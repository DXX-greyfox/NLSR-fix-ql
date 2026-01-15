/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 * Regents of the University of California,
 * Arizona Board of Regents.
 */
 #include "q-learning-calculator.hpp"
 #include "logger.hpp"
 #include "conf-parameter.hpp"
 #include "lsdb.hpp"
 #include "name-map.hpp"
 #include "routing-calculator.hpp"
 #include <algorithm>
 #include <cmath>
 #include <limits>
 
 namespace nlsr {
 
 INIT_LOGGER(route.QLearningCalculator);

 
 QLearningCalculator::QLearningCalculator(LinkCostManager& linkCostManager)
   : m_linkCostManager(linkCostManager)
   , m_rng(std::random_device{}())
 {
   initializeQTable();
 
   //核心：改为订阅 LinkCostManager 的信号
   // 不再使用 setLoadAwareCostCalculator 注册回调
   m_signalConnection = m_linkCostManager.onLinkMetricsUpdated.connect(
     [this](const ndn::Name& neighbor, const LinkCostManager::LinkMetrics& metrics) {
        this->processLinkUpdates(neighbor, metrics);
     });
     
   NLSR_LOG_INFO("QLearningCalculator initialized and listening to LinkCostManager updates.");
 }
 
 QLearningCalculator::~QLearningCalculator()
 {
   // ✅ 核心修改：无需手动清除回调
   // m_signalConnection 会在析构时自动断开连接，安全无忧
   NLSR_LOG_INFO("QLearningCalculator destroyed.");
 }
 
 void
 QLearningCalculator::calculatePath(NameMap& map, RoutingTable& rt, 
                                    ConfParameter& confParam, const Lsdb& lsdb)
 {
   // Q-Learning 只负责计算 Cost，路径计算仍然交给标准的 Dijkstra
   // 这里只是一个代理调用，但未来可以在这里加入策略路由逻辑
   calculateLinkStateRoutingPath(map, rt, confParam, lsdb);
 }

 // ✅ 新增：处理链路更新的主逻辑（原 Lambda 内容迁移至此）
 void 
 QLearningCalculator::processLinkUpdates(const ndn::Name& neighbor, 
                                        const LinkCostManager::LinkMetrics& metrics)
 {
    if (metrics.status != Adjacent::STATUS_ACTIVE) {
      NLSR_LOG_TRACE("QLearning: Neighbor " << neighbor << " is not active. Skipping.");
      return;
    }
    // 1. 观察环境 (Observe)
    State currentState = getCurrentState(metrics);
    
    // 2. 计算奖励 (Reward) - 基于当前链路表现
    double reward = calculateReward(metrics);
    
    // 3. 学习 (Learn) - 如果有历史动作，根据当前结果更新旧动作的 Q 值
    auto historyIt = m_history.find(neighbor);
    if (historyIt != m_history.end()) {
      const StepRecord& lastStep = historyIt->second;
      
      // Q-Learning 核心更新公式：
      // Q(S,A) += alpha * [R + gamma * max(Q(S', all_a')) - Q(S,A)]
      updateQTable(lastStep.state, lastStep.action, reward, currentState);

      if (m_epsilon > m_epsilonMin) {
        m_epsilon *= m_epsilonDecay;
      }
      m_learnCycles++;
    }
    
    // 4. 决策 (Decide) - Epsilon-Greedy 策略
    Action newAction = selectAction(currentState);
    
    // 5. 执行 (Act) - 计算新的 Link Cost
    // 注意：我们基于原始配置成本进行调整，以保证策略的可解释性
    double newCost = applyAction(newAction, metrics.originalCost);
    
    // 6. 记录 (Record) - 存入历史供下一轮学习
    StepRecord newRecord;
    newRecord.state = currentState;
    newRecord.action = newAction;
    newRecord.timestamp = ndn::time::steady_clock::now();
    m_history[neighbor] = newRecord;
    
    NLSR_LOG_TRACE("QLearning " << neighbor << ": "
                   << "S=[" << (int)currentState.rtt << "," << (int)currentState.loss << "] "
                   << "-> R=" << reward 
                   << " -> A=" << (int)newAction 
                   << " -> Cost=" << newCost);
    
    // ✅ 7. 核心修改：主动调用 LCM 接口更新 Cost
    // 完成闭环：算法 -> LCM
    m_linkCostManager.updateNeighborCost(neighbor, newCost);
 }
 
 // ============================================================================
 // 核心逻辑实现
 // ============================================================================
 
 // ✅ 修改：直接使用传入的 metrics，避免重复查询
 QLearningCalculator::State 
 QLearningCalculator::getCurrentState(const LinkCostManager::LinkMetrics& metrics)
 {
   State s;
   
   // 1. RTT 状态判定
   double rttMs = 9999.0;
   if (metrics.currentRtt) {
     rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
   }
   
   if (rttMs < 50.0) s.rtt = RttState::GOOD;
   else if (rttMs < 200.0) s.rtt = RttState::MEDIUM;
   else s.rtt = RttState::POOR;
   
   // 2. 丢包状态判定
   double loss = metrics.packetLoss.value_or(0.0);
   
   // 阈值设为 1% (0.01)
   if (loss < 0.01) s.loss = LossState::LOW;
   else s.loss = LossState::HIGH;
   return s;
 }
 
 double 
 QLearningCalculator::calculateReward(const LinkCostManager::LinkMetrics& metrics)
 {
   // 获取 RTT 和丢包率
   double loss = metrics.packetLoss.value_or(0.0);
   double rttMs = 100.0;
   if (metrics.currentRtt) {
     rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
   }
 
   // 1. RTT 评分 (0.0 - 1.0)
   // <50ms 为满分，>200ms 为 0 分，中间线性插值 
   double rttScore = 0.0;
   if (rttMs <= 50.0) rttScore = 1.0;
   else if (rttMs >= 200.0) rttScore = 0.0;
   else rttScore = 1.0 - ((rttMs - 50.0) / 150.0);
   
   // 2. 丢包评分 (0.0 - 1.0)
   // >10% 丢包直接 0 分 
   double lossScore = 0.0;
   if (loss >= 0.1) lossScore = 0.0;
   else lossScore = 1.0 - (loss / 0.1);
 
   // 3. 综合奖励并限幅 (Range: -5.0 to 5.0) 
   // 基础分：RTT占6成，丢包占4成。减去0.5是为了让结果有正有负。
   double rawReward = (0.6 * rttScore + 0.4 * lossScore - 0.5) * 10.0;
   
   // 丢包严重时的额外惩罚
   if (loss > 0.05) {
       rawReward -= 5.0;
   }
 
   // 严格限幅
   return std::max(-5.0, std::min(5.0, rawReward));
 }
 
 void 
 QLearningCalculator::updateQTable(const State& state, Action action, 
                                  double reward, const State& nextState)
 {
   int s_idx = state.toIndex();
   int a_idx = static_cast<int>(action);
   int next_s_idx = nextState.toIndex();
   
   // 寻找下一状态的最大 Q 值 (Max Q)
   double maxNextQ = -9999.0;
   for (int a = 0; a < ACTION_COUNT; ++a) {
     if (m_qTable[next_s_idx][a] > maxNextQ) {
       maxNextQ = m_qTable[next_s_idx][a];
     }
   }
   
   // Bellman 更新公式
   double currentQ = m_qTable[s_idx][a_idx];
   double newQ = currentQ + m_learningRate * (reward + m_discountFactor * maxNextQ - currentQ);
   
   m_qTable[s_idx][a_idx] = newQ;
 }
 
 QLearningCalculator::Action 
 QLearningCalculator::selectAction(const State& state)
 {
   // Epsilon-Greedy: 探索 vs 利用
   std::uniform_real_distribution<double> dist(0.0, 1.0);
   if (dist(m_rng) < m_epsilon) {
     // 探索：随机选择动作
     std::uniform_int_distribution<int> actionDist(0, ACTION_COUNT - 1);
     return static_cast<Action>(actionDist(m_rng));
   }
   
   // 利用：选择当前 Q 值最大的动作
   int s_idx = state.toIndex();
   int bestAction = 0;
   double maxQ = -99999.0;
   
   for (int a = 0; a < ACTION_COUNT; ++a) {
     if (m_qTable[s_idx][a] > maxQ) {
       maxQ = m_qTable[s_idx][a];
       bestAction = a;
     }
   }
   
   return static_cast<Action>(bestAction);
 }
 
 double 
 QLearningCalculator::applyAction(Action action, double originalCost)
 {
   switch (action) {
     case Action::MAINTAIN:
       return originalCost;
     case Action::PENALIZE:
       // 惩罚：增加 50% 开销，抑制流量
       return originalCost * 1.2;
     case Action::REWARD:
       // 奖励：减少 20% 开销，吸引流量
       return std::max(1.0, originalCost * 0.9);
     default:
       return originalCost;
   }
 }
 
 void 
 QLearningCalculator::initializeQTable()
 {
   // 启发式初始化：给 Agent 植入“常识”
  
   for (int s = 0; s < STATE_COUNT; ++s) {
     for (int a = 0; a < ACTION_COUNT; ++a) {
       m_qTable[s][a] = 0.0; // 默认值
     }
   }
   
   // 定义状态索引辅助函数
   auto idx = [](RttState r, LossState l) { 
     return static_cast<int>(r) * 2 + static_cast<int>(l); 
   };
   
   // 常识 1: 网络很差 (POOR + HIGH LOSS) -> 应该惩罚
   int badState = idx(RttState::POOR, LossState::HIGH);
   m_qTable[badState][static_cast<int>(Action::PENALIZE)] = 10.0; // 高分推荐
   m_qTable[badState][static_cast<int>(Action::REWARD)] = -10.0;  // 极力反对
   
   // 常识 2: 网络很好 (GOOD + LOW LOSS) -> 应该奖励
   int goodState = idx(RttState::GOOD, LossState::LOW);
   m_qTable[goodState][static_cast<int>(Action::REWARD)] = 5.0;
   
   NLSR_LOG_DEBUG("Q-Table initialized with heuristics.");
 }
 
 } // namespace nlsr