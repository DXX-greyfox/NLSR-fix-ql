/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 * Regents of the University of California,
 * Arizona Board of Regents.
 */

 #ifndef NLSR_Q_LEARNING_CALCULATOR_HPP
 #define NLSR_Q_LEARNING_CALCULATOR_HPP
 
 #include "common.hpp"
 #include "link-cost-manager.hpp"
 #include "route/routing-table.hpp"
 
 #include <vector>
 #include <map>
 #include <random>
 #include <ndn-cxx/util/signal.hpp> // ✅ 新增：用于 ScopedConnection
 
 namespace nlsr {
 
 // 前向声明
 class NameMap;
 class ConfParameter; 
 class Lsdb;
 
 /**
  * @brief 基于 Q-Learning 的智能路由计算器
  * * 该类实现了一个强化学习代理，通过与网络环境交互（调整 Cost -> 观察 RTT/丢包）
  * 来学习最优的链路成本调整策略。
  */
 class QLearningCalculator {
 public:
   explicit QLearningCalculator(LinkCostManager& linkCostManager);
   ~QLearningCalculator();
 
   /**
    * @brief 执行路由计算（入口函数）
    */
   void calculatePath(NameMap& map, RoutingTable& rt, 
                      ConfParameter& confParam, const Lsdb& lsdb);
 
 private:
   // ================= 核心定义 =================
 
   // 状态定义：RTT状态 (3) x 丢包状态 (2) = 6种状态
   enum class RttState { GOOD, MEDIUM, POOR };     // 优(<50ms), 良(50-200ms), 差(>200ms)
   enum class LossState { LOW, HIGH };             // 低(<1%), 高(>=1%)
   
   struct State {
     RttState rtt;
     LossState loss;
     
     // 为了让 State 可以作为 map 的 key 或数组索引
     int toIndex() const {
       return static_cast<int>(rtt) * 2 + static_cast<int>(loss);
     }
   };
 
   // 动作定义
   enum class Action {
     MAINTAIN = 0,   // 保持当前 Cost
     PENALIZE = 1,   // 增加 Cost (x1.5)
     REWARD = 2      // 减少 Cost (x0.8)
   };
 
   static constexpr int STATE_COUNT = 6;
   static constexpr int ACTION_COUNT = 3;
 
   // ================= 核心逻辑函数 =================
 
   // ✅ 核心修改：作为信号回调的主处理函数
   void processLinkUpdates(const ndn::Name& neighbor, const LinkCostManager::LinkMetrics& metrics);

   // 1. 获取当前状态 (✅ 修改：直接使用传入的 metrics)
   State getCurrentState(const LinkCostManager::LinkMetrics& metrics);
 
   // 2. 选择动作 (Epsilon-Greedy 策略)
   Action selectAction(const State& state);
 
   // 3. 执行动作 (计算新的 Cost)
   double applyAction(Action action, double originalCost);
 
   // 4. 计算奖励 (基于环境反馈)
   double calculateReward(const LinkCostManager::LinkMetrics& metrics);
 
   // 5. 更新 Q 表 (学习过程)
   void updateQTable(const State& state, Action action, double reward, const State& nextState);
 
   // 6. 启发式初始化 (让算法赢在起跑线)
   void initializeQTable();
 
   // ================= 数据成员 =================
 
   LinkCostManager& m_linkCostManager;
   
   // ✅ 新增：管理信号连接的生命周期
   ndn::signal::ScopedConnection m_signalConnection;

   // Q-Table: [状态][动作] -> 价值
   // 使用原生数组以获得最高效率
   double m_qTable[STATE_COUNT][ACTION_COUNT];
 
   // 学习参数
   double m_learningRate = 0.1;   // Alpha: 学习速率
   double m_discountFactor = 0.1; // Gamma: 对未来奖励的重视程度
   double m_epsilon = 0.2;        // Epsilon: 探索概率 (初始值)

   // ✅ 新增：Epsilon 衰减参数 (建议 B)
   double m_epsilonMin = 0.01;    // 探索率下限
   double m_epsilonDecay = 0.995; // 每次更新后的衰减系数

   // 记录上一次对每个邻居采取的动作和状态，用于下一次更新 Q 表
   struct StepRecord {
     State state;
     Action action;
     ndn::time::steady_clock::time_point timestamp;
   };
   std::map<ndn::Name, StepRecord> m_history;
 
   // 随机数生成器
   std::mt19937 m_rng;
   
   // 预热/暗中学习标志
   bool m_isStable = false;
   uint64_t m_learnCycles = 0;
 };
 
 } // namespace nlsr
 
 #endif // NLSR_Q_LEARNING_CALCULATOR_HPP