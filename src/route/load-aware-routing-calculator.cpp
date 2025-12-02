/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 * Regents of the University of California
 */

 #include "load-aware-routing-calculator.hpp"
 #include "routing-calculator.hpp"
 #include "name-map.hpp"
 #include "nexthop.hpp"
 #include "adjacent.hpp"
 #include "logger.hpp"
 #include "conf-parameter.hpp"
 
 #include <boost/multi_array.hpp>
 #include <algorithm>
 #include <cmath>
 
 namespace nlsr {
 
 INIT_LOGGER(route.LoadAwareRoutingCalculator);
 
 // 算法常量
 static constexpr double RTT_THRESHOLD_EXCELLENT = 10.0;  // ms
 static constexpr double RTT_THRESHOLD_GOOD = 50.0;       // ms
 static constexpr double RTT_THRESHOLD_FAIR = 100.0;      // ms
 static constexpr double RTT_THRESHOLD_POOR = 200.0;      // ms
 
 
 LoadAwareRoutingCalculator::LoadAwareRoutingCalculator(LinkCostManager& linkCostManager)
   : m_linkCostManager(linkCostManager)
  {
   //核心：改为订阅 LinkCostManager 的信号
   m_signalConnection = m_linkCostManager.onLinkMetricsUpdated.connect(
     [this](const ndn::Name& neighbor, const LinkCostManager::LinkMetrics& metrics) {
       this->processLinkUpdates(neighbor, metrics);
     });
   
   NLSR_LOG_INFO("LoadAwareRoutingCalculator: Initialized and listening to LinkCostManager updates");
  }
 
 LoadAwareRoutingCalculator::~LoadAwareRoutingCalculator()
  {
   // ScopedConnection 会自动断开连接
   NLSR_LOG_INFO("LoadAwareRoutingCalculator: Destroyed");
  }
 
 void
 LoadAwareRoutingCalculator::calculatePath(NameMap& map, RoutingTable& rt,ConfParameter& confParam, const Lsdb& lsdb)
  {
   NLSR_LOG_DEBUG("LoadAwareRoutingCalculator::calculatePath called");
   ++m_calculationCount;
   
   // 这里的逻辑不变，依然是调用标准路由计算
   calculateLinkStateRoutingPath(map, rt, confParam, lsdb);
   
   NLSR_LOG_DEBUG("Load-aware routing calculation completed. Adjustments: " << m_costAdjustmentCount);
  }
 
 //核心：处理链路更新信号
 void 
 LoadAwareRoutingCalculator::processLinkUpdates(const ndn::Name& neighbor, 
                                               const LinkCostManager::LinkMetrics& metrics)
 {
   // 使用 originalCost 作为基础进行调整
   double newCost = calculateLoadAwareCost(neighbor, metrics.originalCost, metrics);
   
   // 主动更新 LinkCostManager
   m_linkCostManager.updateNeighborCost(neighbor, newCost);
 }
 
 double
 LoadAwareRoutingCalculator::calculateLoadAwareCost(const ndn::Name& neighbor, double baseCost, const LinkCostManager::LinkMetrics& metrics)
  {
   // 如果基础成本无效，直接返回
   if (baseCost <= 0) {
     return baseCost;
   }
   
   // 计算各种影响因子
   double rttFactor = getRttFactor(metrics);
   double loadFactor = getLoadFactor(metrics);
   double stabilityFactor = getStabilityFactor(metrics);
   
   // 计算综合调整因子
   double adjustmentFactor = m_rttWeight * rttFactor + 
                            m_loadWeight * loadFactor +
                            m_stabilityWeight * stabilityFactor;
   
   // 应用调整因子到基础成本
   double adjustedCost = baseCost * (1.0 + adjustmentFactor);
   
   // 限制在合理范围内
   adjustedCost = std::min(adjustedCost, metrics.originalCost * 3.0);
   adjustedCost = std::max(adjustedCost, metrics.originalCost * 0.5);
   
   // 更新RTT历史（用于下次负载计算）
   if (metrics.currentRtt) {
     auto rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
     updateRttHistory(neighbor, rttMs);
   }
   
   ++m_costAdjustmentCount;
   
   NLSR_LOG_TRACE("Load-aware cost for " << neighbor
                 << ": Base=" << baseCost
                 << ", factors(rtt=" << rttFactor 
                 << ", load=" << loadFactor
                 << ", stability=" << stabilityFactor << ")"
                 << ", final=" << adjustedCost);
   
   return adjustedCost;
  }
 
 double
 LoadAwareRoutingCalculator::getRttFactor(const LinkCostManager::LinkMetrics& metrics)
 {
   if (!metrics.currentRtt) {
     return 0.0; // 没有RTT数据，不调整
   }
   
   auto rttMsValue = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
   
   if (rttMsValue <= RTT_THRESHOLD_EXCELLENT) {
     return 0.0;  // 优秀
   } else if (rttMsValue <= RTT_THRESHOLD_GOOD) {
     return 0.2;  // 良好
   } else if (rttMsValue <= RTT_THRESHOLD_FAIR) {
     return 0.5;  // 一般
   } else if (rttMsValue <= RTT_THRESHOLD_POOR) {
     return 1.0;  // 较差
   } else {
     return 2.0;  // 很差
   }
 }
 
 double
 LoadAwareRoutingCalculator::getLoadFactor(const LinkCostManager::LinkMetrics& metrics)
 {
   // 优先使用metrics中的RTT历史更新本地历史
   if (metrics.currentRtt) {
     auto rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
     updateRttHistory(metrics.neighbor, rttMs);
   }
   
   // 基于RTT历史计算负载因子（变化率）
   auto it = m_rttHistory.find(metrics.neighbor);
   if (it == m_rttHistory.end() || it->second.size() < 3) {
     return 0.0; // 数据不足，不调整
   }
   
   const auto& history = it->second;
   
   // 计算均值和标准差
   double sum = 0.0;
   for (double rtt : history) {
     sum += rtt;
   }
   double mean = sum / history.size();
   
   double variance = 0.0;
   for (double rtt : history) {
     variance += (rtt - mean) * (rtt - mean);
   }
   double stddev = std::sqrt(variance / history.size());
   
   // 计算变化率（负载指标）
   double variationRate = (mean > 0) ? (stddev / mean) : 0.0;
   
   if (variationRate <= 0.1) {
     return 0.0;  // 很稳定，低负载
   } else if (variationRate <= 0.2) {
     return 0.3;  // 稳定
   } else if (variationRate <= 0.5) {
     return 0.7;  // 一般
   } else {
     return 1.5;  // 不稳定，高负载
   }
 }
 
 double
 LoadAwareRoutingCalculator::getStabilityFactor(const LinkCostManager::LinkMetrics& metrics)
 {
   double factor = 0.0;
   
   // 1. 精准丢包惩罚 (基于滑动窗口)
   if (metrics.packetLoss) {
     double plr = *metrics.packetLoss;
     factor += plr * 2.0;
     NLSR_LOG_TRACE("  > PLR penalty: " << plr << " -> " << (plr * 2.0));
   }
   
   // 2. 严重断连惩罚 (基于 Hello Timeout)
   if (metrics.timeoutCount) {
     factor += *metrics.timeoutCount * 0.5;
   }
 
   // 3. 时间惩罚
   if (metrics.lastSuccessTime) {
     auto now = ndn::time::steady_clock::now();
     auto timeSinceSuccess = now - *metrics.lastSuccessTime;
     auto secondsSinceSuccess = ndn::time::duration_cast<ndn::time::seconds>(timeSinceSuccess).count();
     
     if (secondsSinceSuccess > 60) { // 超过1分钟
       factor += std::min(2.0, secondsSinceSuccess / 60.0 * 0.1);
     }
   }
   
   return factor;
 }
 
 void
 LoadAwareRoutingCalculator::updateRttHistory(const ndn::Name& neighbor, double currentRttMs)
 {
   auto& history = m_rttHistory[neighbor];
   history.push_back(currentRttMs);
   
   // 保持历史记录大小在限制范围内
   while (history.size() > MAX_RTT_HISTORY) {
     history.pop_front();
   }
 }
 
 } // namespace nlsr