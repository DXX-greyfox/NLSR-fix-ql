/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 * Regents of the University of California
 */

 #ifndef NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP
 #define NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP
 
 #include "route/routing-table.hpp"
 #include "link-cost-manager.hpp"
 #include "common.hpp"
 
 #include <ndn-cxx/util/time.hpp>
 #include <ndn-cxx/util/signal.hpp> // ✅ 新增：用于 ScopedConnection
 #include <unordered_map>
 #include <deque>
 
 #include "adjacency-list.hpp"
 #include "lsdb.hpp"
 
 namespace nlsr {
 
 // 前向声明
 class NameMap;
 class ConfParameter;
 class Lsdb;
 
 /**
  * @brief 负载感知路由计算器辅助类
  */
 class LoadAwareRoutingCalculator {
 public:
   explicit LoadAwareRoutingCalculator(LinkCostManager& linkCostManager);
   ~LoadAwareRoutingCalculator();
   
   // 路由计算入口（实际上只是代理调用标准计算，因为Cost已经在后台更新了）
   void calculatePath(NameMap& map, RoutingTable& rt, 
                     ConfParameter& confParam, const Lsdb& lsdb);
 
 private:
   // ✅ 新增：信号处理函数
   void processLinkUpdates(const ndn::Name& neighbor, const LinkCostManager::LinkMetrics& metrics);
 
   // 核心计算逻辑
   double calculateLoadAwareCost(const ndn::Name& neighbor, double baseCost, 
                                const LinkCostManager::LinkMetrics& metrics);
   
   double getRttFactor(const LinkCostManager::LinkMetrics& metrics);
   double getLoadFactor(const LinkCostManager::LinkMetrics& metrics);
   double getStabilityFactor(const LinkCostManager::LinkMetrics& metrics);
   
   void updateRttHistory(const ndn::Name& neighbor, double currentRttMs);
 
 private:
   LinkCostManager& m_linkCostManager;
   
   // ✅ 新增：管理信号连接
   ndn::signal::ScopedConnection m_signalConnection;
 
   double m_rttWeight = 0.3;
   double m_loadWeight = 0.4;
   double m_stabilityWeight = 0.3;
   
   std::unordered_map<ndn::Name, std::deque<double>> m_rttHistory;
   static constexpr size_t MAX_RTT_HISTORY = 10;
   
   uint64_t m_calculationCount = 0;
   uint64_t m_costAdjustmentCount = 0;
 };
 
 } // namespace nlsr
 
 #endif // NLSR_LOAD_AWARE_ROUTING_CALCULATOR_HPP