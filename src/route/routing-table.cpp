/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 * Regents of the University of California
 */

 #include "routing-table.hpp"
 #include "name-map.hpp"
 #include "routing-calculator.hpp"
 #include "routing-table-entry.hpp"
 #include "conf-parameter.hpp"
 #include "logger.hpp"
 #include "nlsr.hpp"
 #include "tlv-nlsr.hpp"
 
 // ✅ 包含具体算法的头文件 (已移除 ML)
 #include "load-aware-routing-calculator.hpp"
 #include "q-learning-calculator.hpp"
 
 namespace nlsr {
 
 INIT_LOGGER(route.RoutingTable);
 
 RoutingTable::RoutingTable(ndn::Scheduler& scheduler, Lsdb& lsdb, ConfParameter& confParam)
   : m_scheduler(scheduler)
   , m_lsdb(lsdb)
   , m_confParam(confParam)
   , m_hyperbolicState(m_confParam.getHyperbolicState())
   , m_routingCalcInterval{confParam.getRoutingCalcInterval()}
   , m_isRoutingTableCalculating(false)
   , m_isRouteCalculationScheduled(false)
   , m_ownAdjLsaExist(false)
   , m_linkCostManager(nullptr)
   , m_loadAwareCalculator(nullptr)
   // ✅ 已移除 m_mlAdaptiveCalculator 初始化
   , m_qLearningCalculator(nullptr)
 {
   m_afterLsdbModified = lsdb.onLsdbModified.connect(
     [this] (std::shared_ptr<Lsa> lsa, LsdbUpdate updateType,
             const auto& namesToAdd, const auto& namesToRemove) {
       auto type = lsa->getType();
       bool updateForOwnAdjacencyLsa = lsa->getOriginRouter() == m_confParam.getRouterPrefix() &&
                                       type == Lsa::Type::ADJACENCY;
       bool scheduleCalculation = false;
 
       if (updateType == LsdbUpdate::REMOVED && updateForOwnAdjacencyLsa) {
         NLSR_LOG_DEBUG("No Adj LSA of router itself, routing table can not be calculated :(");
         clearRoutingTable();
         clearDryRoutingTable();
         NLSR_LOG_DEBUG("Calling Update NPT With new Route");
         afterRoutingChange(m_rTable);
         m_ownAdjLsaExist = false;
       }
 
       if (updateType == LsdbUpdate::INSTALLED && updateForOwnAdjacencyLsa) {
         m_ownAdjLsaExist = true;
       }
 
       if (updateType == LsdbUpdate::INSTALLED || updateType == LsdbUpdate::UPDATED) {
         if ((type == Lsa::Type::ADJACENCY  && m_hyperbolicState != HYPERBOLIC_STATE_ON) ||
             (type == Lsa::Type::COORDINATE && m_hyperbolicState != HYPERBOLIC_STATE_OFF)) {
           scheduleCalculation = true;
         }
       }
 
       if (scheduleCalculation) {
         scheduleRoutingTableCalculation();
       }
     }
   );
 }
 
 RoutingTable::~RoutingTable()
 {
   m_afterLsdbModified.disconnect();
   // unique_ptr 会自动释放 Calculator 对象
 }
 
 void RoutingTable::calculate()
 {
   m_lsdb.writeLog();
   NLSR_LOG_TRACE("Calculating routing table");
 
   if (m_isRoutingTableCalculating == false) {
     m_isRoutingTableCalculating = true;
 
     // ✅ 优先级逻辑：Q-Learning > LoadAware > Standard
     // (已移除 ML 分支)
     if (m_confParam.getQLearningRouting()) {
       NLSR_LOG_INFO("Using Q-Learning routing algorithm");
       calculateQLearningRoutingTable();
     }
     else if (m_confParam.getLoadAwareRouting()) {
       NLSR_LOG_INFO("Using load-aware routing algorithm");
       calculateLoadAwareRoutingTable();
     }
     else if (m_hyperbolicState == HYPERBOLIC_STATE_OFF) {
       NLSR_LOG_INFO("Using standard link-state routing algorithm");
       calculateLsRoutingTable();
     }
     else if (m_hyperbolicState == HYPERBOLIC_STATE_DRY_RUN) {
       NLSR_LOG_INFO("Using hyperbolic routing (dry-run mode)");
       calculateLsRoutingTable();
       calculateHypRoutingTable(true);
     }
     else if (m_hyperbolicState == HYPERBOLIC_STATE_ON) {
       NLSR_LOG_INFO("Using hyperbolic routing algorithm");
       calculateHypRoutingTable(false);
     }
 
     m_isRouteCalculationScheduled = false;
     m_isRoutingTableCalculating = false;
   }
   else {
     scheduleRoutingTableCalculation();
   }
 }
 
 // ✅ Q-Learning 路由计算
 void
 RoutingTable::calculateQLearningRoutingTable()
 {
   NLSR_LOG_TRACE("CalculateQLearningRoutingTable Called");
 
   if (m_lsdb.getIsBuildAdjLsaScheduled()) {
     NLSR_LOG_DEBUG("Adjacency build is scheduled, skipping calculation");
     return;
   }
   
   if (!m_ownAdjLsaExist) {
     return;
   }
 
   clearRoutingTable();
   
   if (m_linkCostManager == nullptr) {
     NLSR_LOG_WARN("LinkCostManager missing, fallback to standard");
     calculateLsRoutingTable();
     return;
   }
 
   auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
   auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
 
   // 懒加载 Q-Learning 计算器
   if (!m_qLearningCalculator) {
     NLSR_LOG_INFO("Initializing persistent QLearningCalculator");
     m_qLearningCalculator = std::make_unique<QLearningCalculator>(*m_linkCostManager);
   }
 
   m_qLearningCalculator->calculatePath(map, *this, m_confParam, m_lsdb);
 
   NLSR_LOG_DEBUG("Calling Update NPT With new Route (Q-Learning)");
   afterRoutingChange(m_rTable);
 }
 
 // ✅ 负载感知路由计算
 void
 RoutingTable::calculateLoadAwareRoutingTable()
 {
   NLSR_LOG_TRACE("CalculateLoadAwareRoutingTable Called");
 
   if (m_lsdb.getIsBuildAdjLsaScheduled()) {
     NLSR_LOG_DEBUG("Adjacency build is scheduled, routing table can not be calculated :(");
     return;
   }
 
   if (!m_ownAdjLsaExist) {
     return;
   }
 
   clearRoutingTable();
   
   if (m_linkCostManager == nullptr) {
     NLSR_LOG_WARN("LinkCostManager not available, falling back to standard routing");
     calculateLsRoutingTable();
     return;
   }
 
   auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
   auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
 
   // 懒加载 LoadAware 计算器
   if (!m_loadAwareCalculator) {
     NLSR_LOG_INFO("Creating persistent LoadAwareRoutingCalculator (first time)");
     m_loadAwareCalculator = std::make_unique<LoadAwareRoutingCalculator>(*m_linkCostManager);
   }
 
   m_loadAwareCalculator->calculatePath(map, *this, m_confParam, m_lsdb);
 
   NLSR_LOG_DEBUG("Calling Update NPT With new Route");
   afterRoutingChange(m_rTable);
 }
 
 void
 RoutingTable::calculateLsRoutingTable()
 {
   NLSR_LOG_TRACE("CalculateLsRoutingTable Called");
 
   if (m_lsdb.getIsBuildAdjLsaScheduled()) {
     NLSR_LOG_DEBUG("Adjacency build is scheduled, routing table can not be calculated :(");
     return;
   }
 
   if (!m_ownAdjLsaExist) {
     return;
   }
 
   clearRoutingTable();
 
   auto lsaRange = m_lsdb.getLsdbIterator<AdjLsa>();
   auto map = NameMap::createFromAdjLsdb(lsaRange.first, lsaRange.second);
   NLSR_LOG_DEBUG(map);
 
   calculateLinkStateRoutingPath(map, *this, m_confParam, m_lsdb);
 
   NLSR_LOG_DEBUG("Calling Update NPT With new Route");
   afterRoutingChange(m_rTable);
 }
 
 void
 RoutingTable::calculateHypRoutingTable(bool isDryRun)
 {
   if (isDryRun) {
     clearDryRoutingTable();
   }
   else {
     clearRoutingTable();
   }
 
   auto lsaRange = m_lsdb.getLsdbIterator<CoordinateLsa>();
   auto map = NameMap::createFromCoordinateLsdb(lsaRange.first, lsaRange.second);
   NLSR_LOG_DEBUG(map);
 
   calculateHyperbolicRoutingPath(map, *this, m_lsdb, m_confParam.getAdjacencyList(),
                                  m_confParam.getRouterPrefix(), isDryRun);
 
   if (!isDryRun) {
     NLSR_LOG_DEBUG("Calling Update NPT With new Route");
     afterRoutingChange(m_rTable);
     NLSR_LOG_DEBUG(*this);
   }
 }
 
 void
 RoutingTable::scheduleRoutingTableCalculation()
 {
   if (!m_isRouteCalculationScheduled) {
     NLSR_LOG_DEBUG("Scheduling routing table calculation in " << m_routingCalcInterval);
     m_scheduler.schedule(m_routingCalcInterval, [this] { calculate(); });
     m_isRouteCalculationScheduled = true;
   }
 }
 
 static bool
 routingTableEntryCompare(RoutingTableEntry& rte, ndn::Name& destRouter)
 {
   return rte.getDestination() == destRouter;
 }
 
 void
 RoutingTable::addNextHop(const ndn::Name& destRouter, NextHop& nh)
 {
   NLSR_LOG_DEBUG("Adding " << nh << " for destination: " << destRouter);
 
   RoutingTableEntry* rteChk = findRoutingTableEntry(destRouter);
   if (rteChk == nullptr) {
     RoutingTableEntry rte(destRouter);
     rte.getNexthopList().addNextHop(nh);
     m_rTable.push_back(rte);
   }
   else {
     rteChk->getNexthopList().addNextHop(nh);
   }
   m_wire.reset();
 }
 
 RoutingTableEntry*
 RoutingTable::findRoutingTableEntry(const ndn::Name& destRouter)
 {
   auto it = std::find_if(m_rTable.begin(), m_rTable.end(),
                          std::bind(&routingTableEntryCompare, _1, destRouter));
   if (it != m_rTable.end()) {
     return &(*it);
   }
   return nullptr;
 }
 
 void
 RoutingTable::addNextHopToDryTable(const ndn::Name& destRouter, NextHop& nh)
 {
   NLSR_LOG_DEBUG("Adding " << nh << " to dry table for destination: " << destRouter);
 
   auto it = std::find_if(m_dryTable.begin(), m_dryTable.end(),
                          std::bind(&routingTableEntryCompare, _1, destRouter));
   if (it == m_dryTable.end()) {
     RoutingTableEntry rte(destRouter);
     rte.getNexthopList().addNextHop(nh);
     m_dryTable.push_back(rte);
   }
   else {
     it->getNexthopList().addNextHop(nh);
   }
   m_wire.reset();
 }
 
 void
 RoutingTable::clearRoutingTable()
 {
   m_rTable.clear();
   m_wire.reset();
 }
 
 void
 RoutingTable::clearDryRoutingTable()
 {
   m_dryTable.clear();
   m_wire.reset();
 }
 
 template<ndn::encoding::Tag TAG>
 size_t
 RoutingTableStatus::wireEncode(ndn::EncodingImpl<TAG>& block) const
 {
   size_t totalLength = 0;
 
   for (auto it = m_dryTable.rbegin(); it != m_dryTable.rend(); ++it) {
     totalLength += it->wireEncode(block);
   }
 
   for (auto it = m_rTable.rbegin(); it != m_rTable.rend(); ++it) {
     totalLength += it->wireEncode(block);
   }
 
   totalLength += block.prependVarNumber(totalLength);
   totalLength += block.prependVarNumber(nlsr::tlv::RoutingTable);
 
   return totalLength;
 }
 
 NDN_CXX_DEFINE_WIRE_ENCODE_INSTANTIATIONS(RoutingTableStatus);
 
 const ndn::Block&
 RoutingTableStatus::wireEncode() const
 {
   if (m_wire.hasWire()) {
     return m_wire;
   }
 
   ndn::EncodingEstimator estimator;
   size_t estimatedSize = wireEncode(estimator);
 
   ndn::EncodingBuffer buffer(estimatedSize, 0);
   wireEncode(buffer);
 
   m_wire = buffer.block();
 
   return m_wire;
 }
 
 void
 RoutingTableStatus::wireDecode(const ndn::Block& wire)
 {
   m_rTable.clear();
 
   m_wire = wire;
 
   if (m_wire.type() != nlsr::tlv::RoutingTable) {
     NDN_THROW(Error("RoutingTable", m_wire.type()));
   }
 
   m_wire.parse();
   auto val = m_wire.elements_begin();
 
   std::set<ndn::Name> destinations;
   for (; val != m_wire.elements_end() && val->type() == nlsr::tlv::RoutingTableEntry; ++val) {
     auto entry = RoutingTableEntry(*val);
 
     if (destinations.emplace(entry.getDestination()).second) {
       m_rTable.push_back(entry);
     }
     else {
       m_dryTable.push_back(entry);
     }
   }
 
   if (val != m_wire.elements_end()) {
     NDN_THROW(Error("Unrecognized TLV of type " + ndn::to_string(val->type()) + " in RoutingTable"));
   }
 }
 
 std::ostream&
 operator<<(std::ostream& os, const RoutingTableStatus& rts)
 {
   os << "Routing Table:\n";
   for (const auto& rte : rts.getRoutingTableEntry()) {
     os << rte;
   }
 
   if (!rts.getDryRoutingTableEntry().empty()) {
     os << "Dry-Run Hyperbolic Routing Table:\n";
     for (const auto& rte : rts.getDryRoutingTableEntry()) {
       os << rte;
     }
   }
   return os;
 }
 
 } // namespace nlsr