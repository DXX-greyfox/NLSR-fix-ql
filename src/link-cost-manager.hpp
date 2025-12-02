/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 * Regents of the University of California,
 * Arizona Board of Regents.
 *
 * This file is part of NLSR (Named-data Link State Routing).
 * See AUTHORS.md for complete list of NLSR authors and contributors.
 *
 * NLSR is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NLSR is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NLSR, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

 #ifndef NLSR_LINK_COST_MANAGER_HPP
 #define NLSR_LINK_COST_MANAGER_HPP
 
 #include "adjacency-list.hpp"
 #include "lsdb.hpp"
 #include "route/routing-table.hpp"
 #include "conf-parameter.hpp"
 #include "common.hpp"
 
 #include <ndn-cxx/face.hpp>
 #include <ndn-cxx/security/key-chain.hpp>
 #include <ndn-cxx/util/scheduler.hpp>
 #include <ndn-cxx/util/time.hpp>
 #include <ndn-cxx/util/signal.hpp>
 
 #include <unordered_map>
 #include <deque>
 #include <functional>
 #include <optional>
 
 namespace nlsr {

 class LinkCostManager {
 public:
  // ✅ 链路指标结构体（为负载感知算法提供完整数据）
  struct LinkMetrics {
    ndn::Name neighbor;
    double originalCost;
    double currentCost;
    std::optional<ndn::time::steady_clock::duration> currentRtt;
    std::optional<uint32_t> timeoutCount;
    std::optional<ndn::time::steady_clock::time_point> lastSuccessTime;
    std::vector<ndn::time::steady_clock::duration> rttHistory;
    Adjacent::Status status;
    
    // === 新增：外部设定的多维度指标 ===
    std::optional<double> bandwidth;          // Mbps
    std::optional<double> bandwidthUtil;      // 利用率 0-1
    std::optional<double> packetLoss;         // 丢包率 0-1
    std::optional<double> spectrumStrength;   // 频谱强度 dBm
    
    // === 新增：多因素预览成本（不影响实际路由）===
    std::optional<double> multiDimensionalCostPreview;
  };

   // ML反馈回调类型定义
   using MLFeedbackCallback = std::function<void(const ndn::Name&, double)>;
   // 设置ML反馈回调方法
   void setMLFeedbackCallback(MLFeedbackCallback callback);
   void clearMLFeedbackCallback();
   bool isMLFeedbackEnabled() const { return static_cast<bool>(m_mlFeedbackCallback);}

   /**
    * @brief Simple RTT measurement for single neighbor
    */
   struct RttMeasurement {
     ndn::time::steady_clock::duration rtt;
     ndn::time::steady_clock::time_point timestamp;
     bool isValid;
     
     RttMeasurement(ndn::time::steady_clock::duration r, 
                   ndn::time::steady_clock::time_point t, 
                   bool valid = true)
       : rtt(r), timestamp(t), isValid(valid) {}
   };
 
   /**
    * @brief Outgoing link state tracking
    */
   struct OutgoingLinkState {
     ndn::Name neighbor;
     Adjacent::Status status;
     double originalCost;
     double currentCost;
     uint32_t timeoutCount;
     ndn::time::steady_clock::time_point lastSuccess;
     ndn::time::steady_clock::time_point lastLsaTriggerTime;
     std::deque<RttMeasurement> rttHistory;
     // ========= 新增开始：记录探测结果 =========
     // 滑动窗口：记录最近的探测结果 (true=成功, false=丢包)
     std::deque<bool> probeResults; 
     static constexpr size_t PACKET_WINDOW_SIZE = 10; // 窗口大小：最近10次

     // 辅助函数：计算当前窗口内的丢包率 (0.0 - 1.0)
     double getPacketLossRate() const {
       if (probeResults.empty()) {
         return 0.0; // 没有数据时默认无丢包
       }
       size_t lostCount = std::count(probeResults.begin(), probeResults.end(), false);
       return static_cast<double>(lostCount) / probeResults.size();
     }
     
     // 辅助函数：添加探测结果并维护窗口大小
     void addProbeResult(bool isSuccess) {
       probeResults.push_back(isSuccess);
       if (probeResults.size() > PACKET_WINDOW_SIZE) {
         probeResults.pop_front();
       }
     }
     
     //最大保存样本数量
     static constexpr size_t MAX_RTT_SAMPLES = 2;
     
     bool isStable() const {
       return status == Adjacent::STATUS_ACTIVE && 
              timeoutCount == 0 && 
              (ndn::time::steady_clock::now() - lastSuccess) < ndn::time::minutes(1);
     }
     
     void addRttMeasurement(ndn::time::steady_clock::duration rtt) {
       RttMeasurement measurement(rtt, ndn::time::steady_clock::now(), true);
       rttHistory.push_back(measurement);
       if (rttHistory.size() > MAX_RTT_SAMPLES) {
         rttHistory.pop_front();
       }
     }
     
     ndn::time::steady_clock::duration getAverageRtt() const {
       if (rttHistory.empty()) {
         return ndn::time::milliseconds(0);
       }
       
       auto total = ndn::time::milliseconds(0);
       for (const auto& measurement : rttHistory) {
         total += ndn::time::duration_cast<ndn::time::milliseconds>(measurement.rtt);
       }
       return total / rttHistory.size();
     }
   };

 public:
   LinkCostManager(ndn::Face& face, ndn::KeyChain& keyChain,
                  ConfParameter& confParam, AdjacencyList& adjacencyList, 
                  Lsdb& lsdb, RoutingTable& routingTable, Fib& fib);

   ~LinkCostManager();

  // ✅ 获取链路完整指标
  std::optional<LinkMetrics> getLinkMetrics(const ndn::Name& neighbor) const;

  // ===== 新增：外部指标管理接口 =====
  /**
   * @brief 外部指标结构体（用于nlsrc命令输入）
   */
  struct ExternalMetrics {
    std::optional<double> bandwidth;
    std::optional<double> bandwidthUtil;
    std::optional<double> packetLoss;
    std::optional<double> spectrumStrength;
    ndn::time::steady_clock::time_point lastUpdate;
  };
  
  /**
   * @brief 多维度成本计算配置
   */
  struct MultiDimensionalCostConfig {
    double rttWeight = 0.4;
    double bandwidthWeight = 0.3;
    double reliabilityWeight = 0.2;
    double spectrumWeight = 0.1;
  };
  
  /**
   * @brief 设置邻居的外部指标（通过nlsrc调用）
   * @note 仅影响预览计算，不影响实际路由
   */
  void setExternalMetrics(const ndn::Name& neighbor, const ExternalMetrics& metrics);
  
  /**
   * @brief 获取邻居的完整指标快照（用于nlsrc展示）
   */
  std::optional<LinkMetrics> getMetricsSnapshot(const ndn::Name& neighbor) const;
  
  /**
   * @brief 计算多因素预览成本（不应用到实际路由）
   */
  double calculateMultiDimensionalCostPreview(const ndn::Name& neighbor) const;
  
  /**
   * @brief 获取当前权重配置
   */
  MultiDimensionalCostConfig getMultiDimensionalConfig() const { 
    return m_multiDimConfig; 
  }
 
   /**
    * @brief Initialize the manager with current neighbor list
    */
   void initialize();
 
   /**
    * @brief Start dynamic cost management
    */
   void start();
 
   /**
    * @brief Stop dynamic cost management and restore original costs
    */
   void stop();
 
   /**
    * @brief Check if manager is active
    */
   bool isActive() const { return m_isActive; }

   // 新增：为负载感知路由提供的接口
   /**
    * @brief 获取邻居的RTT历史统计信息
    */
   std::vector<ndn::time::steady_clock::duration> getRttHistory(const ndn::Name& neighbor) const;
  
   /**
    * @brief 获取邻居的超时统计信息
    */
   std::optional<uint32_t> getTimeoutCount(const ndn::Name& neighbor) const;
  
   /**
    * @brief 获取邻居的最后成功时间
    */
   std::optional<ndn::time::steady_clock::time_point> getLastSuccessTime(const ndn::Name& neighbor) const;
  
   /**
    * @brief 获取邻居的原始配置成本
    */
   double getOriginalLinkCost(const ndn::Name& neighbor) const;
 
   // Hello Protocol Event Handlers (called by HelloProtocol)
   void onHelloInterestSent(const ndn::Name& neighbor);
   void onHelloDataReceived(const ndn::Name& neighbor);
   void onHelloTimeout(const ndn::Name& neighbor, uint32_t timeouts);
   void onNeighborStatusChanged(const ndn::Name& neighbor, Adjacent::Status newStatus);
 
   // Debug/Status functions
   double getCurrentCost(const ndn::Name& neighbor) const;
   std::optional<ndn::time::steady_clock::duration> getCurrentRtt(const ndn::Name& neighbor) const;
   std::optional<double> getLinkCost(const ndn::Name& neighbor) const;
   
   //核心：发布-订阅模式的信号
   // 当收集到新的链路数据时发出此信号，由外部算法订阅并处理
   ndn::signal::Signal<LinkCostManager, const ndn::Name&, const LinkMetrics&> onLinkMetricsUpdated;
   //补回缺失的 Cost 变化通知信号 (供 NLSR 主类使用)
   ndn::signal::Signal<LinkCostManager, const ndn::Name&, double> onNeighborCostUpdated;
   //核心：将 Cost 更新接口公开
   // 供外部算法在计算完成后调用，LinkCostManager 只负责执行更新，不负责计算
   void updateNeighborCost(const ndn::Name& neighbor, double newCost);

   // 设置是否启用负载感知模式 (保留作为开关标志，但不控制逻辑流)//后期再修改完善
   void setLoadAwareMode(bool enabled) { m_loadAwareMode = enabled; }
   bool isLoadAwareMode() const { return m_loadAwareMode; }
 
 private:
   // RTT Measurement
   void scheduleRttMeasurement(const ndn::Name& neighbor);
   void performRttMeasurement(const ndn::Name& neighbor);
   void handleRttResponse(const ndn::Name& neighbor, uint32_t seq,
                         ndn::time::steady_clock::time_point sendTime,
                         const ndn::Data& data);
   void handleRttTimeout(const ndn::Name& neighbor, uint32_t seq);
 
   // Cost Calculation and Update
   bool shouldUpdateCost(const ndn::Name& neighbor, double newCost);
   // calculateNewCost 已移除，由外部算法负责
 
   // ✅ 添加验证机制
   void verifyUpdateSuccess(const ndn::Name& neighbor, double expectedCost);
 
  // Timing Safety
  bool canMeasureNow(const ndn::Name& neighbor) const;
  ndn::time::steady_clock::time_point calculateSafeMeasurementTime(const ndn::Name& neighbor) const;

  // Status and Debug
  void generateStatusReport();

  // ===== nlsrc命令处理 =====
  /**
   * @brief 处理来自nlsrc的set-metrics命令
   */
  void handleSetMetricsCommand(const ndn::Interest& interest);
  
  /**
   * @brief 处理来自nlsrc的get-metrics命令
   */
  void handleGetMetricsCommand(const ndn::Interest& interest);
  
  /**
   * @brief 发送错误响应
   */
  void sendNack(const ndn::Interest& interest);
 
   // NLSR Component References
   ndn::Face& m_face;
   ndn::KeyChain& m_keyChain;
   ConfParameter& m_confParam;
   AdjacencyList& m_adjacencyList;
   Lsdb& m_lsdb;
   RoutingTable& m_routingTable;
   Fib& m_fib;
 
   // State Management
   std::unordered_map<ndn::Name, OutgoingLinkState> m_outgoingLinks;
   std::unordered_map<uint32_t, std::pair<ndn::Name, ndn::time::steady_clock::time_point>> m_pendingMeasurements;
   
   ndn::Scheduler m_scheduler;
   bool m_isActive;
   uint32_t m_nextSequenceNumber;
   
   // Configuration Parameters 相关变化阈值参数
  ndn::time::steady_clock::duration m_measurementInterval = ndn::time::seconds(2);
  ndn::time::milliseconds m_measurementTimeout = ndn::time::seconds(1);
  double m_costChangeThreshold = 0.05;
  double m_maxCostMultiplier = 2.0;
 
   // Statistics
   uint64_t m_totalMeasurements = 0;
   uint64_t m_successfulMeasurements = 0;
   uint64_t m_costUpdates = 0;
   bool m_loadAwareMode = false;
   
  // ===== 新增：外部指标存储 =====
  std::unordered_map<ndn::Name, ExternalMetrics> m_externalMetrics;
  MultiDimensionalCostConfig m_multiDimConfig;
 private:
    // ===== ML性能计算核心方法 =====
  /**
   * @brief 计算实时链路性能分数，用于ML反馈
   * @param neighbor 邻居节点名称
   * @param currentRtt 当前测量的RTT值
   * @return 性能分数 (0.0=最佳, 1.0=最差)
   */
  double calculateRealTimePerformance(const ndn::Name& neighbor, 
                                     ndn::time::steady_clock::duration currentRtt);
  
  /**
   * @brief 基于RTT计算性能分数
   * @param rttMs RTT毫秒值
   * @return RTT性能分数 (0.0=优秀, 1.0=很差)
   */
  double calculateRttPerformanceScore(double rttMs);
  
  /**
   * @brief 基于RTT历史计算稳定性性能分数
   * @param neighbor 邻居节点名称
   * @return 稳定性性能分数 (0.0=很稳定, 1.0=很不稳定)
   */
  double calculateStabilityPerformanceScore(const ndn::Name& neighbor);
  
  /**
   * @brief 基于超时情况计算可靠性性能分数
   * @param linkState 链路状态引用
   * @return 可靠性性能分数 (0.0=很可靠, 1.0=很不可靠)//后期再修改完善
   */
  double calculateReliabilityPerformanceScore(const OutgoingLinkState& linkState);
  
  /**
   * @brief 基于RTT变化趋势计算趋势性能分数
   * @param neighbor 邻居节点名称
   * @return 趋势性能分数 (0.0=改善中, 1.0=恶化中)
   */
  double calculateTrendPerformanceScore(const ndn::Name& neighbor);

 private:
  MLFeedbackCallback m_mlFeedbackCallback;
  
  // ===== 性能计算配置参数 =====
  static constexpr size_t MIN_SAMPLES_FOR_ML_FEEDBACK = 3;
  
  // RTT性能评估阈值 (毫秒)
  static constexpr double RTT_EXCELLENT_THRESHOLD = 10.0;
  static constexpr double RTT_GOOD_THRESHOLD = 50.0;
  static constexpr double RTT_FAIR_THRESHOLD = 100.0;
  static constexpr double RTT_POOR_THRESHOLD = 200.0;
  
  // 性能分数权重
  static constexpr double RTT_WEIGHT = 0.4;
  static constexpr double STABILITY_WEIGHT = 0.3;
  static constexpr double RELIABILITY_WEIGHT = 0.2;
  static constexpr double TREND_WEIGHT = 0.1;

 };
 
 } // namespace nlsr
 
 #endif // NLSR_LINK_COST_MANAGER_HPP