#include "link-cost-manager.hpp"
#include "logger.hpp"

#include <ndn-cxx/util/random.hpp>
#include <cmath>

#include <iomanip>  // 用于std::setprecision
#include <cmath>    // 用于std::sqrt
#include <sstream>  // 用于std::ostringstream

namespace nlsr {

INIT_LOGGER(LinkCostManager);

LinkCostManager::LinkCostManager(ndn::Face& face, ndn::KeyChain& keyChain,
                                ConfParameter& confParam, AdjacencyList& adjacencyList, 
                                Lsdb& lsdb, RoutingTable& routingTable,Fib& fib)
  : m_face(face)
  , m_keyChain(keyChain)
  , m_confParam(confParam)
  , m_adjacencyList(adjacencyList)
  , m_lsdb(lsdb)
  , m_routingTable(routingTable)  //初始化RoutingTable
  , m_fib(fib)
  , m_scheduler(face.getIoContext())  // 正确：使用getIoContext()
  , m_isActive(false)
  , m_nextSequenceNumber(1)
{
 ndn::Name rttPrefix(m_confParam.getRouterPrefix());
 rttPrefix.append("link-cost").append("rtt-probe");
 
 NLSR_LOG_DEBUG("Registering RTT probe prefix: " << rttPrefix);
 
 m_face.setInterestFilter(rttPrefix,
   [this](const auto& name, const auto& interest) {
     auto data = std::make_shared<ndn::Data>(interest.getName());
     data->setContent(ndn::encoding::makeStringBlock(ndn::tlv::Content, "rtt-response"));
     data->setFreshnessPeriod(ndn::time::milliseconds(1000));
     m_keyChain.sign(*data, m_confParam.getSigningInfo());
     m_face.put(*data);
     NLSR_LOG_TRACE("RTT response sent for: " << interest.getName());
   },
   [](const auto& name) {
     NLSR_LOG_DEBUG("RTT probe prefix registered: " << name);
   },
   [](const auto& name, const auto& reason) {
     NLSR_LOG_ERROR("Failed to register RTT probe prefix: " << reason);
   });

 // 注册nlsrc命令处理器
 ndn::Name cmdPrefix("/localhost");
 cmdPrefix.append("nlsr").append("link-cost-manager");
 
 NLSR_LOG_DEBUG("Registering link-cost-manager command prefix: " << cmdPrefix);
 
 // 处理set-metrics命令
 ndn::Name setMetricsPrefix = cmdPrefix;
 setMetricsPrefix.append("set-metrics");
 m_face.setInterestFilter(setMetricsPrefix,
   [this](const auto& filter, const auto& interest) {
     handleSetMetricsCommand(interest);
   },
   [](const auto& name) {
     NLSR_LOG_DEBUG("set-metrics prefix registered: " << name);
   },
   [](const auto& name, const auto& reason) {
     NLSR_LOG_ERROR("Failed to register set-metrics prefix: " << reason);
   });

 // 处理get-metrics命令
 ndn::Name getMetricsPrefix = cmdPrefix;
 getMetricsPrefix.append("get-metrics");
 m_face.setInterestFilter(getMetricsPrefix,
   [this](const auto& filter, const auto& interest) {
     handleGetMetricsCommand(interest);
   },
   [](const auto& name) {
     NLSR_LOG_DEBUG("get-metrics prefix registered: " << name);
   },
   [](const auto& name, const auto& reason) {
     NLSR_LOG_ERROR("Failed to register get-metrics prefix: " << reason);
   });
}

LinkCostManager::~LinkCostManager()
{
  stop();
}

void
LinkCostManager::initialize()
{
  NLSR_LOG_INFO("Initializing Link Cost Manager");
  
  for (const auto& adjacent : m_adjacencyList.getAdjList()) {
    OutgoingLinkState linkState;
    linkState.neighbor = adjacent.getName();
    linkState.status = adjacent.getStatus();
    linkState.originalCost = adjacent.getOriginalLinkCost();  // 使用原始配置成本
    linkState.currentCost = adjacent.getLinkCost();
    linkState.timeoutCount = adjacent.getInterestTimedOutNo();
    linkState.lastSuccess = ndn::time::steady_clock::now();
    
    m_outgoingLinks[adjacent.getName()] = linkState;
    
    NLSR_LOG_DEBUG("Initialized link state for " << adjacent.getName() 
                  << " with original cost " << linkState.originalCost);
  }
  
  NLSR_LOG_INFO("Link Cost Manager initialized with " << m_outgoingLinks.size() << " neighbors");
}

void
LinkCostManager::start()
{
  if (m_isActive) {
    NLSR_LOG_WARN("Link Cost Manager already active");
    return;
  }
  
  m_isActive = true;
  //延迟设定事件后，再开始RTT测量
  m_scheduler.schedule(ndn::time::seconds(30), [this] {
    for (auto& pair : m_outgoingLinks) {
      if (pair.second.isStable()) {
        scheduleRttMeasurement(pair.first);
      }
    }
    
    m_scheduler.schedule(ndn::time::minutes(10), [this] {
      generateStatusReport();
    });
  });
  
  NLSR_LOG_INFO("Link Cost Manager started");
}

void
LinkCostManager::stop()
{
  if (!m_isActive) {
    return;
  }
  
  m_isActive = false;
  m_scheduler.cancelAllEvents();
  m_pendingMeasurements.clear();
  
  // 恢复原始成本
  for (const auto& pair : m_outgoingLinks) {
    auto adjacent = m_adjacencyList.findAdjacent(pair.first);
    if (adjacent != m_adjacencyList.end() && adjacent->getLinkCost() != pair.second.originalCost) {
      adjacent->setLinkCost(pair.second.originalCost);
      NLSR_LOG_INFO("Restored original cost " << pair.second.originalCost 
                   << " for " << pair.first);
    }
  }
  
  // 触发LSA重建
  m_lsdb.scheduleAdjLsaBuild();
  
  NLSR_LOG_INFO("Link Cost Manager stopped and original costs restored");
}

// 实现所有事件处理函数
void
LinkCostManager::onHelloInterestSent(const ndn::Name& neighbor)
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    NLSR_LOG_TRACE("Hello Interest sent to " << neighbor);
  }
}

void
LinkCostManager::onHelloDataReceived(const ndn::Name& neighbor)
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    auto& linkState = it->second;
    linkState.status = Adjacent::STATUS_ACTIVE;
    linkState.timeoutCount = 0;
    linkState.lastSuccess = ndn::time::steady_clock::now();
    
    NLSR_LOG_TRACE("Hello Data received from " << neighbor << ", link stable");
    
    if (m_isActive && linkState.isStable() && linkState.rttHistory.empty()) {
      scheduleRttMeasurement(neighbor);
    }
  }
} 

/******************检测到hellotimeout时的处理函数 */
void
LinkCostManager::onHelloTimeout(const ndn::Name& neighbor, uint32_t timeouts)
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    auto& linkState = it->second;
    linkState.timeoutCount = timeouts;
    
    NLSR_LOG_DEBUG("Hello timeout for " << neighbor << ", count: " << timeouts);
    
    if (timeouts >= m_confParam.getInterestRetryNumber()) {
      linkState.status = Adjacent::STATUS_INACTIVE;
      linkState.rttHistory.clear();
      NLSR_LOG_INFO("Neighbor " << neighbor << " became INACTIVE due to timeouts");
    }
  }
}

/*收到邻居状态变化通知时的处理函数*/
void
LinkCostManager::onNeighborStatusChanged(const ndn::Name& neighbor, 
                                        Adjacent::Status newStatus)
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it == m_outgoingLinks.end()) {
    return;
  }
  
  auto& linkState = it->second;
  Adjacent::Status oldStatus = linkState.status;
  linkState.status = newStatus;
  
  NLSR_LOG_INFO("Neighbor " << neighbor << " status: " 
               << oldStatus << " -> " << newStatus);
  
  if (newStatus == Adjacent::STATUS_INACTIVE) {
    // 清理状态
    linkState.rttHistory.clear();//清除RTT历史记录
    linkState.probeResults.clear();
    linkState.timeoutCount = m_confParam.getInterestRetryNumber();
    
    // 取消所有待处理的RTT测量
    auto measurementIt = m_pendingMeasurements.begin();
    while (measurementIt != m_pendingMeasurements.end()) {
      if (measurementIt->second.first == neighbor) {
        measurementIt = m_pendingMeasurements.erase(measurementIt);
      }
      else {
        ++measurementIt;
      }
    }
    
    NLSR_LOG_INFO("Cleaned up state for INACTIVE neighbor " << neighbor);
    // 不在这里触发LSA，由HelloProtocol处理
  }
  else if (newStatus == Adjacent::STATUS_ACTIVE && 
           oldStatus != Adjacent::STATUS_ACTIVE) {
    // 恢复到原始配置成本
    auto adjacent = m_adjacencyList.findAdjacent(neighbor);
    if (adjacent != m_adjacencyList.end()) {
      adjacent->setLinkCost(adjacent->getOriginalLinkCost());
      linkState.currentCost = adjacent->getOriginalLinkCost();
      NLSR_LOG_INFO("Restored neighbor " << neighbor << " to original cost " 
                   << adjacent->getOriginalLinkCost());
    }
    
    linkState.timeoutCount = 0;
    linkState.lastSuccess = ndn::time::steady_clock::now();
    
    if (m_isActive) {
      scheduleRttMeasurement(neighbor);
    }
  }
}

void
LinkCostManager::scheduleRttMeasurement(const ndn::Name& neighbor)
{
  if (!m_isActive) {
    return;
  }
  
  auto safeTime = calculateSafeMeasurementTime(neighbor);
  auto now = ndn::time::steady_clock::now();
  auto delay = safeTime - now;
  
  if (delay <= ndn::time::nanoseconds(0)) {
    delay = ndn::time::seconds(1);
  }
  
  m_scheduler.schedule(delay, [this, neighbor] {
    if (canMeasureNow(neighbor)) {
      performRttMeasurement(neighbor);
    }
    scheduleRttMeasurement(neighbor);
  });
}

void
LinkCostManager::performRttMeasurement(const ndn::Name& neighbor)
{
  uint32_t seq = m_nextSequenceNumber++;
  
  ndn::Name probeName = neighbor;
  probeName.append("link-cost")
           .append("rtt-probe")
           .append(std::to_string(seq));
  
  auto interest = std::make_shared<ndn::Interest>(probeName);
  interest->setInterestLifetime(m_measurementTimeout);
  interest->setMustBeFresh(true);
  
  auto sendTime = ndn::time::steady_clock::now();
  m_pendingMeasurements[seq] = std::make_pair(neighbor, sendTime);
  
  m_face.expressInterest(*interest,
    [this, neighbor, seq, sendTime](const ndn::Interest&, const ndn::Data& data) {
      this->handleRttResponse(neighbor, seq, sendTime, data);
    },
    [this, neighbor, seq](const ndn::Interest&, const ndn::lp::Nack&) {
      this->handleRttTimeout(neighbor, seq);
    },
    [this, neighbor, seq](const ndn::Interest&) {
      this->handleRttTimeout(neighbor, seq);
    });
  
  m_totalMeasurements++;
  NLSR_LOG_TRACE("RTT probe sent to " << neighbor << " with seq " << seq);
}

//出现异常值的处理方法
void
LinkCostManager::handleRttResponse(const ndn::Name& neighbor, uint32_t seq,
                                  ndn::time::steady_clock::time_point sendTime,
                                  const ndn::Data& data)
{
  auto it = m_pendingMeasurements.find(seq);
  if (it == m_pendingMeasurements.end()) {
    return;
  }
  
  auto receiveTime = ndn::time::steady_clock::now();
  auto rtt = receiveTime - sendTime;
  
  m_pendingMeasurements.erase(it);
  m_successfulMeasurements++;
  
  auto rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(rtt).count();
  if (rttMs < 1) {
    NLSR_LOG_DEBUG("RTT too small (" << rttMs << "ms) for " << neighbor 
                  << ", using minimum 1ms");
    rttMs = 1;  // 设置最小值，继续处理，避免警告
  } else if (rttMs > 5000) {
    NLSR_LOG_WARN("RTT too large (" << rttMs << "ms) for " << neighbor 
                 << ", discarding measurement");
    return;  // 只有超大值才丢弃
  }
  
  auto linkIt = m_outgoingLinks.find(neighbor);
  if (linkIt != m_outgoingLinks.end() && linkIt->second.isStable()) {
    // ========= 新增开始：记录探测成功 =========
    linkIt->second.addProbeResult(true);
    linkIt->second.addRttMeasurement(rtt);
    
    NLSR_LOG_DEBUG("RTT measurement for " << neighbor << ": " << rttMs 
                  << "ms (samples: " << linkIt->second.rttHistory.size() << ")");

    // ✅ 核心修改：发布-订阅模式
    // 收集到新数据后，只负责通知订阅者（外部算法），不主动计算
    // 只有当数据量足够时才发送信号
    if (linkIt->second.rttHistory.size() >= OutgoingLinkState::MAX_RTT_SAMPLES) {
      auto metricsOpt = getLinkMetrics(neighbor);
      if (metricsOpt) {
        onLinkMetricsUpdated(neighbor, *metricsOpt);
        NLSR_LOG_TRACE("Emitted link metrics signal for " << neighbor);
      }
    }
    
    // ✅ ML性能反馈机制 (保留用于其他目的，如日志或调试)
    if (m_mlFeedbackCallback && 
        linkIt->second.rttHistory.size() >= MIN_SAMPLES_FOR_ML_FEEDBACK) {
      
      double performance = calculateRealTimePerformance(neighbor, rtt);
      m_mlFeedbackCallback(neighbor, performance);
      
      NLSR_LOG_DEBUG("ML feedback sent for " << neighbor 
                    << ": performance=" << std::fixed << std::setprecision(3) 
                    << performance << " (RTT=" << rttMs << "ms)");
    }
  }
}

void
LinkCostManager::handleRttTimeout(const ndn::Name& neighbor, uint32_t seq)
{
  auto it = m_pendingMeasurements.find(seq);
  if (it != m_pendingMeasurements.end()) {
    m_pendingMeasurements.erase(it);
    NLSR_LOG_DEBUG("RTT probe timeout for " << neighbor << " seq " << seq);
    // ========= 新增开始：记录探测丢包 =========
    auto linkIt = m_outgoingLinks.find(neighbor);
    if (linkIt != m_outgoingLinks.end() && linkIt->second.status == Adjacent::STATUS_ACTIVE) {
      linkIt->second.addProbeResult(false);
      NLSR_LOG_TRACE("Recorded packet loss for " << neighbor 
                    << " (PLR: " << linkIt->second.getPacketLossRate() << ")");
      
      // 超时也视为一种数据更新（丢包率变化），也应该触发信号
      // 但为了避免过于频繁，可以设置一些条件，这里简单起见也触发
      auto metricsOpt = getLinkMetrics(neighbor);
      if (metricsOpt) {
        onLinkMetricsUpdated(neighbor, *metricsOpt);
      }
    }
  }
}

bool
LinkCostManager::shouldUpdateCost(const ndn::Name& neighbor, double newCost)
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it == m_outgoingLinks.end()) {
    return false;
  }
  
  const auto& linkState = it->second;
  double changeRatio = std::abs(newCost - linkState.currentCost) / linkState.currentCost;
  
  return changeRatio >= m_costChangeThreshold;
}


// ✅ 修正：updateNeighborCost实现,逻辑检查
void
LinkCostManager::updateNeighborCost(const ndn::Name& neighbor, double newCost)
{
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end()) {
    NLSR_LOG_ERROR("Cannot find adjacent for " << neighbor);
    return;
  }
  
  auto it = m_outgoingLinks.find(neighbor);
  if (it == m_outgoingLinks.end()) {
    return;
  }

  // 检查邻居状态
  if (it->second.status == Adjacent::STATUS_INACTIVE) {
    NLSR_LOG_DEBUG("Skipping cost update for INACTIVE neighbor: " << neighbor);
    return;
  }
  
  double finalCost = newCost;
  // ✅ 核心修改：移除了此处对 m_loadAwareCostCalculator 的调用
  
  double oldCost = adjacent->getLinkCost();
  
  // 检查变化阈值
  if (!shouldUpdateCost(neighbor, finalCost)) {
    NLSR_LOG_TRACE("Cost change too small, skipping update. Old: " << oldCost << " New: " << finalCost);
    return;
  }
  
  // 更新成本
  adjacent->setLinkCost(finalCost);
  it->second.currentCost = finalCost;
 
  
  // 只在邻居稳定时触发LSA构建
  if (it->second.timeoutCount == 0) {
    // 使用专用的 COST_UPDATE 类型触发，避免被常规构建覆盖
    m_lsdb.scheduleAdjLsaBuildWithType(Lsdb::AdjLsaBuildType::COST_UPDATE);
    m_routingTable.scheduleRoutingTableCalculation();
    
    // 发出信号通知（用于日志或其他模块）
    onNeighborCostUpdated(neighbor, finalCost);
    
    NLSR_LOG_INFO("Triggered COST_UPDATE LSA build for " << neighbor);
  }
  
  m_costUpdates++;
  NLSR_LOG_INFO("Updated cost for " << neighbor 
               << ": " << oldCost << " -> " << finalCost);
}


// ✅ 实现验证机制
void
LinkCostManager::verifyUpdateSuccess(const ndn::Name& neighbor, double expectedCost)
{
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end()) {
    NLSR_LOG_WARN("Verification failed: neighbor " << neighbor << " not found");
    return;
  }
  //触发更新的机制
  double actualCost = adjacent->getLinkCost();
  if (std::abs(actualCost - expectedCost) > 0.02) {
    NLSR_LOG_WARN("Cost update verification failed for " << neighbor 
                 << ": expected " << expectedCost << ", actual " << actualCost);
  } else {
    NLSR_LOG_DEBUG("Cost update verification successful for " << neighbor);
  }
}

bool
LinkCostManager::canMeasureNow(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it == m_outgoingLinks.end()) {
    return false;
  }
  
  return m_isActive && it->second.isStable();
}

ndn::time::steady_clock::time_point
LinkCostManager::calculateSafeMeasurementTime(const ndn::Name& neighbor) const
{
  auto baseInterval = m_measurementInterval;
  auto randomOffset = ndn::time::milliseconds(ndn::random::generateWord32() % 500);
  
  return ndn::time::steady_clock::now() + baseInterval + randomOffset;
}

void
LinkCostManager::generateStatusReport()
{
  if (!m_isActive) {
    return;
  }
  
  NLSR_LOG_INFO("=== Link Cost Manager Status ===");
  NLSR_LOG_INFO("Total measurements: " << m_totalMeasurements);
  NLSR_LOG_INFO("Successful measurements: " << m_successfulMeasurements);
  NLSR_LOG_INFO("Cost updates: " << m_costUpdates);
  NLSR_LOG_INFO("Active neighbors: " << m_outgoingLinks.size());
  
  for (const auto& pair : m_outgoingLinks) {
    const auto& linkState = pair.second;
    auto avgRtt = linkState.getAverageRtt();
    auto avgRttMs = ndn::time::duration_cast<ndn::time::milliseconds>(avgRtt).count();

    NLSR_LOG_INFO("  " << pair.first 
                 << ": status=" << (linkState.status == Adjacent::STATUS_ACTIVE ? "ACTIVE" : "INACTIVE")
                 << ", cost=" << linkState.currentCost 
                 << " (orig=" << linkState.originalCost << ")"
                 << ", samples=" << linkState.rttHistory.size()
                 << ", avg_rtt=" << avgRttMs << "ms"
                 << ", timeouts=" << linkState.timeoutCount);
  }
  
  m_scheduler.schedule(ndn::time::minutes(10), [this] {
    generateStatusReport();
  });
}

double
LinkCostManager::getCurrentCost(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    return it->second.currentCost;
  }
  return 0.0;
}

std::optional<ndn::time::steady_clock::duration>
LinkCostManager::getCurrentRtt(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    return it->second.getAverageRtt();
  }
  return std::nullopt;  // 返回空值而不是 0
}

std::vector<ndn::time::steady_clock::duration>
LinkCostManager::getRttHistory(const ndn::Name& neighbor) const
{
  std::vector<ndn::time::steady_clock::duration> history;
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    for (const auto& measurement : it->second.rttHistory) {
      history.push_back(measurement.rtt);
    }
  }
  return history;
}

std::optional<uint32_t>
LinkCostManager::getTimeoutCount(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    return it->second.timeoutCount;
  }
  return std::nullopt;
}

std::optional<ndn::time::steady_clock::time_point>
LinkCostManager::getLastSuccessTime(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    return it->second.lastSuccess;
  }
  return std::nullopt;
}

// 添加新方法
std::optional<double>
LinkCostManager::getLinkCost(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it != m_outgoingLinks.end()) {
    return it->second.currentCost;
  }
  return std::nullopt;
}

double
LinkCostManager::getOriginalLinkCost(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  return (it != m_outgoingLinks.end()) ? it->second.originalCost : 0.0;
}

//修正：getmetrics的实现
std::optional<LinkCostManager::LinkMetrics>
LinkCostManager::getLinkMetrics(const ndn::Name& neighbor) const
{
  auto it = m_outgoingLinks.find(neighbor);
  if (it == m_outgoingLinks.end()) {
    return std::nullopt;  // 找不到邻居
  }

  LinkMetrics metrics{};
  metrics.neighbor = neighbor;
  
  const auto& linkState = it->second;
  metrics.originalCost = linkState.originalCost;
  metrics.packetLoss = linkState.getPacketLossRate();
  metrics.currentCost = linkState.currentCost;
  metrics.timeoutCount = linkState.timeoutCount;
  metrics.lastSuccessTime = linkState.lastSuccess;
  metrics.status = linkState.status;
  
  // 构建RTT历史
  for (const auto& measurement : linkState.rttHistory) {
    metrics.rttHistory.push_back(measurement.rtt);
  }
  
  if (!metrics.rttHistory.empty()) {
    metrics.currentRtt = linkState.getAverageRtt();
  }
  
  return metrics;
}

// ✅ 核心修改：移除了 LoadAwareCostCalculator 的相关设置和清除函数
// void LinkCostManager::setLoadAwareCostCalculator(...) {}
// void LinkCostManager::clearLoadAwareCostCalculator() {}

 //ML新增方法实现
void LinkCostManager::setMLFeedbackCallback(MLFeedbackCallback callback)
  {
    m_mlFeedbackCallback = std::move(callback);
    NLSR_LOG_INFO("ML feedback callback registered");
  }

void LinkCostManager::clearMLFeedbackCallback()
  {
    m_mlFeedbackCallback = nullptr;
    NLSR_LOG_INFO("ML feedback callback cleared");
  }

double LinkCostManager::calculateRealTimePerformance(const ndn::Name& neighbor, 
                                             ndn::time::steady_clock::duration currentRtt)
  {
    auto it = m_outgoingLinks.find(neighbor);
    if (it == m_outgoingLinks.end()) {
      return 0.5; // 默认中等性能
    }
    
    const auto& linkState = it->second;
    auto currentRttMs = ndn::time::duration_cast<ndn::time::milliseconds>(currentRtt).count();
    
    // 多维度性能评估
    double rttPerformance = calculateRttPerformanceScore(currentRttMs);
    double stabilityPerformance = calculateStabilityPerformanceScore(neighbor);
    double reliabilityPerformance = calculateReliabilityPerformanceScore(linkState);
    double trendPerformance = calculateTrendPerformanceScore(neighbor);
    
    // 加权综合评分
    double totalPerformance = 
      RTT_WEIGHT * rttPerformance +
      STABILITY_WEIGHT * stabilityPerformance +
      RELIABILITY_WEIGHT * reliabilityPerformance +
      TREND_WEIGHT * trendPerformance;
    
    return std::max(0.0, std::min(1.0, totalPerformance));
  }

double LinkCostManager::calculateRttPerformanceScore(double rttMs)
  {
    // 基于网络体验的分级映射
    if (rttMs <= RTT_EXCELLENT_THRESHOLD) {
      return 0.0;  // 优秀：几乎无感知延迟
    } else if (rttMs <= RTT_GOOD_THRESHOLD) {
      // 线性插值：10ms-50ms映射到0.0-0.3
      return (rttMs - RTT_EXCELLENT_THRESHOLD) / 
            (RTT_GOOD_THRESHOLD - RTT_EXCELLENT_THRESHOLD) * 0.3;
    } else if (rttMs <= RTT_FAIR_THRESHOLD) {
      // 线性插值：50ms-100ms映射到0.3-0.6
      return 0.3 + (rttMs - RTT_GOOD_THRESHOLD) / 
            (RTT_FAIR_THRESHOLD - RTT_GOOD_THRESHOLD) * 0.3;
    } else if (rttMs <= RTT_POOR_THRESHOLD) {
      // 线性插值：100ms-200ms映射到0.6-0.9
      return 0.6 + (rttMs - RTT_FAIR_THRESHOLD) / 
            (RTT_POOR_THRESHOLD - RTT_FAIR_THRESHOLD) * 0.3;
    } else {
      // 超过200ms：0.9-1.0，有上限
      return 0.9 + std::min(0.1, (rttMs - RTT_POOR_THRESHOLD) / 800.0 * 0.1);
    }
  }

double LinkCostManager::calculateStabilityPerformanceScore(const ndn::Name& neighbor)
  {
    auto it = m_outgoingLinks.find(neighbor);
    if (it == m_outgoingLinks.end() || it->second.rttHistory.size() < 3) {
      return 0.5; // 数据不足，给中等分数
    }
    
    const auto& history = it->second.rttHistory;
    
    // 计算最近几次测量的变异系数
    size_t sampleCount = std::min(history.size(), size_t(5));
    double sum = 0.0, sumSquares = 0.0;
    
    auto start = history.end() - sampleCount;
    for (auto iter = start; iter != history.end(); ++iter) {
      double rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(iter->rtt).count();
      sum += rttMs;
      sumSquares += rttMs * rttMs;
    }
    
    double mean = sum / sampleCount;
    double variance = (sumSquares / sampleCount) - (mean * mean);
    double stddev = std::sqrt(variance);
    
    // 变异系数 = 标准差 / 均值
    double coefficientOfVariation = (mean > 0) ? (stddev / mean) : 0.0;
    
    // 变异系数到性能分数的映射
    if (coefficientOfVariation <= 0.1) {
      return 0.0;  // 非常稳定
    } else if (coefficientOfVariation <= 0.3) {
      return coefficientOfVariation / 0.3 * 0.4;  // 比较稳定
    } else {
      return 0.4 + std::min(0.6, (coefficientOfVariation - 0.3) / 0.7 * 0.6);
    }
  }

double LinkCostManager::calculateReliabilityPerformanceScore(const OutgoingLinkState& linkState)
  {
    // 基于超时计数的简单映射
    if (linkState.timeoutCount == 0) {
      return 0.0;  // 完全可靠
    } else if (linkState.timeoutCount <= 2) {
      return 0.2;  // 偶尔超时
    } else if (linkState.timeoutCount <= 5) {
      return 0.5;  // 经常超时
    } else {
      return 0.8;  // 很不可靠
    }
  }
//这个是加了趋势分析的算法函数
double LinkCostManager::calculateTrendPerformanceScore(const ndn::Name& neighbor)
  {
    auto it = m_outgoingLinks.find(neighbor);
    if (it == m_outgoingLinks.end() || it->second.rttHistory.size() < 6) {
      return 0.0; // 数据不足，给最好分数
    }
    
    const auto& history = it->second.rttHistory;
    size_t size = history.size();
    
    // 比较最近3次 vs 之前3次的平均RTT
    double recentAvg = 0.0, previousAvg = 0.0;
    
    for (size_t i = size - 3; i < size; ++i) {
      recentAvg += ndn::time::duration_cast<ndn::time::milliseconds>(history[i].rtt).count();
    }
    for (size_t i = size - 6; i < size - 3; ++i) {
      previousAvg += ndn::time::duration_cast<ndn::time::milliseconds>(history[i].rtt).count();
    }
    
    recentAvg /= 3.0;
    previousAvg /= 3.0;
    
    if (previousAvg > 0) {
      double changeRatio = (recentAvg - previousAvg) / previousAvg;
      
      if (changeRatio <= -0.1) {
        return 0.0;  // RTT明显改善
      } else if (changeRatio <= 0.1) {
        return 0.2;  // RTT基本稳定
      } else if (changeRatio <= 0.3) {
        return 0.5;  // RTT轻微恶化
      } else {
        return 0.8;  // RTT明显恶化
      }
    }
    
    return 0.0;
  }

// ===== 新增：外部指标管理功能 =====

void
LinkCostManager::setExternalMetrics(const ndn::Name& neighbor, const ExternalMetrics& metrics)
{
  // ✅ 只验证AdjacencyList（配置文件中是否存在该邻居）
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end()) {
    NLSR_LOG_WARN("Cannot set external metrics: neighbor " << neighbor 
                 << " not found in configuration file");
    return;
  }
  
  // 存储外部指标到独立的map
  m_externalMetrics[neighbor] = metrics;
  
  NLSR_LOG_INFO("External metrics updated for " << neighbor 
               << " (OriginalCost=" << adjacent->getOriginalLinkCost() << ")");
}

std::optional<LinkCostManager::LinkMetrics>
LinkCostManager::getMetricsSnapshot(const ndn::Name& neighbor) const
{
  // ✅ 只从AdjacencyList读取（权威数据源，配置文件）
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end()) {
    return std::nullopt;  // 配置文件中不存在该邻居
  }
  
  LinkMetrics metrics;
  metrics.neighbor = neighbor;
  metrics.originalCost = adjacent->getOriginalLinkCost();
  metrics.currentCost = adjacent->getLinkCost();
  metrics.status = adjacent->getStatus();
  
  // ✅ 从m_externalMetrics读取用户设置（如果有）
  auto extIt = m_externalMetrics.find(neighbor);
  if (extIt != m_externalMetrics.end()) {
    metrics.bandwidth = extIt->second.bandwidth;
    metrics.bandwidthUtil = extIt->second.bandwidthUtil;
    metrics.packetLoss = extIt->second.packetLoss;
    metrics.spectrumStrength = extIt->second.spectrumStrength;
  }
  
  // 计算多维度预览成本（使用默认值或用户设置）
  double previewCost = calculateMultiDimensionalCostPreview(neighbor);
  if (previewCost > 0) {
    metrics.multiDimensionalCostPreview = previewCost;
  }
  
  return metrics;
}

double
LinkCostManager::calculateMultiDimensionalCostPreview(const ndn::Name& neighbor) const
{
  // ✅ 只从AdjacencyList读取originalCost（权威数据源）
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end()) {
    NLSR_LOG_DEBUG("Cannot calculate preview cost: neighbor " << neighbor << " not found in AdjacencyList");
    return -1.0;
  }
  double originalCost = adjacent->getOriginalLinkCost();
  
  // ===== 1. RTT因子（使用默认值，不读取实际测量）=====
  double rttFactor = 1.1;  // 默认假设RTT=20ms
  
  // ===== 2. 带宽因子（使用默认值或用户设置）=====
  double bwFactor = 1.3;  // 默认假设利用率=30%
  auto extIt = m_externalMetrics.find(neighbor);
  if (extIt != m_externalMetrics.end() && extIt->second.bandwidthUtil) {
    double util = *extIt->second.bandwidthUtil;
    if (util <= 0.0) {
      bwFactor = 1.0;
    } else if (util >= 1.0) {
      bwFactor = 2.0;
    } else {
      bwFactor = 1.0 + util;
    }
  }
  
  // ===== 3. 可靠性因子（使用默认值或用户设置）=====
  double reliabilityFactor = 1.02;  // 默认假设丢包率=1%
  if (extIt != m_externalMetrics.end() && extIt->second.packetLoss) {
    double loss = *extIt->second.packetLoss;
    if (loss <= 0.0) {
      reliabilityFactor = 1.0;
    } else if (loss >= 0.5) {
      reliabilityFactor = 2.0;
    } else {
      reliabilityFactor = 1.0 + (loss * 2.0);
    }
  }
  
  // ===== 4. 频谱因子（使用默认值或用户设置）=====
  double spectrumFactor = 1.4;  // 默认假设频谱=-50dBm
  if (extIt != m_externalMetrics.end() && extIt->second.spectrumStrength) {
    double strength = *extIt->second.spectrumStrength;
    if (strength >= -30) {
      spectrumFactor = 1.0;
    } else if (strength <= -80) {
      spectrumFactor = 2.0;
    } else {
      spectrumFactor = 1.0 + ((-30.0 - strength) / 50.0);
    }
  }
  
  // ===== 5. 加权融合 =====
  double compositeFactor = 
    m_multiDimConfig.rttWeight * rttFactor +
    m_multiDimConfig.bandwidthWeight * bwFactor +
    m_multiDimConfig.reliabilityWeight * reliabilityFactor +
    m_multiDimConfig.spectrumWeight * spectrumFactor;
  
  double finalCost = originalCost * compositeFactor;
  
  NLSR_LOG_DEBUG("Multi-dimensional cost preview for " << neighbor 
                << ": originalCost=" << originalCost
                << ", rttFactor=" << std::fixed << std::setprecision(2) << rttFactor
                << ", bwFactor=" << bwFactor
                << ", reliabilityFactor=" << reliabilityFactor
                << ", spectrumFactor=" << spectrumFactor
                << ", compositeFactor=" << compositeFactor
                << ", finalCost=" << finalCost);
  
  return std::round(finalCost);
}

// ===== nlsrc命令处理实现 =====

void
LinkCostManager::handleSetMetricsCommand(const ndn::Interest& interest)
{
  const auto& name = interest.getName();
  
  // 解析Interest名称：/localhost/nlsr/link-cost-manager/set-metrics/<neighbor>/[options]
  // Interest结构固定：[0]=localhost, [1]=nlsr, [2]=link-cost-manager, [3]=set-metrics, [4]=neighbor开始
  size_t baseSize = 4; // neighbor从第4个组件开始
  
  if (name.size() < baseSize + 1) {
    NLSR_LOG_ERROR("Invalid set-metrics command format");
    sendNack(interest);
    return;
  }
  
  // 提取neighbor名称
  // Interest格式：/<prefix>/nlsr/link-cost-manager/set-metrics/<neighbor-components>/--bandwidth/100/...
  // 策略：从baseSize开始，遇到以"--"开头的组件就停止
  ndn::Name neighborName;
  size_t optionsStart = name.size(); // 默认没有选项
  
  for (size_t i = baseSize; i < name.size(); ++i) {
    std::string comp = name.get(i).toUri();
    
    // 检查是否是选项（以"--"或"%2D%2D"开头，NDN可能会编码）
    if (comp.size() >= 2 && (comp[0] == '-' && comp[1] == '-')) {
      optionsStart = i;
      break;
    }
    if (comp.size() >= 6 && comp.substr(0, 6) == "%2D%2D") {
      optionsStart = i;
      break;
    }
    
    neighborName.append(name.get(i));
  }
  
  if (neighborName.size() == 0) {
    NLSR_LOG_ERROR("Failed to extract neighbor name from Interest");
    sendNack(interest);
    return;
  }
  
  // 解析选项参数
  ExternalMetrics metrics;
  metrics.lastUpdate = ndn::time::steady_clock::now();
  
  for (size_t i = optionsStart; i + 1 < name.size(); i += 2) {
    std::string option = name.get(i).toUri();
    std::string value = name.get(i + 1).toUri();
    
    try {
      if (option == "--bandwidth") {
        metrics.bandwidth = std::stod(value);
      }
      else if (option == "--bandwidth-util") {
        metrics.bandwidthUtil = std::stod(value);
      }
      else if (option == "--packet-loss") {
        metrics.packetLoss = std::stod(value);
      }
      else if (option == "--spectrum") {
        metrics.spectrumStrength = std::stod(value);
      }
    }
    catch (const std::exception& e) {
      NLSR_LOG_WARN("Failed to parse option " << option << ": " << e.what());
    }
  }
  
  // 设置外部指标
  setExternalMetrics(neighborName, metrics);
  
  // 发送成功响应
  auto data = std::make_shared<ndn::Data>(interest.getName());
  std::string response = "External metrics updated successfully for " + neighborName.toUri();
  data->setContent(ndn::encoding::makeStringBlock(ndn::tlv::Content, response));
  // 缩短新命令响应的Freshness，降低CS缓存影响
  data->setFreshnessPeriod(ndn::time::milliseconds(100));
  m_keyChain.sign(*data, m_confParam.getSigningInfo());
  m_face.put(*data);
  
  NLSR_LOG_INFO("Sent set-metrics response for " << neighborName);
}

void
LinkCostManager::handleGetMetricsCommand(const ndn::Interest& interest)
{
  const auto& name = interest.getName();
  
  // 解析Interest名称：/localhost/nlsr/link-cost-manager/get-metrics/<neighbor>
  // Interest结构固定：[0]=localhost, [1]=nlsr, [2]=link-cost-manager, [3]=get-metrics, [4]=neighbor开始
  size_t baseSize = 4; // neighbor从第4个组件开始
  
  if (name.size() < baseSize + 1) {
    NLSR_LOG_ERROR("Invalid get-metrics command format");
    sendNack(interest);
    return;
  }
  
  // 提取neighbor名称（从baseSize到末尾的所有组件）
  ndn::Name neighborName;
  for (size_t i = baseSize; i < name.size(); ++i) {
    neighborName.append(name.get(i));
  }
  
  // 获取指标快照
  auto metricsOpt = getMetricsSnapshot(neighborName);
  if (!metricsOpt) {
    NLSR_LOG_WARN("Neighbor not found: " << neighborName);
    sendNack(interest);
    return;
  }
  
  const auto& metrics = *metricsOpt;
  
  // 构建响应内容
  std::ostringstream response;
  response << "Neighbor: " << neighborName << "\n";
  response << "OriginalCost: " << metrics.originalCost << "\n";
  response << "CurrentCost: " << metrics.currentCost << "\n";
  
  // 外部指标
  if (metrics.bandwidth) {
    response << "Bandwidth: " << *metrics.bandwidth << "\n";
  }
  if (metrics.bandwidthUtil) {
    response << "BandwidthUtil: " << *metrics.bandwidthUtil << "\n";
  }
  if (metrics.packetLoss) {
    response << "PacketLoss: " << *metrics.packetLoss << "\n";
  }
  if (metrics.spectrumStrength) {
    response << "SpectrumStrength: " << *metrics.spectrumStrength << "\n";
  }
  
  // RTT信息
  if (metrics.currentRtt) {
    auto rttMs = ndn::time::duration_cast<ndn::time::milliseconds>(*metrics.currentRtt).count();
    response << "AverageRTT: " << rttMs << "\n";
  }
  
  // 多维度成本预览
  if (metrics.multiDimensionalCostPreview) {
    response << "MultiDimensionalCost: " << *metrics.multiDimensionalCostPreview << "\n";
  }
  
  // 权重配置
  response << "Weights: " << m_multiDimConfig.rttWeight << ","
           << m_multiDimConfig.bandwidthWeight << ","
           << m_multiDimConfig.reliabilityWeight << ","
           << m_multiDimConfig.spectrumWeight << "\n";
  
  // 发送响应
  auto data = std::make_shared<ndn::Data>(interest.getName());
  data->setContent(ndn::encoding::makeStringBlock(ndn::tlv::Content, response.str()));
  data->setFreshnessPeriod(ndn::time::milliseconds(100));
  m_keyChain.sign(*data, m_confParam.getSigningInfo());
  m_face.put(*data);
  
  NLSR_LOG_INFO("Sent get-metrics response for " << neighborName);
}

void
LinkCostManager::sendNack(const ndn::Interest& interest)
{
  auto data = std::make_shared<ndn::Data>(interest.getName());
  data->setContent(ndn::encoding::makeStringBlock(ndn::tlv::Content, "ERROR"));
  data->setFreshnessPeriod(ndn::time::milliseconds(100));
  m_keyChain.sign(*data, m_confParam.getSigningInfo());
  m_face.put(*data);
}

} // namespace nlsr