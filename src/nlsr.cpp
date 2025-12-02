/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 */

#include "nlsr.hpp"
#include "adjacent.hpp"
#include "logger.hpp"

#include <cstdlib>
#include <cstdio>
#include <unistd.h>

#include <ndn-cxx/mgmt/nfd/control-command.hpp>
#include <ndn-cxx/mgmt/nfd/status-dataset.hpp>
#include <ndn-cxx/net/face-uri.hpp>

namespace nlsr {

INIT_LOGGER(Nlsr);

Nlsr::Nlsr(ndn::Face& face, ndn::KeyChain& keyChain, ConfParameter& confParam)
  : m_face(face)
  , m_scheduler(face.getIoContext())
  , m_confParam(confParam)
  , m_adjacencyList(confParam.getAdjacencyList())
  , m_namePrefixList(confParam.getNamePrefixList())
  , m_fib(m_face, m_scheduler, m_adjacencyList, m_confParam, keyChain)
  , m_lsdb(m_face, keyChain, m_confParam)
  , m_routingTable(m_scheduler, m_lsdb, m_confParam)
  , m_namePrefixTable(confParam.getRouterPrefix(), m_fib, m_routingTable,
                      m_routingTable.afterRoutingChange, m_lsdb.onLsdbModified)
  , m_helloProtocol(m_face, keyChain, confParam, m_routingTable, m_lsdb, *this)
  , m_linkCostManager(std::make_unique<LinkCostManager>(m_face, keyChain, m_confParam, 
                                                       m_adjacencyList, m_lsdb, m_routingTable, m_fib))
  , m_onNewLsaConnection(m_lsdb.getSync().onNewLsa.connect(
      [this] (const ndn::Name& updateName, uint64_t sequenceNumber,
              const ndn::Name& originRouter, uint64_t incomingFaceId) {
        registerStrategyForCerts(originRouter);
      }))
  , m_onPrefixRegistrationSuccess(m_fib.onPrefixRegistrationSuccess.connect(
      [this] (const ndn::Name& name) {
        m_helloProtocol.sendHelloInterest(name);
      }))
  , m_onInitialHelloDataValidated(m_helloProtocol.onInitialHelloDataValidated.connect(
      [this] (const ndn::Name& neighbor) {
        auto it = m_adjacencyList.findAdjacent(neighbor);
        if (it != m_adjacencyList.end()) {
          m_fib.registerPrefix(m_confParam.getSyncPrefix(), it->getFaceUri(), it->getLinkCost(),
                               ndn::time::milliseconds::max(), ndn::nfd::ROUTE_FLAG_CAPTURE, 0);
        }
      }))
  , m_dispatcher(m_face, keyChain)
  , m_datasetHandler(m_dispatcher, m_lsdb, m_routingTable)
  , m_controller(m_face, keyChain)
  , m_faceDatasetController(m_face, keyChain)
  , m_prefixUpdateProcessor(m_dispatcher,
      m_confParam.getPrefixUpdateValidator(),
      m_namePrefixList,
      m_lsdb,
      m_confParam.getConfFileNameDynamic())
  , m_nfdRibCommandProcessor(m_dispatcher,
      m_namePrefixList,
      m_lsdb)
  , m_statsCollector(m_lsdb, m_helloProtocol)
  , m_faceMonitor(m_face)
  , m_terminateSignals(face.getIoContext(), SIGINT, SIGTERM)
{
  NLSR_LOG_DEBUG("Initializing Nlsr");

  m_faceMonitor.onNotification.connect(std::bind(&Nlsr::onFaceEventNotification, this, _1));
  m_faceMonitor.start();

  m_fib.setStrategy(m_confParam.getLsaPrefix(), Fib::MULTICAST_STRATEGY, 0);
  m_fib.setStrategy(m_confParam.getSyncPrefix(), Fib::MULTICAST_STRATEGY, 0);

  NLSR_LOG_DEBUG("Default NLSR identity: " << m_confParam.getSigningInfo().getSignerName());

  addDispatcherTopPrefix(ndn::Name(m_confParam.getRouterPrefix()).append("nlsr"));
  addDispatcherTopPrefix(LOCALHOST_PREFIX);

  enableIncomingFaceIdIndication();

  initializeFaces(std::bind(&Nlsr::processFaceDataset, this, _1),
                  std::bind(&Nlsr::onFaceDatasetFetchTimeout, this, _1, _2, 0));

  m_adjacencyList.writeLog();
  NLSR_LOG_DEBUG(m_namePrefixList);

  if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
    for (auto&& neighbor : m_adjacencyList.getAdjList()) {
      neighbor.setLinkCost(0);
    }
  }

  m_terminateSignals.async_wait([this] (auto&&... args) {
    terminate(std::forward<decltype(args)>(args)...);
  });

  // ✅ 教学要点：HelloProtocol事件连接的重要性
  // 这些连接让LinkCostManager能够实时感知邻居状态变化
  // 以下信号函数是HelloProtocol的事件连接，用于触发LinkCostManager的更新
  m_helloProtocol.onDataReceived.connect(
   [this](const ndn::Name& neighbor) { onHelloDataReceived(neighbor); });

  m_helloProtocol.onTimeout.connect(
     [this](const ndn::Name& neighbor, uint32_t timeoutCount) { onHelloTimeout(neighbor, timeoutCount); });
     
  m_helloProtocol.onNeighborStatusChanged.connect(
       [this](const ndn::Name& neighbor, Adjacent::Status status) { onHelloNeighborStatusChanged(neighbor, status); });
       
  m_helloProtocol.onInterestSent.connect(
         [this](const ndn::Name& neighbor) { onHelloInterestSent(neighbor); });

  // ✅ 教学要点：立即设置LinkCostManager到RoutingTable的重要性
  // 这个设置必须在LinkCostManager启动之前完成，确保路由表可以使用智能成本计算
  // 无论是负载感知算法还是ML算法，都需要这个基础设施
  if ((m_confParam.getLoadAwareRouting() || m_confParam.getMLAdaptiveRouting()) && m_linkCostManager) {
    NLSR_LOG_INFO("🔧 Setting LinkCostManager to RoutingTable for intelligent routing");
    m_routingTable.setLinkCostManager(m_linkCostManager.get());
    NLSR_LOG_INFO("✅ LinkCostManager integration completed");
  }

  // ✅ 教学要点：配置验证的简洁性原则
  // 对于ML算法，我们只需要简单的配置验证，不需要复杂的初始化逻辑
  // 实际的ML计算器会在RoutingTable中按需创建，遵循懒加载原则
  if (m_confParam.getMLAdaptiveRouting()) {
    NLSR_LOG_INFO("🧠 ML-adaptive routing enabled, intelligent learning will start with routing calculations");
  }

  // ✅ 教学要点：延迟启动策略的重要性
  // LinkCostManager需要在HelloProtocol稳定后才能开始工作
  // 这确保了邻居发现和状态同步都已经完成
  m_scheduler.schedule(ndn::time::seconds(100), [this] {
    m_linkCostManager->initialize();
    
    // ✅ 关键：连接成本更新信号，这是智能路由的核心机制
    m_linkCostManager->onNeighborCostUpdated.connect(
      [this](const ndn::Name& neighbor, double newCost) {
        this->onNeighborCostUpdated(neighbor, newCost);
      });
    
    m_linkCostManager->start();
    NLSR_LOG_INFO("✅ LinkCostManager started - intelligent routing infrastructure ready");
  });
}

// ✅ 教学要点：事件处理方法的设计模式
// 这些方法实现了系统各组件之间的解耦通信
// HelloProtocol负责邻居发现，LinkCostManager负责成本计算，Nlsr负责协调

void
Nlsr::onHelloInterestSent(const ndn::Name& neighbor)
{
  if (m_linkCostManager && m_linkCostManager->isActive()) {
    m_linkCostManager->onHelloInterestSent(neighbor);
  }
}

void
Nlsr::onHelloDataReceived(const ndn::Name& neighbor)
{
  if (m_linkCostManager && m_linkCostManager->isActive()) {
    m_linkCostManager->onHelloDataReceived(neighbor);
  }
}
/************这是有关linkcost的超时处理函数 */
void
Nlsr::onHelloTimeout(const ndn::Name& neighbor, uint32_t timeoutCount)
{
  if (m_linkCostManager && m_linkCostManager->isActive()) {
    m_linkCostManager->onHelloTimeout(neighbor, timeoutCount);
  }
}

void
Nlsr::onHelloNeighborStatusChanged(const ndn::Name& neighbor, Adjacent::Status status)
{
  if (m_linkCostManager && m_linkCostManager->isActive()) {
    m_linkCostManager->onNeighborStatusChanged(neighbor, status);
  }
}

void
Nlsr::onNeighborCostUpdated(const ndn::Name& neighbor, double newCost)
{
  auto adjacent = m_adjacencyList.findAdjacent(neighbor);
  if (adjacent == m_adjacencyList.end() || adjacent->getFaceId() == 0) {
    NLSR_LOG_WARN("Cannot update adjacency prefix for " << neighbor 
                 << ": neighbor not found or no valid face");
    return;
  }
  
  // ✅ 教学要点：前缀重新注册的重要性
  // 当邻居成本发生变化时，需要重新注册相关前缀
  // 这确保了路由系统能够使用最新的成本信息
  NLSR_LOG_INFO("Re-registering adjacency prefixes for " << neighbor 
               << " with updated cost " << newCost);
  
  registerAdjacencyPrefixes(*adjacent, ndn::time::milliseconds::max());
  
  NLSR_LOG_DEBUG("Successfully updated adjacency prefixes for " << neighbor);
}

// ✅ 教学要点：terminate方法中的清理逻辑
// 系统关闭时需要优雅地停止所有智能路由组件
// 这确保了资源的正确释放和状态的恢复
void
Nlsr::terminate(const boost::system::error_code& error, int signalNo)
{
  if (error)
    return;
  NLSR_LOG_INFO("Caught signal " << signalNo << " (" << ::strsignal(signalNo) << "), exiting...");
  
  // ✅ 重要：停止LinkCostManager并恢复原始成本
  // 这确保了网络在NLSR关闭后能够恢复到配置的静态成本
  if (m_linkCostManager && m_linkCostManager->isActive()) {
    m_linkCostManager->stop();
    NLSR_LOG_INFO("✅ LinkCostManager stopped and original costs restored");
  }
  
  m_face.getIoContext().stop();
}

// ✅ 其余方法保持完全不变，维持系统的稳定性
void
Nlsr::registerStrategyForCerts(const ndn::Name& originRouter)
{
  for (const ndn::Name& router : m_strategySetOnRouters) {
    if (router == originRouter) {
      return;
    }
  }

  m_strategySetOnRouters.push_back(originRouter);

  ndn::Name routerKey(originRouter);
  routerKey.append(ndn::security::Certificate::KEY_COMPONENT);
  ndn::Name instanceKey(originRouter);
  instanceKey.append("nlsr").append(ndn::security::Certificate::KEY_COMPONENT);

  m_fib.setStrategy(routerKey, Fib::BEST_ROUTE_STRATEGY, 0);
  m_fib.setStrategy(instanceKey, Fib::BEST_ROUTE_STRATEGY, 0);

  ndn::Name siteKey;
  for (size_t i = 0; i < originRouter.size(); ++i) {
    if (originRouter[i].toUri() == "%C1.Router") {
      break;
    }
    siteKey.append(originRouter[i]);
  }
  ndn::Name opPrefix(siteKey);
  siteKey.append(ndn::security::Certificate::KEY_COMPONENT);
  m_fib.setStrategy(siteKey, Fib::BEST_ROUTE_STRATEGY, 0);

  opPrefix.append(std::string("%C1.Operator"));
  m_fib.setStrategy(opPrefix, Fib::BEST_ROUTE_STRATEGY, 0);
}

void
Nlsr::addDispatcherTopPrefix(const ndn::Name& topPrefix)
{
  registerPrefix(topPrefix);
  try {
    m_dispatcher.addTopPrefix(topPrefix, false, m_confParam.getSigningInfo());
  }
  catch (const std::exception& e) {
    NLSR_LOG_ERROR("Error setting top-level prefix in dispatcher: " << e.what());
  }
}

void
Nlsr::registerPrefix(const ndn::Name& prefix)
{
  m_face.registerPrefix(prefix,
    [] (const ndn::Name& name) {
      NLSR_LOG_DEBUG("Successfully registered prefix " << name);
    },
    [] (const ndn::Name& name, const std::string& reason) {
      NLSR_LOG_ERROR("Failed to register prefix " << name << " (" << reason << ")");
      NDN_THROW(Error("Prefix registration failed: " + reason));
    });
}

void
Nlsr::onFaceEventNotification(const ndn::nfd::FaceEventNotification& faceEventNotification)
{
  NLSR_LOG_TRACE("onFaceEventNotification called");

  switch (faceEventNotification.getKind()) {
    case ndn::nfd::FACE_EVENT_DESTROYED: {
      uint64_t faceId = faceEventNotification.getFaceId();

      auto adjacent = m_adjacencyList.findAdjacent(faceId);

      if (adjacent != m_adjacencyList.end()) {
        NLSR_LOG_DEBUG("Face to " << adjacent->getName() << " with face id: " << faceId << " destroyed");

        adjacent->setFaceId(0);

        if (adjacent->getStatus() == Adjacent::STATUS_ACTIVE) {
          adjacent->setStatus(Adjacent::STATUS_INACTIVE);
          adjacent->setInterestTimedOutNo(m_confParam.getInterestRetryNumber());

          if (m_confParam.getHyperbolicState() == HYPERBOLIC_STATE_ON) {
            m_routingTable.scheduleRoutingTableCalculation();
          }
          else {
            m_lsdb.scheduleAdjLsaBuild();
          }
        }
      }
      break;
    }
    case ndn::nfd::FACE_EVENT_CREATED: {
      ndn::FaceUri faceUri;
      try {
        faceUri = ndn::FaceUri(faceEventNotification.getRemoteUri());
      }
      catch (const std::exception& e) {
        NLSR_LOG_WARN(e.what());
        return;
      }
      auto adjacent = m_adjacencyList.findAdjacent(faceUri);
      uint64_t faceId = faceEventNotification.getFaceId();

      if (adjacent != m_adjacencyList.end() &&
          (adjacent->getFaceId() == 0 || adjacent->getFaceId() != faceId))
      {
        NLSR_LOG_DEBUG("Face creation event matches neighbor: " << adjacent->getName()
                        << ". New Face ID: " << faceId << ". Registering prefixes.");
        adjacent->setFaceId(faceId);

        registerAdjacencyPrefixes(*adjacent, ndn::time::milliseconds::max());
      }
      break;
    }
    default:
      break;
  }
}

void
Nlsr::initializeFaces(const FetchDatasetCallback& onFetchSuccess,
                      const FetchDatasetTimeoutCallback& onFetchFailure)
{
  NLSR_LOG_TRACE("Initializing faces");
  m_faceDatasetController.fetch<ndn::nfd::FaceDataset>(onFetchSuccess, onFetchFailure);
}

void
Nlsr::processFaceDataset(const std::vector<ndn::nfd::FaceStatus>& faces)
{
  NLSR_LOG_DEBUG("Processing face dataset");

  for (auto&& adjacent : m_adjacencyList.getAdjList()) {
    const std::string& faceUriString = adjacent.getFaceUri().toString();
    for (const auto& faceStatus : faces) {
      if (adjacent.getFaceId() == 0 && faceUriString == faceStatus.getRemoteUri()) {
        NLSR_LOG_DEBUG("FaceUri: " << faceStatus.getRemoteUri() <<
                   " FaceId: "<< faceStatus.getFaceId());
        adjacent.setFaceId(faceStatus.getFaceId());
        this->registerAdjacencyPrefixes(adjacent, ndn::time::milliseconds::max());
      }
    }
    if (adjacent.getFaceId() == 0) {
      NLSR_LOG_WARN("The adjacency " << adjacent.getName() <<
                " has no Face information in this dataset.");
    }
  }

  scheduleDatasetFetch();
}

void
Nlsr::registerAdjacencyPrefixes(const Adjacent& adj, ndn::time::milliseconds timeout)
{
  ndn::FaceUri faceUri = adj.getFaceUri();
  double linkCost = adj.getLinkCost();
  const ndn::Name& adjName = adj.getName();

  m_fib.registerPrefix(adjName, faceUri, linkCost,
                       timeout, ndn::nfd::ROUTE_FLAG_CAPTURE, 0);

  m_fib.registerPrefix(m_confParam.getLsaPrefix(),
                       faceUri, linkCost, timeout,
                       ndn::nfd::ROUTE_FLAG_CAPTURE, 0);
}

void
Nlsr::onFaceDatasetFetchTimeout(uint32_t code,
                                const std::string& msg,
                                uint32_t nRetriesSoFar)
{
  if (nRetriesSoFar++ < m_confParam.getFaceDatasetFetchTries()) {
    NLSR_LOG_DEBUG("Failed to fetch dataset: " << msg << ". Attempting retry #" << nRetriesSoFar);
    m_faceDatasetController.fetch<ndn::nfd::FaceDataset>(std::bind(&Nlsr::processFaceDataset, this, _1),
                                                         std::bind(&Nlsr::onFaceDatasetFetchTimeout,
                                                                   this, _1, _2, nRetriesSoFar));
  }
  else {
    NLSR_LOG_ERROR("Failed to fetch dataset: " << msg << ". Exceeded limit of " <<
               m_confParam.getFaceDatasetFetchTries() << ", so not trying again this time.");
    scheduleDatasetFetch();
  }
}

void
Nlsr::scheduleDatasetFetch()
{
  NLSR_LOG_DEBUG("Scheduling dataset fetch in " << m_confParam.getFaceDatasetFetchInterval());

  m_scheduler.schedule(m_confParam.getFaceDatasetFetchInterval(), [this] {
    initializeFaces(
      [this] (const auto& faces) { processFaceDataset(faces); },
      [this] (uint32_t code, const std::string& msg) { onFaceDatasetFetchTimeout(code, msg, 0); });
  });
}

void
Nlsr::enableIncomingFaceIdIndication()
{
  NLSR_LOG_DEBUG("Enabling incoming face id indication for local face.");

  m_controller.start<ndn::nfd::FaceUpdateCommand>(
    ndn::nfd::ControlParameters()
      .setFlagBit(ndn::nfd::FaceFlagBit::BIT_LOCAL_FIELDS_ENABLED, true),
    [] (const ndn::nfd::ControlParameters& cp) {
      NLSR_LOG_DEBUG("Successfully enabled incoming face id indication"
                     << "for face id " << cp.getFaceId());
    },
    [] (const ndn::nfd::ControlResponse& cr) {
      NLSR_LOG_WARN("Failed to enable incoming face id indication feature: " <<
                    "(code: " << cr.getCode() << ", reason: " << cr.getText() << ")");
    });
}

} // namespace nlsr