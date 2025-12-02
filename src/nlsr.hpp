/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2025,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
 */

#ifndef NLSR_NLSR_HPP
#define NLSR_NLSR_HPP

#include "adjacency-list.hpp"
#include "conf-parameter.hpp"
#include "hello-protocol.hpp"
#include "lsdb.hpp"
#include "name-prefix-list.hpp"
#include "test-access-control.hpp"
#include "publisher/dataset-interest-handler.hpp"
#include "route/fib.hpp"
#include "route/name-prefix-table.hpp"
#include "route/routing-table.hpp"
#include "update/prefix-update-processor.hpp"
#include "update/nfd-rib-command-processor.hpp"
#include "utility/name-helper.hpp"
#include "stats-collector.hpp"
#include "link-cost-manager.hpp"

#include <ndn-cxx/face.hpp>
#include <ndn-cxx/encoding/nfd-constants.hpp>
#include <ndn-cxx/mgmt/dispatcher.hpp>
#include <ndn-cxx/mgmt/nfd/face-event-notification.hpp>
#include <ndn-cxx/mgmt/nfd/face-monitor.hpp>
#include <ndn-cxx/mgmt/nfd/face-status.hpp>
#include <ndn-cxx/mgmt/nfd/control-parameters.hpp>
#include <ndn-cxx/mgmt/nfd/control-response.hpp>
#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/util/scheduler.hpp>

#include <boost/asio/signal_set.hpp>

namespace nlsr {

class Nlsr
{
public:
  
  using FetchDatasetCallback = std::function<void(const std::vector<ndn::nfd::FaceStatus>&)>;
  using FetchDatasetTimeoutCallback = std::function<void(uint32_t, const std::string&)>;

  class Error : public std::runtime_error
  {
  public:
    using std::runtime_error::runtime_error;
  };

  Nlsr(ndn::Face& face, ndn::KeyChain& keyChain, ConfParameter& confParam);

  Lsdb&
  getLsdb()
  {
    return m_lsdb;
  }

  Fib&
  getFib()
  {
    return m_fib;
  }

  // ✅ 教学要点：公共接口设计的简洁性原则
  // 只暴露必要的接口，避免不必要的复杂性
  LinkCostManager& getLinkCostManager() { return *m_linkCostManager; }

private:
  void
  registerStrategyForCerts(const ndn::Name& originRouter);

  void
  addDispatcherTopPrefix(const ndn::Name& topPrefix);

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  void
  initializeFaces(const FetchDatasetCallback& onFetchSuccess,
                  const FetchDatasetTimeoutCallback& onFetchFailure);

  void
  onFaceDatasetFetchTimeout(uint32_t code,
                            const std::string& reason,
                            uint32_t nRetriesSoFar);

  void
  processFaceDataset(const std::vector<ndn::nfd::FaceStatus>& faces);

private:
  void
  registerAdjacencyPrefixes(const Adjacent& adj, ndn::time::milliseconds timeout);

  void
  registerPrefix(const ndn::Name& prefix);

  void
  onFaceEventNotification(const ndn::nfd::FaceEventNotification& faceEventNotification);

  void
  scheduleDatasetFetch();

  void
  enableIncomingFaceIdIndication();

  void
  terminate(const boost::system::error_code& error, int signalNo);

public:
  static inline const ndn::Name LOCALHOST_PREFIX{"/localhost/nlsr"};

  // ✅ 教学要点：HelloProtocol事件处理器设计
  // 这些方法实现了HelloProtocol与LinkCostManager的集成
  // 通过事件驱动的方式，LinkCostManager可以实时获取邻居状态变化
  void onHelloInterestSent(const ndn::Name& neighbor);
  void onHelloDataReceived(const ndn::Name& neighbor);
  void onHelloTimeout(const ndn::Name& neighbor, uint32_t timeoutCount);
  void onHelloNeighborStatusChanged(const ndn::Name& neighbor, Adjacent::Status status);
  void onNeighborCostUpdated(const ndn::Name& neighbor, double newCost);

private:
  ndn::Face& m_face;
  ndn::Scheduler m_scheduler;
  ConfParameter& m_confParam;
  AdjacencyList& m_adjacencyList;
  NamePrefixList& m_namePrefixList;
  std::vector<ndn::Name> m_strategySetOnRouters;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  Fib m_fib;
  Lsdb m_lsdb;
  RoutingTable m_routingTable;
  NamePrefixTable m_namePrefixTable;
  HelloProtocol m_helloProtocol;
  
 
  // 将LinkCostManager放在HelloProtocol之后，确保依赖关系正确
  // 这样LinkCostManager可以在构造时安全地引用HelloProtocol相关的组件
  std::unique_ptr<LinkCostManager> m_linkCostManager;

private:
  ndn::signal::ScopedConnection m_onNewLsaConnection;
  ndn::signal::ScopedConnection m_onPrefixRegistrationSuccess;
  ndn::signal::ScopedConnection m_onInitialHelloDataValidated;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  ndn::mgmt::Dispatcher m_dispatcher;
  DatasetInterestHandler m_datasetHandler;

private:
  ndn::nfd::Controller m_controller;
  ndn::nfd::Controller m_faceDatasetController;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  update::PrefixUpdateProcessor m_prefixUpdateProcessor;
  update::NfdRibCommandProcessor m_nfdRibCommandProcessor;

  StatsCollector m_statsCollector;

private:
  ndn::nfd::FaceMonitor m_faceMonitor;
  boost::asio::signal_set m_terminateSignals;
  
  // ✅ 教学要点：避免重复的系统级ML对象
  // 之前的设计中考虑过在Nlsr类中添加ML计算器，但这会与RoutingTable中的产生冲突
  // 最终决定只在RoutingTable中管理ML计算器，保持架构的一致性和简洁性
  // 这样的设计避免了生命周期管理的复杂性，也避免了回调注册的冲突
};

} // namespace nlsr

#endif // NLSR_NLSR_HPP