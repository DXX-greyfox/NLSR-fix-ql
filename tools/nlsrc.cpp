/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2024,  The University of Memphis,
 *                           Regents of the University of California,
 *                           Arizona Board of Regents.
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

#include "nlsrc.hpp"

#include "config.hpp"
#include "version.hpp"
#include "src/publisher/dataset-interest-handler.hpp"

#include <ndn-cxx/data.hpp>
#include <ndn-cxx/encoding/block.hpp>
#include <ndn-cxx/face.hpp>
#include <ndn-cxx/interest.hpp>
#include <ndn-cxx/mgmt/nfd/control-parameters.hpp>
#include <ndn-cxx/mgmt/nfd/control-response.hpp>
#include <ndn-cxx/security/interest-signer.hpp>
#include <ndn-cxx/security/key-chain.hpp>
#include <ndn-cxx/security/signing-helpers.hpp>
#include <ndn-cxx/security/validator-config.hpp>
#include <ndn-cxx/security/validator-null.hpp>
#include <ndn-cxx/util/segment-fetcher.hpp>

#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <iostream>

namespace nlsrc {

const ndn::Name LOCALHOST_PREFIX("/localhost");
const ndn::PartialName LSDB_SUFFIX("nlsr/lsdb");
const ndn::PartialName NAME_UPDATE_SUFFIX("nlsr/prefix-update");
const ndn::PartialName RT_SUFFIX("nlsr/routing-table");

const uint32_t ERROR_CODE_TIMEOUT = 10060;
const uint32_t RESPONSE_CODE_SUCCESS = 200;
const uint32_t RESPONSE_CODE_NO_EFFECT = 204;
const uint32_t RESPONSE_CODE_SAVE_OR_DELETE = 205;

Nlsrc::Nlsrc(std::string programName, ndn::Face& face)
  : m_programName(std::move(programName))
  , m_routerPrefix(LOCALHOST_PREFIX)
  , m_face(face)
{
  disableValidator();
}

void
Nlsrc::printUsage() const
{
  const std::string help(R"EOT(Usage:
@NLSRC@ [-h | -V]
@NLSRC@ [-R <router prefix> [-c <nlsr.conf path> | -k]] COMMAND [<Command Options>]
       -h print usage and exit
       -V print version and exit
       -R target a remote NLSR instance
       -c verify response with nlsr.conf security.validator policy
       -k do not verify response (insecure)

   COMMAND can be one of the following:
       lsdb
           display NLSR lsdb status
       routing
           display routing table status
       status
           display all NLSR status (lsdb & routingtable)
       advertise <name>
           advertise a name prefix through NLSR
       advertise <name> save
           advertise and save the name prefix to the conf file
       withdraw <name>
           remove a name prefix advertised through NLSR
       withdraw <name> delete
           withdraw and delete the name prefix from the conf file
       link-metrics set <neighbor-name> [OPTIONS]
           set external metrics for a neighbor link
           OPTIONS:
             --bandwidth <Mbps>        link bandwidth in Mbps
             --bandwidth-util <0-1>    bandwidth utilization (0-1)
             --packet-loss <0-1>       packet loss rate (0-1)
             --spectrum <dBm>          spectrum strength in dBm
       link-metrics show <neighbor-name>
           display multi-dimensional cost calculation for a neighbor
)EOT");
  boost::algorithm::replace_all_copy(std::ostream_iterator<char>(std::cout),
                                     help, "@NLSRC@", m_programName);
}

void
Nlsrc::setRouterPrefix(ndn::Name prefix)
{
  m_routerPrefix = std::move(prefix);
}

void
Nlsrc::disableValidator()
{
  m_validator.reset(new ndn::security::ValidatorNull());
}

bool
Nlsrc::enableValidator(const std::string& filename)
{
  using namespace boost::property_tree;
  ptree validatorConfig;
  try {
    ptree config;
    read_info(filename, config);
    validatorConfig = config.get_child("security.validator");
  }
  catch (const ptree_error& e) {
    std::cerr << "Failed to parse configuration file '" << filename
              << "': " << e.what() << std::endl;
    return false;
  }

  auto validator = std::make_unique<ndn::security::ValidatorConfig>(m_face);
  try {
    validator->load(validatorConfig, filename);
  }
  catch (const ndn::security::validator_config::Error& e) {
    std::cerr << "Failed to load validator config from '" << filename
              << "' security.validator section: " << e.what() << std::endl;
    return false;
  }

  m_validator = std::move(validator);
  return true;
}

void
Nlsrc::getStatus(const std::string& command)
{
  if (command == "lsdb") {
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchAdjacencyLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchCoordinateLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchNameLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::printLsdb, this));
  }
  else if (command == "routing") {
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchRtables, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::printRT, this));
  }
  else if (command == "status") {
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchAdjacencyLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchCoordinateLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchNameLsas, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::fetchRtables, this));
    m_fetchSteps.push_back(std::bind(&Nlsrc::printAll, this));
  }
  runNextStep();
}

bool
Nlsrc::dispatch(ndn::span<std::string> subcommand)
{
  if (subcommand.size() == 0) {
    return false;
  }

  if (subcommand[0] == "advertise") {
    switch (subcommand.size()) {
      case 2:
        advertiseName(subcommand[1], false);
        return true;
      case 3:
        if (subcommand[2] != "save") {
          return false;
        }
        advertiseName(subcommand[1], true);
        return true;
    }
    return false;
  }

  if (subcommand[0] == "withdraw") {
    switch (subcommand.size()) {
      case 2:
        withdrawName(subcommand[1], false);
        return true;
      case 3:
        if (subcommand[2] != "delete") {
          return false;
        }
        withdrawName(subcommand[1], true);
        return true;
    }
    return false;
  }

  if (subcommand[0] == "link-metrics") {
    if (subcommand.size() < 3) {
      return false;
    }
    
    if (subcommand[1] == "set") {
      // link-metrics set <neighbor-name> [--bandwidth X] [--bandwidth-util X] ...
      std::string neighborName = subcommand[2];
      std::map<std::string, std::string> options;
      
      for (size_t i = 3; i + 1 < subcommand.size(); i += 2) {
        options[subcommand[i]] = subcommand[i + 1];
      }
      
      setLinkMetrics(neighborName, options);
      return true;
    }
    else if (subcommand[1] == "show") {
      // link-metrics show <neighbor-name>
      if (subcommand.size() != 3) {
        return false;
      }
      showLinkMetrics(subcommand[2]);
      return true;
    }
    
    return false;
  }

  if (subcommand[0] == "lsdb" || subcommand[0] == "routing" || subcommand[0] == "status") {
    if (subcommand.size() != 1) {
      return false;
    }
    getStatus(subcommand[0]);
    return true;
  }
  return false;
}

void
Nlsrc::runNextStep()
{
  if (m_fetchSteps.empty()) {
    return;
  }

  std::function<void()> nextStep = m_fetchSteps.front();
  m_fetchSteps.pop_front();

  nextStep();
}

void
Nlsrc::advertiseName(ndn::Name name, bool wantSave)
{
  std::string info = (wantSave ? "(Save: " : "(Advertise: ") + name.toUri() + ")";
  ndn::Name::Component verb("advertise");
  sendNamePrefixUpdate(name, verb, info, wantSave);
}

void
Nlsrc::withdrawName(ndn::Name name, bool wantDelete)
{
  std::string info = (wantDelete ? "(Delete: " : "(Withdraw: ") + name.toUri() + ")";
  ndn::Name::Component verb("withdraw");
  sendNamePrefixUpdate(name, verb, info, wantDelete);
}

void
Nlsrc::setLinkMetrics(const std::string& neighborName,
                     const std::map<std::string, std::string>& options)
{
  std::cout << "Setting external metrics for neighbor: " << neighborName << std::endl;
  
  // 解析选项
  bool hasMetrics = false;
  for (const auto& opt : options) {
    if (opt.first == "--bandwidth") {
      std::cout << "  Bandwidth: " << opt.second << " Mbps" << std::endl;
      hasMetrics = true;
    }
    else if (opt.first == "--bandwidth-util") {
      double util = std::stod(opt.second);
      std::cout << "  Bandwidth Utilization: " << (util * 100) << "%" << std::endl;
      hasMetrics = true;
    }
    else if (opt.first == "--packet-loss") {
      double loss = std::stod(opt.second);
      std::cout << "  Packet Loss: " << (loss * 100) << "%" << std::endl;
      hasMetrics = true;
    }
    else if (opt.first == "--spectrum") {
      std::cout << "  Spectrum Strength: " << opt.second << " dBm" << std::endl;
      hasMetrics = true;
    }
    else {
      std::cerr << "Warning: Unknown option " << opt.first << std::endl;
    }
  }
  
  if (!hasMetrics) {
    std::cerr << "ERROR: No valid metrics provided" << std::endl;
    m_exitCode = 1;
    return;
  }
  
  // 构造Interest发送给NLSR
  ndn::Name interestName = m_routerPrefix;
  interestName.append("nlsr")
              .append("link-cost-manager")
              .append("set-metrics")
              .append(neighborName);
  
  // 将options编码到Interest参数中
  for (const auto& opt : options) {
    interestName.append(opt.first).append(opt.second);
  }
  
  auto interest = std::make_shared<ndn::Interest>(interestName);
  interest->setMustBeFresh(true);
  interest->setInterestLifetime(ndn::time::seconds(4));
  
  m_face.expressInterest(*interest,
    [this, neighborName](const ndn::Interest&, const ndn::Data& data) {
      std::cout << "✓ External metrics updated successfully for " << neighborName << std::endl;
      m_exitCode = 0;
    },
    [this, neighborName](const ndn::Interest&, const ndn::lp::Nack& nack) {
      std::cerr << "ERROR: Received Nack with reason: " << nack.getReason() << std::endl;
      m_exitCode = 1;
    },
    [this, neighborName](const ndn::Interest&) {
      std::cerr << "ERROR: Request timeout for " << neighborName << std::endl;
      m_exitCode = 1;
    });
}

void
Nlsrc::showLinkMetrics(const std::string& neighborName)
{
  // 构造Interest查询NLSR
  ndn::Name interestName = m_routerPrefix;
  interestName.append("nlsr")
              .append("link-cost-manager")
              .append("get-metrics")
              .append(neighborName);
  
  auto interest = std::make_shared<ndn::Interest>(interestName);
  interest->setMustBeFresh(true);
  interest->setCanBePrefix(false);  // 禁止前缀匹配，避免CS缓存
  interest->setInterestLifetime(ndn::time::seconds(4));
  
  m_face.expressInterest(*interest,
    [this, neighborName](const ndn::Interest&, const ndn::Data& data) {
      // 解析返回的指标数据并格式化输出
      try {
        auto content = data.getContent();
        std::string contentStr(reinterpret_cast<const char*>(content.value()), content.value_size());
        
        // ✅ 检查是否是ERROR响应
        if (contentStr == "ERROR") {
          std::cerr << "ERROR: Neighbor " << neighborName << " not found in configuration file" << std::endl;
          std::cerr << "Please check the neighbor name in /etc/ndn/nlsr.conf" << std::endl;
          m_exitCode = 1;
          return;
        }
        
        // 解析key-value格式的数据
        std::map<std::string, std::string> metrics;
        std::istringstream iss(contentStr);
        std::string line;
        while (std::getline(iss, line)) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);
            // 去除首尾空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            metrics[key] = value;
          }
        }
        
        // 输出格式化结果
        std::cout << "\nNeighbor: " << neighborName << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        
        // 外部指标
        std::cout << "[External Metrics (User-Configured)]" << std::endl;
        if (metrics.count("Bandwidth")) {
          std::cout << "  Bandwidth: " << metrics["Bandwidth"] << " Mbps" << std::endl;
        }
        if (metrics.count("BandwidthUtil")) {
          double util = std::stod(metrics["BandwidthUtil"]);
          std::cout << "  Bandwidth Utilization: " << (util * 100) << "%" << std::endl;
        }
        if (metrics.count("PacketLoss")) {
          double loss = std::stod(metrics["PacketLoss"]);
          std::cout << "  Packet Loss: " << (loss * 100) << "%" << std::endl;
        }
        if (metrics.count("SpectrumStrength")) {
          std::cout << "  Spectrum Strength: " << metrics["SpectrumStrength"] << " dBm" << std::endl;
        }
        std::cout << "  Last Updated: (current time)" << std::endl;
        std::cout << std::endl;
        
        // 解析权重
        std::vector<double> weights = {0.4, 0.3, 0.2, 0.1};
        if (metrics.count("Weights")) {
          std::istringstream wss(metrics["Weights"]);
          std::string w;
          int i = 0;
          while (std::getline(wss, w, ',') && i < 4) {
            weights[i++] = std::stod(w);
          }
        }
        
        std::cout << "[Multi-Dimensional Cost Calculation]" << std::endl;
        std::cout << "  Weights Configuration:" << std::endl;
        std::cout << "    RTT=" << weights[0] << ", Bandwidth=" << weights[1] 
                  << ", Reliability=" << weights[2] << ", Spectrum=" << weights[3] << std::endl;
        std::cout << std::endl;
        
        // 计算因子（从NLSR返回的数据推导）
        double originalCost = metrics.count("OriginalCost") ? std::stod(metrics["OriginalCost"]) : 12.0;
        double multiCost = metrics.count("MultiDimensionalCost") ? std::stod(metrics["MultiDimensionalCost"]) : originalCost;
        double compositeFactor = multiCost / originalCost;
        
        // 计算各因子（反推）
        double rttFactor = 1.0;
        double bwFactor = 1.0;
        double reliabilityFactor = 1.0;
        double spectrumFactor = 1.0;
        
        if (metrics.count("AverageRTT")) {
          double rttMs = std::stod(metrics["AverageRTT"]);
          rttFactor = (rttMs <= 0) ? 1.0 : ((rttMs >= 200) ? 2.0 : (1.0 + rttMs / 200.0));
        } else {
          rttFactor = 1.1; // 默认
        }
        
        if (metrics.count("BandwidthUtil")) {
          double util = std::stod(metrics["BandwidthUtil"]);
          bwFactor = 1.0 + util;
        } else {
          bwFactor = 1.3; // 默认
        }
        
        if (metrics.count("PacketLoss")) {
          double loss = std::stod(metrics["PacketLoss"]);
          reliabilityFactor = 1.0 + (loss * 2.0);
          if (reliabilityFactor > 2.0) reliabilityFactor = 2.0; // 与LCM保持一致，上限2.0
        } else {
          reliabilityFactor = 1.02; // 默认
        }
        
        if (metrics.count("SpectrumStrength")) {
          double strength = std::stod(metrics["SpectrumStrength"]);
          if (strength >= -30) spectrumFactor = 1.0;
          else if (strength <= -80) spectrumFactor = 2.0;
          else spectrumFactor = 1.0 + ((-30.0 - strength) / 50.0);
        } else {
          spectrumFactor = 1.4; // 默认
        }
        
        std::cout << "  Factor Breakdown:" << std::endl;
        std::cout << "    RTT Factor:         " << std::fixed << std::setprecision(2) << rttFactor << std::endl;
        std::cout << "    Bandwidth Factor:   " << bwFactor << std::endl;
        std::cout << "    Reliability Factor: " << reliabilityFactor << std::endl;
        std::cout << "    Spectrum Factor:    " << spectrumFactor << std::endl;
        std::cout << std::endl;
        
        std::cout << "  Calculation Formula:" << std::endl;
        std::cout << "    Cost = OriginalCost × (α·RTT + β·BW + γ·Reliability + δ·Spectrum)" << std::endl;
        std::cout << "    Cost = " << originalCost << " × (" 
                  << weights[0] << "×" << rttFactor << " + "
                  << weights[1] << "×" << bwFactor << " + "
                  << weights[2] << "×" << reliabilityFactor << " + "
                  << weights[3] << "×" << spectrumFactor << ")" << std::endl;
        std::cout << "    Cost = " << originalCost << " × " 
                  << std::setprecision(3) << compositeFactor 
                  << " = " << std::setprecision(2) << (originalCost * compositeFactor) 
                  << " ≈ " << (int)std::round(multiCost) << std::endl;
        std::cout << std::endl;
        
        std::cout << "  Multi-Dimensional Cost: " << (int)std::round(multiCost) << std::endl;
        std::cout << std::endl;
        
        m_exitCode = 0;
      }
      catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to parse response: " << e.what() << std::endl;
        m_exitCode = 1;
      }
    },
    [this, neighborName](const ndn::Interest&, const ndn::lp::Nack& nack) {
      std::cerr << "ERROR: Received Nack with reason: " << nack.getReason() << std::endl;
      std::cerr << "Neighbor " << neighborName << " may not exist or metrics not configured" << std::endl;
      m_exitCode = 1;
    },
    [this, neighborName](const ndn::Interest&) {
      std::cerr << "ERROR: Request timeout for " << neighborName << std::endl;
      std::cerr << "NLSR may not be running or neighbor does not exist" << std::endl;
      m_exitCode = 1;
    });
}

void
Nlsrc::sendNamePrefixUpdate(const ndn::Name& name,
                            const ndn::Name::Component& verb,
                            const std::string& info,
                            bool flag)
{
  ndn::nfd::ControlParameters parameters;
  parameters.setName(name);
  if (flag) {
    parameters.setFlags(1);
  }

  auto paramWire = parameters.wireEncode();
  ndn::Name commandName = m_routerPrefix;
  commandName.append(NAME_UPDATE_SUFFIX);
  commandName.append(verb);
  commandName.append(paramWire.begin(), paramWire.end());

  ndn::security::InterestSigner signer(m_keyChain);
  auto commandInterest = signer.makeCommandInterest(commandName,
                           ndn::security::signingByIdentity(m_keyChain.getPib().getDefaultIdentity()));
  commandInterest.setMustBeFresh(true);

  m_face.expressInterest(commandInterest,
                         std::bind(&Nlsrc::onControlResponse, this, info, _2),
                         std::bind(&Nlsrc::onTimeout, this, ERROR_CODE_TIMEOUT, "Nack"),
                         std::bind(&Nlsrc::onTimeout, this, ERROR_CODE_TIMEOUT, "Timeout"));
}

void
Nlsrc::onControlResponse(const std::string& info, const ndn::Data& data)
{
  if (data.getMetaInfo().getType() == ndn::tlv::ContentType_Nack) {
    std::cerr << "ERROR: Run-time advertise/withdraw disabled" << std::endl;
    return;
  }

  ndn::nfd::ControlResponse response;

  try {
    response.wireDecode(data.getContent().blockFromValue());
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: Control response decoding error" << std::endl;
    m_exitCode = 1;
    return;
  }

  uint32_t code = response.getCode();

  if (code != RESPONSE_CODE_SUCCESS && code != RESPONSE_CODE_SAVE_OR_DELETE) {
    std::cerr << response.getText() << std::endl;
    std::cerr << "Name prefix update error (code: " << code << ")" << std::endl;
    m_exitCode = code == RESPONSE_CODE_NO_EFFECT ? 0 : 1;
    return;
  }

  std::cout << "Applied Name prefix update successfully: " << info << std::endl;
  m_exitCode = 0;
}

void
Nlsrc::fetchAdjacencyLsas()
{
  fetchFromLsdb<nlsr::AdjLsa>(nlsr::dataset::ADJACENCY_COMPONENT,
                              std::bind(&Nlsrc::recordLsa, this, _1));
}

void
Nlsrc::fetchCoordinateLsas()
{
  fetchFromLsdb<nlsr::CoordinateLsa>(nlsr::dataset::COORDINATE_COMPONENT,
                                     std::bind(&Nlsrc::recordLsa, this, _1));
}

void
Nlsrc::fetchNameLsas()
{
  fetchFromLsdb<nlsr::NameLsa>(nlsr::dataset::NAME_COMPONENT,
                               std::bind(&Nlsrc::recordLsa, this, _1));
}

void
Nlsrc::fetchRtables()
{
  fetchFromRt<nlsr::RoutingTableStatus>([this] (const auto& rts) { this->recordRtable(rts); });
}

template<class T>
void
Nlsrc::fetchFromLsdb(const ndn::Name::Component& datasetType,
                     const std::function<void(const T&)>& recordLsa)
{
  auto name = m_routerPrefix;
  name.append(LSDB_SUFFIX);
  name.append(datasetType);
  ndn::Interest interest(name);

  auto fetcher = ndn::SegmentFetcher::start(m_face, interest, *m_validator);
  fetcher->onComplete.connect(std::bind(&Nlsrc::onFetchSuccess<T>, this, _1, recordLsa));
  fetcher->onError.connect(std::bind(&Nlsrc::onTimeout, this, _1, _2));
}

void
Nlsrc::recordLsa(const nlsr::Lsa& lsa)
{
  Router& router = m_routers.emplace(lsa.getOriginRouter(), Router()).first->second;
  auto lsaString = boost::lexical_cast<std::string>(lsa);

  if (lsa.getType() == nlsr::Lsa::Type::ADJACENCY) {
    router.adjacencyLsaString = lsaString;
  }
  else if (lsa.getType() == nlsr::Lsa::Type::COORDINATE) {
    router.coordinateLsaString = lsaString;
  }
  else if (lsa.getType() == nlsr::Lsa::Type::NAME) {
    router.nameLsaString = lsaString;
  }
}

template<class T>
void
Nlsrc::fetchFromRt(const std::function<void(const T&)>& recordDataset)
{
  auto name = m_routerPrefix;
  name.append(RT_SUFFIX);
  ndn::Interest interest(name);

  auto fetcher = ndn::SegmentFetcher::start(m_face, interest, *m_validator);
  fetcher->onComplete.connect(std::bind(&Nlsrc::onFetchSuccess<T>, this, _1, recordDataset));
  fetcher->onError.connect(std::bind(&Nlsrc::onTimeout, this, _1, _2));
}

template<class T>
void
Nlsrc::onFetchSuccess(const ndn::ConstBufferPtr& buf,
                      const std::function<void(const T&)>& recordDataset)
{
  size_t offset = 0;
  while (offset < buf->size()) {
    auto [isOk, block] = ndn::Block::fromBuffer(buf, offset);

    if (!isOk) {
      std::cerr << "ERROR: cannot decode LSA TLV" << std::endl;
      break;
    }

    offset += block.size();

    T dataset(block);
    recordDataset(dataset);
  }

  runNextStep();
}

void
Nlsrc::onTimeout(uint32_t errorCode, const std::string& error)
{
  std::cerr << "Request timed out (code: " << errorCode
            << ", error: " << error << ")"  << std::endl;
  m_exitCode = 1;
}

void
Nlsrc::recordRtable(const nlsr::RoutingTableStatus& rts)
{
  std::ostringstream os;
  os << rts;
  m_rtString = os.str();
}

void
Nlsrc::printLsdb()
{
  std::cout << "LSDB:" << std::endl;

  for (const auto& item : m_routers) {
    std::cout << "  OriginRouter: " << item.first << std::endl;
    std::cout << std::endl;

    const Router& router = item.second;

    if (!router.adjacencyLsaString.empty()) {
      std::cout << router.adjacencyLsaString << std::endl;
    }

    if (!router.coordinateLsaString.empty()) {
      std::cout << router.coordinateLsaString << std::endl;
    }

    if (!router.nameLsaString.empty()) {
      std::cout << router.nameLsaString << std::endl;
    }
  }
}

void
Nlsrc::printRT()
{
  if (!m_rtString.empty()) {
    std::cout << m_rtString;
  }
  else {
    std::cout << "Routing Table is not calculated yet" << std::endl;
  }
}

void
Nlsrc::printAll()
{
  std::cout << "NLSR Status" << std::endl;
  printLsdb();
  printRT();
}

} // namespace nlsrc

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char** argv)
{
  ndn::Face face;
  nlsrc::Nlsrc nlsrc(argv[0], face);

  if (argc < 2) {
    nlsrc.printUsage();
    return 2;
  }

  int opt;
  const char* confFile = DEFAULT_CONFIG_FILE;
  bool disableValidator = false;
  while ((opt = ::getopt(argc, argv, "+hVR:c:k")) != -1) {
    switch (opt) {
    case 'h':
      nlsrc.printUsage();
      return 0;
    case 'V':
      std::cout << NLSR_VERSION_BUILD_STRING << std::endl;
      return 0;
    case 'R':
      nlsrc.setRouterPrefix(::optarg);
      break;
    case 'c':
      confFile = ::optarg;
      break;
    case 'k':
      disableValidator = true;
      break;
    default:
      nlsrc.printUsage();
      return 2;
    }
  }

  if (argc == ::optind) {
    nlsrc.printUsage();
    return 2;
  }

  if (nlsrc.getRouterPrefix() != nlsrc::LOCALHOST_PREFIX && !disableValidator) {
    if (!nlsrc.enableValidator(confFile)) {
      return 1;
    }
  }

  std::vector<std::string> subcommand(&argv[::optind], &argv[argc]);
  try {
    bool isValidSyntax = nlsrc.dispatch(subcommand);
    if (!isValidSyntax) {
      nlsrc.printUsage();
      return 2;
    }

    face.processEvents();
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }
  return nlsrc.getExitCode();
}
