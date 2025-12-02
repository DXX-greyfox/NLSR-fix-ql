/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */


#include "logger.hpp" // 用于 NLSR_LOG_ERROR
#include "conf-parameter.hpp" // 用于获取 state-dir
#include "route/name-map.hpp" // 用于 NameMap
#include "adjacent.hpp" // 用于 Adjacent::NON_ADJACENT_COST

#include <ndn-cxx/name.hpp>
#include "topology.hpp"
#include <fstream>   // 用于 std::ofstream
#include <sstream>   // 用于 std::ostringstream
#include <filesystem> // 用于 std::filesystem (C++17, 标准3)
#include <stdexcept> // 用于 std::exception

namespace nlsr {

INIT_LOGGER(route.TopologyExporter);

void
exportTopology(const AdjMatrix& matrix, const NameMap& map, 
               const ConfParameter& confParam)
{
  // 步骤 1: 确定写入路径 (基于 nlsr.conf 中的 state-dir)
  std::filesystem::path stateDir = confParam.getStateFileDir();
  std::filesystem::path topologyPath = stateDir / "topology.json";
  std::filesystem::path tempPath = stateDir / "topology.json.tmp";

  try {
    std::ostringstream jsonStream;
    size_t nRouters = map.size();

    // 步骤 2: 构建JSON字符串
    jsonStream << "{\n  \"nodes\": [\n";

    // A. 转换 NameMap (字典) 为 "nodes" 列表
    for (size_t i = 0; i < nRouters; ++i) {
      auto nameOpt = map.getRouterNameByMappingNo(static_cast<int32_t>(i));
      if (nameOpt) {
        jsonStream << "    {\"id\": " << i << ", \"name\": \"" << nameOpt->toUri() << "\"}";
        if (i < nRouters - 1) {
          jsonStream << ",\n";
        }
      }
    }
    jsonStream << "\n  ],\n";

    // B. 转换 AdjMatrix (地图) 为 "links" 列表
    jsonStream << "  \"links\": [\n";
    bool firstLink = true;

    // 优化：只遍历上三角 (j = i + 1)，因为矩阵是对称的
    for (size_t i = 0; i < nRouters; ++i) {
      for (size_t j = i + 1; j < nRouters; ++j) {
        
        double cost = matrix[i][j];

        // 检查链路是否真实存在 (成本不是 "INF")
        if (cost != Adjacent::NON_ADJACENT_COST) {
          if (!firstLink) {
            jsonStream << ",\n";
          }
          jsonStream << "    {\"source\": " << i << ", \"target\": " << j << ", \"cost\": " << cost << "}";
          firstLink = false;
        }
      }
    }
    jsonStream << "\n  ]\n}"; // JSON 结束

    // 步骤 3: 原子性写入文件
    
    // A. 写入临时文件
    std::ofstream tempFile(tempPath, std::ios::out | std::ios::trunc);
    if (!tempFile.is_open()) {
        throw std::runtime_error("Cannot open temporary file: " + tempPath.string());
    }
    tempFile << jsonStream.str();
    tempFile.close();

    // B. 原子性重命名 (调包)
    std::filesystem::rename(tempPath, topologyPath);

  }
  catch (const std::exception& e) {
    // 步骤 4: 故障隔离 (标准5)
    // 我们不希望导出失败导致路由计算崩溃。
    NLSR_LOG_ERROR("Failed to export topology: " << e.what());
    // 安静地失败，不抛出异常
  }
}

} // namespace nlsr