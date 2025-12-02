/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef NLSR_TOPOLOGY_HPP
#define NLSR_TOPOLOGY_HPP

#include "common.hpp"
#include <boost/multi_array.hpp>

// 运行Dijkstra算法所需的邻接矩阵
using AdjMatrix = boost::multi_array<double, 2>;

namespace nlsr {

// 前向声明 (标准3：保持头文件简洁，避免不必要的#include)
class NameMap;
class ConfParameter;

/**
 * @brief 将当前的网络拓扑（邻接矩阵和名称映射）导出为JSON文件。
 *
 * @param matrix 完整的N x N邻接矩阵 (地图)
 * @param map NameMap (字典)
 * @param confParam NLSR配置参数 (用于获取state-dir)
 *

 */
void
exportTopology(const AdjMatrix& matrix, const NameMap& map, 
               const ConfParameter& confParam);

} // namespace nlsr

#endif // NLSR_TOPOLOGY_EXPORTER_HPP