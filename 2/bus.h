#ifndef BUS_H
#define BUS_H

// 公交换乘优化模块（中文注释）
// 提供最少换乘与综合（换乘+总站数）求解接口

#include <vector>
#include <string>

// 计算最少换乘次数（BFS）
// 参数：
// L：线路数量
// lineStations：每条线路的站点序列
// stationLines：每个站点所属线路列表
// inLine：inLine[l][sid] 表示站点 sid 是否在线路 l 上
// startSid / endSid：起止站点编号
// nameOf：站点名称
void solveMinTransfers(
    int L,
    const std::vector< std::vector<int> >& lineStations,
    const std::vector< std::vector<int> >& stationLines,
    const std::vector< std::vector<char> >& inLine,
    int startSid,
    int endSid,
    const std::vector<std::string>& nameOf
);

// 综合最优：先最少换乘，再最少总站数（Dijkstra，字典序权重）
// 参数同上，posInLine 提供站点在各线路中的位置索引
void solveWeighted(
    int L,
    const std::vector< std::vector<int> >& lineStations,
    const std::vector< std::vector<int> >& stationLines,
    const std::vector< std::vector<int> >& posInLine,
    int startSid,
    int endSid,
    const std::vector<std::string>& nameOf
);

#endif
