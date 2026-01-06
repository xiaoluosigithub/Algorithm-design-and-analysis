#ifndef GRAPH_H
#define GRAPH_H
#include <vector>

struct Edge {
    int to;    // to：目标节点编号
    double length;    // length：道路长度
    double congestion;    // congestion：拥堵系数（>=1）
    int lights;    // lights：红绿灯数量
    Edge();
    Edge(int t, double len, double cong, int l);
};

// Dijkstra 最短路
// 参数：
// g：邻接表图（1..N）
// s：起点
// t：终点
// mode：0=仅长度；1=长度*拥堵+红绿灯等待（30秒/灯）
// 输出：
// prevNode：前驱数组用于还原路径
// dist：到各点的最短代价
bool dijkstra(const std::vector< std::vector<Edge> >& g, int s, int t, int mode, std::vector<int>& prevNode, std::vector<double>& dist);

// 根据前驱数组还原 s->t 的路径节点序列
std::vector<int> buildPath(int s, int t, const std::vector<int>& prevNode);

// 打印路径节点序列
void printPath(const std::vector<int>& path);

#endif
