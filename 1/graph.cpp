#include "graph.h"
#include <queue>
#include <limits>
#include <iostream>

using namespace std;

// 边结构体构造
Edge::Edge() : to(0), length(0), congestion(1), lights(0) {}
Edge::Edge(int t, double len, double cong, int l) : to(t), length(len), congestion(cong), lights(l) {}

static const double LIGHT_WAIT_SECONDS = 30.0;

// Dijkstra 用的节点-距离结构（用于优先队列）
struct NodeDist {
    int node;
    double dist;
    NodeDist() : node(0), dist(0) {}
    NodeDist(int n, double d) : node(n), dist(d) {}
};

// 小根堆比较器
struct NodeDistGreater {
    bool operator()(const NodeDist& a, const NodeDist& b) const {
        return a.dist > b.dist;
    }
};

// 统一边权计算
static double edgeWeight(const Edge& e, int mode) {
    if (mode == 1) {
        // 加权模式：长度*拥堵+红绿灯等待（30秒/灯）
        return e.length * e.congestion + (double)e.lights * LIGHT_WAIT_SECONDS;
    }
    return e.length;
}

// Dijkstra 最短路径
bool dijkstra(const vector< vector<Edge> >& g, int s, int t, int mode, vector<int>& prevNode, vector<double>& dist) {
    int n = (int)g.size() - 1; // 节点数（1..N）
    dist.assign(n + 1, numeric_limits<double>::infinity()); // 到各点的最短代价（初始为无穷大）
    prevNode.assign(n + 1, -1); // 前驱数组（用于路径还原）

    priority_queue<NodeDist, vector<NodeDist>, NodeDistGreater> pq; // 优先队列（小根堆）
    dist[s] = 0.0; // 起点到自身的距离为0
    pq.push(NodeDist(s, 0.0)); // 起点入队

    vector<char> used(n + 1, 0);

    while (!pq.empty()) {
        NodeDist cur = pq.top();
        pq.pop();
        int u = cur.node;
        if (used[u]) continue;
        used[u] = 1;
        if (u == t) break;

        for (size_t i = 0; i < g[u].size(); ++i) {
            const Edge& e = g[u][i];
            int v = e.to;
            double w = edgeWeight(e, mode);
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prevNode[v] = u;
                pq.push(NodeDist(v, dist[v]));
            }
        }
    }

    return dist[t] != numeric_limits<double>::infinity();
}

// 路径还原
vector<int> buildPath(int s, int t, const vector<int>& prevNode) {
    vector<int> path;
    int cur = t;
    while (cur != -1) {
        path.push_back(cur);
        if (cur == s) break;
        cur = prevNode[cur];
    }
    if (path.empty() || path.back() != s) return vector<int>();
    for (size_t i = 0, j = path.size() - 1; i < j; ++i, --j) {
        int tmp = path[i];
        path[i] = path[j];
        path[j] = tmp;
    }
    return path;
}

// 打印路径
void printPath(const vector<int>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) cout << " -> ";
        cout << path[i];
    }
    cout << "\n";
}

