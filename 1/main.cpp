#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <ctime>
#include "graph.h"
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);
    #ifdef _WIN32
    SetConsoleOutputCP(65001);
    #endif

    // 读取城市图基础信息：节点数 N、道路数 M
    int N, M;
    if (!(cin >> N >> M)) {
        cout << "输入格式:\n";
        cout << "N M\n";
        cout << "接下来 M 行: u v 长度 拥堵系数 红绿灯数量\n";
        cout << "Q\n";
        cout << "接下来 Q 行: type s t   (type=1 按长度最短路径, type=2 按加权通行时间)\n";
        return 0;
    }

    // 构建无向图（邻接表）
    vector< vector<Edge> > g(N + 1);
    for (int i = 0; i < M; ++i) {
        int u, v, lights;
        double len, cong;
        cin >> u >> v >> len >> cong >> lights;
        if (u < 1 || u > N || v < 1 || v > N) continue;
        if (cong < 1.0) cong = 1.0;
        g[u].push_back(Edge(v, len, cong, lights));
        g[v].push_back(Edge(u, len, cong, lights));
    }

    // 处理 Q 次查询
    int Q;
    cin >> Q;
    for (int qi = 0; qi < Q; ++qi) {
        int type, s, t;
        cin >> type >> s >> t;
        if (s < 1 || s > N || t < 1 || t > N) {
            cout << "节点编号无效\n";
            continue;
        }
        // mode=0 按长度；mode=1 按长度*拥堵+红绿灯等待
        int mode = (type == 2 ? 1 : 0);
        vector<int> prevNode;
        vector<double> dist;
        clock_t st = clock();
        bool ok = dijkstra(g, s, t, mode, prevNode, dist);
        clock_t ed = clock();
        if (!ok) {
            cout << "无可达路径\n";
            continue;
        }
        vector<int> path = buildPath(s, t, prevNode);
        if (mode == 0) {
            cout << fixed << setprecision(3);
            cout << "最短距离: " << dist[t] << "\n";
            cout << "路径: ";
            printPath(path);
            cout << "耗时(毫秒): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        } else {
            cout << fixed << setprecision(3);
            cout << "最短加权通行时间: " << dist[t] << "\n";
            cout << "路径: ";
            printPath(path);
            cout << "耗时(毫秒): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        }
    }
    return 0;
}
