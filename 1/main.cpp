#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <iomanip>
#include <ctime>

using namespace std;

struct Edge {
    int to;
    double length;
    double congestion;
    int lights;
    Edge() : to(0), length(0), congestion(1), lights(0) {}
    Edge(int t, double len, double cong, int l) : to(t), length(len), congestion(cong), lights(l) {}
};

static const double LIGHT_WAIT_SECONDS = 30.0;

struct NodeDist {
    int node;
    double dist;
    NodeDist() : node(0), dist(0) {}
    NodeDist(int n, double d) : node(n), dist(d) {}
};

struct NodeDistGreater {
    bool operator()(const NodeDist& a, const NodeDist& b) const {
        return a.dist > b.dist;
    }
};

static double edgeWeight(const Edge& e, int mode) {
    if (mode == 1) {
        return e.length * e.congestion + (double)e.lights * LIGHT_WAIT_SECONDS;
    }
    return e.length;
}

static bool dijkstra(const vector< vector<Edge> >& g, int s, int t, int mode, vector<int>& prevNode, vector<double>& dist) {
    int n = (int)g.size() - 1;
    dist.assign(n + 1, numeric_limits<double>::infinity());
    prevNode.assign(n + 1, -1);

    priority_queue<NodeDist, vector<NodeDist>, NodeDistGreater> pq;
    dist[s] = 0.0;
    pq.push(NodeDist(s, 0.0));

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

static vector<int> buildPath(int s, int t, const vector<int>& prevNode) {
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

static void printPath(const vector<int>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) cout << " -> ";
        cout << path[i];
    }
    cout << "\n";
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

    int N, M;
    if (!(cin >> N >> M)) {
        cout << "Input format:\n";
        cout << "N M\n";
        cout << "M lines: u v length congestion lights\n";
        cout << "Q\n";
        cout << "Q lines: type s t   (type=1 length shortest path, type=2 weighted time)\n";
        return 0;
    }

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

    int Q;
    cin >> Q;
    for (int qi = 0; qi < Q; ++qi) {
        int type, s, t;
        cin >> type >> s >> t;
        if (s < 1 || s > N || t < 1 || t > N) {
            cout << "Invalid nodes\n";
            continue;
        }
        int mode = (type == 2 ? 1 : 0);
        vector<int> prevNode;
        vector<double> dist;
        clock_t st = clock();
        bool ok = dijkstra(g, s, t, mode, prevNode, dist);
        clock_t ed = clock();
        if (!ok) {
            cout << "No path\n";
            continue;
        }
        vector<int> path = buildPath(s, t, prevNode);
        if (mode == 0) {
            cout << fixed << setprecision(3);
            cout << "Shortest distance: " << dist[t] << "\n";
            cout << "Path: ";
            printPath(path);
            cout << "Elapsed(ms): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        } else {
            cout << fixed << setprecision(3);
            cout << "Shortest weighted time: " << dist[t] << "\n";
            cout << "Path: ";
            printPath(path);
            cout << "Elapsed(ms): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        }
    }

    return 0;
}
