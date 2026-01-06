#include "bus.h"
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <limits>

using namespace std;

static const int BIG = 100000;

// Dijkstra 用最小堆元素（状态ID + 距离）
struct PQItem {
    int state;
    int dist;
    PQItem() : state(0), dist(0) {}
    PQItem(int s, int d) : state(s), dist(d) {}
};

// 最小堆比较器
struct PQGreater {
    bool operator()(const PQItem& a, const PQItem& b) const {
        return a.dist > b.dist;
    }
};

// 拼接某条线路上 A->B 的站点序列到 out
static bool appendSegment(const vector<int>& lineStations, int fromSid, int toSid, vector<int>& out) {
    int posFrom = -1, posTo = -1;
    for (size_t i = 0; i < lineStations.size(); ++i) {
        if (lineStations[i] == fromSid) posFrom = (int)i;
        if (lineStations[i] == toSid) posTo = (int)i;
    }
    if (posFrom == -1 || posTo == -1) return false;

    if (posFrom <= posTo) {
        for (int i = posFrom; i <= posTo; ++i) {
            int sid = lineStations[i];
            if (!out.empty() && out.back() == sid) continue;
            out.push_back(sid);
        }
    } else {
        for (int i = posFrom; i >= posTo; --i) {
            int sid = lineStations[i];
            if (!out.empty() && out.back() == sid) continue;
            out.push_back(sid);
        }
    }
    return true;
}

// 打印站点序列（用名称）
static void printStationSeq(const vector<int>& stationSeq, const vector<string>& nameOf) {
    for (size_t i = 0; i < stationSeq.size(); ++i) {
        if (i) cout << " -> ";
        int sid = stationSeq[i];
        cout << nameOf[sid];
    }
    cout << "\n";
}

// 最少换乘（以线路为节点做 BFS）
void solveMinTransfers(
    int L,
    const vector< vector<int> >& lineStations,
    const vector< vector<int> >& stationLines,
    const vector< vector<char> >& inLine,
    int startSid,
    int endSid,
    const vector<string>& nameOf
) {
    vector<char> isGoal(L + 1, 0);
    for (size_t i = 0; i < stationLines[endSid].size(); ++i) isGoal[stationLines[endSid][i]] = 1;

    vector<int> dist(L + 1, -1);
    vector<int> prevLine(L + 1, -1);
    vector<int> prevStation(L + 1, -1);
    queue<int> q;

    for (size_t i = 0; i < stationLines[startSid].size(); ++i) {
        int l = stationLines[startSid][i];
        dist[l] = 0;
        q.push(l);
    }

    int goalLine = -1;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        if (isGoal[u]) {
            goalLine = u;
            break;
        }
        for (int sid = 0; sid < (int)stationLines.size(); ++sid) {
            if (!inLine[u][sid]) continue;
            const vector<int>& linesHere = stationLines[sid];
            for (size_t k = 0; k < linesHere.size(); ++k) {
                int v = linesHere[k];
                if (v == u) continue;
                if (dist[v] != -1) continue;
                dist[v] = dist[u] + 1;
                prevLine[v] = u;
                prevStation[v] = sid;
                q.push(v);
            }
        }
    }

    if (goalLine == -1) {
        cout << "无可达路线\n";
        return;
    }

    vector<int> lineSeq;
    vector<int> transferStations;
    int cur = goalLine;
    while (cur != -1) {
        lineSeq.push_back(cur);
        int ps = prevStation[cur];
        if (ps != -1) transferStations.push_back(ps);
        cur = prevLine[cur];
    }
    for (size_t i = 0, j = lineSeq.size() - 1; i < j; ++i, --j) {
        int tmp = lineSeq[i]; lineSeq[i] = lineSeq[j]; lineSeq[j] = tmp;
    }
    for (size_t i = 0, j = transferStations.size() - 1; i < j; ++i, --j) {
        int tmp = transferStations[i]; transferStations[i] = transferStations[j]; transferStations[j] = tmp;
    }

    int transfers = (int)lineSeq.size() - 1;
    cout << "最少换乘次数: " << transfers << "\n";
    cout << "线路序列: ";
    for (size_t i = 0; i < lineSeq.size(); ++i) {
        if (i) cout << " -> ";
        cout << "L" << lineSeq[i];
    }
    cout << "\n";

    vector<int> stationSeq;
    if (lineSeq.size() == 1) {
        appendSegment(lineStations[lineSeq[0]], startSid, endSid, stationSeq);
    } else {
        int firstTransferSid = transferStations[0];
        appendSegment(lineStations[lineSeq[0]], startSid, firstTransferSid, stationSeq);
        for (size_t i = 1; i + 1 < lineSeq.size(); ++i) {
            int aSid = transferStations[i - 1];
            int bSid = transferStations[i];
            appendSegment(lineStations[lineSeq[i]], aSid, bSid, stationSeq);
        }
        int lastTransferSid = transferStations[transferStations.size() - 1];
        appendSegment(lineStations[lineSeq[lineSeq.size() - 1]], lastTransferSid, endSid, stationSeq);
    }
    cout << "站点序列: ";
    printStationSeq(stationSeq, nameOf);
}

// 状态键：站点 + 当前线路（0 表示未选择线路）
struct StateKey {
    int station;
    int line;
    StateKey() : station(0), line(0) {}
    StateKey(int s, int l) : station(s), line(l) {}
};

static long long packKey(int station, int line, int L) {
    return (long long)station * (long long)(L + 1) + (long long)line;
}

void solveWeighted(
    int L,
    const vector< vector<int> >& lineStations,
    const vector< vector<int> >& stationLines,
    const vector< vector<int> >& posInLine,
    int startSid,
    int endSid,
    const vector<string>& nameOf
) {
    if (startSid == endSid) {
        cout << "换乘次数: 0, 总站数: 0\n";
        cout << "站点序列: " << nameOf[startSid] << "\n";
        cout << "线路序列: （无）\n";
        return;
    }

    map<long long, int> stateId;
    vector<StateKey> idToKey;

    int startState = 0;
    stateId[packKey(startSid, 0, L)] = 0;
    idToKey.push_back(StateKey(startSid, 0));

    for (int sid = 0; sid < (int)stationLines.size(); ++sid) {
        for (size_t i = 0; i < stationLines[sid].size(); ++i) {
            int l = stationLines[sid][i];
            long long k = packKey(sid, l, L);
            if (stateId.find(k) != stateId.end()) continue;
            int id = (int)idToKey.size();
            stateId[k] = id;
            idToKey.push_back(StateKey(sid, l));
        }
    }

    int S = (int)idToKey.size();
    vector<int> dist(S, numeric_limits<int>::max());
    vector<int> prev(S, -1);

    priority_queue<PQItem, vector<PQItem>, PQGreater> pq;
    dist[startState] = 0;
    pq.push(PQItem(startState, 0));

    while (!pq.empty()) {
        PQItem cur = pq.top();
        pq.pop();
        int u = cur.state;
        if (cur.dist != dist[u]) continue;

        int station = idToKey[u].station;
        int line = idToKey[u].line;

        if (station == endSid && line != 0) break;

        const vector<int>& linesHere = stationLines[station];
        for (size_t i = 0; i < linesHere.size(); ++i) {
            int l2 = linesHere[i];
            int add = 0;
            if (line == 0) add = 0;
            else if (l2 != line) add = BIG;

            int v = stateId[packKey(station, l2, L)];
            if (dist[u] + add < dist[v]) {
                dist[v] = dist[u] + add;
                prev[v] = u;
                pq.push(PQItem(v, dist[v]));
            }
        }

        if (line != 0) {
            int pos = posInLine[line][station];
            if (pos != -1) {
                if (pos - 1 >= 0) {
                    int ns = lineStations[line][pos - 1];
                    int v = stateId[packKey(ns, line, L)];
                    if (dist[u] + 1 < dist[v]) {
                        dist[v] = dist[u] + 1;
                        prev[v] = u;
                        pq.push(PQItem(v, dist[v]));
                    }
                }
                if (pos + 1 < (int)lineStations[line].size()) {
                    int ns = lineStations[line][pos + 1];
                    int v = stateId[packKey(ns, line, L)];
                    if (dist[u] + 1 < dist[v]) {
                        dist[v] = dist[u] + 1;
                        prev[v] = u;
                        pq.push(PQItem(v, dist[v]));
                    }
                }
            }
        }
    }

    int bestState = -1;
    int bestDist = numeric_limits<int>::max();
    for (size_t i = 0; i < stationLines[endSid].size(); ++i) {
        int l = stationLines[endSid][i];
        int id = stateId[packKey(endSid, l, L)];
        if (dist[id] < bestDist) {
            bestDist = dist[id];
            bestState = id;
        }
    }
    if (bestState == -1 || bestDist == numeric_limits<int>::max()) {
        cout << "无可达路线\n";
        return;
    }

    vector<int> states;
    int cur = bestState;
    while (cur != -1) {
        states.push_back(cur);
        cur = prev[cur];
    }
    for (size_t i = 0, j = states.size() - 1; i < j; ++i, --j) {
        int tmp = states[i]; states[i] = states[j]; states[j] = tmp;
    }

    vector<int> stationSeq;
    vector<int> lineSeq;
    int lastLine = 0;
    for (size_t i = 0; i < states.size(); ++i) {
        int sid = idToKey[states[i]].station;
        int l = idToKey[states[i]].line;
        if (stationSeq.empty() || stationSeq.back() != sid) stationSeq.push_back(sid);
        if (l != 0 && l != lastLine) {
            lineSeq.push_back(l);
            lastLine = l;
        }
    }

    int transfers = bestDist / BIG;
    int stops = bestDist % BIG;
    cout << "换乘次数: " << transfers << ", 总站数: " << stops << "\n";
    cout << "线路序列: ";
    if (lineSeq.empty()) cout << "（无）\n";
    else {
        for (size_t i = 0; i < lineSeq.size(); ++i) {
            if (i) cout << " -> ";
            cout << "L" << lineSeq[i];
        }
        cout << "\n";
    }
    cout << "站点序列: ";
    printStationSeq(stationSeq, nameOf);
}
