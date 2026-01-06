#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <limits>
#include <ctime>

using namespace std;

static const int BIG = 100000;

struct PQItem {
    int state;
    int dist;
    PQItem() : state(0), dist(0) {}
    PQItem(int s, int d) : state(s), dist(d) {}
};

struct PQGreater {
    bool operator()(const PQItem& a, const PQItem& b) const {
        return a.dist > b.dist;
    }
};

static int getStationId(map<string, int>& idOf, vector<string>& nameOf, const string& name) {
    map<string, int>::iterator it = idOf.find(name);
    if (it != idOf.end()) return it->second;
    int id = (int)nameOf.size();
    idOf[name] = id;
    nameOf.push_back(name);
    return id;
}

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

static void printStationSeq(const vector<int>& stationSeq, const vector<string>& nameOf) {
    for (size_t i = 0; i < stationSeq.size(); ++i) {
        if (i) cout << " -> ";
        int sid = stationSeq[i];
        cout << nameOf[sid];
    }
    cout << "\n";
}

static void solveMinTransfers(
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
        cout << "No route\n";
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
    cout << "Min transfers: " << transfers << "\n";
    cout << "Line sequence: ";
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
    cout << "Station sequence: ";
    printStationSeq(stationSeq, nameOf);
}

struct StateKey {
    int station;
    int line;
    StateKey() : station(0), line(0) {}
    StateKey(int s, int l) : station(s), line(l) {}
};

static long long packKey(int station, int line, int L) {
    return (long long)station * (long long)(L + 1) + (long long)line;
}

static void solveWeighted(
    int L,
    const vector< vector<int> >& lineStations,
    const vector< vector<int> >& stationLines,
    const vector< vector<int> >& posInLine,
    int startSid,
    int endSid,
    const vector<string>& nameOf
) {
    if (startSid == endSid) {
        cout << "Transfers: 0, Stops: 0\n";
        cout << "Station sequence: " << nameOf[startSid] << "\n";
        cout << "Line sequence: (none)\n";
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
            else continue;

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
        cout << "No route\n";
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
    cout << "Transfers: " << transfers << ", Stops: " << stops << "\n";
    cout << "Line sequence: ";
    if (lineSeq.empty()) cout << "(none)\n";
    else {
        for (size_t i = 0; i < lineSeq.size(); ++i) {
            if (i) cout << " -> ";
            cout << "L" << lineSeq[i];
        }
        cout << "\n";
    }
    cout << "Station sequence: ";
    printStationSeq(stationSeq, nameOf);
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

    int L;
    if (!(cin >> L)) {
        cout << "Input format:\n";
        cout << "L\n";
        cout << "For each line i=1..L: k stationName1 ... stationNameK\n";
        cout << "Q\n";
        cout << "Q lines: type startStation endStation  (type=1 min transfers, type=2 weighted)\n";
        return 0;
    }

    map<string, int> stationId;
    vector<string> nameOf;

    vector< vector<int> > lineStations(L + 1);
    for (int i = 1; i <= L; ++i) {
        int k;
        cin >> k;
        lineStations[i].resize(k);
        for (int j = 0; j < k; ++j) {
            string s;
            cin >> s;
            int sid = getStationId(stationId, nameOf, s);
            lineStations[i][j] = sid;
        }
    }

    int S = (int)nameOf.size();
    vector< vector<int> > stationLines(S);
    for (int i = 1; i <= L; ++i) {
        for (size_t j = 0; j < lineStations[i].size(); ++j) {
            int sid = lineStations[i][j];
            stationLines[sid].push_back(i);
        }
    }

    vector< vector<char> > inLine(L + 1, vector<char>(S, 0));
    for (int i = 1; i <= L; ++i) {
        for (size_t j = 0; j < lineStations[i].size(); ++j) {
            inLine[i][lineStations[i][j]] = 1;
        }
    }

    vector< vector<int> > posInLine(L + 1, vector<int>(S, -1));
    for (int i = 1; i <= L; ++i) {
        for (size_t j = 0; j < lineStations[i].size(); ++j) {
            posInLine[i][lineStations[i][j]] = (int)j;
        }
    }

    int Q;
    cin >> Q;
    for (int qi = 0; qi < Q; ++qi) {
        int type;
        string startName, endName;
        cin >> type >> startName >> endName;

        if (stationId.find(startName) == stationId.end() || stationId.find(endName) == stationId.end()) {
            cout << "No route\n";
            continue;
        }

        int startSid = stationId[startName];
        int endSid = stationId[endName];

        if (stationLines[startSid].empty() || stationLines[endSid].empty()) {
            cout << "No route\n";
            continue;
        }

        if (type == 1) {
            clock_t st = clock();
            solveMinTransfers(L, lineStations, stationLines, inLine, startSid, endSid, nameOf);
            clock_t ed = clock();
            cout << "Elapsed(ms): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        } else {
            clock_t st = clock();
            solveWeighted(L, lineStations, stationLines, posInLine, startSid, endSid, nameOf);
            clock_t ed = clock();
            cout << "Elapsed(ms): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        }
    }

    return 0;
}
