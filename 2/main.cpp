#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <limits>
#include <ctime>
#include "bus.h"
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

static int getStationId(map<string, int>& idOf, vector<string>& nameOf, const string& name) {
    map<string, int>::iterator it = idOf.find(name);
    if (it != idOf.end()) return it->second;
    int id = (int)nameOf.size();
    idOf[name] = id;
    nameOf.push_back(name);
    return id;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);
    #ifdef _WIN32
    SetConsoleOutputCP(65001);
    #endif

    int L;
    if (!(cin >> L)) {
        cout << "输入格式:\n";
        cout << "L\n";
        cout << "每条线路：k 站点1 ... 站点k\n";
        cout << "Q\n";
        cout << "Q 行查询: type 起点站 终点站  (type=1 最少换乘, type=2 综合最优)\n";
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
            cout << "无可达路线\n";
            continue;
        }

        int startSid = stationId[startName];
        int endSid = stationId[endName];

        if (stationLines[startSid].empty() || stationLines[endSid].empty()) {
            cout << "无可达路线\n";
            continue;
        }

        if (type == 1) {
            clock_t st = clock();
            solveMinTransfers(L, lineStations, stationLines, inLine, startSid, endSid, nameOf);
            clock_t ed = clock();
            cout << "耗时(毫秒): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        } else {
            clock_t st = clock();
            solveWeighted(L, lineStations, stationLines, posInLine, startSid, endSid, nameOf);
            clock_t ed = clock();
            cout << "耗时(毫秒): " << (1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC) << "\n";
        }
    }

    return 0;
}
