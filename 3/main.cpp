#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <ctime>
#include "parking.h"
#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

// 程序入口：读取输入，分别执行贪心与回溯调度，并输出结果
int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

#ifdef _WIN32
    // 设置控制台使用 UTF-8，避免中文乱码
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
#endif

    int M, N;
    if (!(cin >> M >> N)) {
        cout << "输入格式：\n";
        cout << "M N\n";
        cout << "K\n";
        cout << "K 行：到达时间 持续时间\n";
        cout << "快照时刻（snapshotTime）\n";
        return 0;
    }
    int K;
    cin >> K;
    vector<Vehicle> cars;
    cars.reserve(K);
    for (int i = 1; i <= K; ++i) {
        int a, d;
        cin >> a >> d;
        cars.push_back(Vehicle(i, a, d));
    }
    int snapshotTime;
    cin >> snapshotTime;

    ScheduleResult greedy = runGreedy(M, N, cars);
    cout << "[贪心算法]\n";
    cout << "总行驶距离：" << greedy.totalDistance << "\n";
    cout << "耗时(毫秒)：" << greedy.elapsedMs << "\n";
    cout << "时刻快照 t=" << snapshotTime << "\n";
    printSnapshot(M, N, greedy, snapshotTime);
    cout << "离场顺序：\n";
    printDepartureOrder(greedy.departureOrder);

    ScheduleResult best;
    if (K <= 12) {
        best = runBacktracking(M, N, cars);
        cout << "[回溯搜索]\n";
        cout << "总行驶距离：" << best.totalDistance << "\n";
        cout << "耗时(毫秒)：" << best.elapsedMs << "\n";
        cout << "时刻快照 t=" << snapshotTime << "\n";
        printSnapshot(M, N, best, snapshotTime);
        cout << "离场顺序：\n";
        printDepartureOrder(best.departureOrder);
    } else {
        cout << "[回溯搜索]\n";
        cout << "已跳过（K>12，回溯耗时指数级）\n";
    }

    return 0;
}
