#ifndef PARKING_H
#define PARKING_H

#include <vector>

// 车辆信息：id（编号，从1开始）、arrive（到达时刻）、duration（停放时长）
struct Vehicle {
    int id;
    int arrive;
    int duration;
    Vehicle();
    Vehicle(int i, int a, int d);
};

// 调度结果：每辆车的车位、入场与离场时刻、离场顺序、总距离与耗时
struct ScheduleResult {
    std::vector<int> spotOf;           // spotOf[vid] = 车位索引（r*N + c），未停为 -1
    std::vector<int> parkTimeOf;       // parkTimeOf[vid] = 实际入场时刻
    std::vector<int> endTimeOf;        // endTimeOf[vid] = 实际离场时刻
    std::vector<int> departureOrder;   // 离场顺序（按时刻升序，时刻相同按编号升序）
    long long totalDistance;           // 累计行驶距离（以 r+c 作为距离度量）
    double elapsedMs;                  // 算法运行耗时（毫秒）
    ScheduleResult();
};

// 贪心调度：每次选择距离入口最近的可用车位（r+c 最小），无法入场则等待最近离场事件
ScheduleResult runGreedy(int M, int N, const std::vector<Vehicle>& cars);

// 回溯搜索：枚举所有可能停车位分配，结合下界剪枝以求全局最优
ScheduleResult runBacktracking(int M, int N, const std::vector<Vehicle>& cars);

// 输出在给定时刻的停车场快照（M 行 N 列，显示车编号；空位显示 0）
void printSnapshot(int M, int N, const ScheduleResult& res, int snapshotTime);

// 输出离场顺序（用空格分隔的车编号）
void printDepartureOrder(const std::vector<int>& order);

#endif
