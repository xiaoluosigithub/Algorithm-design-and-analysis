#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <ctime>

using namespace std;

struct Vehicle {
    int id;
    int arrive;
    int duration;
    Vehicle() : id(0), arrive(0), duration(0) {}
    Vehicle(int i, int a, int d) : id(i), arrive(a), duration(d) {}
};

struct ActiveCar {
    int id;
    int endTime;
    int spot;
    ActiveCar() : id(0), endTime(0), spot(-1) {}
    ActiveCar(int i, int e, int s) : id(i), endTime(e), spot(s) {}
};

static int spotDist(int r, int c) {
    return r + c;
}

static void releaseByTime(vector<ActiveCar>& active, int t, vector<int>& departureOrder) {
    bool changed = true;
    while (changed) {
        changed = false;
        int bestEnd = numeric_limits<int>::max();
        int bestIdx = -1;
        for (size_t i = 0; i < active.size(); ++i) {
            if (active[i].endTime < bestEnd) {
                bestEnd = active[i].endTime;
                bestIdx = (int)i;
            } else if (active[i].endTime == bestEnd && active[i].id < active[bestIdx].id) {
                bestIdx = (int)i;
            }
        }
        if (bestIdx != -1 && bestEnd <= t) {
            departureOrder.push_back(active[bestIdx].id);
            active.erase(active.begin() + bestIdx);
            changed = true;
        }
    }
}

static int earliestDepartureTime(const vector<ActiveCar>& active) {
    int bestEnd = numeric_limits<int>::max();
    for (size_t i = 0; i < active.size(); ++i) bestEnd = min(bestEnd, active[i].endTime);
    return bestEnd;
}

static int pickGreedySpot(int M, int N, const vector<char>& occupied) {
    int best = -1;
    int bestD = numeric_limits<int>::max();
    for (int r = 0; r < M; ++r) {
        for (int c = 0; c < N; ++c) {
            int idx = r * N + c;
            if (occupied[idx]) continue;
            int d = spotDist(r, c);
            if (d < bestD) {
                bestD = d;
                best = idx;
            } else if (d == bestD && best != -1) {
                int br = best / N, bc = best % N;
                if (r < br || (r == br && c < bc)) best = idx;
            }
        }
    }
    return best;
}

struct ScheduleResult {
    vector<int> spotOf;
    vector<int> parkTimeOf;
    vector<int> endTimeOf;
    vector<int> departureOrder;
    long long totalDistance;
    double elapsedMs;
    ScheduleResult() : totalDistance(0), elapsedMs(0) {}
};

static ScheduleResult runGreedy(int M, int N, const vector<Vehicle>& cars) {
    clock_t st = clock();

    int K = (int)cars.size();
    ScheduleResult res;
    res.spotOf.assign(K + 1, -1);
    res.parkTimeOf.assign(K + 1, -1);
    res.endTimeOf.assign(K + 1, -1);

    vector<char> occupied(M * N, 0);
    vector<int> spotOwner(M * N, 0);
    vector<ActiveCar> active;
    int currentTime = 0;

    for (int i = 0; i < K; ++i) {
        int vid = cars[i].id;
        int t = cars[i].arrive;
        if (t < currentTime) t = currentTime;

        releaseByTime(active, t, res.departureOrder);
        for (int idx = 0; idx < M * N; ++idx) {
            occupied[idx] = 0;
            spotOwner[idx] = 0;
        }
        for (size_t j = 0; j < active.size(); ++j) {
            occupied[active[j].spot] = 1;
            spotOwner[active[j].spot] = active[j].id;
        }

        while (true) {
            int spot = pickGreedySpot(M, N, occupied);
            if (spot != -1) {
                int r = spot / N;
                int c = spot % N;
                int endT = t + cars[i].duration;
                occupied[spot] = 1;
                spotOwner[spot] = vid;
                active.push_back(ActiveCar(vid, endT, spot));
                res.spotOf[vid] = spot;
                res.parkTimeOf[vid] = t;
                res.endTimeOf[vid] = endT;
                res.totalDistance += spotDist(r, c);
                currentTime = t;
                break;
            }
            int nextT = earliestDepartureTime(active);
            t = max(t, nextT);
            releaseByTime(active, t, res.departureOrder);
            for (int idx = 0; idx < M * N; ++idx) occupied[idx] = 0;
            for (size_t j = 0; j < active.size(); ++j) occupied[active[j].spot] = 1;
        }
    }

    releaseByTime(active, numeric_limits<int>::max(), res.departureOrder);

    clock_t ed = clock();
    res.elapsedMs = 1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC;
    return res;
}

struct BTContext {
    int M;
    int N;
    vector<Vehicle> cars;
    vector<int> bestSpotOf;
    vector<int> bestParkTimeOf;
    vector<int> bestEndTimeOf;
    long long bestDistance;
    vector<int> allSpotDistancesSorted;
    clock_t st;
};

static long long lowerBoundRemain(const BTContext& ctx, int remain) {
    long long s = 0;
    for (int i = 0; i < remain && i < (int)ctx.allSpotDistancesSorted.size(); ++i) s += ctx.allSpotDistancesSorted[i];
    return s;
}

static void dfsBacktrack(
    BTContext& ctx,
    int idx,
    int currentTime,
    vector<char>& occupied,
    vector<ActiveCar>& active,
    vector<int>& spotOf,
    vector<int>& parkTimeOf,
    vector<int>& endTimeOf,
    long long curDist
) {
    int K = (int)ctx.cars.size();
    if (curDist >= ctx.bestDistance) return;
    long long lb = curDist + lowerBoundRemain(ctx, K - idx);
    if (lb >= ctx.bestDistance) return;

    if (idx == K) {
        ctx.bestDistance = curDist;
        ctx.bestSpotOf = spotOf;
        ctx.bestParkTimeOf = parkTimeOf;
        ctx.bestEndTimeOf = endTimeOf;
        return;
    }

    int vid = ctx.cars[idx].id;
    int t = ctx.cars[idx].arrive;
    if (t < currentTime) t = currentTime;

    vector<int> tmpDepart;
    releaseByTime(active, t, tmpDepart);
    for (int i = 0; i < ctx.M * ctx.N; ++i) occupied[i] = 0;
    for (size_t j = 0; j < active.size(); ++j) occupied[active[j].spot] = 1;

    if ((int)active.size() == ctx.M * ctx.N) {
        int nextT = earliestDepartureTime(active);
        dfsBacktrack(ctx, idx, max(t, nextT), occupied, active, spotOf, parkTimeOf, endTimeOf, curDist);
        return;
    }

    vector<int> candidates;
    candidates.reserve(ctx.M * ctx.N);
    for (int r = 0; r < ctx.M; ++r) {
        for (int c = 0; c < ctx.N; ++c) {
            int sp = r * ctx.N + c;
            if (!occupied[sp]) candidates.push_back(sp);
        }
    }
    for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            int a = candidates[i], b = candidates[j];
            int da = spotDist(a / ctx.N, a % ctx.N);
            int db = spotDist(b / ctx.N, b % ctx.N);
            if (db < da) {
                int tmp = candidates[i]; candidates[i] = candidates[j]; candidates[j] = tmp;
            }
        }
    }

    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        int sp = candidates[ci];
        int r = sp / ctx.N;
        int c = sp % ctx.N;
        int endT = t + ctx.cars[idx].duration;

        vector<ActiveCar> active2 = active;
        active2.push_back(ActiveCar(vid, endT, sp));
        vector<char> occupied2 = occupied;
        occupied2[sp] = 1;

        spotOf[vid] = sp;
        parkTimeOf[vid] = t;
        endTimeOf[vid] = endT;

        dfsBacktrack(ctx, idx + 1, t, occupied2, active2, spotOf, parkTimeOf, endTimeOf, curDist + spotDist(r, c));

        spotOf[vid] = -1;
        parkTimeOf[vid] = -1;
        endTimeOf[vid] = -1;
    }
}

static ScheduleResult runBacktracking(int M, int N, const vector<Vehicle>& cars) {
    clock_t st = clock();
    int K = (int)cars.size();

    BTContext ctx;
    ctx.M = M;
    ctx.N = N;
    ctx.cars = cars;
    ctx.bestDistance = (long long)numeric_limits<int>::max();
    ctx.bestSpotOf.assign(K + 1, -1);
    ctx.bestParkTimeOf.assign(K + 1, -1);
    ctx.bestEndTimeOf.assign(K + 1, -1);
    ctx.st = st;

    vector<int> allD;
    for (int r = 0; r < M; ++r) for (int c = 0; c < N; ++c) allD.push_back(spotDist(r, c));
    for (size_t i = 0; i < allD.size(); ++i) {
        for (size_t j = i + 1; j < allD.size(); ++j) {
            if (allD[j] < allD[i]) {
                int tmp = allD[i]; allD[i] = allD[j]; allD[j] = tmp;
            }
        }
    }
    ctx.allSpotDistancesSorted = allD;

    vector<char> occupied(M * N, 0);
    vector<ActiveCar> active;
    vector<int> spotOf(K + 1, -1);
    vector<int> parkTimeOf(K + 1, -1);
    vector<int> endTimeOf(K + 1, -1);

    dfsBacktrack(ctx, 0, 0, occupied, active, spotOf, parkTimeOf, endTimeOf, 0);

    ScheduleResult res;
    res.spotOf = ctx.bestSpotOf;
    res.parkTimeOf = ctx.bestParkTimeOf;
    res.endTimeOf = ctx.bestEndTimeOf;
    res.totalDistance = ctx.bestDistance;

    vector< pair<int,int> > ends;
    for (int i = 0; i < K; ++i) {
        int vid = cars[i].id;
        ends.push_back(make_pair(res.endTimeOf[vid], vid));
    }
    for (size_t i = 0; i < ends.size(); ++i) {
        for (size_t j = i + 1; j < ends.size(); ++j) {
            if (ends[j].first < ends[i].first || (ends[j].first == ends[i].first && ends[j].second < ends[i].second)) {
                pair<int,int> tmp = ends[i]; ends[i] = ends[j]; ends[j] = tmp;
            }
        }
    }
    for (size_t i = 0; i < ends.size(); ++i) res.departureOrder.push_back(ends[i].second);

    clock_t ed = clock();
    res.elapsedMs = 1000.0 * (double)(ed - st) / (double)CLOCKS_PER_SEC;
    return res;
}

static void printSnapshot(int M, int N, const ScheduleResult& res, int snapshotTime) {
    vector<int> grid(M * N, 0);
    int K = (int)res.spotOf.size() - 1;
    for (int vid = 1; vid <= K; ++vid) {
        int sp = res.spotOf[vid];
        int pt = res.parkTimeOf[vid];
        int et = res.endTimeOf[vid];
        if (sp == -1 || pt == -1 || et == -1) continue;
        if (pt <= snapshotTime && snapshotTime < et) grid[sp] = vid;
    }
    for (int r = 0; r < M; ++r) {
        for (int c = 0; c < N; ++c) {
            if (c) cout << " ";
            cout << grid[r * N + c];
        }
        cout << "\n";
    }
}

static void printDepartureOrder(const vector<int>& order) {
    for (size_t i = 0; i < order.size(); ++i) {
        if (i) cout << " ";
        cout << order[i];
    }
    cout << "\n";
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);

    int M, N;
    if (!(cin >> M >> N)) {
        cout << "Input format:\n";
        cout << "M N\n";
        cout << "K\n";
        cout << "K lines: arrive duration\n";
        cout << "snapshotTime\n";
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
    cout << "[Greedy]\n";
    cout << "Total distance: " << greedy.totalDistance << "\n";
    cout << "Elapsed(ms): " << greedy.elapsedMs << "\n";
    cout << "Snapshot at t=" << snapshotTime << "\n";
    printSnapshot(M, N, greedy, snapshotTime);
    cout << "Departure order:\n";
    printDepartureOrder(greedy.departureOrder);

    ScheduleResult best;
    if (K <= 12) {
        best = runBacktracking(M, N, cars);
        cout << "[Backtracking]\n";
        cout << "Total distance: " << best.totalDistance << "\n";
        cout << "Elapsed(ms): " << best.elapsedMs << "\n";
        cout << "Snapshot at t=" << snapshotTime << "\n";
        printSnapshot(M, N, best, snapshotTime);
        cout << "Departure order:\n";
        printDepartureOrder(best.departureOrder);
    } else {
        cout << "[Backtracking]\n";
        cout << "Skipped (K>12, exponential time)\n";
    }

    return 0;
}
