#ifndef TIME_COST_H
#define TIME_COST_H

#include <chrono>
#include <cstdlib>
#include <ctime>

class TimeCost {
 public:
  TimeCost() { time_start(); }

  void time_start() { start = std::chrono::system_clock::now(); }

  double time_end() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> cost_seconds = end - start;
    return cost_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif  // TIME_COST_H
