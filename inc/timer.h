#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class timer {
public:
  timer() : start_time_() {}

  void start() { start_time_ = std::chrono::high_resolution_clock::now(); }

  double elapsed_time_nano() {
    return elapsed_time<std::chrono::nanoseconds>();
  }
  double elapsed_time_micro() {
    return elapsed_time<std::chrono::microseconds>();
  }
  double elapsed_time_sec() { return elapsed_time<std::chrono::seconds>(); }

  double get_time_nano() { return get_time<std::chrono::nanoseconds>(); }
  double get_time_micro() { return get_time<std::chrono::microseconds>(); }
  double get_time_sec() { return get_time<std::chrono::seconds>(); }

private:
  std::chrono ::time_point<std::chrono::high_resolution_clock> start_time_;

  template <class Units> inline double elapsed_time() {
    return std::chrono::duration_cast<Units>(
               std::chrono::high_resolution_clock::now() - start_time_)
        .count();
  }

  template <class Units> inline double get_time() {
    return std::chrono::time_point_cast<Units>(
               std::chrono::high_resolution_clock::now())
        .time_since_epoch()
        .count();
  }
};

#endif // TIMER_H
