
#include "k1_control/k1_executor.h"

namespace k1_control {
using namespace std::chrono_literals;

Executor::Executor() : executor_spin_([this] { run(); }) {
  while (!this->spinning) {
    // Need to wait until the executor starts spinning
    std::this_thread::sleep_for(100ms);
  }
}

Executor::~Executor() {
  // if the executor still spinning cancel it
  this->shutdown();
  executor_spin_.join();
}

void Executor::run() {
  // spin the executor
  spin();
}

void Executor::shutdown() {
  if (this->spinning) {
    this->cancel();
  }
}

}
