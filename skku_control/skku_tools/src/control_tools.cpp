#include <skku_tools/control_tools.h>

#include <cstring>
#include <exception>
#include <fstream>
#include <string>
#include <pthread.h>


// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)

namespace SKKU {

bool hasRealtimeKernel() {
  std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
  bool is_realtime;
  realtime >> is_realtime;
  return is_realtime;
}

bool setCurrentThreadToHighestSchedulerPriority(std::string* error_message) {
  const int thread_priority = sched_get_priority_max(SCHED_FIFO);
  if (thread_priority == -1) {
    if (error_message != nullptr) {
      *error_message =
          "Error: unable to get maximum possible thread priority: "s + std::strerror(errno);
    }
    return false;
  }

  sched_param thread_param{};
  thread_param.sched_priority = thread_priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (error_message != nullptr) {
      *error_message = "DSR: unable to set realtime scheduling: "s + std::strerror(errno);
    }
    return false;
  }
  return true;
}

}  // namespace franka
