#ifndef REALTIME_SETTING_HPP
#define REALTIME_SETTING_HPP

#include <linux/sched.h>
#include <linux/sched/types.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
// #include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

extern long syscall(long number, ...);

inline long sched_setattr(pid_t pid, const struct sched_attr* attr,
                          unsigned int flags) {
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

inline bool setProcessHighPriority(unsigned int priority) {
  /* Get high priority */
  struct sched_attr attr;
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_RR;
  attr.sched_priority = priority;
  if (sched_setattr(0, &attr, 0) < 0) {
    printf("sched_setattr failed\n");
    return false;
  }
  return true;
}

inline bool StickThisThreadToCore(int core_id) {
  int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
  if (core_id < 0 || core_id >= num_cores) {
    printf("invalid core_id\n");
    return false;
  }

  size_t size;
  cpu_set_t* cpuset;

  cpuset = CPU_ALLOC(1);
  size = CPU_ALLOC_SIZE(1);

  CPU_ZERO_S(size, cpuset);
  CPU_SET_S(core_id, size, cpuset);

  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, size, cpuset) != 0) return false;

  return true;
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // !REALTIME_SETTING_HPP
