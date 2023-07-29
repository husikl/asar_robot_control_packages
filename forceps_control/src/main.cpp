#include <forceps_control/forceps_control.h>

// C
#include <signal.h>

// C++
#include <thread>

using namespace forceps_nu;

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_WARN_STREAM("SHUTDOWN SIGNAL RECEIVED");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // Ros related
  ros::init(argc, argv, "forceps_control");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  ForcepsControl fc(node_handle, &kill_this_process);

  //   RealtimeSchedulerInterface::activateRRScheduler(99);
  //   RealtimeSchedulerInterface::display_thread_sched_attr(
  //       "Trying to upgrade to real-time SCHED_DEADLINE scheduler...");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  std::thread t(std::bind(&ForcepsControl::ControlLoop, &fc));

  t.join();
  spinner.stop();

  return 0;
}
