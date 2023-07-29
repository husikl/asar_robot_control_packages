// C

// C++
#include <csignal>

// External
#include <ros/ros.h>

// Internal
#include <usbquad08/usbquad08.h>

/**
 * Main Function
 */

bool kill_this_process = false;

void SigIntHandler(int sig)
{
  kill_this_process = true;
  ROS_INFO("Shutting down");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  int cycle_freq;

  // ROS Initialization
  ros::init(argc, argv, "usbquad08_stream_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigIntHandler);

  if (!nh.getParam("usbquad08_stream_freq", cycle_freq))
    cycle_freq = 100;
  ROS_INFO("USBQUAD08_stream_node: freq = %d Hz", cycle_freq);
  ros::Rate loop_rate(cycle_freq);

  USBQUAD08 daq_enc(nh, "1001633");

  daq_enc.InitENC();
  // daq_enc.set_offset(5000);
  daq_enc.StartScanENC();

  // Control loop
  while (!kill_this_process)
  {
    // daq_enc.UpdateStateENC();
    daq_enc.UpdateScanStateENC();
    daq_enc.PublishStateENC();

    loop_rate.sleep();
    ros::spinOnce();
  }

  daq_enc.Quit();

  return 0;
}
