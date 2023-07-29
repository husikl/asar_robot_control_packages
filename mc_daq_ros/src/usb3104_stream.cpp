// C

// C++
#include <csignal>

// External
#include <ros/ros.h>

// Internal
#include <usb3104/usb3104.h>

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
  ros::init(argc, argv, "usb3104_stream_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigIntHandler);

  if (!nh.getParam("usb3104_stream_freq", cycle_freq))
    cycle_freq = 100;
  ROS_INFO("USB3104_stream_node: freq = %d Hz", cycle_freq);
  ros::Rate loop_rate(cycle_freq);

  USB3104 daq_ao(nh, "01F7E9D2");

  daq_ao.InitAO();

  // Control loop
  while (!kill_this_process)
  {
    // daq_ao.UpdateStateAO();
    daq_ao.PublishStateAO();

    loop_rate.sleep();
    ros::spinOnce();
  }

  daq_ao.Quit();

  return 0;
}
