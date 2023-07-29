// C

// C++
#include <csignal>

// External
#include <ros/ros.h>

// Internal
#include <usb1608/usb1608.h>

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
  ros::init(argc, argv, "usb1608_stream_node");
  ros::NodeHandle nh;
  signal(SIGINT, SigIntHandler);

  if (!nh.getParam("usb1608_stream_freq", cycle_freq))
    cycle_freq = 100;
  ROS_INFO("USB1608_stream_node: freq = %d Hz", cycle_freq);
  ros::Rate loop_rate(cycle_freq);

  USB1608 daq_ai(nh, "01F92A95");

  daq_ai.InitAI();
  daq_ai.InitDIO(4, 4);
  daq_ai.StartScanAI();

  // Control loop
  while (!kill_this_process)
  {
    daq_ai.UpdateScanStateAI();
    daq_ai.PublishStateAI();
    daq_ai.UpdateStateDI();
    daq_ai.PublishStateDI();
    loop_rate.sleep();
    ros::spinOnce();
  }

  daq_ai.StopScanAI();
  daq_ai.Quit();

  return 0;
}
