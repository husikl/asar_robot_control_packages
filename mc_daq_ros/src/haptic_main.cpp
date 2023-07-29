
// C++
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// External
// ROS
#include <ros/ros.h>
// MC DAQ
#include <ul_lib/utility.h>
#include "uldaq.h"

// Internal
// USB DAQ
// #include <mc_daq_ros/daq_cmd.h>
#include <usb1608/usb1608.h>
#include <usb3104/usb3104.h>
#include <usbquad08/usbquad08.h>

class HAPTICDAQ
{
  const int k_PULSES_PER_TURN = 4096;

public:
  //   Constructor
  HAPTICDAQ(ros::NodeHandle &node_handle) : nh_(node_handle)
  {

    bool daq_ai_enable = false;
    node_handle.getParam("daq_ai", daq_ai_enable);

    daq0_.reset(new USB1608(node_handle, "01F92A95"));

    daq0_->InitAI();
    state_ = 0;

    daq0_->StartScanAI();
  }
  ~HAPTICDAQ()
  {
    ROS_INFO("Closing DAQ ports");
    if (daq0_->IsEnabledAI() or daq0_->IsEnabledDIO())
    {
      daq0_->StopScanAI();
      daq0_->Quit();
    }
  }

  void control_loop()
  {
    if (daq0_->IsEnabledAI())
    {
      daq0_->UpdateScanStateAI();
      daq0_->PublishStateAI();
    }
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<USB1608> daq0_;
  int state_;

  std::vector<double> ai_state_;
};

/**
 * Main function:
 */

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

/**
 * Main function:
 */
int main(int argc, char **argv)
{
  int cycle_freq;

  // Initialize ROS
  ros::init(argc, argv, "haptic_main_node");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  if (!node_handle.getParam("daq_freq", cycle_freq))
    cycle_freq = 1;
  ROS_INFO_STREAM("Acquisition sampling frequency: " << cycle_freq << "hz");
  ros::Rate loop_rate(cycle_freq);

  HAPTICDAQ hd(node_handle);

  // Control Loop
  while (!kill_this_process)
  {
    hd.control_loop();
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}