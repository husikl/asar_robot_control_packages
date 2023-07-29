// C++
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
// MC DAQ
#include <ul_lib/utility.h>
#include "uldaq.h"

// USB DAQ
#include <usb1608/usb1608.h>
#include <usb3104/usb3104.h>
#include <usbquad08/usbquad08.h>

#include <mc_daq_ros/daq_cmd.h>

class MCDAQ
{
public:
  //   Constructor
  MCDAQ(ros::NodeHandle &node_handle) : nh_(node_handle)
  {
    // Services
    srv_server_daq_cmd =
        nh_.advertiseService("daq_command", &MCDAQ ::SrvDaqCommandCb, this);

    daq0_.reset(new USB1608(node_handle, "01F92A95"));
    daq1_.reset(new USB3104(node_handle, "01F7E9D2"));

    daq0_->InitAI();
    daq0_->InitDIO(4, 4);
    daq1_->InitAO();

    daq0_->StartScanAI();
  }
  ~MCDAQ()
  {
    ROS_INFO("Closing DAQ ports");
    if (daq0_->IsEnabledAI() or daq0_->IsEnabledDIO())
      daq0_->Quit();
    if (daq1_->IsEnabledAO())
      daq1_->Quit();
    if (daq2_->IsEnabledENC())
      daq2_->Quit();
  }

  bool SrvDaqCommandCb(mc_daq_ros::daq_cmd::Request &req,
                       mc_daq_ros::daq_cmd::Response &res)
  {
    bool result = false;
    ROS_INFO_STREAM("DAQ Command Called with request: " << req.message);
    if (req.message == "reset_enc")
    {
      daq2_->SetZero(req.port);
      return true;
    }
    else if (req.message == "reset_all_enc")
    {
      daq2_->SetAllZero();
      return true;
    }

    return result;
  }

  void control_loop()
  {
    if (daq0_->IsEnabledAI())
    {
      daq0_->UpdateScanStateAI();
      daq0_->PublishStateAI();
    }
    if (daq0_->IsEnabledDIO())
    {
      daq0_->UpdateStateDI();
      daq0_->PublishStateDI();
      daq0_->UpdateStateDO(7, 1);
    }
    if (daq1_->IsEnabledAO())
    {
      daq1_->UpdateStateAO();
      daq1_->PublishStateAO();
    }
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<USB1608> daq0_;
  std::unique_ptr<USB3104> daq1_;
  std::unique_ptr<USBQUAD08> daq2_;
  ros::ServiceServer srv_server_daq_cmd;
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
  ros::init(argc, argv, "mc_daq_node");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  if (!node_handle.getParam("daq_freq", cycle_freq))
    cycle_freq = 1;
  ROS_INFO_STREAM("Acquisition sampling frequency: " << cycle_freq << "hz");
  ros::Rate loop_rate(cycle_freq);

  MCDAQ md(node_handle);

  // Control Loop
  while (!kill_this_process)
  {
    md.control_loop();
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}