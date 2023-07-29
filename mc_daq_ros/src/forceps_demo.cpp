
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
  const int k_PULSES_PER_TURN = 4096;

public:
  //   Constructor
  MCDAQ(ros::NodeHandle &node_handle) : nh_(node_handle)
  {
    bool daq_di_enable = false;
    bool daq_do_enable = false;
    bool daq_ai_enable = false;
    bool daq_ao_enable = false;
    bool daq_enc_enable = false;

    // Services
    srv_server_daq_cmd =
        nh_.advertiseService("daq_command", &MCDAQ ::SrvDaqCommandCb, this);

    node_handle.getParam("daq_di", daq_di_enable);
    node_handle.getParam("daq_do", daq_do_enable);
    node_handle.getParam("daq_ai", daq_ai_enable);
    node_handle.getParam("daq_ao", daq_ao_enable);
    node_handle.getParam("daq_enc", daq_enc_enable);

    daq0_.reset(new USB1608(node_handle, "01F92A95"));
    daq1_.reset(new USB3104(node_handle, "01F7E9D2"));
    daq2_.reset(new USBQUAD08(node_handle, "1001633"));

    daq0_->InitAI();
    daq0_->InitDIO(4, 4);
    daq1_->InitAO();
    daq2_->InitENC();
    state_ = 0;

    enc_pitch_ = 0;
    enc_yaw_1_ = 0;
    enc_yaw_2_ = 0;

    v_pitch_ = 0.0;
    v_yaw_1_ = 0.0;
    v_yaw_2_ = 0.0;
    pitch_state_ = 0;
    yaw_1_state_ = 0;
    yaw_2_state_ = 0;
  }
  ~MCDAQ()
  {
    v_pitch_ = 0.0;
    v_yaw_1_ = 0.0;
    v_yaw_2_ = 0.0;

    std::vector<double> ao_cmd = {v_yaw_1_, 0.0, v_yaw_2_, 0.0,
                                  v_pitch_, 0.0, 0.0, 0.0};
    daq1_->set_ao_cmd(ao_cmd);
    daq1_->UpdateStateAO();
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
      daq0_->UpdateStateAI();
      daq0_->PublishStateAI();
    }
    if (daq0_->IsEnabledDIO())
    {
      daq0_->UpdateStateDI();
      daq0_->PublishStateDI();

      // daq0.UpdateStateDO(7, 1);
    }
    if (daq1_->IsEnabledAO())
    {
      daq1_->PublishStateAO();
    }
    if (daq2_->IsEnabledENC())
    {
      daq2_->UpdateStateENC();
      daq2_->PublishStateENC();
    }
    if (state_ == 0)
    {
      daq2_->SetAllZero();
      state_ = 1;
    }
    if (state_ == 1)
    {
      enc_state_ = daq2_->GetEncState();
      enc_yaw_1_ = enc_state_[0];
      enc_yaw_2_ = enc_state_[2];
      enc_pitch_ = enc_state_[4];

      if (enc_pitch_ <= 4096)
      {
        v_pitch_ = -10.0;
      }
      else
      {
        v_pitch_ = 0.0;
        pitch_state_ = 1;
      }

      if (enc_yaw_1_ <= 256)
      {
        v_yaw_1_ = -10.0;
      }
      else
      {
        v_yaw_1_ = 0.0;
        yaw_1_state_ = 1;
      }

      if (enc_yaw_2_ >= -200)
      {
        v_yaw_2_ = 10.0;
      }
      else
      {
        v_yaw_2_ = 0.0;
        yaw_2_state_ = 1;
      }

      std::vector<double> ao_cmd = {v_yaw_1_, 0.0, v_yaw_2_, 0.0,
                                    v_pitch_, 0.0, 0.0, 0.0};
      daq1_->set_ao_cmd(ao_cmd);
      daq1_->UpdateStateAO();

      if (pitch_state_ == 1 && yaw_1_state_ == 1 && yaw_2_state_ == 1)
      {
        state_ = 2;
        v_pitch_ = 0.0;
        v_yaw_1_ = 0.0;
        v_yaw_2_ = 0.0;
      }
    }

    if (state_ == 2)
    {
      enc_state_ = daq2_->GetEncState();
      // ai_state_ = daq0_->GetAiState();

      ss_yaw_1_ = ai_state_[0];
      ss_yaw_2_ = ai_state_[1];
      ss_pitch_ = ai_state_[2];

      enc_yaw_1_ = enc_state_[0];
      enc_yaw_2_ = enc_state_[2];
      enc_pitch_ = enc_state_[4];

      int dir_coeff = 1;

      if (enc_pitch_ <= -k_PULSES_PER_TURN / 2)
      {
        dir_coeff = 1;
        ROS_INFO("Should increase");
      }
      else if (enc_pitch_ >= k_PULSES_PER_TURN / 2)
      {
        dir_coeff = -1;
        ROS_INFO("Should decrease");
        // pitch_state_ = 0;
      }

      if (ss_pitch_ < 0.15)
        v_pitch_ = -0.5 * (dir_coeff * k_PULSES_PER_TURN / 2 - enc_pitch_);
      else
        v_pitch_ = 0.0;

      if (v_pitch_ >= 10)
        v_pitch_ = 10.0;
      if (v_pitch_ <= -10)
        v_pitch_ = -10.0;

      std::vector<double> ao_cmd = {v_yaw_1_, 0.0, v_yaw_2_, 0.0,
                                    v_pitch_, 0.0, 0.0, 0.0};
      daq1_->set_ao_cmd(ao_cmd);
      daq1_->UpdateStateAO();

      if (pitch_state_ == 0 && yaw_1_state_ == 0 && yaw_2_state_ == 0)
      {
        state_ = 1;
        v_pitch_ = 0.0;
        v_yaw_1_ = 0.0;
        v_yaw_2_ = 0.0;
      }
    }
    ROS_INFO_STREAM("State: " << state_);
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<USB1608> daq0_;
  std::unique_ptr<USB3104> daq1_;
  std::unique_ptr<USBQUAD08> daq2_;
  ros::ServiceServer srv_server_daq_cmd;
  int state_;
  int enc_pitch_;
  int enc_yaw_1_;
  int enc_yaw_2_;
  double v_pitch_;
  double v_yaw_1_;
  double v_yaw_2_;

  double ss_pitch_;
  double ss_yaw_1_;
  double ss_yaw_2_;

  int pitch_state_;
  int yaw_1_state_;
  int yaw_2_state_;
  std::vector<int> enc_state_;
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