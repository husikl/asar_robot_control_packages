#ifndef KORTEX_ARM_H
#define KORTEX_ARM

#include <kortex_api/common/KDetailedException.h>

#include <kortex_api/client_stubs/BaseClientRpc.h>
#include <kortex_api/client_stubs/BaseCyclicClientRpc.h>
#include <kortex_api/client_stubs/SessionClientRpc.h>
#include <kortex_api/client/SessionManager.h>

#include <kortex_api/client/RouterClient.h>

#include <kortex_api/client/TransportClientUdp.h>
#include <kortex_api/client/TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>
#include <time.h>

namespace k_api = Kinova::Api;

struct ArmArgs
{
  ArmArgs()
      : ip_address("192.168.1.10"), username("admin"), password("admin"), port(10000),
        port_real_time(10001)
  {
  }
  std::string ip_address;
  std::string username;
  std::string password;
  const int port;
  const int port_real_time;
};

class KortexArm
{
  const std::chrono::duration<int64_t> ACTION_WAITING_TIME = std::chrono::seconds(1);

private:
  ArmArgs arm_args_;
  k_api::TransportClientTcp *transport_;
  k_api::TransportClientUdp *transport_real_time_;
  k_api::RouterClient *router_;
  k_api::RouterClient *router_real_time_;
  k_api::SessionManager *session_manager_;
  k_api::SessionManager *session_manager_real_time_;
  k_api::Base::BaseClient *base_;
  k_api::BaseCyclic::BaseCyclicClient *base_cyclic_;
  std::vector<double> joint_base_;
  k_api::BaseCyclic::Command base_command_;
  k_api::BaseCyclic::Feedback base_feedback_;
  std::vector<float> commands_;

  int n_actuators_;

public:
  KortexArm(ArmArgs _arm_args);
  ~KortexArm();
  int connect();
  int disconnect();
  void moveToHome(k_api::Base::BaseClient *base);

  std::function<void(k_api::Base::ActionNotification)> checkEndOrAbort(bool &finished);

  int lowLevelServoModeOn();
  int lowLevelServoModeOff();

  int getJointsPosition(std::vector<double> &joint_pos);
  int setJointsPosition(const std::vector<float> joint_pos_cmd);
  int setJointsPosition(const VectorXd joint_pos_cmd);

  int setJointsPositionWithFeedback(const std::vector<double> joint_pos_cmd,
                                    std::vector<double> &joint_pos);
  int initSetJointsPosition();
};

KortexArm::KortexArm(ArmArgs _arm_args) : arm_args_(_arm_args) {}

KortexArm::~KortexArm()
{
  // Destroy the API
  delete base_;
  delete base_cyclic_;
  delete session_manager_;
  delete session_manager_real_time_;
  delete router_;
  delete router_real_time_;
  delete transport_;
  delete transport_real_time_;
}

int KortexArm::connect()
{
  // Create API objects
  auto error_callback = [](k_api::KError err)
  {
    cout << "_________ callback error _________" << err.toString();
  };

  transport_ = new k_api::TransportClientTcp();
  router_ = new k_api::RouterClient(transport_, error_callback);
  transport_->connect(arm_args_.ip_address, arm_args_.port);

  transport_real_time_ = new k_api::TransportClientUdp();
  router_real_time_ = new k_api::RouterClient(transport_real_time_, error_callback);
  transport_real_time_->connect(arm_args_.ip_address, arm_args_.port_real_time);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(arm_args_.username);
  create_session_info.set_password(arm_args_.password);
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  std::cout << "Creating sessions for communication" << std::endl;
  session_manager_ = new k_api::SessionManager(router_);
  session_manager_->CreateSession(create_session_info);
  session_manager_real_time_ = new k_api::SessionManager(router_real_time_);
  session_manager_real_time_->CreateSession(create_session_info);
  std::cout << "Sessions created" << std::endl;

  // Create services
  base_ = new k_api::Base::BaseClient(router_);
  base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_real_time_);

  n_actuators_ = base_->GetActuatorCount().count();

  std::cout << "Kortex Arm conecction established successfully" << std::endl;
  return 0;
}

int KortexArm::disconnect()
{
  // Close API session
  session_manager_->CloseSession();
  session_manager_real_time_->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_->SetActivationStatus(false);
  transport_->disconnect();
  router_real_time_->SetActivationStatus(false);
  transport_real_time_->disconnect();
  std::cout << "Kortex Arm connection closed" << std::endl;
  return 0;
}

void KortexArm::moveToHome(k_api::Base::BaseClient *base)
{
  // Make sure the arm is in Single Level Servoing before executing an Action
  auto servoingMode = k_api::Base::ServoingModeInformation();
  servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base->SetServoingMode(servoingMode);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move arm to ready position
  std::cout << "Moving the arm to a safe position" << std::endl;
  auto action_type = k_api::Base::RequestedActionType();
  action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
  auto action_list = base->ReadAllActions(action_type);
  auto action_handle = k_api::Base::ActionHandle();
  action_handle.set_identifier(0);
  for (auto action : action_list.action_list())
  {
    if (action.name() == "Home")
    {
      action_handle = action.handle();
    }
  }

  if (action_handle.identifier() == 0)
  {
    std::cout << "Can't reach safe position, exiting" << std::endl;
  }
  else
  {
    bool action_finished = false;
    // Notify of any action topic event
    auto options = k_api::Common::NotificationOptions();
    auto notification_handle =
        base->OnNotificationActionTopic(checkEndOrAbort(action_finished), options);

    base->ExecuteActionFromReference(action_handle);

    while (!action_finished)
    {
      std::this_thread::sleep_for(ACTION_WAITING_TIME);
    }

    base->Unsubscribe(notification_handle);
  }
}

std::function<void(k_api::Base::ActionNotification)>
KortexArm::checkEndOrAbort(bool &finished)
{
  return [&finished](k_api::Base::ActionNotification notification)
  {
    std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event())
              << std::endl;

    // The action is finished when we receive a END or ABORT event
    switch (notification.action_event())
    {
    case k_api::Base::ActionEvent::ACTION_ABORT:
    case k_api::Base::ActionEvent::ACTION_END:
      finished = true;
      break;
    default:
      break;
    }
  };
}

int KortexArm::lowLevelServoModeOn()
{
  try
  {
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);

    base_->SetServoingMode(servoingMode);
  }
  catch (k_api::KDetailedException &ex)
  {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return 1;
  }
  catch (std::runtime_error &ex2)
  {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return 1;
  }
  std::cout << "Kortex in Low level mode " << std::endl;
  return 0;
}

int KortexArm::lowLevelServoModeOff()
{
  auto servoingMode = k_api::Base::ServoingModeInformation();

  // Set back the servoing mode to Single Level Servoing
  servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_->SetServoingMode(servoingMode);

  // Wait for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

int KortexArm::getJointsPosition(std::vector<double> &joint_pos)
{
  // k_api::BaseCyclic::Feedback base_feedback;

  try
  {
    base_feedback_ = base_cyclic_->RefreshFeedback();

    joint_pos.clear();
    for (int i = 0; i < n_actuators_; i++)
    {
      joint_pos.push_back(base_feedback_.actuators(i).position());
    }
  }
  catch (k_api::KDetailedException &ex)
  {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return 1;
  }
  catch (std::runtime_error &ex2)
  {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return 1;
  }

  return 0;
}

int KortexArm::initSetJointsPosition()
{
  // std::vector<float> commands;
  base_feedback_ = base_cyclic_->RefreshFeedback();
  try
  {
    for (int i = 0; i < n_actuators_; i++)
    {
      commands_.push_back(base_feedback_.actuators(i).position());
      base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
    }
  }
  catch (k_api::KDetailedException &ex)
  {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return 1;
  }
  catch (std::runtime_error &ex2)
  {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return 1;
  }
  ROS_INFO_STREAM("Actuators initialized");

  return 0;
}

int KortexArm::setJointsPosition(const std::vector<float> joint_pos_cmd)
{

  // k_api::BaseCyclic::Command base_command;
  auto lambda_fct_callback = [](const Kinova::Api::Error &err,
                                const k_api::BaseCyclic::Feedback data)
  {
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1),
                                                &serialized_data);
    // std::cout << serialized_data << std::endl
    //           << std::endl;
  };
  try
  {
    std::cout << "Setting ";
    for (int i = 0; i < n_actuators_; i++)
    {
      // base_command.add_actuators()->set_position(joint_pos_cmd[i]);
      // commands_[i] = fmod(joint_pos_cmd[i], 360.0f);
      // std::cout << commands_[i] << " " ;
      std::cout << joint_pos_cmd[i] << " ";
      // base_command_.mutable_actuators(i)->set_position(fmod(commands_[i], 360.0f));
      // base_command_.mutable_actuators(i)->set_position(fmod(commands_[i], 360.0f));
      base_command_.mutable_actuators(i)->set_position(joint_pos_cmd[i]);
    }
    std::cout << std::endl;

    base_cyclic_->Refresh_callback(base_command_, lambda_fct_callback, 0);

    // base_cyclic_->RefreshCommand(base_command_,0);
  }
  catch (k_api::KDetailedException &ex)
  {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return 1;
  }
  catch (std::runtime_error &ex2)
  {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return 1;
  }

  return 0;
}

int KortexArm::setJointsPosition(const VectorXd joint_pos_cmd)
{

  // k_api::BaseCyclic::Command base_command;
  auto lambda_fct_callback = [](const Kinova::Api::Error &err,
                                const k_api::BaseCyclic::Feedback data)
  {
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1),
                                                &serialized_data);
    // std::cout << serialized_data << std::endl
    //           << std::endl;
  };
  try
  {
    // std::cout << "Setting ";
    for (int i = 0; i < n_actuators_; i++)
    {
      // base_command.add_actuators()->set_position(joint_pos_cmd[i]);
      // commands_[i] = fmod(joint_pos_cmd[i], 360.0f);
      // std::cout << commands_[i] << " " ;

      // std::cout << joint_pos_cmd[i] << " ";
      // base_command_.mutable_actuators(i)->set_position(fmod(commands_[i], 360.0f));
      // base_command_.mutable_actuators(i)->set_position(fmod(commands_[i], 360.0f));
      base_command_.mutable_actuators(i)->set_position(joint_pos_cmd[i]);
    }
    // std::cout << std::endl;

    base_cyclic_->Refresh_callback(base_command_, lambda_fct_callback, 0);

    // base_cyclic_->RefreshCommand(base_command_,0);
  }
  catch (k_api::KDetailedException &ex)
  {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return 1;
  }
  catch (std::runtime_error &ex2)
  {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return 1;
  }

  return 0;
}

int KortexArm::setJointsPositionWithFeedback(const std::vector<double> joint_pos_cmd,
                                             std::vector<double> &joint_pos)
{
}

#endif