// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

#pragma once


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// Hardware
#include "pico_comm.hpp"

// NaN
#include <limits>

// ostringstream
#include <sstream>

class FourWheelSteering : public hardware_interface::RobotHW
{
public:
  FourWheelSteering()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &FourWheelSteering::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &FourWheelSteering::stop_callback, this))
  {
    std::vector<std::string> velocity_joints_name = {"ffl_wheel_joint", "ffr_wheel_joint",
                                                     "rfl_wheel_joint", "rfr_wheel_joint"};
    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < velocity_joints_name.size(); ++i)
    {

      hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(state_handle, &joints_[i].velocity_command);
      jnt_vel_interface_.registerHandle(vel_handle);
    }

    std::vector<std::string> position_joints_name = {"front_left_steering_joint", "front_right_steering_joint",
                                                     "rear_left_steering_joint", "rear_right_steering_joint"};
    // Connect and register the joint state and position interface
    for (unsigned int i = 0; i < position_joints_name.size(); ++i)
    {
      hardware_interface::JointStateHandle state_handle(position_joints_name[i], &steering_joints_[i].position, &steering_joints_[i].velocity, &steering_joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(state_handle, &steering_joints_[i].position_command);
      jnt_pos_interface_.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void start_hardware()
  {

    // Start Hardware
    pico_fl.connect(SERIAL_PORT_FL);
    // pico_fr.connect();
    // pico_rl.connect();
    // pico_rr.connect();
  }

  void read()
  {
    // Read the joint state of the robot into the hardware interface
    if (running_)
    {
      for (unsigned int i = 0; i < 3; ++i)
      {
        // Note that joints_[i].position will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        joints_[i].position += joints_[i].velocity*getPeriod().toSec(); // update position
        joints_[i].velocity = joints_[i].velocity_command; // might add smoothing here later
      }
      for (unsigned int i = 0; i < 3; ++i)
      {
        steering_joints_[i].position = steering_joints_[i].position_command; // might add smoothing here later
      }

      for (unsigned int i = 0; i < 1; ++i)
      {
      // Real hardware
      float steering_position, driving_position, driving_velocity;
      // pico_fl.readEncoder(steering_position, driving_position);
      // pico_fl.readDrivingEncoderVelocity(driving_velocity);

      pico_fl.readSteeringDriving(steering_position, driving_position, driving_velocity);

      joints_[i].position = driving_position; // update position
      joints_[i].velocity = driving_velocity;
      steering_joints_[i].position = steering_position;

      ROS_INFO("Read encoders: %.2f, %.2f", steering_joints_[i].position, joints_[i].velocity);
      }
    }
    else
    {
      for (unsigned int i = 0; i < 4; ++i)
      {
        joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
      for (unsigned int i = 0; i < 4; ++i)
      {
        steering_joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        steering_joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  void write()
  {
    for (unsigned int i = 0; i < 1; ++i)
      {
      // Real hardware
      ROS_INFO("Target: %.2f , %.2f",steering_joints_[i].position_command, joints_[i].velocity_command);
      pico_fl.controlLeg(steering_joints_[i].position_command, joints_[i].velocity_command);
      }

  }

    void kill_all()
  {
    for (unsigned int i = 0; i < 1; ++i)
      {
      // Real hardware
      ROS_INFO("Sending: %.2f , %.2f",0.0, 0.0);
      pico_fl.writeMotor(0, 0);
      }
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  //Hardware
  std::string SERIAL_PORT_FL;
  std::string SERIAL_PORT_FR;
  std::string SERIAL_PORT_RL;
  std::string SERIAL_PORT_RR;

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
  } joints_[4];

  struct SteeringJoint
  {
    double position;
    double velocity;
    double effort;
    double position_command;

    SteeringJoint() : position(0), velocity(0), effort(0), position_command(0) { }
  } steering_joints_[4];
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  PicoComms pico_fl;
  PicoComms pico_fr;
  PicoComms pico_rl;
  PicoComms pico_rr;

};
