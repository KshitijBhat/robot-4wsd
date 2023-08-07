// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

#include <four_wheel_steering.h>
#include <chrono>
#include <thread>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_wheel_steering");
  ros::NodeHandle nh;

  // This should be set in launch files as well
  nh.setParam("/use_sim_time", true);

  FourWheelSteering robot;

  nh.getParam("rover/hardware/serial_port_fl", robot.SERIAL_PORTS[0]);
  std::cout << "FL Serial Port:" << robot.SERIAL_PORTS[0]<< std::endl;

  nh.getParam("rover/hardware/serial_port_fr", robot.SERIAL_PORTS[1]);
  std::cout << "FR Serial Port:" << robot.SERIAL_PORTS[1]<< std::endl;

  nh.getParam("rover/hardware/serial_port_rl", robot.SERIAL_PORTS[2]);
  std::cout << "RL Serial Port:" << robot.SERIAL_PORTS[2]<< std::endl;

  nh.getParam("rover/hardware/serial_port_rr", robot.SERIAL_PORTS[3]);
  std::cout << "RR Serial Port:" << robot.SERIAL_PORTS[3]<< std::endl;
  
  nh.getParam("rover/hardware/pid_gains_steering", robot.pid_gains_steering);
  nh.getParam("rover/hardware/pid_gains_driving", robot.pid_gains_driving);
  
  // Set PID values
  for (unsigned int i = 0; i < 4; ++i)
  {
      robot.controllers[i].setPID(robot.pid_gains_steering);
      robot.controllers[i].setvPID(robot.pid_gains_driving);
  }
  
  std::cout<< "P    "<< robot.pid_gains_steering[0]<< "     "<<robot.pid_gains_driving[0]<<std::endl;
  std::cout<< "I    "<< robot.pid_gains_steering[1]<< "     "<<robot.pid_gains_driving[1]<<std::endl;
  std::cout<< "D    "<< robot.pid_gains_steering[2]<< "     "<<robot.pid_gains_driving[2]<<std::endl;

  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point end   = std::chrono::system_clock::now();

  ros::Time internal_time(0);
  const ros::Duration dt = robot.getPeriod();
  double elapsed_secs = 0;

  robot.start_hardware();

  while(ros::ok())
  {
    begin = std::chrono::system_clock::now();

    robot.read();
    cm.update(internal_time, dt);
    robot.write();

    end = std::chrono::system_clock::now();

    elapsed_secs = std::chrono::duration_cast<std::chrono::duration<double> >((end - begin)).count();

    if (dt.toSec() - elapsed_secs < 0.0)
    {
      ROS_WARN_STREAM_THROTTLE(
            0.1, "Control cycle is taking to much time, elapsed: " << elapsed_secs);
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Control cycle is, elapsed: " << elapsed_secs);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt.toSec() - elapsed_secs));
    }

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);
    internal_time += dt;
  }
  spinner.stop();
  robot.kill_all();
  robot.kill_all();
  robot.kill_all();

  return 0;
}
