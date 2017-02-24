/*
 * Copyright (c) 2015-2017, Georg Bartels, <georg.bartels@cs.uni-bremen.de>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Institute of Artificial Intelligence, 
 *     University of Bremen nor the names of its contributors may be used 
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iai_naive_kinematics_sim/iai_naive_kinematics_sim.hpp>
#include <sensor_msgs/JointState.h>

template <class T>
T readParam(const ros::NodeHandle& nh, const std::string& param_name)
{
  T param;
  if(!nh.getParam(param_name, param))
    throw std::runtime_error("Could not find parameter with name '" +
        param_name + "' in namespace '" + nh.getNamespace() + "'.");

  return param;
}
   
class SimulatorNode
{
  public:
    SimulatorNode(const ros::NodeHandle& nh): 
      nh_(nh), sim_frequency_(0.0) {}
    ~SimulatorNode() {}

    void init()
    {
      readSimFrequency();

      sim_.init(readUrdf(), readControlledJoints(), readWatchdogPeriod());
      sim_.setSubJointState(readStartConfig());

      sub_ = nh_.subscribe("commands", 1, &SimulatorNode::callback, this,
            ros::TransportHints().tcpNoDelay());
      pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
      ok_ = true;
    }

    void run()
    {
      ros::Rate sim_rate(sim_frequency_);
      ros::Duration period = sim_rate.expectedCycleTime();
      while(ros::ok() && ok())
      {
        sim_.update(ros::Time::now(), period);
        pub_.publish(sim_.getJointState());
        ros::spinOnce();
        sim_rate.sleep();
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double sim_frequency_;
    iai_naive_kinematics_sim::SimulatorVelocityResolved sim_;
    bool ok_;

    void stop()
    {
      ok_ = false;
    }

    bool ok() const
    {
      return ok_;
    }

    void callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      try
      {
        ros::Time now = ros::Time::now();
        sim_.setSubCommand(*msg, now);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("%s", e.what());
        stop();
      }
    }

    urdf::Model readUrdf() const
    {
      std::string urdf_descr_ = readParam<std::string>(nh_, "/robot_description");
      urdf::Model model;
      if(!model.initString(urdf_descr_))
        throw std::runtime_error("Could not parse given robot description.");
      return model;
    }

    void readSimFrequency()
    {
      sim_frequency_ = readParam<double>(nh_, "sim_frequency");
      if(sim_frequency_ <= 0.0)
        throw std::runtime_error("Read a non-positive simulation frequency.");
      ROS_INFO("sim_frequency: %f", sim_frequency_);
    }

    ros::Duration readWatchdogPeriod() const
    {
      double watchdog_period = readParam<double>(nh_, "watchdog_period");
      if(watchdog_period <= 0.0)
        throw std::runtime_error("Read a non-positive watchdog period.");
      ROS_INFO("watchdog_period: %f", watchdog_period);

      return ros::Duration(watchdog_period);
    }

    std::vector<std::string> readControlledJoints() const
    {
      std::vector<std::string> controlled_joints =
        readParam< std::vector<std::string> >(nh_, "controlled_joints");
      std::string out_string;
      for(size_t i =0; i < controlled_joints.size(); ++i)
        out_string += " " + controlled_joints[i];
      ROS_INFO("controlled joints:%s", out_string.c_str());

      return controlled_joints;
    }

    sensor_msgs::JointState readStartConfig() const
    {
      std::map<std::string, double> start_config;
      nh_.getParam("start_config", start_config);
      sensor_msgs::JointState joint_state;
      for(std::map<std::string, double>::const_iterator it=start_config.begin(); 
          it!=start_config.end(); ++it)
      {
        joint_state.name.push_back(it->first);
        joint_state.position.push_back(it->second);
        joint_state.velocity.push_back(0.0);
        joint_state.effort.push_back(0.0);
        ROS_INFO("start config for '%s': %f", it->first.c_str(), it->second);
      }

      return joint_state;
    }

};

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"velocity_resolved_sim");

  SimulatorNode sim(ros::NodeHandle("~"));

  try
  {
    sim.init();
    sim.run();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
