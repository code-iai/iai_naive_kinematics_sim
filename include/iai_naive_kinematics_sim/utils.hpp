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

#ifndef IAI_NAIVE_KINEMATICS_SIM_UTILS_HPP
#define IAI_NAIVE_KINEMATICS_SIM_UTILS_HPP

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <exception>
#include <iai_naive_kinematics_sim/watchdog.hpp>

namespace iai_naive_kinematics_sim
{
  inline std::string messageRangeErrorJointState(const std::string field_name, 
      size_t index, size_t field_size)
  {
    return "Asked to set joint " + field_name + " at index " + std::to_string(index) + 
          " of a message of type sensor_msgs::JointState with " + std::to_string(field_size) + 
          " elements.";
  }

  inline void setJointName(sensor_msgs::JointState& state, size_t index, const std::string& name)
  {
    if (index >= state.name.size())
      throw std::range_error(messageRangeErrorJointState("name", index, state.name.size()));
    state.name[index] = name;
  }

  inline void setJointPosition(sensor_msgs::JointState& state, size_t index, double position)
  {
    if (index >= state.position.size())
      throw std::range_error(messageRangeErrorJointState("position", index, state.position.size()));
    state.position[index] = position;
  }

  inline void setJointVelocity(sensor_msgs::JointState& state, size_t index, double velocity)
  {
    if (index >= state.velocity.size())
      throw std::range_error(messageRangeErrorJointState("velocity", index, state.velocity.size()));
    state.velocity[index] = velocity;
  }

  inline void setJointEffort(sensor_msgs::JointState& state, size_t index, double effort)
  {
    if (index >= state.effort.size())
      throw std::range_error(messageRangeErrorJointState("effort", index, state.effort.size()));
    state.effort[index] = effort;
  }

  inline void setJointState(sensor_msgs::JointState& state, size_t index, const std::string& name,
      double position, double velocity, double effort)
  {
    setJointName(state, index, name);
    setJointPosition(state, index, position);
    setJointVelocity(state, index, velocity);
    setJointEffort(state, index, effort);
  }

  inline void pushBackJointState(sensor_msgs::JointState& state, const std::string& name,
      double position, double velocity, double effort)
  {
    state.name.push_back(name);
    state.position.push_back(position);
    state.velocity.push_back(velocity);
    state.effort.push_back(effort);
  }

  inline void clearJointState(sensor_msgs::JointState& state)
  {
    state.name.clear();
    state.position.clear();
    state.velocity.clear();
    state.effort.clear();
  }

  inline void sanityCheckJointState(const sensor_msgs::JointState& state)
  {
    if (state.name.size() != state.position.size())
      throw std::range_error(std::string("State of type sensor_msgs::JointState") +
          " has fields 'name' and 'position' with different sizes: " +
          std::to_string(state.name.size()) + " compared to " +
          std::to_string(state.position.size()) + ".");
    if (state.name.size() != state.velocity.size())
      throw std::range_error(std::string("State of type sensor_msgs::JointState") +
          " has fields 'name' and 'velocity' with different sizes: " +
          std::to_string(state.name.size()) + " compared to " +
          std::to_string(state.velocity.size()) + ".");
  }

  inline bool isMovingJoint(int type)
  {
    return (type == urdf::Joint::REVOLUTE ||
            type == urdf::Joint::CONTINUOUS ||
            type == urdf::Joint::PRISMATIC);
  }

  inline bool modelHasMovableJoint(const urdf::Model& model, const std::string& name)
  {
    urdf::JointConstSharedPtr joint = model.getJoint(name);
    return joint.get() && isMovingJoint(joint->type);
  }

  inline std::map<std::string, size_t> makeJointIndexMap(const std::vector<std::string>& joint_names)
  {
    std::map<std::string, size_t> map;
    for(size_t i=0; i<joint_names.size(); ++i)
      map[joint_names[i]] = i;
    return map;
  }

  inline sensor_msgs::JointState bootstrapJointState(const urdf::Model& model,
      const std::vector<std::string>& joint_names)
  {
    sensor_msgs::JointState state;

    for (size_t i=0; i<joint_names.size(); ++i)
      if (!modelHasMovableJoint(model, joint_names[i]))
        throw std::runtime_error("URDF model has no movable joint with name '" +
            joint_names[i] + "'.");
      else
        pushBackJointState(state, joint_names[i], 0.0, 0.0, 0.0);

    return state;
  }

  inline std::map<std::string, Watchdog> makeWatchdogs(const urdf::Model& model,
      const std::vector<std::string>& controlled_joints, const ros::Duration watchdog_period)
  {
    std::map<std::string, Watchdog> watchdogs;
    for(size_t i=0; i<controlled_joints.size(); ++i)
      if (!modelHasMovableJoint(model, controlled_joints[i]))
        throw std::runtime_error("URDF model has no movable joint with name '" +
            controlled_joints[i] + "'.");
      else
        watchdogs[controlled_joints[i]] = Watchdog(watchdog_period);

    return watchdogs;
  }

  template <class T>
  inline T readParam(const ros::NodeHandle& nh, const std::string& param_name)
  {
    T param;
    if(!nh.getParam(param_name, param))
      throw std::runtime_error("Could not find parameter with name '" +
          param_name + "' in namespace '" + nh.getNamespace() + "'.");
  
    return param;
  }
}

#endif
