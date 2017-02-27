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

#ifndef IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SIMULATOR_HPP

#include <iai_naive_kinematics_sim/utils.hpp>
#include <iai_naive_kinematics_sim/watchdog.hpp>

namespace iai_naive_kinematics_sim
{
  class Simulator
  {
    public:
      Simulator() {}

      ~Simulator() {}

      void init(const urdf::Model& model, const std::vector<std::string>& controlled_joints,
          const ros::Duration& watchdog_period)
      {
        model_ = model;
        state_ = bootstrapJointState(model);
        command_ = state_;
        index_map_ = makeJointIndexMap(state_.name);
        watchdogs_ = makeWatchdogs(model, controlled_joints, watchdog_period);
      }

      size_t size() const
      {
        return index_map_.size();
      }

      void update(const ros::Time& now, const ros::Duration& dt)
      {
        if (dt.toSec() <= 0)
          throw std::runtime_error("Time interval given to update function not bigger than 0.");
        // FIXME: consider dropping these checks to increase speed
        if (state_.name.size() != state_.position.size())
          throw std::range_error(std::string("Internal state of type sensor_msgs::JointState") +
              " has fields 'name' and 'position' with different sizes: " +
              std::to_string(state_.name.size()) + " compared to " +
              std::to_string(state_.position.size()) + ".");
        if (state_.name.size() != state_.velocity.size())
          throw std::range_error(std::string("Internal state of type sensor_msgs::JointState") +
              " has fields 'name' and 'velocity' with different sizes: " +
              std::to_string(state_.name.size()) + " compared to " +
              std::to_string(state_.velocity.size()) + ".");

        // ask the watchdogs, and stop joints that have not received a new command in a while
        for (std::map<std::string, Watchdog>::const_iterator it=watchdogs_.begin(); it!=watchdogs_.end(); ++it)
          if (it->second.barks(now))
            setJointVelocity(command_, getJointIndex(it->first), 0.0);
        
        for(size_t i=0; i<state_.position.size(); ++i)
        {
          // FIXME: having this check might be inefficient, profile this
          if (hasControlledJoint(state_.name[i]))
            state_.velocity[i] = command_.velocity[i];
          state_.position[i] += state_.velocity[i] * dt.toSec();
          enforceJointLimits(state_.name[i]);
        } 

        state_.header.stamp = now;
        state_.header.seq++;
      }

      const sensor_msgs::JointState& getJointState() const
      {
        return state_;
      }

      const sensor_msgs::JointState& getCommand() const
      {
        return command_;
      }

      bool hasJoint(const std::string& name) const
      {
        std::map<std::string, size_t>::const_iterator it = index_map_.find(name);
        
        return it!=index_map_.end();
      }

      bool hasControlledJoint(const std::string& name) const
      {
        std::map<std::string, Watchdog>::const_iterator it = watchdogs_.find(name);
        
        return it!=watchdogs_.end();
      }

      void setSubJointState(const sensor_msgs::JointState& state)
      {
        // FIXME: code duplication! think about refactoring
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

        for(size_t i=0; i<state.name.size(); ++i)
          setJointState(state_, getJointIndex(state.name[i]), state.name[i], 
              state.position[i], state.velocity[i], state.effort[i]);
      }

      void setSubCommand(const sensor_msgs::JointState& command, const ros::Time& now)
      {
        // FIXME: code duplication! think about refactoring
        if (command.name.size() != command.velocity.size())
          throw std::range_error(std::string("Command of type sensor_msgs::JointState") +
              " has fields 'name' and 'velocity' with different sizes: " +
              std::to_string(command.name.size()) + " compared to " +
              std::to_string(command.velocity.size()) + ".");

        for (size_t i=0; i<command.name.size(); ++i)
        {
          std::map<std::string, Watchdog>::iterator it = watchdogs_.find(command.name[i]);

          if (it != watchdogs_.end())
          {
            it->second.pet(now);
            setJointVelocity(command_, getJointIndex(command.name[i]), command.velocity[i]);
          }
        }
      }

    private:
      // internal state and commands of the simulator
      sensor_msgs::JointState state_, command_;

      // a map from joint-state names to their index in the joint-state message
      std::map<std::string, size_t> index_map_;

      // urdf model to lookup information about the joints
      urdf::Model model_;

      // a map holding the watchdogs for our command interfaces
      std::map<std::string, Watchdog> watchdogs_;

      size_t getJointIndex(const std::string& name) const
      {
        std::map<std::string, size_t>::const_iterator it = index_map_.find(name);
        
        if (it==index_map_.end())
          throw std::runtime_error("Could not find joint index for joint with name '" +
              name + "'.");

        return it->second;
      }

      boost::shared_ptr<urdf::Joint> getJoint(const std::string& name) const
      {
        std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it =
          model_.joints_.find(name);

        if (it == model_.joints_.end())
          throw std::runtime_error("URDF has no joint with name '" + name + "'.");

        return it->second;
      }

      void enforceJointLimits(const std::string& name)
      {
        boost::shared_ptr<urdf::Joint> joint = getJoint(name);

        if ((joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) &&
            joint->limits.get())
        {
          // joint should be limited, and has limits specified
          
          size_t index = getJointIndex(name);
          if(state_.position[index] < joint->limits->lower ||
             state_.position[index] > joint->limits->upper)
          {
            state_.position[index] = 
              std::max(joint->limits->lower, std::min(state_.position[index], joint->limits->upper));
            state_.velocity[index] = 0.0;
          }
        }
      }
  };
}

#endif
