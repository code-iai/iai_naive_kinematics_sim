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

#ifndef IAI_NAIVE_KINEMATICS_SIM_SYSTEM_HPP
#define IAI_NAIVE_KINEMATICS_SIM_SYSTEM_HPP

#include <iai_naive_kinematics_sim/simulator.hpp>
#include <iai_naive_kinematics_sim/watchdog.hpp>

namespace iai_naive_kinematics_sim
{
  class System
  {
    public:
      System() {}
      ~System() {}

      void init(const urdf::Model& model, const std::vector<std::string>& controlled_joints,
          double watchdog_period)
      {
        for(size_t i=0; i<controlled_joints.size(); ++i)
          if (!modelHasMovableJoint(model, controlled_joints[i]))
            throw std::runtime_error("URDF model has no movable joint with name '" +
                controlled_joints[i] + "'.");
        
        sim_.init(model);

        dogs_.clear();
        for(size_t i=0; i<controlled_joints.size(); ++i)
          dogs_[controlled_joints[i]] = Watchdog(ros::Duration(watchdog_period));
      }

      const sensor_msgs::JointState& getJointState() const
      {
        return sim_.getJointState();
      }

      void setSubJointState(const sensor_msgs::JointState& state)
      {
        sim_.setSubJointState(state);
      }

      void update(const ros::Time& now, double dt)
      {
        // FIXME: Now that I use sensor_msgs/JointState as a command message, it feels over-complicated
        //        to hold the commands within the watchdogs: I basically store the velocity commands twice.
        //        Watchdogs could be reduced to just remember the last time a command came in.
        for(std::map<std::string, Watchdog>::iterator it=dogs_.begin(); it!=dogs_.end(); ++it)
        {
          it->second.update(now);
          sim_.setNextJointVelocity(it->first, it->second.getCommand());
        }
        sim_.update(now, dt);
      }

      void setVelocityCommand(const sensor_msgs::JointState& msg, const ros::Time& now)
      {
        if (msg.name.size() != msg.velocity.size())
          throw std::runtime_error("Command message invalid: " + std::to_string(msg.name.size()) + 
              " names and " + std::to_string(msg.velocity.size()) + " velocities given. Do not match.");

        for (size_t i=0; i<msg.name.size(); ++i)
        {
          std::map<std::string, Watchdog>::iterator it = dogs_.find(msg.name[i]);

          if (it==dogs_.end())
            throw std::runtime_error("No velocity interface for joint with name '" + 
                msg.name[i] + "'.");

          it->second.setNewCommand(now, msg.velocity[i]);
        }
      }

      const SimulatorVelocityResolved& getSim() const
      {
        return sim_;
      }

      const std::map<std::string, Watchdog>& getWatchdogs() const
      {
        return dogs_;
      }

    private:
      SimulatorVelocityResolved sim_;
      std::map<std::string, Watchdog> dogs_;
  };
}

#endif
