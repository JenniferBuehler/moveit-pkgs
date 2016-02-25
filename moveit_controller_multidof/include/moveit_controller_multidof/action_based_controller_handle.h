/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

// XXX MULTIDOF_CHANGE : canged include protectors 
#ifndef MOVEIT_CONTROLLER_MULTIDOF_ACTIONBASEDCONTROLLERHANDLEBASE_H
#define MOVEIT_CONTROLLER_MULTIDOF_ACTIONBASEDCONTROLLERHANDLEBASE_H

#include <moveit/controller_manager/controller_manager.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/macros/class_forward.h>


// XXX MULTIDOF_CHANGE : global replace of
// moveit_simple_controller_manager for moveit_controller_multidof 
namespace moveit_controller_multidof
{

/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string &name) :
    moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  virtual void addJoint(const std::string &name) = 0;
  virtual void getJoints(std::vector<std::string> &joints) = 0;
};

MOVEIT_CLASS_FORWARD(ActionBasedControllerHandleBase);


// XXX BEGIN MULTIDOF_CHANGE : New class based on ActionBasedControllerHandleBase
/*
 * This is a simple base class which only adds the joints.
 */
class ActionBasedControllerJointsHandle : public ActionBasedControllerHandleBase
{

public:
  ActionBasedControllerJointsHandle(const std::string &name) :
    ActionBasedControllerHandleBase(name)
  {
  }

  virtual void addJoint(const std::string &name)
  {
    joints_.push_back(name);
  }

  virtual void getJoints(std::vector<std::string> &joints)
  {
    joints = joints_;
  }

protected:
  /* the joints controlled by this controller */
  std::vector<std::string> joints_;
};
// XXX END MULTIDOF_CHANGE : New class based on ActionBasedControllerHandleBase


} // end namespace moveit_controller_multidof

#endif // MOVEIT_CONTROLLER_MULTIDOF_ACTIONBASEDCONTROLLERHANDLEBASE_H
