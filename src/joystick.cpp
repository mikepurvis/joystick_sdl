/**
Software License Agreement (BSD)

\file      joystick.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "SDL_gamecontroller.h"
#include "SDL.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "joystick_sdl/joystick.h"


namespace joystick_sdl
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link directly to the
 * Joystick class.
 */
struct Joystick::Impl
{
  void timerCallback(const ros::TimerEvent&);
  bool attemptConnection();
  void addMappingsFromFile(std::string filename);
  double scaleAxis(int32_t unscaled_value);

  ros::Time last_connection_attempt_time;
  ros::Duration connection_attempt_period;
  ros::Publisher joy_pub;
  ros::Timer poll_timer;
  sensor_msgs::Joy joy_msg;

  int poll_frequency_hz;
  double dead_zone;

  SDL_Joystick* joy_handle;
  int num_axes;
  int num_buttons;

  Impl();
  ~Impl();
};

Joystick::Joystick(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  /*std::string mappings_file;
  nh_param->param<std::string>("mappings_file", mappings_file,
     ros::package::getPath("joystick_sdl") + "/mappings/gamecontrollerdb.txt");
  pimpl_->addMappingsFromFile(mappings_file);*/

  nh_param->param<double>("dead_zone", pimpl_->dead_zone, 0.05);

  pimpl_->joy_pub = nh->advertise<sensor_msgs::Joy>("joy", 1, true);
  nh_param->param<int>("poll_frequency_hz", pimpl_->poll_frequency_hz, 100);
  ros::Duration poll_period(1.0 / pimpl_->poll_frequency_hz);
  pimpl_->poll_timer = nh->createTimer(poll_period, &Joystick::Impl::timerCallback, pimpl_);
  pimpl_->poll_timer.start();
}

Joystick::Impl::Impl() :
  joy_handle(NULL),
  connection_attempt_period(1.0)
{
  if (SDL_Init(0)) //SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK))
  {
    ROS_FATAL("SDL initialization failed. Joystick will not be available.");
    return;
  }
  if (SDL_JoystickEventState(SDL_DISABLE) != 0)
  {
    ROS_FATAL("SDL initialization failed. Joystick will not be available.");
    return;
  }
}

void Joystick::Impl::addMappingsFromFile(std::string filename)
{
  int mappings_count = SDL_GameControllerAddMappingsFromFile(filename.c_str());
  if (mappings_count > 0)
  {
    ROS_INFO_STREAM("Added " << mappings_count << " joystick mappings from file: " << filename);
  }
  else
  {
    ROS_ERROR_STREAM("Unable to add joystick mappings from file: " << filename);
  }
}

Joystick::Impl::~Impl()
{
  if (joy_handle)
  {
    SDL_JoystickClose(joy_handle);
  }

  SDL_Quit();
}

void Joystick::Impl::timerCallback(const ros::TimerEvent&)
{
  if (!joy_handle)
  {
    // Don't retry connection too frequently.
    if (ros::Time::now() - last_connection_attempt_time > connection_attempt_period)
    {
      last_connection_attempt_time = ros::Time::now();
      attemptConnection();
    }
    return;
  }

  SDL_JoystickUpdate();
  joy_msg.header.stamp = ros::Time::now();

  for (int axis = 0; axis < num_axes; axis++)
  {
    joy_msg.axes[axis] = scaleAxis(SDL_JoystickGetAxis(joy_handle, axis));
  }

  for (int button = 0; button < num_buttons; button++)
  {
    joy_msg.buttons[button] = SDL_JoystickGetButton(joy_handle, button);
  }

  joy_pub.publish(joy_msg);
}

bool Joystick::Impl::attemptConnection()
{
  // Must reinitialize the joystick subsystem in order to scan for newly-connected or
  // newly-available devices. Once initialized, it's essential to "quit" the subsystem
  // once we're through with this connection attempt, or the next initialization will
  // be a no-op.
  if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0)
  {
    ROS_ERROR("Unable to initialize SDL joystick subsystem. Will try again in 1 second.");
    return false;
  }

  int num_joysticks = SDL_NumJoysticks();
  if (num_joysticks == 0)
  {
    ROS_ERROR("No joystick found. Will look again in 1 second.");
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    return false;
  }

  joy_handle = SDL_JoystickOpen(0);
  if (!joy_handle)
  {
    ROS_ERROR("Failed to connect to joystick. Will try again in 1 second.");
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    return false;
  }

  if (num_joysticks > 1)
  {
    ROS_WARN("Found %d joysticks. Connected arbitrarily to the first joystick found.", num_joysticks);
  }

  ROS_INFO("Successfully connected to %s", SDL_JoystickName(joy_handle));
  num_axes = SDL_JoystickNumAxes(joy_handle);
  num_buttons = SDL_JoystickNumButtons(joy_handle);
  joy_msg.axes.resize(num_axes);
  joy_msg.buttons.resize(num_buttons);
  return true;
}

double Joystick::Impl::scaleAxis(int32_t unscaled_value)
{
  if (unscaled_value > 0)
  {
    unscaled_value++;
  }

  double scaled_value = unscaled_value / -32768.0;

  if (std::abs(scaled_value) < dead_zone)
  {
    return 0;
  }

  return scaled_value;
}

}  // namespace joystick_sdl
