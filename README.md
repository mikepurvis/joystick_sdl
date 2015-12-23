joystick_sdl
============

One of the limitations of the standard ROS [joy][1] package is that it
depends on the `linux/joystick.h` header. This sucks, especially for ROS
users on Windows and OS X, who would enjoy being able to remotely teleop
robots using a locally-connected game controller... without a VM.

So the idea here is to piggy-back on the cross-platform joystick support available in [SDL][2]. So far, it works with a Logitech F710 in Ubuntu
Trusty 14.04 and Mac OS X 10.10.3, as long as the F710 is in DirectInput
mode ("D" on the top switch).

[1]: https://github.com/ros-drivers/joystick_drivers/tree/indigo-devel/joy
[2]: https://wiki.libsdl.org/CategoryJoystick

