joystick_sdl [![Build Status](https://travis-ci.org/mikepurvis/joystick_sdl.svg?branch=master)](https://travis-ci.org/mikepurvis/joystick_sdl)
============

One of the limitations of the standard ROS [joy][1] package is that it
depends on the `linux/joystick.h` header. This sucks, especially for ROS
users on Windows and OS X, who would enjoy being able to remotely teleop
robots using a locally-connected game controller... without a VM.

So the idea here is to piggy-back on the cross-platform joystick support
available in [SDL][2]. So far, it is tested to work with a Logitech F710
and DualShock 3 (wired) in Ubuntu Trusty 14.04 and Mac OS X 10.10.3,
with the following caveats:

  * For Mac OS X, the F710 must be in DirectInput mode ("D" on the top switch).
  * Detecting disconnects does _not_ work under OS X. SDL 2.0.4 fixes the
    issue, which you can upgrade to with `brew reinstall sdl2 --devel`.
  * Analog triggers don't work, nor does motion control or force feedback
    (SDL limitations). Triggers register as ordinary binary button presses.
  * No attempt is made to expose "hats" or "balls" from the SDL API.
  * Unlike `joy_node`, which uses IO blocking, this package polls the
    joystick at a fixed interval (default 25 Hz). This is a consequence of
    building it on a game platform, where input is typically processed at
    a set point in the main loop.
  * Unlike `joy_node`, there's (currently) no way to differentiate multiple
    devices connected to the same computer.

Finally, SDL provides (via the [GameController][3] API, some ability to abstract
joystick mappings so that disparate units can swap in for each other more
easily (eg, every device has an "A" button, even if it's the one with an
X on it on a PlayStation pad). There's some rough-in to support this here,
but it's unclear how desirable it really is going forward, as most robots
have a standard joystick they ship with, and a button mapping to match. For
now, this is unused, as it's more valuable to have this package be a drop-in
replacement for [joy](http://wiki.ros.org/joy).

This package is under active development. Caveat emptor.

[1]: https://github.com/ros-drivers/joystick_drivers/tree/indigo-devel/joy
[2]: https://wiki.libsdl.org/CategoryJoystick
[3]: https://wiki.libsdl.org/CategoryGameController

