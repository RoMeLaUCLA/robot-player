# Robot Player

The Robot Player package is a small, self contained package that aims to simplify communicating with simulators and robots. Often it's required to have little 'toggle' switches everywhere in your code to have your programs communicate with a simulator and the hardware. This package aims to simplify that by presenting a uniform interface through the MotionManager class.

Each 'player' is a class designed to abstract the boilerplate and small details like port numbers and handles away from the user.

**This project currently has a few dependencies on Numpy, but they will be eliminated shortly.**
** Currently, all that is needed is to build the C libraries ** 