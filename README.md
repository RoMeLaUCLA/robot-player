# Robot Player

The Robot Player package is a small, self contained package that aims to simplify communicating with simulators and robots. Often it's required to have little 'toggle' switches everywhere in your code to have your programs communicate with a simulator and the hardware. This package aims to simplify that by presenting a uniform interface through the MotionManager class.

Each 'player' is a class designed to abstract the boilerplate and small details like port numbers and handles away from the user.

** Currently, all that is needed is to build the C libraries ** 


## Installation instructions

### Dependencies
Currently robot-player is being tested with python 2.7. It might work with python 3 as well, but I haven't tested it.

This tutorial assumes that you are using a python virtual environment. If you've installed virtualenv and virtualenvwrapper, you can set things up very easily with the following lines of code:
```
mkvirtualenv -p /usr/bin/python2.7 --no-site-packages player2
```

```bash
git clone https://github.com/RoMeLaUCLA/robot-player.git
cd robot-player
git checkout release
pip install .
```

That's it!