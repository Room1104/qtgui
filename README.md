# qtgui
ROS Qt UI package - consists of a C++ Qt gui which invokes a service,
and that service script (in python).

Requires:
* ros-indigo-qt-build
* ros-indigo-qt-app
* ros-indigo-qt-create
* ros-indigo-qt-core

QtGui is a Qt-4 app which has some buttons. These should call
`qnode.triggerService(servicename)` where `servicename` is something like
`/qtgui/dispatcher/randomgreet` or whatever.

The `dispatcher.py` script creates services to respond to these. The services
then perform real actions, such as calling `say()`, which would use the MARY
speech system to say some text.

The basic idea is to split the UI stuff, which is better written in C++
and Qt form editing, from the logic, which is better written in Python.

## Launching

Launch with `roslaunch qtgui qtgui.launch`.
