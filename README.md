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

## AES

The `aes.py` script
* publishes the hormone level from -1 to 1 on the output topic `/hlevel`
* releases the hormone according to the level of the input topic `/hrelease`
* kicks the hormone down if the service `/aes/nasty` is called
* kicks the hormone up if the service `/aes/nice` is called
* geometrically decays the hormone level to zero over time
