^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlesim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2018-04-11)
------------------
* add melodic turtle (`#41 <https://github.com/ros/ros_tutorials/issues/41>`_)

0.8.1 (2017-07-27)
------------------
* theta ranges from -pi to +pi (`#31 <https://github.com/ros/ros_tutorials/issues/31>`_)

0.8.0 (2017-03-10)
------------------
* add lunar turtle (`#39 <https://github.com/ros/ros_tutorials/pull/39>`_)

0.7.1 (2016-10-24)
------------------
* check pen_on\_ when processing teleport requests (`#35 <https://github.com/ros/ros_tutorials/pull/35>`_)

0.7.0 (2016-03-18)
------------------
* add kinetic image
* update to Qt5
* fix size of Jade image to not exceed other images in order to not get positioned incorrectly
* fix compiler warnings

0.6.1 (2015-09-19)
------------------
* update the coordinate system in /spawn service for consistency (`#25 <https://github.com/ros/ros_tutorials/pull/25>`_)

0.6.0 (2015-05-21)
------------------
* add jade turtle (`#22 <https://github.com/ros/ros_tutorials/pull/22>`_)

0.5.3 (2015-05-04)
------------------

0.5.2 (2014-12-23)
------------------

0.5.1 (2014-05-08)
------------------

0.5.0 (2014-05-07)
------------------
* add indigo turtle
* add disabled code to easily spawn all available turtle types

0.4.3 (2014-01-07)
------------------

0.4.2 (2013-10-04)
------------------
* fix missing install of hydro.svg (`#12 <https://github.com/ros/ros_tutorials/issues/12>`_)

0.4.1 (2013-09-11)
------------------
* add hydro image to turtlesim

0.4.0 (2013-09-06)
------------------
* Adding png version of hydro for wiki linking
* TurtleApp accepts argc by reference
* Restoring all the changes appropriate for Hydro

0.3.13 (2013-08-21)
-------------------
* TurtleApp accepts argc by reference
* add hydro image to turtlesim
* remove mainpage.dox

0.3.12 (2013-03-29)
-------------------
* reverting velocity -> twist for groovy
* Revert "chaning command_velocity to cmd_vel" for groovy
  This reverts commit 96e5174d3a5c961b6e1195b90b4024e2858df010.
* Revert "adding geometry_msgs dependency in package.xml and CMakelist" for groovy
  This reverts commit c7ac1155d70269909b55af03d13fe2e089d6215d.
* Revert "alaphabetic order" for groovy
  This reverts commit f928765ed08773517c195b74c55231c0e4fcc5e5.

0.3.11 (2013-03-21)
-------------------
* update email in package.xml

0.3.10 (2013-03-08)
-------------------
* Fix a moc generation error with boost >= 1.48
  See:
  https://bugreports.qt-project.org/browse/QTBUG-22829
* Revert "Merge pull request `#6 <https://github.com/ros/ros_tutorials/issues/6>`_ from ros/fix_qt_moc"
  This reverts commit 0e11b41ac53aad0e043b77d4d5950889245eaceb, reversing
  changes made to fc19df449d9ac297e8ab829ff22e99323c33ae93.
* Revert "fix missing include (regression of `#5 <https://github.com/ros/ros_tutorials/issues/5>`_)"
  This reverts commit 546dabe05c00e87296952cb2ca655e01895bd5ed.
* fix missing include (regression of `#5 <https://github.com/ros/ros_tutorials/issues/5>`_)
* Fix a mod generation error with boost >= 1.48
  See:
  https://bugreports.qt-project.org/browse/QTBUG-22829
* alaphabetic order
* adding geometry_msgs dependency in package.xml and CMakelist
* chaning command_velocity to cmd_vel
* remove turtlesim velocity and use Twist msg

0.3.9 (2012-12-21)
------------------
* add groovy turtle
* modified dep type of catkin

0.3.8 (2012-12-13)
------------------
* add missing downstream depend
* switched from langs to message_* packages

0.3.7 (2012-12-06)
------------------

0.3.6 (2012-10-30)
------------------
* fix catkin function order

0.3.5 (2012-10-18)
------------------

0.3.4 (2012-10-06)
------------------

0.3.3 (2012-10-05)
------------------
* fixed missing genmsg stuff
* updated to latest catkin
* added package.xml files

0.3.2 (2012-09-05)
------------------
* updated catkin variables
* updated pkg-config in manifest.xml

0.3.1 (2012-09-03)
------------------
* use install destination variables, removed manual installation of manifests

0.3.0 (2012-08-29)
------------------
* updated to current catkin

0.2.20 (2013-02-08)
-------------------
* fixed compilation on platforms with different qreal type

0.2.19 (2012-06-15 03:13:40 +0000)
----------------------------------
* make find_package REQUIRED
* removed obsolete catkin tag from manifest files
* added missing install of turtlesim images
* using fuerte image in turtlesim
* fuerte icon
* remove old Makefiles and bump to 0.2.13
* fix find boost component for turtlesim
* change deps for turtlesim from wx to qt
* migrate turtlesim from wx to qt
* updated export for messages/catkin
* add missing libs for oneiric
* add missing dependency on wx, and take out conditional build logic from turtlesim
* conditionally build based on wx, for now
* turn on turtlesim
* adding <catkin/>, removing depends and platform tags
* remove old rosbuild2 stuff
* adios rosbuild2 in manifests
* changed number of turtles to a #define to prevent future mistakes with adding new turtles
* electric turtle
* rosbuild2/windows tweaks, they keep on comin'
* rosbuild2 taking shape.
* rosbuild2 taking shape
* moving teleop keyboard into turtlesim to remove tutorial deps on keyboard
* diamondback
* Added Ubuntu platform tags
* fix to actually paint on OSX
* Only update the path image every 3 frames, because ConvertToImage on a 500x500 bitmap is somehow very expensive
* Move bitmap->image conversion outside of loop (that was boneheaded)
* Add color sensor to turtles
* Switch turtlesim to x-forward (theta=0 now faces to the right)
* Optionally name your turtles yourself
* Fix coordinate system
* adding a little more description to manifest
* Add absolute and relative teleport service calls
* changing turtlesim to turtlesim_node for tutorial clarity
* * Multi-turtle support
  * turtle_pose and command_velocity now exist per-turtle.  turtle_pose has been renamed "pose"
  * "spawn" service call to spawn a new turtle, which returns the turtle name
  * "kill" service call, to kill a turtle by name
  * Switch to "meters" as the distance unit, where 1 meter is defined as the height of the turtle
* adding export to manifest
* Change default background/pen colors
* Randomly choose one of the 3 turtles
* 3 turtle set by metamanda
* throttling refresh rate so that xorg doesn't use all the cpu
* adding debug statements
* the drawing file used to create turtle.png
* new turtle made by melonee
* Apply Melonee's diff to set the background color parameters on the param server at startup
* Add error output if the turtle hits the wall
* Add turtlesim to the ros_tutorials stack
