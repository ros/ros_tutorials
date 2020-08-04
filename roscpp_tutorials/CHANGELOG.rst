^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roscpp_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2020-08-04)
------------------

0.9.2 (2020-02-26)
------------------
* explicitly state Boost dependencies manifests (`#83 <https://github.com/ros/ros_tutorials/issues/83>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#81 <https://github.com/ros/ros_tutorials/issues/81>`_)
* add cached parameter sample (`#52 <https://github.com/ros/ros_tutorials/issues/52>`_)

0.9.1 (2019-03-04)
------------------

0.9.0 (2018-04-11)
------------------

0.8.1 (2017-07-27)
------------------

0.8.0 (2017-03-10)
------------------

0.7.1 (2016-10-24)
------------------

0.7.0 (2016-03-18)
------------------

0.6.1 (2015-09-19)
------------------

0.6.0 (2015-05-21)
------------------

0.5.3 (2015-05-04)
------------------
* add talker_listener launch file (`#21 <https://github.com/ros/ros_tutorials/pull/21>`_)

0.5.2 (2014-12-23)
------------------

0.5.1 (2014-05-08)
------------------

0.5.0 (2014-05-07)
------------------

0.4.3 (2014-01-07)
------------------

0.4.2 (2013-10-04)
------------------

0.4.1 (2013-09-11)
------------------

0.4.0 (2013-09-06)
------------------

0.3.13 (2013-08-21)
-------------------

0.3.12 (2013-03-29)
-------------------

0.3.11 (2013-03-21)
-------------------
* update email in package.xml

0.3.10 (2013-03-08)
-------------------

0.3.9 (2012-12-21)
------------------
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
* fixed installation of cpp talker and listener

0.3.4 (2012-10-06)
------------------

0.3.3 (2012-10-05)
------------------
* fixed missing genmsg stuff
* updated to latest catkin
* updated package.xml files
* added package.xml files

0.3.2 (2012-09-05)
------------------
* updated catkin variables

0.3.1 (2012-09-03)
------------------
* use install destination variables, removed manual installation of manifests

0.3.0 (2012-08-29)
------------------
* fixed location of built binaries
* updated to current catkin

0.2.20 (2013-02-08)
-------------------

0.2.19 (2012-06-15 03:13:40 +0000)
----------------------------------
* make find_package REQUIRED
* removed obsolete catkin tag from manifest files
* remove old Makefiles and bump to 0.2.13
* adding <catkin/>, removing depends and platform tags
* catkin cleanups
* remove old rosbuild2 stuff
* adios rosbuild2 in manifests
* catkin updates
* Debugging build, "it works" but should be revisited.
* catkin_project
* catkin tweaks
* catkin tweaks
* catkinize
* Make installation follow the spec
* Updated cpp versions of talker/listener to compile properly with catkin
* Support for catkin
* warning stoppers
* Windows and rosbuild2 tweaks
* url fixes
* rosbuild2 taking shape
* fix end tag
* wiki tag the timers tutorial code
* tag code
* refactor service server w/class method and tag for wiki
* add listener with class method tutorial code
* add parameters tutorial code
* add FULLTEXT tag
* remove comment from the init tag
* add rosconsole tag
* tag talker/listener
* Added Ubuntu platform tags
* Tutorial for listening to a single message, though not compiling because the functionality won't exist until ROS 0.11
  Also enabled compilation of the async listener tutorial, since 0.10 is out
* Forgot roscpp_tutorials is not yet released -- remove compilation of code that only works against roscpp trunk
* * Add listener_async_spin sample
  * Fix rosbuild cmake deprecation warnings
* Create trunk/branches/tags
