^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospy_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2020-08-04)
------------------

0.9.2 (2020-02-26)
------------------
* add print parenthesis for Python 3 (`#82 <https://github.com/ros/ros_tutorials/issues/82>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#81 <https://github.com/ros/ros_tutorials/issues/81>`_)

0.9.1 (2019-03-04)
------------------
* add missing dependency for tests (`#50 <https://github.com/ros/ros_tutorials/issues/50>`_)

0.9.0 (2018-04-11)
------------------

0.8.1 (2017-07-27)
------------------

0.8.0 (2017-03-10)
------------------

0.7.1 (2016-10-24)
------------------
* add example of periodical publishing with rospy.Timer (`#34 <https://github.com/ros/ros_tutorials/issues/34>`_)

0.7.0 (2016-03-18)
------------------
* fix minor misleading comment (`#27 <https://github.com/ros/ros_tutorials/pull/27>`_)

0.6.1 (2015-09-19)
------------------

0.6.0 (2015-05-21)
------------------

0.5.3 (2015-05-04)
------------------

0.5.2 (2014-12-23)
------------------
* update code style of examples (`#18 <https://github.com/ros/ros_tutorials/pull/18>`_, `#19 <https://github.com/ros/ros_tutorials/pull/19>`_)

0.5.1 (2014-05-08)
------------------
* fix install when building Debian package

0.5.0 (2014-05-07)
------------------
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* use queue_size for rospy.Publisher (`ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_)
* use catkin_install_python() to install Python scripts (`#17 <https://github.com/ros/ros_tutorials/issues/17>`_)

0.4.3 (2014-01-07)
------------------
* python 3 compatibility (`#13 <https://github.com/ros/ros_tutorials/issues/13>`_)
* remove load_manifest calls

0.4.2 (2013-10-04)
------------------

0.4.1 (2013-09-11)
------------------
* check for CATKIN_ENABLE_TESTING

0.4.0 (2013-09-06)
------------------

0.3.13 (2013-08-21)
-------------------
* check for CATKIN_ENABLE_TESTING
* remove mainpage.dox

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
* fix node names (fix `#2 <https://github.com/ros/ros_tutorials/issues/2>`_)

0.3.6 (2012-10-30)
------------------
* fix catkin function order

0.3.5 (2012-10-18)
------------------

0.3.4 (2012-10-06)
------------------

0.3.3 (2012-10-05)
------------------
* added missing dep to rostest
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
* updated to current catkin

0.2.20 (2013-02-08)
-------------------

0.2.19 (2012-06-15 03:13:40 +0000)
----------------------------------
* make find_package REQUIRED
* removed obsolete catkin tag from manifest files
* fixed permissions of installed rospy_tutorials files
* remove old Makefiles and bump to 0.2.13
* removed catkin_export_python() from rospy_tutorials
* clean up build; tests turned on (but failing)
* adding <catkin/>, removing depends and platform tags
* updated to new catkin_export_python macro
* Don't install version control files
* catkin tweaks
* catkin tweaks
* initial add of rospy_tutorials
* url fixes
* now uses myargv
* Added Ubuntu platform tags
* fixed deprecated use
* a2i_client: simplified conversion
* added msg/srv export
* Remove use of deprecated rosbuild macros
* updated to load_manifest
* Create trunk/branches/tags
