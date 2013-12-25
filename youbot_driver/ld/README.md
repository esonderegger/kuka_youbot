Setting a Global ROS LD_LIBRARY_PATH
====================================

In order to run executables which use the youbot_driver without root, [setcap](http://linux.die.net/man/8/setcap) must be run. This will disable local LD_LIBRARY_PATH. The added conf file can be installed to ensure the driver can still run with the ROS libraries. This is installed with the Debian package.
