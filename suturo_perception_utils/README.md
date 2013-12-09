suturo_perception_utils
-----------------------
since cmake check macros for include and platform checking are completely fucked up, 
you have to manually specify whether you want to build this lib with/without ROS headers.

Build the workspace or this project with -DHAVE_NO_ROS:BOOL=TRUE to use std methods where ROS would
normally be used
