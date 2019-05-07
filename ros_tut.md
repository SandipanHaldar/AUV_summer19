# ROS Tutorial - beginners
ROS- robot oprating system, ros service is service provided by robot
## Creating a ROS Workspace
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/
 $ catkin_make
 Source the setup files - $ source devel/setup.bash
 Consists of three folders
➔ src-contains our ros package
➔ devel
➔ build
## Creating a ROS Package
 Catkin package consists of
➔ CMakeLists.txt
➔ The package.xml file - provides meta information about the
package.
➔ Each package must have its own folder.
$ cd ~/catkin_ws/src
$catkin_create_pkg <package_name> [depend1] [depend2][depend3]
(ex:$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp)
Building catkin workspace
$ cd ~/catkin_ws
$ catkin_make
source the generated setup file - $ . ~/catkin_ws/devel/setup.bash
 First order dependencies
➔ The dependencies are stored in package.xml
➔ roscpp
rospy
std_msgs
 Indirect dependencies : there are many indirect independencies,can be known through - $ rospack depends [Package Name]
## Building packages
 $ source /opt/ros/melodic/setup.bash
In a catkin workspace -
$ catkin_make
This process is run for each CMake project.
## Ros Nodes
![ROS node](http://www.clearpathrobotics.com/assets/guides/ros/_images/ros101one.png)
     

