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
1. roscore = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server (parameter server will be introduced later) 
1. rosnode = ros+node : ROS tool to get information about a node. 
1. rosrun = ros+run : runs a node from a given package. 
Rosnode has some interesting functions which can be seen be typing  $rosnode topic
Some important functions are given 
* $rosnode list - The rosnode list command lists these active nodes.
* $rosnode info - The rosnode info command returns information
Rosrun allows you to use the package name to directly run a node within a package (without having to know the package path). 
## Ros Topics
* rqt_graph
    it creates a dynamic graph of what's going on in the system
* rostopic
  $ rostopic -h
1. rostopic bw : display bandwidth used by topic
1. rostopic echo : print messages to screen
1. rostopic hz : display publishing rate of topic
1. rostopic list : print information about active topics
1. rostopic pub : publish data to topic
1. rostopic type : print topic type
* rostopic echo
$ rostopic echo [topic] <br />
 rostopic echo shows the data published on a topic.
* rostopic list
➔ rostopic list returns a list of all topics currently subscribed to
and published.<br />
➔ Usage: $ rostopic list [/topic]
Options:
-h, --help  show this help message and exit  <br />
-b BAGFILE, --bag=BAGFILE list topics in .bag file <br />
-v, --verbose list full details about each topic <br />
-p list only publishers <br />
-s list only subscribers <br />
* rostopic pub
➔ rostopic pub publishes data on to a topic currently advertised. <br />
➔ $ rostopic pub [topic] [msg_type] [args]
* rostopic hz
➔ rostopic hz reports the rate at which data is published. <br />
➔ rostopic hz [topic]
* ROS Messages
➔ Communication on topics happens by sending ROS messages between nodes. <br />
➔ rostopic type returns the message type of any topic being published.<br />
➔ $ rostopic type [topic] <br />
## ROS Services
Services are another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response.<br />
rosservice can easily attach to ROS's client/service framework with services. rosservice has many commands that can be used on services, as shown below: <br />
rosservice list  -       print information about active services <br />
rosservice call   -      call the service with the provided args <br />
rosservice type    -     print service type<br />
rosservice find     -    find services by service type<br />
rosservice uri       -   print service ROSRPC uri<br />
## Ros parameters
rosparam allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax. In simple cases, YAML looks very natural: 1 is an integer, 1.0 is a float, one is a string, true is a boolean, [1, 2, 3] is a list of integers, and {a: b, c: d} is a dictionary. rosparam has many commands that can be used on parameters, as shown below: 
rosparam set    -        set parameter<br />
rosparam get     -       get parameter<br />
rosparam load     -      load parameters from file<br />
rosparam dump      -     dump parameters to file<br />
rosparam delete     -    delete parameter<br />
rosparam list        -   list parameter names<br />
<br />
$ rosparam set /background_r 150 -- will change the red channel of the background color.
* Rosparam dump and load
➔ rosparam dump [file_name] [namespace]<br />
➔ rosparam load [file_name] [namespace]<br />
## Using rqt_console and roslaunch
![img1](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_console%28start%29.png)
$ rosrun rqt_console rqt_console<br />

$ rosrun rqt_logger_level rqt_logger_level<br />
The above commands creates the above mentioned pop-ups
* Using roslaunch
roslaunch starts nodes as defined in a launch file. <br />
create a launch file called turtlemimic.launch and write the code for for two mimicing turtles
<br /> In a new terminal type <br />$roslaunch beginner_tutorials turtlemimic.launch<br />
Then $ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'<br />
You will see the two turtlesims start moving even though the publish command is only being sent to turtlesim1. 
## ROS msg and srv
* msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages. 
* an srv file describes a service. It is composed of two parts: a request and a response. 
msg files are stored in the msg directory of a package, and srv files are stored in the srv directory. <br />
msgs are just simple text files with a field type and field name per line. The field types you can use are: 
int8, int16, int32, int64 (plus uint*) <br />
float32, float64 <br />
string <br />
time, duration <br />
other msg files <br />
variable-length array[] and fixed-length array[C]  <br />
 <br /> <br />
 There is also a special type in ROS: Header, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have Header header. 
 Here is an example of a msg that uses a Header, a string primitive, and two other msgs : <br />
 ```
 Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
  ```
  srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. Here is an example of a srv file: 
```
int64 A
int64 B
---
int64 Sum
```
## Review
    rospack = ros+pack(age) : provides information related to ROS packages

    roscd = ros+cd : changes directory to a ROS package or stack

    rosls = ros+ls : lists files in a ROS package

    roscp = ros+cp : copies files from/to a ROS package
    rosmsg = ros+msg : provides information related to ROS message definitions
    rossrv = ros+srv : provides information related to ROS service definitions
    catkin_make : makes (compiles) a ROS package
        rosmake = ros+make : makes (compiles) a ROS package (if you're not using a catkin workspace) 

# Intermidiate 
## Creating a ROS package by hand.
There is a tool for creating ROS Packages (catkin_create_pkg), but, as you will see, there is nothing actually difficult here. catkin_create_pkg prevents mistakes and saves effort, but packages are just a directory and a simple XML file.

Now we'll create a new foobar package. This tutorial assumes that we're working your catkin workspace and sourcing of the setup file is already done.
```
catkin_ws_top $ mkdir -p src/foobar
catkin_ws_top $ cd src/foobar
```
The very first thing we'll do is add our manifest file. The package.xml file allows tools like rospack to determine information about what your package depends upon.
```
<package format="2">
  <name>foobar</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="foobar@foo.bar.willowgarage.com">PR-foobar</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```
Now that your package has a manifest, ROS can find it. Try executing the command:
```
rospack find foobar
```
If ROS is set up correctly you should see something like: /home/user/ros/catkin_ws_top/src/foobar. This is how ROS finds packages behind the scenes.

Note that this package now also has dependencies on roscpp and std_msgs.

Such dependencies are used by catkin to configure packages in the right order.

Now we need the CMakeLists.txt file so that catkin_make, which uses CMake for its more powerful flexibility when building across multiple platforms, builds the package.
In Foobar/CMmakeList.txt put
```
cmake_minimum_required(VERSION 2.8.3)
project(foobar)
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package()
```
That's all you need to start building a package in ROS using catkin. Of course, if you want it to actually start building something, you're going to need to learn a couple more CMake macros. See our CMakeLists.txt guide for more information. Also always go back to beginner level tutorial (CreatingPackage and so on) to customize your package.xml and CMakeLists.txt.
## Managing System dependencies
 This explains how to use rosdep to install system dependencies.
### System Dependencies
ROS packages sometimes require external libraries and tools that must be provided by the operating system. These required libraries and tools are commonly referred to as system dependencies. In some cases these system dependencies are not installed by default. ROS provides a simple tool, rosdep, that is used to download and install system dependencies.

ROS packages must declare that they need these system dependencies in the package manifest. Let's look at the manifest for the turtlesim package:
Rosdep is a tool you can use to install system dependencies required by ROS packages.
Download and install the system dependencies for turtlesim
* Rosdistro/rosdep
rosdep actually retrieves the rules from the rosdistro github repository.

As of version 0.14.0 rosdep update will only fetch ROS package names for non-EOL ROS distributions. If you are still using an EOL ROS distribution (which you probably shouldn't) you can pass the argument --include-eol-distros to also fetch the ROS package names of those.

These rules are used when a dependency is listed that doesn't match the name of a ROS package built on the buildfarm. Then rosdep checks if there exists a rule to resolve it for the proper platform and package manager you are using.

When creating a new package, you might need to declare new system dependencies to the rosdep rules if they are not there yet. Just edit the file, add the dependency needed (following a strict alphabetical order and a similar structure as the other dependencies already registered) and send a pull request.
### Roslaunch tips for large projects

     

