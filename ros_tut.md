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




     

