# Actionlib Tutorials
## Writing a Simple Action Server using the Execute Callback
### Creating the Action Messages
Before writing an action it is important to define the goal, result, and feedback messages. The action messages are generated automatically from the .action file, for more information on action files see the actionlib documentation. This file defines the type and format of the goal, result, and feedback topics for the action.
* actionlib/server/simple_action_server.h is the action library used from implementing simple actions. 
This includes the action message generated from the Fibonacci.action file. This is a header generated automatically from the FibonacciAction.msg file.
* There are  protected variables of the action class. The node handle is constructed and passed into the action server during construction of the action. The action server is constructed in the constructor of the action and has been discussed below. The feedback and result messages are created for publishing in the action. 
* In the action constructor, an action server is created. The action server takes arguments of a node handle, name of the action, and optionally an executeCB.
* Now the executeCB function referenced in the constructor is created. The callback function is passed a pointer to the goal message. Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type. 
* An important component of an action server is the ability to allow an action client to request that the current goal execution be cancelled. When a client requests that the current goal be preempted the action server should cancel the goal, perform necessary clean-up, and call the function setPreempted(), which signals that the action has been preempted by user request. Setting the rate at which the action server checks for preemption requests is left to the implementor of the server.
### Compiling
Modify the Cmakelist as required.
### Running the action server
Start a roscore terminal. And then run the action server. 
You will see something similar to:

    [ INFO] 1250790662.410962000: Started node [/fibonacci], pid [29267], bound on [aqy], xmlrpc port [39746], tcpros port [49573], logging to [~/ros/ros/log/fibonacci_29267.log], using [real] time
  you can look at the nodes: 
  ![pic1](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29?action=AttachFile&do=get&target=fibonacci_server.png)
  This shows that your action server is publishing the feedback, status, and result channels as expected and subscribed to the goal and cancel channels as expected. The action server is up and running properly. 
