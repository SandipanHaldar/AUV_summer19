# Smach tutorials
Smach, which stands for "State Machine", is a powerful and scalable Python-based library for hierarchical state machines. The Smach library does not depend on ROS, and can be used in any Python project. The executive_smach stack however provides very nice integration with ROS, including smooth actionlib integration and a powerful Smach viewer to visualize and introspect state machines. 
![pic1](http://wiki.ros.org/pr2_plugs_executive?action=AttachFile&do=get&target=smach.png)
## Creating a state machine
### Creating a state
To create a state, you simply inherit from the State base class, and implement the State.execute(userdata) method:<br />
```
class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2']):
       # Your state initialization goes here

     def execute(self, userdata):
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'
```
* In the init method you initialize your state class. Make sure to never block in the init method! If you need to wait for other parts of your system to start up, do this from a separate thread.<br />
* In the execute method of a state the actual work is done. Here you can execute any code you want. It is okay to block in this method as long as you like. Once you return from this method, the current state is finished.<br />
* When a state finishes, it returns an outcome. Each state has a number of possible outcomes associated with it. An outcome is a user-defined string that describes how a state finishes.
### Adding states to a state machine
A state machine is a container that holds a number of states. When adding a state to a state machine container, you specify the transitions between the states.
```
sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
  with sm:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'}
 ```
 The resulting state machine looks like this:
 
 ![pic2](http://wiki.ros.org/smach/Tutorials/Getting%20Started?action=AttachFile&do=get&target=simple.png)
* The red boxes show the possible outcomes of the state machine container: outcome4 and outcome5, as specified in line 1.

* In line 3-5 we add the first state to the container, and call it FOO. The convention is to name states with all caps. If the outcome of state FOO is 'outcome1', then we transition to state BAR. If the outcome of state FOO is 'outcome2', then the whole state machine will exit with 'outcome4'.

* Every state machine container is also a state. So you can nest state machines by adding a state machine container to another state machine container.
 ## Pre-defined States and Containers
 ### State Library
 Smach comes with a whole library of pre-implemented states that cover many common usecases
 ###  Container library
 Similarly, Smach also comes with a set of useful containers:

* StateMachine: the generic state machine container

* Concurrence: a state machine that can run multiple states in parallel.

* Sequence: a state machine that makes it easy to execute a set of states in sequence. The 'Smach Containers' section on the tutorials page gives an overview of all available containers.
## Passing User Data between States
### Specifying User Data
A state could require some input data to do its work, and/or it could have some output data it wants to provide to other states. The input and output data of a state is called userdata of the state. When you construct a state, you can specify the names of the userdata fields it needs/provides. 
* The input_keys list enumerates all the inputs that a state needs to run. A state declares that it expect these fields to exist in the userdata. The execute method is provided a copy of the userdata struct. The state can read from all userdata fields that it enumerates in the input_keys list, but it can't write to any of these fields. 
* The output_keys list enumerates all the outputs that a state provides. The state can write to all fields in the userdata struct that are enumerated in the output_keys list. 
The interface to a state is defined by its outcomes, its input keys and its output keys. 
### Connecting User Data
When adding states to a state machine, you also need to connect the user data fields, to allow states to pass data to each other. For example, if state FOO produces 'foo_output', and state BAR needs 'bar_input', then you can attach these two user data ports together using name remapping:
The remapping field maps the in/output_key of a state to a userdata field of the state machine. So when you remap 'x':'y':

    x needs to be an input_key or an output_key of the state, and
    y will automatically become part of the userdata of the state machine.
 We can use the remapping mechanism to pass data from state FOO to state BAR. To accomplish this, we need one remapping when adding FOO, and one remapping when adding BAR:

    FOO: remapping={'foo_output':'sm_user_data'}
    BAR: remapping={'bar_input':'sm_user_data'} 
 We can also use the remapping mechanism to pass data from a state BAR to the state machine that contains BAR. If 'sm_output' is an output key of the state machine:

    BAR: remapping={'bar_output':'sm_output'} 

Or, the opposite, we can pass data from the state machine to a state FOO. If 'sm_input' is an input key of the state machine:

    FOO: remapping={'foo_input':'sm_input'} 
 ![pic3](  http://wiki.ros.org/smach/Tutorials/User%20Data?action=AttachFile&do=get&target=user_data.png)
## Create a Hierarchical State Machine
 We create a number of states, each with a number of outcomes, input keys and output keys specified.
We create a top level state machine, and start adding states to it. One of the states we add is another state machine:
The only point to take away from this is that every state machine is also a normal state. So you can add a state machine to another state machine in the same way you add a state to a state machine. So dealing with userdata is not any different when you deal with hierarchical state machines: the sub state machine specifies input and output keys, and they get remapped when you add the sub state machine to the top level state machine.
![pic3](http://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine?action=AttachFile&do=get&target=sm_expanded.png)

## Calling Actions from a State Machine (ROS)
You could simply call any action from a generic state, but SMACH has specific support to call actions, saving you a lot of code! SMACH provides a state class that acts as a proxy to an actionlib action. The instantiation of the state takes a topic name, action type, and some policy for generating a goal. The possible outcomes of the simple action state are 'succeeded', 'preempted' and 'aborted'.

Depending on how you get your goal, there are simple and more complex ways to use the simple action state.
### Goal Messages
The types of goal messages are given
* Empty Goal Message
* Fixed Goal Message
* Goal from User Data
* Goal callback
### Result Message
* Result Frrom user
* Result Callback
## Viewing State Machines (ROS)
SMACH containers can provide a debugging interface (over ROS) which allows a developer to get full introspection into a state machine. The SMACH viewer can use this debugging interface to visualize and interact with your state machine. To add this debugging interface to a state machine, add the following lines to your code: 
```
# First you create a state machine sm
# .....
# Creating of state machine sm finished

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

# Execute the state machine
outcome = sm.execute()

# Wait for ctrl-c to stop the application
rospy.spin()
sis.stop()

```
* server_name: this name is used to create a namespace for the ROS introspection topics. You can name this anything you like, as long as this name is unique in your system. This name is not shown in the smach viewer.
* SM_ROOT: your state machine will show up under this name in the smach viewer. So you can pretty much choose any name you like. If you have sub-state machines that are in different executables, you can make them show up as hierarchical state machines by choosing this name in a clever way.
The smach viewer will automatically traverse the child containers of sm, if any exist, and add ros hooks to each of them. So you only need to hook up one introspection server to the top level state machine, not to the sub state machines. Once the introspection server has been instantiated, it will advertise a set of topics with names constructed by appending to the server name given to it on construction
Once you have one or more introspection servers running in your ROS system, you can start the smach viewer using:
```
  rosrun smach_viewer smach_viewer.py
```
The viewer will automatically connect to all running introspection servers. 
## SMACH Containers
### StateMachine container
First import the state machine. Since a SMACH StateMachine also provides a State interface, its outcomes and userdata interactions must be specified on construction.Similarly to the SMACH State interface, input keys and output keys are optional.<br />
When adding states to a state machine you first specify the state machine you want to add states to. This can be done by using Python's "with" statement. You can think of this like "opening" the container for construction. It creates a context in which all subsequent add* calls will apply to the open container.
### Concurrence container
The outcome map of a SMACH concurrence specifies the policy for determining the outcome of the concurrence based on the outcomes of its children. Specifically, the map is a dictionary where the keys are potential outcomes of the concurrence, and the values are dictionaries mapping child labels onto child outcomes. Once all the states in the concurrence have terminated, if one of these child-outcome mappings is satisfied, the concurrence will return its associated outcome. If none of the mappings are satisfied, the concurrence will return its default outcome.
### Sequence container
The Sequence container is a StateMachine container, extended with auto-generated transitions that create a sequence of states from the order in which said states are added to the container. Only the differences are noted here.
<br/>First import the sequence type . A container Sequence has its outcomes, specified on construction, along with the 'connector_outcome' which is used for the automatic transitions. The constructor signature is:
```
 __init__(self, outcomes, connector_outcome):
```
Adding states to a sequence is the same as to a container.<br />
But, each state added will receive an additional transition from it to the state which is added after it. The transition will follow the outcome specified at construction of this container.<br />
If one of the transitions given in the dictionary mapping parameter to 'Sequence.add()' follows the connector outcome specified in the constructor, the provided transition will override the automatically generated connector transition.<br />
For example, when creating a sequence of action states, no transition mapping is needed. All action states have the usual outcome triple, which the sequence needs to have as its outcomes, too:

```

   1 sq = Sequence(
   2         outcomes = ['succeeded','aborted','preempted'],
   3         connector_outcome = 'succeeded')
   4 with sq:
   5     Sequence.add('MOVE_ARM_GRAB_PRE', MoveVerticalGripperPoseActionState())
   6     Sequence.add('MOVE_GRIPPER_OPEN', MoveGripperState(GRIPPER_MAX_WIDTH))
   7     Sequence.add('MOVE_ARM_GRAB',     MoveVerticalGripperPoseActionState())
   8     Sequence.add('MOVE_GRIPPER_CLOSE', MoveGripperState(grab_width))
   9     Sequence.add('MOVE_ARM_GRAB_POST', MoveVerticalGripperPoseActionState())
```
### Iterator container
The iterator allows you to loop through a state or states until success conditions are met. This tutorial demonstrates how to use an iterator to sort a list of numbers into evens and odds.
<br />
A sample of forming loops is given below<br />
[link](https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/iterator_tutorial.py)
The smach viewer would give the following result
![pic](http://wiki.ros.org/smach/Tutorials/Iterator%20container?action=AttachFile&do=get&target=smach_tutorial.png)
### Wrapping a Container With actionlib
SMACH provides the top-level container called ActionServerWrapper. This class advertises an actionlib action server. Instead of being executed by a parent, its contained state goes active when the action server receives a goal. Accordingly, this container does not inherit from the smach.State base class and cannot be put inside of another container.

The action server wrapper can inject the goal message received by the action server into the contained state, as well as extract the result message from that state when it terminates. When constructing the action server wrapper, the user specifies which state machine outcomes correspond to a succeeded, aborted, or preempted result.
Consider this example, which wraps a SMACH state machine as an action:


```
import rospy

from smach import StateMachine
from smach_ros import ActionServerWrapper

# Construct state machine
sm = StateMachine(outcomes=['did_something',
                            'did_something_else',
                            'aborted',
                            'preempted'])
with sm:
    ### Add states in here...

# Construct action server wrapper
asw = ActionServerWrapper(
    'my_action_server_name', MyAction,
    wrapped_container = sm,
    succeeded_outcomes = ['did_something','did_something_else'],
    aborted_outcomes = ['aborted'],
    preempted_outcomes = ['preempted'] )

# Run the server in a background thread
asw.run_server()

# Wait for control-c
rospy.spin()
```
The above code will call sm.execute(), but it will not load the goal into the contained state machine, nor will it extract a result. In order to do these things, you need to tell the action server wrapper what it should call the goal and result messages in the context of SMACH. You can replace the action server wrapper construction call with the following:
```# Construct action server wrapper
asw = ActionServerWrapper(
    'my_action_server_name', MyAction, sm,
    ['did_something','did_something_else'], ['aborted'], ['preempted'],
    goal_key = 'my_awesome_goal',
    result_key = 'egad_its_a_result' )
 ```
The keyword arguments goal_key and result_key are the SMACH userdata keys in the context of the ActionServerWrapper. Like any other container, this means that the wrapper's contained state (in this case the state machine sm) will receive a reference to this userdata structure when its execute() method is called. Similarly to how userdata is passed between scopes in nested state machines, in this case, you need to set these key identifiers in the state machine sm as well.

In order to copy in the keys form the parent, you can replace the construction call for the state machine sm with this:
```
 Construct state machine
sm = StateMachine(
        outcomes=['did_something','did_something_else','aborted','preempted'],
        input_keys = ['my_awesome_goal'],
        output_keys = ['egad_its_a_result'])
```
## Generic State
This tutorial shows how to implement a genericstate in smach
The most general (but often least efficient) way to create a state is to derive from the 'State' base class. You can pretty much do anything you want in the execute method of a state. Know that SMACH comes with a library of useful states that you can use, without having to write a whole custom state.
```
from smach import State
```
```
 class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2']):
       # Your state initialization goes here
     
     def execute(self, userdata):
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'
```
Some good code practices:

Do not block in your constructor. If your state needs to wait for another component to come up, do this in a separate thread
## CBState
This willshow hw to use a CBState , a state that simpley uses a callback when active.
This state simply executes a single callback when the state is executed. This is useful for executing arbitrary code in a state, without having to declare a new state class. This class supports the use of the smach.cb_interface decorator, and its API can be found [here](http://www.ros.org/doc/api/smach/html/python/smach.state.CBState-class.html#__init__).

The CBState calls the callback with at least one argument: the container's userdata. Additional arguments and keyword arguments can be given to the CBState on construction. These args will be passed into the callback when the CBState is executed.





