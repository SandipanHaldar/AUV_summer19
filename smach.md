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

