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

