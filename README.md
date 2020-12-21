# Research Track I - First Assignment S4482512 Enrico Fiasche'

The assignment requires controlling a holonomic robot in a 2d space with a simple 2d simulator, Stage. 
The simulator can be launched by executing the command:

```
rosrun stage_ros stageros $(rospack find assignment1)/world/exercise.world
```

If you want to run the solution, you have to run the following commands in two separate shells in the same order:

```
rosrun assignment1 target_server.py
```

and

```
rosrun assignment1 robot_controller.py 
```

### The following behaviour should be achieved
- 1. The robot asks for a random target, with both coordinates in the interval (-6.0, 6.0).
- 2. The robot reaches the target.
- 3. Go to step 1.

### Programs and nodes
- I have two different programs, **target_server.py** and **robot_controller.py**. I have chosen to do it in Python, because I already know the c++ code and I want to improve my skills knowledge in Python.
- The first one, **target_server.py**, is a server called __target__ that generates each time a random point (x,y) given the range (min,max), required by the file Target.srv.
- The second one, **robot_controller.py**, control the moviment of the robot in the space, trying to reach the target position.
  It has a subscriber, to know the actual position of the robot in the space through the topic odom. Inside the callBack function it checks if the robot has reached the target position and modifies its velocity.
  It has also a client, to generate a new target position when the robot reaches it.

### Services and messages
- In this project, I didn't use any custom messages to send between processes.
- I have created a service called **Target.srv**, that requires two float parameters, the range (the minimum and maximum value) to generate a random target.
The response is the random float target coordinates (x,y).
This service is used by the **target_server.py**.

### Nodes and communication
I have three nodes that run toghether to reach the project goal.
- The first one is the **stageros**. It is the 2D space simulator that sends the topic **odom**, to know the robot actual position, and receives the topic **cmd_vel**, to modify the robot velocity, from another node, the **robot_controller**.
- The second one is **robot_controller**. This node controls the moviment of the robot. It receives the actual position from the **stageros** node, to see if the target is reached, if it isn't reached modifies the velocity based by the distance between the goal and robot position, with the topic **cmd_vel**.
- The thrid node is **target_server**. It is a service server, that provides a random target position to the **robot_controller** node.

There is a graph inside folder **graph**, which is figured what is explained above.

