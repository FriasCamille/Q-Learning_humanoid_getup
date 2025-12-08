# Getup with classical Q learning
Today all the approaches to resolve the getup in humanoid robots uses some sort of Deep RL to approximate the getup robot behaviour they work well but the computational requirements increase with each  solution forcing to update the hardware, especially the GPU one of the most expensive components.
This is a series of experiments to try to solve the getup behaviour whitout the most expensive part of the usual systems, the neural network.
## Contents
- A c++ class to solve basic Q-learning
- Mujoco simulator class
- Mujoco Visualization class
- Robot enviroment
- A Darwin OP mujoco xml modified from another repo (The credits are in the same file i will put it later)
## Dependencies
- For now only works in linux, i'm using Ubuntu 20.04
- Mujoco Simulator
    - Need to install Mujoco from: https://github.com/google-deepmind/mujoco

## System Approach
Currently i'm trying to solve the getup simplifying the Robot only looking the sagital plane making virtually a robot with only 5 motors. After that i discretize all the motor ranges, you can change it in the "resources/robot_config.yaml" so motors and sensros will divide the range of action in N steps where N is the number of positions that a motor can take.
![[resources/robot.png]]
### State
The state vector is compossed by 7 variables:


| Vector index | What it is                       |
| ------------ | -------------------------------- |
| 0-4          | Each of the motor position index |
| 5            | The imu readen angular velocity  |
| 6            | The imu angular position index   |

 As there can be N posible values for each of the index the number of states will be:
 $states = N^7$

Where N can be any value greater than 2 just have in mind that greater the value more memory and more time will take to compleate i recomend maximum 9 but you can experiment yourself changing the variable "discretization" in "src/main.cpp"

### Action

The action vector is compossed by 5 values, one for each motor, than can take the value of -1,0,1 representing if the motor should move to the down, next posible position, stay in the same position or move to the upper next posible position.

So the Actions can take $3^5 = 243$ posible values.

### Q table

The resulting Q table will be saved by default in the "resources/trainings/getup_trainment" binary file, you can change the location as the name in the main.cpp file in variable "trainment_path"

## Reward function 
You can change it in the function reward in "src/include/Robot.h"
The current reward function is:
$$Reward = collision_reward + ground_reward + angle_reward + COM_reward + torso_height_reward + goal_reward - 0.1$$
where:

### Collision_reward
Just a minus 100 if the enviroment detects any colission, now is reading the torso mesh collision you can change it in main.cpp and a collision is detected when the force between the torso and the ground exceds 40 N you can change the value in the function collision in "src/include/environment.h"

### ground reward
Just a minus 5 if the feet are not touching the ground

### angle reward 
Is described by:
$$reward-= 5*cos(abs(angle))$$

### COM_reward
Described by:
$$reward-= 10*Com_err$$
Where the com error is the distance between the feet COM and the robot COM

### torso height
Described by:
$$reward+= 2* torso_height$$

### goal_reward 
Just a 100 if the robot reach the goal position described in the variable target_position in "src/include/robot.h" 

## How to run it

Remember to satisfy the Dependencies listed here after that just git clone this repo and run the make file and if it compiles succesfully just type in cmd ./getup
You can add true to visualize and a file path to your own robot model for mujoco. Now it currently has the Darwin OP mujoco model.



