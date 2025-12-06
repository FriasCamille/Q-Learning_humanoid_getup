## Getup with classical Q learning
Today all the approaches to resolve the getup in humanoid robots uses some sort of Deep RL to approximate the getup robot behaviour they work well but the computational requirements increase with each  solution forcing to update the hardware, especially the GPU one of the most expensive components.
This is a series of experiments to try to solve the getup behaviour whitout the most expensive part of the usual systems, the neural network.
## Contents
- A c++ class to solve basic Q-learning
- Mujoco simulator class
- Mujoco Visualization class
- Robot enviroment
- A Darwin OP mujoco xml modified from another repo (The credits are in the same file i will put it later)

