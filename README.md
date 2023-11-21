
# SPHERE COLLISION SYSTEM

## Overview

The goal of this project is to develop a simple sphere collision system that showcases different physical quantities like gravity, friction, and energy loss. The implementation will include collision detection and resolution between multiple spheres and the floor. Additionally, the user will be able to modify physical quantities, radius, mass, initial positions and velocities of the spheres during runtime, which will be implemented dynamically.
The system is written in C++ using NGL, the NCCA Graphics Library, and QtCreator for the UI .

## Link for the collision system demo

Demo Link: https://youtu.be/jn2dYeo2kVg

## Planning

To implement a simple sphere collision system, I divided the project into smaller parts and executed them by one by one.

For ease of implementation, I started with BlankNGL, which already had functions to project the camera and a shader setup to create and load basic primitive objects.

I will need a bounding box to contain the spheres, so I created a class instance from the BBox header.

I also need a Sphere class to facilitate drawing a sphere onto the screen. It should also contain all the necessary attributes to implement physics on the sphere, and to update it's position accordingly. I used the timerEvent function from qt to make a program loop for the simulation.

## Next steps

Further on, I will look at how to implement gravity and collision with the floor. This is covered in [report1.md](report1.md)



