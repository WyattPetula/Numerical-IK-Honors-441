# Honors Project for MATH 441: Matrix Algebra

# Section 1 - Introduction

## Project Motivation

One of my favorite video games is Rain World. In Rain World, you play as a nomadic slugcat who must learn to navigate an alien ecosystem as predator and prey. One element of the game that captivates me is its rich use of procedural animation, which animates creatures using algorithms rather than keyframes. 

![Demo GIF](https://i.imgur.com/osZj8BC.gif)

*Inverse kinematics enables the slugcat to climb, walk, and crawl realistically in a variety of environments—a feat difficult to accomplish using traditional animation methods.*

A common procedural animation technique is inverse kinematics, which calculates the joint angles and positions necessary for the end of a kinematic chain (e.g., a hand) to reach a target. As a hobbyist game developer, I want to make a game like Rain World for my next project; however, I have no experience with procedural animation. Fortunately, several inverse kinematics methods are rooted in matrix algebra, which makes them a great topic for my MATH 441 honors project.

## Project Goals

1. Implement four iterative inverse kinematics algorithms:
- Cyclic Coordinate Descent (baseline)
- Jacobian Transpose
- Jacobian Pseudoinverse
- Damped Least Squares
2. Review literature to understand how and why these methods work
3. Benchmark each method's performance and compare their visual behavior

# Section 2 - Implementation

I used the Unity Game Engine for this project. I chose Unity because I use it for game development, and it supports MathNet.Numerics, an easy-to-use linear algebra library. Unity is written in C#.

## Cyclic Coordinate Descent
https://github.com/user-attachments/assets/89317682-64c4-4b02-8c31-c0a2f38c6eb5

## Jacobian Transpose
https://github.com/user-attachments/assets/0af39a8f-e61d-49c1-8b62-5ab88edc9485

## Jacobian Pseudoinverse
https://github.com/user-attachments/assets/c29a9c8c-39de-436d-a199-b59f26d76db6

## (Jacobian) Dampened Least Squares
https://github.com/user-attachments/assets/28c8d139-9640-43ef-9f1f-8fe9aa660d2a

# Section 3 - How Each Method Works

# Section 4 - Comparative Analysis

# Section 5 - Conclusion

# License
This project is licensed under Creative Commons Attribution (CC BY). You are free to copy and redistribute the material in any medium or format for any purpose. You are also free to remix, transform, and build upon the material for any purpose. However, you must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use. Additionally, you may not apply legal terms or technological measures that legally restrict others from doing anything this license permits.

# Author
Wyatt P. Petula
