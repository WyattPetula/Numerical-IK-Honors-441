# Honors Project for MATH 441: Matrix Algebra

# Section 1 - Introduction

## Project Motivation

One of my favorite video games is Rain World. In Rain World, you play as a nomadic slugcat who must learn to navigate an alien ecosystem as predator and prey. One element of the game that captivates me is its rich use of procedural animation, which animates creatures using algorithms rather than keyframes. 

![Demo GIF](https://i.imgur.com/osZj8BC.gif)

*Inverse kinematics enables the slugcat to climb, walk, and crawl realistically in a variety of environments—a feat difficult to accomplish using traditional animation methods.*

A common procedural animation technique is inverse kinematics, which calculates the joint angles and positions necessary for the end of a kinematic chain (e.g., a hand) to reach a target. As a hobbyist game developer, I want to make a game like Rain World for my next project; however, I have no experience with procedural animation. Fortunately, several inverse kinematics methods are rooted in matrix algebra, which makes them a great topic for a MATH 441 honors project.

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

**Steps:**

For each joint in the kinematic chain, from end effector to base:

1. Get the normalized vector from the joint to the end effector
2. Get the normalized vector from the joint to the target
3. Find the angle between the vectors
4. Rotate the joint by that angle

## Jacobian Transpose
https://github.com/user-attachments/assets/0af39a8f-e61d-49c1-8b62-5ab88edc9485

**Steps:**

For each step of the simulation:

1. Update the Jacobian
```math
J = \begin{bmatrix}
\frac{\partial ee_{1x}}{\partial j\theta_1} & \frac{\partial ee_{1x}}{\partial j\theta_2} & ... & \frac{\partial ee_{1x}}{\partial j\theta_n} \\
\frac{\partial ee_{1y}}{\partial j\theta_1} & \frac{\partial ee_{1y}}{\partial j\theta_2} & ... & \frac{\partial ee_{1y}}{\partial j\theta_n} \\
\frac{\partial ee_{1z}}{\partial j\theta_1} & \frac{\partial ee_{1z}}{\partial j\theta_2} & ... & \frac{\partial ee_{1z}}{\partial j\theta_n} \\
\frac{\partial ee_{2x}}{\partial j\theta_1} & \frac{\partial ee_{2x}}{\partial j\theta_2} & ... & \frac{\partial ee_{2x}}{\partial j\theta_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial ee_{nz}}{\partial j\theta_1} & \frac{\partial ee_{nz}}{\partial j\theta_2} & ... & \frac{\partial ee_{nz}}{\partial j\theta_n} \\
\end{bmatrix}
```
2. Update the end-effector position vector
```math
\textbf{s} = \begin{bmatrix}
ee_{1x} \\
ee_{1y} \\
ee_{1z} \\
ee_{2x} \\
\vdots \\
ee_{nz}
\end{bmatrix}
```

3. Update the target position vector 
```math
\textbf{t} = \begin{bmatrix}
t_{1x} \\
t_{1y} \\
t_{1z} \\
t_{2x} \\
\vdots \\
t_{nz}
\end{bmatrix}
```

4. Update the positional error vector
```math
\textbf{e} = \textbf{t} - \textbf{s}
```

5. Recalculate the step-scaling rate (using dot product)
```math
\alpha = \frac{\langle \textbf{e}, JJ^\top \textbf{e}\rangle}{\langle JJ^\top \textbf{e}, JJ^\top \textbf{e}\rangle}
```

6. Recalculate the per-joint correction angles
```math
\vec{\Delta\theta} = \alpha J^\top \textbf{e}
```

7. Apply the correction angles to each corresponding joint
```math
j_{\theta} = j_{\theta} + \vec{\Delta\theta}_i
```

## Jacobian Pseudoinverse
https://github.com/user-attachments/assets/c29a9c8c-39de-436d-a199-b59f26d76db6

**Steps:**

1. Update the Jacobian
```math
J = \begin{bmatrix}
\frac{\partial ee_{1x}}{\partial j\theta_1} & \frac{\partial ee_{1x}}{\partial j\theta_2} & ... & \frac{\partial ee_{1x}}{\partial j\theta_n} \\
\frac{\partial ee_{1y}}{\partial j\theta_1} & \frac{\partial ee_{1y}}{\partial j\theta_2} & ... & \frac{\partial ee_{1y}}{\partial j\theta_n} \\
\frac{\partial ee_{1z}}{\partial j\theta_1} & \frac{\partial ee_{1z}}{\partial j\theta_2} & ... & \frac{\partial ee_{1z}}{\partial j\theta_n} \\
\frac{\partial ee_{2x}}{\partial j\theta_1} & \frac{\partial ee_{2x}}{\partial j\theta_2} & ... & \frac{\partial ee_{2x}}{\partial j\theta_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial ee_{nz}}{\partial j\theta_1} & \frac{\partial ee_{nz}}{\partial j\theta_2} & ... & \frac{\partial ee_{nz}}{\partial j\theta_n} \\
\end{bmatrix}
```
2. Update the end-effector position vector
```math
\textbf{s} = \begin{bmatrix}
ee_{1x} \\
ee_{1y} \\
ee_{1z} \\
ee_{2x} \\
\vdots \\
ee_{nz}
\end{bmatrix}
```

3. Update the target position vector 
```math
\textbf{t} = \begin{bmatrix}
t_{1x} \\
t_{1y} \\
t_{1z} \\
t_{2x} \\
\vdots \\
t_{nz}
\end{bmatrix}
```

4. Update the positional error vector
```math
\textbf{e} = \textbf{t} - \textbf{s}
```

5. Calculate the Moore-Penrose Pseudoinverse of the Jacobian
```math
J^{+} = V \Sigma^{+} U^\top
```

6. Recalculate the per-joint correction angles (alpha is constant)
```math
\vec{\Delta\theta} = \alpha J^{+} \textbf{e}
```

7. Apply the correction angles to each corresponding joint
```math
j_{\theta} = j_{\theta} + \vec{\Delta\theta}_i
```

## (Jacobian) Damped Least Squares
https://github.com/user-attachments/assets/28c8d139-9640-43ef-9f1f-8fe9aa660d2a

# Section 3 - How Each Method Works

# Section 4 - Comparative Analysis

# Section 5 - Conclusion

# License
This project is licensed under Creative Commons Attribution (CC BY). You are free to copy and redistribute the material in any medium or format for any purpose. You are also free to remix, transform, and build upon the material for any purpose. However, you must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use. Additionally, you may not apply legal terms or technological measures that legally restrict others from doing anything this license permits.

# Author
Wyatt P. Petula
