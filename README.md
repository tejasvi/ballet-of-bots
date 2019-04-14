# Ballet of Bots - 3 DOF Ball-on-plate System

# General Overview
This goal of this project is to effectively control a weighted ball on a flat surface.
Following components are used:
- A resistive touchscreen is used to find ball position
- 2 Servo motors are positioned on two corners of trianglur base for tilting the plate
- A python frontend is used for end user control

The entire project is built around using a PID method to properly move the ball to the setpoint.
For those not familiar, a PID control loop equation looks like the following:

![alt text](https://wikimedia.org/api/rest_v1/media/math/render/svg/242b6798586d4fc1aedf7e4f92bf77416e4fc76c)

Where e is the error and Kp, Ki, and Kd are all constants optimised for maximum stability.

- Concept model 
<img src="https://hackster.imgix.net/uploads/image/file/153228/zostava.JPG?auto=compress%2Cformat&w=740&h=555&fit=max" width="440px">

- Keeping Ball Centered:
<img src="https://github.com/schwertJake/Ball-On-Plate-Machine/blob/master/Sample_Art/center.gif?raw=true" width="480px">

- Moving the ball among "corners":
<img src="https://raw.githubusercontent.com/schwertJake/Ball-On-Plate-Machine/master/Sample_Art/corners.gif" width="440px">

- Circular path:
<img src="https://github.com/schwertJake/Ball-On-Plate-Machine/blob/master/Sample_Art/circle_1.gif?raw=true" width="440px">
<img src="https://github.com/schwertJake/Ball-On-Plate-Machine/blob/master/Sample_Art/circle_2.gif?raw=true" width="440px">
