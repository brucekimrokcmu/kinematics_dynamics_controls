# kinematics, Dynamics and Controls

## **1. Kinematics**

**Unit Quaternions - tracking rigid bodies** <br>
**Planar Rigid Body Transformations using screw theory in the plane R^2**<br>
**Forward kinematics**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181958228-be78bb29-cb91-4e84-b385-7215552e5f96.JPG" width=20% height=20%>
<img src="https://user-images.githubusercontent.com/92174982/181962088-d7914f05-8ad7-45c0-baf0-4691490d2887.jpg" width=40% height=40%>
</p><br>


## **2. Manipulator Kinematics**<br>

**Forward Kinematics, Inverse Kinematics, Jacobian**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181963219-5a97ee0e-4e8d-44b8-b8d4-cddd9ac72f1a.JPG" width=30% height=30%>
</p><br>

**Force a manipulator can generate and forces that it can resist**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181964566-648a54aa-891a-4951-b802-9a101a097d37.JPG" width=40% height=40%>
</p><br>

**Numerical techniques for solving the Inverse Kinematics problem of redundant manipulators with singular configurations**<br>
  ***code only***<br>
**Virtual model control for mobility and manipulation platforms, Parallel chains**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181970599-9ba4445c-fbb0-42f0-a101-4c2cacaf3139.JPG" width=40% height=40%>
<img src="https://user-images.githubusercontent.com/92174982/181983711-485fae10-3419-4a14-aa3f-903aa4fe217e.jpg" width=40% height=40%>
<img src="https://user-images.githubusercontent.com/92174982/181984117-20a3b7b5-fc71-4bf0-9fd0-f6cf893813f3.jpg" width=40% height=40%>
</p><br>

## **3. Rigid Body Dynamics**

**Lagrange method for deriving the EOP of dynamic systems with constraints**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181991346-c2f0d8e8-2aa9-4f0b-99db-7a31131a881b.JPG" width=30% height=30%>
<img src="https://user-images.githubusercontent.com/92174982/181991795-7aeeef3f-2826-45b4-97ae-61b8ead2b9da.JPG" width=30% height=30%>
</p><br>  
  
**Calculating the inertia matrix of rigid bodies with known shapes**<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181994407-76ffa25d-235d-4e62-a131-7c72ace2c2f7.JPG" width=30% height=30%>
</p><br>

**Dynamic parameter identification**<br>
Generating a precise dynamic model of an approaching meteorite to predict its future trajectory based on a small localization information about its inertial pose for a minute at a frequency of 1HZ from the beacon installed on the surface of the asteroid.<br> 
***code only***<br>

## **4. Manipulator Dynamics**<br>
Equations of motion for complicated robot mechanisms <br>
***code only***<br>

## **5. Control Fundamentals**<br>
**LTI systems and feedback design**<br>
State space model for motor-load coupling through series spring system<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181994834-82d8ec77-6d27-49f2-9f6d-a94a69e24f5d.JPG" width=30% height=30%>
<img src="https://user-images.githubusercontent.com/92174982/181994915-d0f88c2b-f780-446c-a9a6-0bdb7c22fd31.JPG" width=30% height=30%>
</p><br>
Designing a state feedback that gives a closed loop system with certain eigenvalues - damping the oscillatory eigenvalues<br>

<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181995265-56dd1f4f-9037-4722-a618-f6c5515ec430.jpg" width=30% height=30%>
<img src="https://user-images.githubusercontent.com/92174982/181995352-a9e921bd-59a3-4791-bf90-b4bb91d4f1f2.jpg" width=30% height=30%>
</p><br>
yet, SEA has higher challenge in attaining covergence with distrubance torque<br>

**Filtering methods to cope with noisy signals**<br>
Estimating position and velocity of a Micro-UAV in a noisy environment<br>
<p float="left">
<img src="https://user-images.githubusercontent.com/92174982/181995521-1eb17086-4cd5-4bc4-85df-03256b17ad2c.jpg" width=30% height=30%>
</p><br>

## **Personal Project - Tree modelling using mass-spring-damper system**
Modeling Trees as Kinematic Stucture with Spring & Damper System - Moonyoung Lee (moonyoul), Kwang Kyun Kim (kwangkyk)<br>

>Abstract— There is a growing need for automation in
agriculture as reports of labor shortages increase. However,
using robots to harvest from trees is challenging because the
dynamics models of trees are often unknown, which can lead
to unsafe robot interaction with the tree. We propose to model
a tree as a series of kinematic chain similar to a robot arm.
With this model simplification, branches can be modeled as
a rigid link where motions are then constraint to revolute
motion around its joints. We then observe tree’s response to
an applied external disturbance in simulation. We investigate
how the response of the spring-damping tree model system
varies as we increase the number of branches in a tree. Our
experimental results show that fully grown tree with 13 links
reach steady state in 43% of the time compared to that
of a less mature tree with only 7 links when an external
disturbance is applied.

