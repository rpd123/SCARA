# SCARA  
  
Instructable: https://create.arduino.cc/projecthub/ricpd/scara-chess-robot-a45793  
    
Arduino code is in https://github.com/rpd123/Arduino-Firmware    
Python code is at https://github.com/rpd123/chess-robot  
  
It uses a mini gripper:  
https://www.thingiverse.com/thing:4394894/files  
manipulator.stl is not required
   
This is for a modified version of pyBot. See original at:  
https://www.instructables.com/Pybot-Python-3D-Printed-Robotic-Arm/ 
  
We have the pyBot stlfiles here. The change from the original is to have a longer shank1 and a replaced shank2 (as well as a different gripper).  
      
Drill through the hub and use a longer screw  
    
The longer shank1 requires a 400mm belt.  
    
Note: this modification is for playing chess, where the load is very light.  
This modification allows shank2 to turn right back against shank1, and so allows the robot to stand next to the chessboard and reach near and far squares.

WIRING

Shank1 goes from the shoulder to the elbow, and shank2 goes from the elbow to the gripper.

Connect:
X controller to motor which moves shank2
Y controller to motor which moves shank1
Z controller to motor which moves the robot vertically

In this diagram
https://github.com/rpd123/community_robot_arm/blob/master/Assembly%20%26%20Control%20Guides/Wiring%20Guide/wiring-lower%20motor.jpg

Controllers are X, Y, Z from left to right.  
