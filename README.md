# AISRA-Code-Log
All the code required to run AISRA's robot, as well as previous iterations of code.

First iterations of code copied from: https://docs.google.com/document/d/1Mb3yHUOSwPBkBjCDocP3Y7CUVXcGVMLf7G2Typuetxg/edit?usp=sharing

Code explanation:

robot - main interface

IK_FK - calculates how to move the robot arm to a specific point in space (assumes the arm being controlled is motor actuated) (need to check)

compensator - takes the numbers from IK_FK and compensates for difference between motor actuated and belt driven (assumes the arm being controlled is belt driven) (need to check)

comm - ensures the arduino(s) are connected to the queen (assumes queen is laptop, later to be replaced with a raspi). Communicates to the arduinos how to move to the desired location.

arduino 1 - in charge of moving the base (joint 0)

arduino 2 - in charge of moving joints 1-3

arduino 3 - in charge of powering the electromagnet on the tool changer (joint 4)
