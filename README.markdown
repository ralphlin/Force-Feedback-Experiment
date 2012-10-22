# Force feedback.cpp

Written January, 2009  
By Ralph Lin  
University of Washington  
Neurobotics Lab   

## Description:

This program is used to collect data for a human-in-the-loop
robotics experiment to characterize neural reflexes while
gripping an object. Briefly, the subject holds a custom
machined aluminum object attached to the end of a SensAble
Technologies Phantom 1.5 Haptic Robot. The subject's grip force
is measured with a fsr force sensor, while the subject's 
electrical muscle signals are recorded via EMG electrodes 
attached to the skin. 

Over the duration of the experiment, the robot will complete a number of trials where it applies a force to the gripped object in an attempt to "jerk" the object from the subject's grasp. The subject's reflex reaction to this motion (grip force and neural response) is recorded via force sensors placed on the object and EMG surface electrodes placed on the finger. These data were used to create artificial robotic
reflexes to help advanced prosthetics integrate with brain
computer interfaces for complex object manipulation (ICORR 2009).

## Notes:

This program was designed to run in Windows (WAIMEA) with the 
SensAble Phantom 1.5 robot, and NiDAQ cards to read force sensor 
and EMG data.
