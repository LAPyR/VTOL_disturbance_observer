# VTOL_disturbance_observer


This repository shows the modified files from the original PX4 firmware that contain the control algorithm described in the summitted paper. The modification consists in the implementation of a disturbance observer to compensate external disturbances added to the UAV system. We have tested the algorithm in a classical VTOL system.

In order to test the proposed algorithm is necessary to follow the next steps:

* Clone the PX4 firmware from https://github.com/PX4/PX4-Autopilot.
* Compile it according to https://docs.px4.io/master/en/dev_setup/building_px4.html.
* Paste the .cpp and hpp files above (4 from this repository) to replace the old ones.
* Compile the firmware again.

Then, the compiled firmware can be run through a SITL simulation following the steps in 
https://docs.px4.io/master/en/simulation/gazebo_vehicles.html.

The vehicle that need to be tested is the default quadrotor with the command: make px4_sitl gazebo

Further information about uploading the modified firmware into a pixhawk-based autopilot (for real flight tests) can also be found at
https://docs.px4.io/master/en/dev_setup/building_px4.html.
