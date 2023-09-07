# Zephyr Drone

![alt text](https://github.com/Luup850/Zephyr-drone/blob/main/Docs/frontpage.jpg?raw=true)

Master thesis project at Technical University Denmark

This project is a fork of the http://rsewiki.elektro.dtu.dk/index.php/Drone_control project, originally made by Jens Christian Andersen at DTU.



## Drone Software
All the code for the drone can be found in `Zephyr-drone/drone/`

The code running on the Teensy is found in `Zephyr-drone/drone/drone_ctrl` which is a modified version of the original logic for the onboard teensy. Follow Jens tutorial on the previously mentioned wiki, but overwrite the drone_ctrl folder.


`Zephyr-drone/drone/mission` contains the code for the mission running on the Raspberry Pi. To be able to recompile the project, it is nessecary to install OpenCV with contributions.

## Matlab Model
Original model was made by Jens Christian Andersen. However, the model had to be modified a bit and is therefore in this repo as an updated version

The Matlab model for the drone is found in `Zephyr-drone
/matlab/`, together with other useful scripts.

`hexacopter.m` contains all the parameters both for the discretized model (`hexacopter_sim_discrete.slx`) and the continuos time model (`hexacopter_sim.slx`), and needs to be executed before any of the models.
## Other Folders
The python folder contains scripts used to convert the logged data on the drone to plots.
Furthermore, it also contains the script used for callibrating the camera on the drone.

The Docs folder mainly contains plots used in the presentation and the thesis, together with some videos of the drone.
