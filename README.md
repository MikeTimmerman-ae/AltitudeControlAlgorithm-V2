
# Altitude Control Algorithm V2

Proportional Integral Derivative (PID) control algorithm which controls the apogee of a rocket

The program is split up in three parts: dynamics, controller and simulator.
### Dynamics
The dynamics class contains all information about the rocket's dynamics and state. Using the 'step' class method, a control input is fed into the system and the output response of the system to this input is obtained by integrating the system's equations of motion using the RK45.  The parameters characterizing the rocket are all contained within its data members. Sensor noise and bias can also be set using the class methods. 

### Controller
The controller contains the structure of a PID controller. It can be configured to have multiple input and outputs as well as multiple inputs and one output. The gains of each input channel can easily be set are reset using the class methods. A reference time-varying trajectory can also be set using polynomial coeffcients or a reference point can be fed at each iteration, The class also contains a subclass, saturator , used to put limits on the controller output, as well as rate limits. Actuator noise and bias can also be added here.

### Simulator
The simulator class is used to simulate a closed-loop system interaction between the controller and the system as to simulate an actual rocket flight. It can also be used to tune the PID gains and to investigate the effect of variations in the initial conditions.

## Installation

The project uses cmake to compile and link the project. This means that the user should have cmake installed to run the code. \

Following steps will enable you to set-up and build the project:

1. Set-up directory and initialize with git:

```console

foo@bar:~$ git init

```

2. Clone this project into the directory:

```console

foo@bar:~$ git clone https://github.com/MikeTimmerman-ae/AltitudeControlAlgorithm-V2.git

```

3. (Skip step in case you have cmake) Download the CMake zip file from: https://cmake.org/download/ and unpack

4. Open the workspace file "Control Algorithm" in vs code

5. Install the Cmake Tools and CMake extensions

6. Go to File > Preferences > Settings > Extensions > CMake Tools > Cmake: Cmake Path\

Input the directory to the cmake.exe executable which is in ${install_directory}\bin and restart vs code

7. Delete the 'eigen' directory in the libraries directory and from git:

```console

foo@bar:~$ git rm -r --cached libraries/eigen

```

8. Install the submodule 'eigen':

```console

foo@bar:~$ git submodule add https://gitlab.com/libeigen/eigen.git libraries/eigen

```

9. Select a kit for compilation in the blue status bar at the bottom of vs code

10. Configure the build folder by pressing the play button in the blue status bar at the bottom of vs code

![image info](./data/vscodestatusbar.png)

11. Select the executable to execute: ${working_directory}\build\ControlSoftware.exe

12. Run code by again clicking the play button at the bottom of vs code