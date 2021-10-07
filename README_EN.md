# 3rd Japan Autonomous AI Challenge
[日本語](./README.md) | English

This repository provides the necessary data for the participants of the [3rd Japan Autonomous AI Challenge](https://www.jsae.or.jp/jaaic/en/index.html), such as environment construction procedures and competition rules.

Please refer to [RULE_EN.md](./RULE_EN.md) for the competition details and rules of this competition.

## System Requirements
### Running both LGSVL simulator & Autoware in one computer
- OS: Ubuntu 20.04
- CPU: Intel Core i7(8 cores) or higher
- GPU:
  - NVIDIA Geforce RTX2080(GTX1080Ti) or higher (when implementing a ROS node using GPU functions)
  - NVIDIA Geforce GTX1080 or higher (when implementing a ROS node without GPU functions)
- Memory: 32GB or more
- Storage: SSD 30GB or more

When running LGSVL simulator & Autoware in separate computer, system requirements are as below.

### Running LGSVL Simulator
- OS: Ubuntu 20.04 or Windows 10
- CPU: Intel Core i7 (4 cores) or higher
- GPU: NVIDIA Geforce GTX 1080 or higher
- Memory: 16GB or more
- Storage: SSD 20GB or more


### Runnning Autoware
- OS: Ubuntu 20.04
- CPU: Intel Core i7 (4 cores) or higher
- GPU: NVIDIA Geforce GTX 1080 or higher (GPU is not required when implementing a ROS node without GPU functions)
- Memory: 16GB or more
- Storage: SSD 10GB or more


## Setup Autoware Environment(ROS2 Environment)
The ROS2 execution environment for this event will be built in a Docker container.

In advance, please install
- [Docker](https://docs.docker.jp/linux/index.html)
  - In order to run without sudo, please follow [this](https://docs.docker.com/engine/install/linux-postinstall/) instruction after installation.
  - If this procedure is not done correctly, the following error will occur during the execution of ade.
```
Got permission denied while trying to connect to the Docker daemon socket at ...
```
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#install-guide)
- [ADE](https://ade-cli.readthedocs.io/en/latest/)

### Clone This Repository
Please install [git lfs](https://packagecloud.io/github/git-lfs/install) beforehand.

```
git clone https://github.com/AutomotiveAIChallenge/aichallenge2021
```

### Install ROS2+Autoware.Auto
Autoware.Auto is recommended to be set up in a Docker environment using ADE.(https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html)

Since lgsvl_bridge is needed in addition to the official Docker files, we provide the Docker files for ADE in this repository.

To install,
```
bash setup_autoware.sh
```

This will add the Autoware.Auto (1.0.0) Docker image and set`autoware/adehome`as the home directory for ADE.
You can also set up Autoware.Auto in your local environment without using the above script, but please use Autoware.Auto version 1.0.0.


### Run ADE Container

```
# In autoware/adehome,

# Run ADE container
ade start --update
# Enter the ADE container
ade enter
```

## Setup Sample Code(ROS2 Package)

### About the Sample Code
Participants will be asked to create a ROS2 package to carry out the assigned scenario.
The following ROS package is provided in this repository in `autoware/adehome/aichallenge_ws/src` as a sample code to be used as a base.
- aichallenge_launch
  - Contains the main launch file `aichallenge.launch.py`. All ROS2 nodes will be launched from this launch file.
- aichallenge_eval
  - Package for score calculation.
- aichallenge_msgs
  - Contains message definitions.
- vehicle_pose_publisher
  - Since the point cloud map we will be running on has few features and NDT(Normal Distributions Transform) matching is easy to lose self-position, we have provided a ground truth data of localization.
  - You can get vehicle pose by subscribing `/aichallenge/vehicle_pose`.
- aichallenge_submit
  - You can freely change the contents of this directory.
  - Please submit only the contents of this directory. So, all ROS2 packages implemented by the participants should be placed in this directory. The following packages are already included.
  - aichallenge_submit_launch
    - `aichallenge_submit_launch.launch.py` will be called from the main launch file `aichallenge.launch.py`, so please modify this launch file appropriately to launch your ROS2 node.
  - aichallenge_sample
    - This is a sample implementation of autonomous driving.
    - It is set to follow the center of the lanelet at a maximum of 35km/h(approximately 21.7mi/h). Vehicle avoidance is not configured, and the vehicle will brake when another vehicle approaches in front.
  - sample_localizer
    - This package performs localization based on `/aichallenge/vehicle_pose`.

### Build the Sample Code
```
# In the ADE container,
source /opt/AutowareAuto/setup.bash
cd aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```

Please place the ROS2 package you created under `aichallenge_ws/src/aichallenge_submit` so that it can be built by the above procedure.

### Start the Sample Code
```
# In the ADE container,
source /opt/AutowareAuto/setup.bash
source ~/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.py
```
This will launch rviz, but for full operation, it needs to communicate with the simulator using lgsvl_bridge. Please open another terminal and launch lgsvl_bridge with the following command.
```
# Enter the ADE container
ade enter

# In the ADE container
source ~/aichallenge_ws/install/setup.bash
lgsvl_bridge
```
Up to this point, the setup and execution of the Autoware side is complete. If the setup is done correctly, the vehicle model and lanelet will be displayed in rviz.
![Screen](/image/rviz.png)

## Setup Simulator
### Download the Simulator Binary
Download the binary of the simulator from the following link.

[simulator.zip](https://drive.google.com/file/d/1Q-MVYxtuSPMz8obmvA6FwbuaP6qJA0ul/view?usp=sharing)

The zip file includes both simulator for Linux/Windows.
- simulator_linux
- simulator_windows
Please appropriate one according to your environment.

*Note: Disclaimer about this simulator
We do not assume any liability or responsibility for any loss or damage arising from downloading this file.*

### Sign Up for LGSVL Simulator
https://wise.svlsimulator.com/sign-in
Register and sign-in to your LGSVL account from the link above.

### Cluster Registration
Launch the downloaded binary.
![Screen](/image/initial.png)
Clicking on the "LINK TO CLOUD" button, a web browser will be open and the cluster registration page will appear.
Then enter an appropriate name and click "Create cluster".
![Screen](/image/cluster.png)

### Maps/Vehicles/Plugins Registration
Store → Maps → IndianapolisMotorSpeedway, then press the "+" button.
![Screen](/image/map.png)
Store → Vehicles → DallaralL15, then press the "+" button.
![Screen](/image/vehicle.png)
Store → Plugins → Timer Sensor, then press the "+" button.
![Screen](/image/plugin.png)

### Adding Sensor Settings
Open the DallaralL15 screen that you just added, and open "Sensor Configurations" from the button in the red circle in the image.
![Screen](/image/sensorconfig_open.png)
Press "Add New Configuration".
![Screen](/image/sensorconfig_addnew.png)
Set the name "Autoware.Auto", Bridge "ROS2", and press Apply.
![Screen](/image/sensorconfig_name.png)
Press the "Upload sensor configuration" in the red circle in the image.
![Screen](/image/sensorconfig_upload.png)
Select DallaralL15_Autoware.Auto.json in this repository.
![Screen](/image/sensorconfig_select.png)
Press the "Save" button.
![Screen](/image/sensorconfig_save.png)


### Set and Run Simulation
Click Simulations → Add New, then "Select Cluster" to choose what you registered.
![Screen](/image/setting1.png)
Press Next, then select "API Only" as Runtime Template.
![Screen](/image/setting2.png)
Select Next → Next → Publish, and after the window is shown, press "Run Simulation". Then simulation will be run.
![Screen](/image/simulation_run.png)
"API Ready!" will be displayed on the simulator window.
![Screen](/image/api_ready.png)

## Scenario Execution & Time Acquisition
The execution of the scenario in the simulator (adding your own vehicles, controlling the movement of NPC vehicles, loading the map, etc.) is done by [python script](./scenario/scenario.train.py).

Before running the scenario, please check the following:
- The simulator
  - must be in the API Ready state.
- ROS2
  - The `aichallenge.launch.py` should be launched and rviz should be displayed.
  - `lgsvl_bridge` is running.

### Setup LGSVL Simulator Python API
The Python API provided by LG is used to run the simulator scenario. We recommend to run the scenario on the same computer that run the simulator.

```
git clone https://github.com/lgsvl/PythonAPI
cd PythonAPI
git checkout refs/tags/2021.2
```

Install PythonAPI following the README.

### Run Scenario
First, get the configuration id of the sensor settings(Autoware.Auto) that you added in the [Adding Sensor Settings](#Adding Sensor Settings) section.
Click the button in the red circle in the image to copy it to the clipboard.
![Image](/image/configuration_id.png)

```
cd scenario
# Enter the IP address of the machine for Autoware for bridge and the IP address of the machine for simulator for host.
python3 scenario.train.py --vehicle_id={configuration id which you get} --bridge=127.0.0.1 --host=127.0.0.1
```

### Time Acquisition

Please check  the [RULE_EN.md](./RULE_EN.md) for the competition details.

Time is published to the `/aichallenge/score` topic.
You can check it as follows.

After you start the simulator, you can check by echoing the topic.
```
source ~/aichallenge_ws/install/setup.bash
ros2 topic echo /aichallenge/score
```

The timing for publishing is as follows
1. When reaching the goal.
2. When scenario.train.py is executed and 5 minutes has passed.

The time is calculated when `/aichallenge/score` is published.

The contents of `/aichallenge/score` are as follows.
- time : Time including penalties. This value will be the official time in the competition.
- rawTime : The time took to reach the goal.
- hasFinished : Turn to be 1 when reaching the goal, 0 otherwise.
- contactPenalty : Penalty for vehicle collision. (5 seconds will be added to the time for each collision)
- trackLimitPenalty : Penalty for going off course.(The time spent off the course is added to the time.)

# Contact us

## Regarding notifications of updates, etc.
Whenever there is an update to github, we will make a new comment on the issue at the following URL.

If you subscribe to this issue, you will be notified when it is updated (please turn on notifications).

https://github.com/AutomotiveAIChallenge/aichallenge2021/issues/1

## Regarding acceptance of inquiries  
If you have any questions about the competition, repository contents, etc., please contact us via the issue on github.  
