# 3rd Japan Autonomous AI Challenge
[日本語](./README.md) | English

This repository provides the necessary data for the participants of the [3rd Japan Autonomous AI Challenge](https://www.jsae.or.jp/jaaic/en/index.html), such as environment construction procedures and competition rules.

Please refer to [RULE_EN.md](./RULE_EN.md) for the competition details and rules of this competition.

In this competition, you will implement an autonomous driving system based on Autoware.Auto.
Please refer to the following link for the tutorial of Autoware.Auto.
- [Introduction to Autoware.Auto for AI Challenge competitors (pdf)](doc/Introduction_to_Autoware_for_AI_Challenge_Competitors.pdf)
- [Introduction to Autoware.Auto for AI Challenge competitors (youtube)](https://www.youtube.com/watch?v=p8ay7KTOANM)

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
sudo apt install git-lfs
git lfs install --skip-repo

git clone https://github.com/AutomotiveAIChallenge/aichallenge2021
```

Please make sure pcd file is downloaded from LFS server.
```
ls -lh aichallenge2021/autoware/adehome/aichallenge_ws/src/aichallenge_launch/data/IndianapolisMotorSpeedway.pcd
```
The file size is about 300MB if it is normally downloaded.

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
Store → Vehicles → DallaraIL15, then press the "+" button.
![Screen](/image/vehicle.png)
Store → Plugins → Timer Sensor, then press the "+" button.
![Screen](/image/plugin.png)

### Adding Sensor Settings
Open the DallaraIL15 screen that you just added, and open "Sensor Configurations" from the button in the red circle in the image.
![Screen](/image/sensorconfig_open.png)
Press "Add New Configuration".
![Screen](/image/sensorconfig_addnew.png)
Set the name "Autoware.Auto", Bridge "ROS2", and press Apply.
![Screen](/image/sensorconfig_name.png)
Press the "Upload sensor configuration" in the red circle in the image.
![Screen](/image/sensorconfig_upload.png)
Select DallaraIL15_Autoware.Auto.json in this repository.
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

# Online Evaluation Environment
## Execution flow in the online environment during evaluation
To calculate the score, submit only the package `aichallenge_submit` from the web page of the online evaluation environment, and it will be scored automatically.
After the submission, the online evaluation environment will evaluate your work using the script under `evaluation/` in the following steps.

### (1) Placement of aichallenge_submit
The `aichallenge_submit.tar.gz` you uploaded will be placed under `evaluation/`.

### (2) docker build
Execute `evaluation/build.sh` to create the docker image defined in `evaluation/Dockerfile`. The procedure to create this image is as follows:

1. Place the pre-built Autoware provided by `binary-foxy:1.0.0` to `/opt/AutowareAuto`, and the source provided by `autoware/adehome/aichallenge_ws` in this repository to `/opt/aichallenge_ws`.
2. Install `ros-foxy-lgsvl-bridge` and `lgsvl/PythonAPI`.
3. Extract the submitted `aichallenge_submit.tar.gz` to `/opt/aichallenge_ws/src/aichallenge_submit`.
4. Run `rosdep install` and `colcon build`.

### (3) Simulation execution
In the online evaluation environment, simulator will be launched and API mode simulation will be started.

On the same machine, `evaluation/run.sh` will be executed, and the docker container will be launched and scored. In the container, the following is done by running `evaluation/main.bash`.
1. Start recording rosbag.
2. Launch lgsvl_bridge
3. Launch the ROS2 nodes
4. Run the scenario

The actual scoring procedure and the commands to be executed are the same as in `evaluation/main.bash`, except that the following procedures are added: automatic startup/shutdown of the simulator, score acquisition, and rosbag upload.

If you run it with `evaluation/run.sh`, the rosbag and runtime log (output of ros2 launch) will be saved under `evaluation/output`.

#### Scenarios to be run
In the online evaluation environment, the following two scenarios will be executed.

- Distributed scenarios (`scenario.train.py`)

    In order to be able to verify the behavior in the online evaluation environment, the distributed scenario will also be run, and the rosbag and runtime logs can be obtained from the web page. The time for this scenario is not related to the ranking.

- Scenarios for evaluation (undisclosed)

    This is a scenario for time evaluation that is not distributed. The output of the time topic when this scenario is run is recorded and used as the time to determine the ranking. Neither the rosbag nor the runtime log can be downloaded.

## Procedure for submitting source code to the online evaluation environment
### (1) Compress your source code.
Use `autoware/adehome/aichallenge_ws/src/aichallenge_submit/create-tar-file.sh` to compress the source code in `aichallenge_submit`.

```sh
cd autoware/adehome/aichallenge_ws/src/aichallenge_submit/
./create-tar-file.sh
```

Make sure that a compressed file is created in `autoware/adehome/aichallenge_ws/src/aichallenge_submit.tar.gz`.

### (2) Make sure that scenario can be executed automatically in docker inside `evaluation/`.
Before uploading to the online evaluation environment, please confirm that you can build and execute in a Docker container similar to the online environment using the local environment by following the steps below.

First, place the file `aichallenge_submit.tar.gz` created in (1) under `evaluation/`. The file structure is as follows.
```
evaluation/
|-- Dockerfile
|-- aichallenge_submit.tar.gz
|-- build.sh
|-- main.bash
`-- run.sh
```

- `evaluation/output/` will store the rosbag and score records from the run. Remove any directories created by previous runs.
- In `evaluation/bringup/`, the contents of this repository (the part you need) will be stored by `./build.sh`.

Next, build the docker image that contains the `aichallenge_submit` you created.
```sh
./build.sh
```

After the build is complete, start the simulator as described in this README and start the API mode simulation. After confirming that the message "API ready!" is displayed, launch the docker container and run the scoring flow by `run.sh`.
The environment variable `LG_VEHICLE_ID` used in `run.sh` should be set to the configuration id obtained when configuring the sensor in this README.
```sh
export LG_VEHICLE_ID='{configuration id you got}'
./run.sh
```

Finally, check the score output in `evaluation/output/score.json`. The rosbag and runtime logs are also output under `evaluation/output/`, so please refer to them for debugging.

### (3) Upload the source from the online evaluation environment web page

After logging in to the [web page](https://aichallenge21.tier4.jp/), follow the instructions on the screen to upload the file `aichallenge_submit.tar.gz` created in (1).

After the upload is finished, the source build and simulation will be executed in order.

- If the simulation is completed successfully, the message `Scoring complete` will be displayed, and the time for both the distribution scenario and the evaluation scenario will be displayed. In addition, you can download the same rosbag and runtime log that is output in `evaluation/output/` for the distribution scenario from the link shown below the time. The time of the last uploaded evaluation scenario will be used as the final time in the ranking.
- Even if the scenario finishes successfully, it will be displayed as `No result` if there is no score output due to launch failure, or `Checkpoint not passed` if not all checkpoints have been passed.
- If the build fails, `Build error` will be displayed. Please follow steps (1) and (2) to check again that the Docker image can be built.
- If the simulator fails to run, the message `Simulator error` will be displayed. In this case, there may be an internal error on the server side, so please upload the file again. If this happens repeatedly, please contact us.

Please note that you cannot upload new sources while evaluation is in progress. Uploading is limited to three times a day and will be reset at midnight Japan time.

# Contact us

## Regarding notifications of updates, etc.
Whenever there is an update to github, we will make a new comment on the issue at the following URL.

If you subscribe to this issue, you will be notified when it is updated (please turn on notifications).

https://github.com/AutomotiveAIChallenge/aichallenge2021/issues/1

## Regarding acceptance of inquiries
If you have any questions about the competition, repository contents, etc., please contact us via the issue on github. Questions can be asked in either English or Japanese. Please close the issue when your question is resolved.

Questions must be directly related to the content of the competition. Questions regarding the use of the software will not be answered in the interest of fairness.

Questions on each issue will basically be answered within two business days. However, please understand that it may take more than two business days if the question requires time to be reviewed or if there are many questions.
