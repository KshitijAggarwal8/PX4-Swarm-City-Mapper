# Set up a PX4 development environment on Ubuntu in the normal way
```bash
# set up the PX4 environment in your root file
cd ~/

# git clone the PX4 repo
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# To run your PX4 in your original simulator (Gazebo classic)
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

cd PX4-Autopilot/

make px4_sitl

# Downgrade for PX4 environment to run properly in your ubuntu
pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# to launch an example environment with 4 drones
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 4
```

# Set up XRCE-DDS for the topics
```bash
# in a new terminal 
# clone this in your root as well
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

# open to Micro-XRCE-DDS-Agent folder
cd Micro-XRCE-DDS-Agent 

# create a build folder 
mkdir build && cd build

# build
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

# to begin MicroXRCEAgent
cd Micro-XRCE-DDS-Agent 
MicroXRCEAgent udp4 -p 8888
```

# to run your custom file
```bash
# create your workspace 
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
# clone the px4_msgs and example folder
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/mun0404/PX4-Swarm-City-Mapper.git

# this is where you extract our file and paste it
# to build
cd ..

cp ~/PX4-Swarm-City-Mapper/src/px4_swarm_contoller/worlds/postoffice.world ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/

source /opt/ros/humble/setup.bash
colcon build

# to run our environment(default = 5 drones)
ros2 launch final_project multi_drone_controller.launch.py

# to change the number of drones
ros2 launch final_project multi_drone_controller.launch.py nb_vehicles:=<number of drones>
```

# after every simulation remeber to kill the gzserver and gzclient
```bash
killall gzserver gzclient
```

# for any other reference vist
[PX4 documentation](https://docs.px4.io/main/en/ros2/user_guide.html#install-px4)