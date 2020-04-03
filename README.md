# DYROS TOCABI CONTROLLER 

# Building
## Prerequisites
### 1. mscl installation
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 
```sh
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
sudo dpkg -i c++-mscl_52.2.1_amd64.deb
```

### 2. SOEM installation
 ```sh
 git clone https://github.com/saga0619/SOEM
 cd SOEM
 mkdir build
 cd build
 cmake ..
 make all
 sudo make install
 ```

### 3. RBDL installation
```sh
git clone https://github.com/saga0619/rbdl-orb
cd rbdl-orb
mkdir build
cd build
cmake ..
make all
sudo make install
```

* If controller can't find librbdl.so.2.6.0, Add following line to .bashrc 
```sh
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>>~/.bashrc
sudo ldconfig
```


### 4. qpOASES installation
```sh
git clone https://github.com/saga0619/qpoases
cd qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```


### 5. Kvaser installation
```sh
wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
tar xvzf linuxcan.tar.gz
cd linuxcan
sudo make install
```


## Tocabi Controller installation
Git clone https://github.com/saga0619/dyros_cc , https://github.com/saga0619/mujoco_ros_sim

```sh
cd ~/catkin_ws/src/
git clone https://github.com/saga0619/dyros_cc
git clone https://github.com/saga0619/mujoco_ros_sim
git clone https://github.com/saga0619/dyros_tocabi
cd ~/catkin_ws/
catkin_make
```

## How to launch
### Simulation Mode 
```sh
roslaunch tocabi_controller simulation.launch
```
### Realrobot Mode
```sh
roslaunch tocabi_controller realrobot.launch
```
### Launch GUI alone
```sh
rosrun tocabi_gui tocabi_gui
```
### Monitor Tocabi Status from controller with RViz
```sh
roslaunch tocabi_description display.launch
```
### Monitor Tocabi Status from joint publisher with RViz
```sh
roslaunch tocabi_description display_with_joint_pub.launch
```


