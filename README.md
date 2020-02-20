# tocabi_controller

## mscl install 
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 
```sh
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
sudo dpkg -i c++-mscl_52.2.1_amd64.deb
```

## SOEM Setup
 ```sh
 git clone https://github.com/saga0619/SOEM
 cd SOEM
 mkdir build
 cd build
 cmake ..
 make all
 sudo make install
 ```

## RBDL Setup
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


## qpOASES setup
```sh
git clone https://github.com/saga0619/qpoases
cd qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```



## Custom Controller & Mujoco & Tocabi setup
Git clone https://github.com/saga0619/dyros_cc , https://github.com/saga0619/mujoco_ros_sim

```sh
cd ~/catkin_ws/src/
git clone https://github.com/saga0619/dyros_cc
git clone https://github.com/saga0619/mujoco_ros_sim
git clone https://github.com/saga0619/dyros_tocabi
```

## Simulation Mode 
```sh
roslaunch tocabi_controller simulation.launch
```

## Realrobot Mode
```sh
roslaunch tocabi_controller real
```
