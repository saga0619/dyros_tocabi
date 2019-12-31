# tocabi_controller

## SOEM Setup
 * install [SOEM](https://github.com/saga0619/SOEM)
 ```sh
 git clone https://github.com/saga0619/SOEM
 cd SOEM
 mkdir build
 cd build
 cmake ..
 make all
 sudo make install
 ```

## mscl install 
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 

```sh
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
sudo dpkg -i c++-mscl_52.2.1_amd64.deb
```

## RBDL Setup
### Installing
```sh
git clone https://github.com/saga0619/rbdl-orb
mkdir build
cd build
cmake ..
make all
sudo make install
```

* If red controller can't find librbdl.so.2.6.0, Add following line to .bashrc 
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```


## qpOASES setup
Download qpOASES [Link](http://www.qpoases.org/go/release) 
```sh
cd qpOASES-3.2.1
mkdir build
cd build
cmake ..
make all
sudo make install
```

### qpOASES error handling
if error occures, add following line to qpOASES-3.2.1/CMakeLists.txt, below PROJECT(qpOASES CXX), which is line 34

```
add_compile_options(-fPIC)
```

## Dyros Red Controller Setup
Download 1.0 version from release list. [1.0 release](https://github.com/saga0619/dyros_red/releases)
Unzip at ros worksapce (ex:catkin_ws)

## ncurse Setup
```sh 
sudo apt-get install libncurses5-dev
```

## Passwordless sudo for ROS launch
add folowinig line to /etc/sudoers, below the line (includedir /etc/sudoers.d)
```
username ALL=(ALL) NOPASSWD: ALL
```
