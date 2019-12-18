# tocabi_controller

## SOEM Setup
 * install [SOEM](https://github.com/saga0619/SOEM)


## mscl install 
 * download [MSCL](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) 

```sh
dpkg -i c++-mscl_52.2.1_amd64.deb
```

## RBDL Setup
### Installing
```sh
wget https://bitbucket.org/rbdl/rbdl/get/default.zip
unzip default.zip
cd rbdl-rbdl-0879ee8c548a
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```

### RBDL Error handling
* If any error related to ROS occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc and remove following line
```cpp
#include <ros.h>
```
* If error " 'boost' does not name a type" occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc and edit boost::shared_ptr to std::shared_ptr. (line 15~18)
```cpp
typedef std::shared_ptr<urdf::Link> LinkPtr;
typedef const std::shared_ptr<const urdf::Link> ConstLinkPtr;
typedef std::shared_ptr<urdf::Joint> JointPtr;
typedef std::shared_ptr<urdf::ModelInterface> ModelPtr;
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
