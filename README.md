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

* If controller can't find librbdl.so.2.6.0, Add following line to .bashrc 
```sh
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib'>>~/.bashrc
sudo ldconfig
```


## qpOASES setup
Download qpOASES [Link](http://www.qpoases.org/go/release) 
```sh
git clone https://github.com/saga0619/qpoases
mkdir build
cd build
cmake ..
make all
sudo make install
```


## Passwordless sudo for ROS launch
add folowinig line to /etc/sudoers, below the line (includedir /etc/sudoers.d)
```
$(username) ALL=(ALL) NOPASSWD: ALL
```
