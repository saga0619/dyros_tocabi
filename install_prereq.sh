#!/bin/bash


echo "Dyros Tocabi Auto Installer"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

while true; do
    read -p "Do you wish to install Dyros Tocabi Prerequisities? " yn
    case $yn in
        [Yy]* ) echo "Starting Install ..."
                mkdir Temp
                echo "Downloading mscl ... "
                cd Temp
                wget https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb
                dpkg -i c++-mscl_52.2.1_amd64.deb
                
                git clone https://github.com/saga0619/SOEM
                cd SOEM
                mkdir build
                cd build
                cmake ..
                make all
                sudo make install
                cd ../..

                git clone https://github.com/saga0619/rbdl-orb
                cd rbdl-orb
                mkdir build
                cd build
                cmake ..
                make all
                sudo make install
                cd ../..

                git clone https://github.com/saga0619/qpoases
                cd qpoases
                mkdir build
                cd build
                cmake ..
                make all
                sudo make install
                cd ../..

                wget --content-disposition "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
                tar xvzf linuxcan.tar.gz
                cd linuxcan
                make all
                sudo make install
                cd ..
                
                wget http://mirror.yongbok.net/gnu/gsl/gsl-2.6.tar.gz
                tar xvzf gsl-2.6.tar.gz
                cd gsl-2.6
                ./configure
                make
                sudo make install
                
                cd ../..

                sudo rm -rf Temp
                echo "Run 'git clone https://github.com/saga0619/dyros_cc & git clone https://github.com/saga0619/mujoco_ros_sim'"
                break;;
        [Nn]* ) echo "Aborting ...";
                exit;;
        * ) echo "Please answer yes or no.";;
    esac
done
