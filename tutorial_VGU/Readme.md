### This is tutorial to run the package DriverlessCarChallenge 
## Installation
1. install opencv (https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)
or 
``` sh
sudo apt-get install python-opencv
```
2. install GNU C Compiler 
``` sh
sudo apt-get install g++
```

3. install library for developer
``` sh
sudo apt-get install libopencv-dev
```
4. install cmake 
```sh
sudo apt-get install cmake
```
## Cmake the package
``` sh
cd ~/tutorial_VGU
mkdir build
cmake ../
make
```
## Running package
```sh
cd ~/tutorial_VGU
./test_radon
```
