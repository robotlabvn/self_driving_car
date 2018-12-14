### Read Me file for package traffic_sign_detection
1. git clone this package
2. Dowload the Dataset available on this link: https://www.kaggle.com/robotlabvn/trafficsigndetectiontrainimgs
3. Unizip and Copy the *Train_Imgs* folder to the directory: /catkin_ws/src/traffic_sign_detection/Train_Imgs
4. Open the Car Simulation Environment
```
cd ~/<car_simulattion folder>/
./test.x86_64
```
 ![alt text](https://github.com/robotlabvn/self_driving_car/blob/master/traffic_sign_detection/Image/Simulation_1.png)
4. Running the package
 ```
 cd ~/catkin_ws
 catkin_make
 roslaunch traffic_sign_detection detection_sign.launch
```

