#### Environments
```
sudo apt-get install python-pip
sudo apt-get install chromium-chromedriver
sudo pip install transforms3d selenium matplotlib gmplot
```

#### Read odom in world coordinates , plot it in Google Map, then Save the trajectory in the dir.
##### Read a bag and save the odom's trajectory in the bag's dir.
```
source devel/setup.bash
rosrun read_bag_plot_in_google_map.py -f /media/test.bag /sensor/novatel/odom
```
##### Read all bag in the dir and save the odom's trajectory in the bag's dir.
```
source devel/setup.bash
rosrun read_bag_plot_in_google_map.py -d /media/test /sensor/novatel/odom
```
#### Read two kinds of odom in world coordinates , plot their XYZ and RPY comparison figure, then Save them in the dir.
##### Read a bag and save the figure in the bag's dir.
* the args 0 0 0 0 0 0 is the odom1 to odom2's extrinsic X Y Z Roll Pitch Yaw, and using radian
```
source devel/setup.bash
rosrun read_bag_plot_in_google_map.py -f /media/test.bag  /sensor/novatel/odom  /pose_optimize/velodyne/odom 0 0 0 0 0 0
```
##### Read all bag in the dir and save the figure in the bag's dir.
```
source devel/setup.bash
rosrun read_bag_plot_in_google_map.py -d /media/test /sensor/novatel/odom /pose_optimize/velodyne/odom 0 0 0 0 0 0
```
