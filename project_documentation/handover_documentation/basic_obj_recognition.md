## Basic object recognition

The following tutorial was very useful in setting up a basic image recognition and play around with it. Do not forget to select /usb_cam/image_raw to get the  https://sudonull.com/post/14732-Detection-and-recognition-of-objects-from-the-camera-in-ROS-using-the-package-find_object_2d

#### What can go wrong?

**E: Unable to locate package ros-kinetic-find-object-2d**

You need to set up your sources.list and keys to get software from [packages.ros.org](http://packages.ros.org/). Following the instructions, you should be able to make it work.

```jsx
sudo sh -c 'echo "deb <http://packages.ros.org/ros/ubuntu> $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

**E: Unable to correct problems, you have held broken packages. (cause: broken catkin)**

The following packages have unmet dependencies: catkin : Depends: python-catkin-pkg but it is not going to be installed E: Unable to correct problems, you have held broken packages.

```jsx
sudo apt-get install ros-kinetic-catkin
source /opt/ros/kinetic/setup.bash //source the ROS environment
catkin_make
```

**[rospack] Error: package 'uvc_camera' not found**

Install the uvc-camera package for your ros distribution

```jsx
sudo apt-get install ros-kinetic-uvc-camera
```