Use the following links and follow the steps for setting up the packages you need to run the D-435 camera on your system.

https://github.com/IntelRealSense/realsense-ros#installation-instructions

In the event of an SDK2 error, try installing this.

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

Additionally, in the event of you trying to run the D-435, it doesn't work, the first thing I would recommend troubleshooting is making sure your system can use your USB port. Just do "lsusb" to see which port has `lsusb`.  You should see some result like "Bus 002 Device 003: ID 8086:0b3a Intel Corp. ". Might be a different USB port. Whichever one it is, just do `sudo chmod o+w /dev/bus/usb/00X/00X` and in place of the Xs put your actual port numbers.

After all that, it should work. Try running 

`roslaunch realsense2_camera demo_pointcloud.launch`

You should get an excellent little demo to play around with. If something else breaks, good luck - you're on your own.