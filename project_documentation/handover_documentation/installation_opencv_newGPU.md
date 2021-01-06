This was done with a Nvidia 3080 and Ubuntu 20.04. 
OpenCV version 5.4.1
CUDA 11.0
Nvidia Driver 455.38

3080 has ARCH_BIN of 8.6 but that did not work, so I changed it to 8.0 as that is the only other supported version it has.

Installing CUDA for Tensorflow:
https://www.tensorflow.org/install/gpu

Download the toolkit using the runfile (The debian did not work for me and the runfile works just as well): https://developer.nvidia.com/cuda-toolkit-archive
At the time of writing only CUDA 11.0 works, as CUDA 11.1 updates some ".so" from version 10.1 to 11.1

After installing CUDA, be sure to also install cuDNN: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html
The only error I had with this is that it was not recognizing some symlinks, this was fixed by executing this command for every link that was missing:
sudo rm libcudnn_adv_train.so && sudo rm libcudnn_adv_train.so.8 && sudo ln -s libcudnn_adv_train.so.8.0.5 libcudnn_adv_train.so.8 && sudo ln -s libcudnn_adv_train.so.8 libcudnn_adv_train.so
Ofcourse replace the name of the file with the name of the file that was missing and the version number with the correct version number.
The version number can be found by doing a ls <name>.so* as it will list all the files with that name.
sudo rm <name>.so && sudo rm <name>.so.<major_version> && sudo ln -s <name>.so.<long_version> <name>.so.<major_version> && sudo ln -s <name>.so.<major_version> <name>.so

After fixing the symlinks, can also be used to see which ones are broken:
 sudo ldconfig
 sudo reboot

Follow the installation instructions for CuDNN and install the sample.
If this shows that there are missing packages, install both the runtime library and the development library for your CuDNN version.
Go to section 2.4 to see how to test the sample, if this worked, you have a working installation of CuDNN.

After installing Cuda build the "deviceQuery" sample, I built all samples but that sample is very useful. It shows information about the CUDA enabled device that you are using.
   ~/NVIDIA_CUDA-11.0_Samples/bin/x86_64/linux/release/deviceQuery

Example important output:
~/NVIDIA_CUDA-11.0_Samples/bin/x86_64/linux/release/deviceQuery Starting...

    CUDA Device Query (Runtime API) version (CUDART static linking)

   Detected 1 CUDA Capable device(s)

   Device 0: "GeForce RTX 3080"
     CUDA Driver Version / Runtime Version          11.1 / 11.0
     CUDA Capability Major/Minor version number:    8.6
     Total amount of global memory:                 10015 MBytes (10501423104 bytes)

   CUDA Capability Major version is what we would use later to build openCV, if it works, otherwise look for other compatible computational numbers from https://en.wikipedia.org/wiki/CUDA#GPUs_supported

Steps OpenCV: \
mkdir opencv \
git clone https://github.com/opencv/opencv.git \
git clone https://github.com/opencv/opencv_contrib.git \
cd opencv \
git checkout <check the latest version in the releases tab (https://github.com/opencv/opencv/releases) \
cd ../opencv_contrib \
git checkout <check the latest version in the releases tab (https://github.com/opencv/opencv/releases) \
mkdir build && cd build \

GUI Explnation for if the Commandline does not work (https://www.youtube.com/watch?v=tjXkW0-4gME)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D OPENCV_ENABLE_NONFREE=ON \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D CUDA_ARCH_BIN=<GPU CUDA Capability Major version> \
-D WITH_CUBLAS=1 \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules/ \
-D HAVE_opencv_python3=ON \
-D PYTHON_EXECUTABLE=<Python executable path> \
-D BUILD_EXAMPLES=ON ..

make -j<number of cores>

sudo make install
sudo ldconfig

Check if the OpenCV so file is in the correct place:  \
ls -l /usr/local/lib/python3.8/site-packages/cv2/python-3.8 \

ln -s /usr/local/lib/python3.8/site-packages/cv2/python-3.8/<file name of so file> <Python Virtual Environment Path>/lib/python3.8/site-packages/cv2.so \

To verify the installation: \
`import cv2 \
count = cv2.cuda.getCudaEnabledDeviceCount() \
print(count) \`

This will print out if there are Cuda enabled devices, you will also see when trying to use OpenCV's dnn module that it is not saying that it is using the CPU because no GPU was found.

Example code for a OpenCV project with the Realsense can be found in the CameraDepthTest project.
The file "opencv_viewer_example.py" should be working if everything is installed correctly, with a D435 camera.
