# install librealsense dependencies first:

echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list  
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE  
sudo apt-get update  
sudo apt-get install librealsense2-dkms  
sudo apt-get install librealsense2-utils  
sudo apt-get install librealsense2-dev  
sudo apt-get install librealsense2-dbg  

# install PCL dependencies:
* install [CUDA 9.0](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604)
* install [OpenCV 3.4.1](https://github.com/opencv/opencv/archive/3.4.1.zip)
* `sudo apt install libeigen3-dev`
* `sudo apt install libflann-dev`
* `sudo apt install libvtk5-*`
* `sudo apt install libboost-all-dev`
* `sudo apt install libqhull-dev`

# build and run
mkdir build  
cd build && cmake ..  
make -j  
cd .. && ./build/test_real_sense  

# NOTE:
there are two config json files in cfg for d435 and d415. you can use the intel gui (realsense-viewer) to to tweak the settings, save to disk and load from that.

