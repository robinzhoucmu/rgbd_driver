# install dependencies first:

echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list  
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE  
sudo apt-get update  
sudo apt-get install librealsense2-dkms  
sudo apt-get install librealsense2-utils  
sudo apt-get install librealsense2-dev  
sudo apt-get install librealsense2-dbg  

# build and run
mkdir build  
cd build && cmake ..  
make -j  
cd .. && ./build/test_real_sense  

# NOTE:
there are two config json files in cfg for d435 and d415. you can use the intel gui (realsense-viewer) to to tweak the settings, save to disk and load from that.

