# Dependencies:  
stated in [`package.xml`](https://github.com/strands-project/v4r/blob/master/package.xml)
There are two options to use the SIFT recognizer:
 - Use V4R third party library SIFT GPU (this requires a decent GPU - see www.cs.unc.edu/~ccwu/siftgpu) [default]
 - Use OpenCV non-free SIFT implementation (this requires the non-free module of OpenCV - can be installed from source). This option is enabled if BUILD_SIFTGPU is disabled in cmake.

# Installation:  

## From Source  
```
cd ~/somewhere
git clone 'https://github.com/ThoMut/v4r.git'
cd v4r
./setup.sh
mkdir build
cd build
cmake ..
make
sudo make install (optional)
```
