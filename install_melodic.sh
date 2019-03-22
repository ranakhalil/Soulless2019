sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-joy
sudo pip install cython
git clone http://github.com/kctess5/range_libc 
cd range_libc/pywrapper
./compile.sh
cd ../../
sudo rm -rf range_libc
