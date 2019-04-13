sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-pid
sudo pip install cython
git clone http://github.com/kctess5/range_libc 
cd range_libc/pywrapper
./compile.sh
cd ../../
sudo rm -rf range_libc
