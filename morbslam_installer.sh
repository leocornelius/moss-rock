
sudo apt update
sudo apt install libpython2.7-dev

sudo apt install -y ninja-build
echo "Installing G2o dependencies"
sudo apt install -y libeigen3-dev
sudo apt install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
sudo apt install -y libceres-dev

echo "Installing Pangolin dependencies"
sudo apt install -y libglew-dev
sudo apt install -y libpython2.7-dev
sudo apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev \
        libdc1394-22-dev libraw1394-dev \
        libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev

sudo apt install -y libeigen3-dev \
        doxygen python3-pydot python3-pydot-ng
sudo apt install -y graphviz # after python-pydot and python-pydot-ng

cd ..
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
echo "Installing Pangolin dependencies"
./scripts/install_prerequisites.sh all 

echo "Installing Pangolin"
cmake -B build
cmake --build build -j12
cd ..


sudo apt install -y libboost-all-dev libssl-dev
cd moss-rock
echo "Building ORB-SLAM2"
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_VERBOSE_MAKEFILE=ON
