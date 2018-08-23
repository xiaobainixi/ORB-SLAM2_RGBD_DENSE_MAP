echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM21
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
