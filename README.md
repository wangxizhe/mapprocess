# mapprocess  
Delete or add points in the SLAM map under the ROS platform.  
## installation method
mkdir -p ~/MApPreprocess/src  
cd MApPreprocess/src  
git clone https://github.com/wangxizhe/mapprocess.git  
cd ..  
catkin_make
## operation method  
source devel/setup.bash
### deletepoints.cpp
roslaunch MapProcess deletepoints.launch  
Press P twice to select a range.  
Press G once to return once, only once.
### addpoints.cpp  
roslaunch MapProcess addpoints.launch  
Press P once to add a point.  
Press G once to return once, only once.
