# mapprocess  
delete or add points in SLAM map  
## installation method
mkdir -p ~/MApPreprocess/src
cd MApPreprocess/src
git clone https://github.com/wangxizhe/Beidou_Navigation.git
cd
catkin_make
## operation method
### deletepoints.cpp
roslaunch MapProcess deletepoints.launch  
Press P twice to select a range.  
Press G once to return once, only once.
### addpoints.cpp  
Press P once to add a point.  
Press G once to return once, only once.
