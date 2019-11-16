# ROS2-GPS2xyz

1.需要安装eigen3：  
    # edition: eigen 3.3.4 from 2017  
    $ git clone https://github.com/eigenteam/eigen-git-mirror  
    $ cd eigen-git-mirror  
    $ mkdir build  
    $ cd build  
    $ cmake ..  
    $ sudo make install  
    $ cp -Rf /usr/local/include/eigen3/Eigen /usr/local/include/  
    
  
2.使用方法  
  编译后  
  $ ros2 launch geodetic_utils geod_tran.launch.py  
  数据：  
  两个发布转换过的数据topic: /gps_position和/gps_odom  
  
