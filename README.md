## rviz_maps

This package is an adapted version of [rviz_satellite](https://github.com/gareth-cross/rviz_satellite). It extends this version and is able to plot marker points of incoming gps positions. This package is basically a rviz plugin

![demo_pic](https://github.com/CesMak/rviz_maps/blob/master/data/demo.png)

Load Tiles from tile server around a latitude and longitude coordinate and plot gps points as markers of message Type: sensor_msgs/NavSatFix to a topic with name /gps/fix.

## Demo

``` 
roslaunch rviz_maps demo.launch
``` 

** If you do not see the map** -> change the latitute and click enter!

## Errors:

``` 
Errors     << rviz_maps:make /home/markus/ros_space/alfons_ws/logs/rviz_maps/build.make.000.log
In file included from /home/markus/ros_space/alfons_ws/src/alfons/rviz_maps/src/aerialmap_display.cpp:38:0:
/home/markus/ros_space/alfons_ws/src/alfons/rviz_maps/src/aerialmap_display.h:20:53: fatal error: rviz_maps/rviz_scale.h: No such file or directory
compilation terminated.
``` 

**Solution**

* source ws
* try rosmsg show rviz_maps/rviz_scale.msg
* roscd rviz_maps
* catkin build --this
* catkin build

**mapscache is stored in package of different folder**
``` 
  const std::string package_path = ros::package::getPath("rviz_pics");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_pics' not found to create storage folder");
  }

  std::hash<std::string> hash_fn;
  cache_path_ =
      QDir::cleanPath(QString::fromStdString(package_path) + QDir::separator() +
                      QString("mapscache") + QDir::separator() +
                      QString::number(hash_fn(object_uri_)));
``` 

Change storage folder here ^^^

If this does not work change a little thing in the tileloader.cpp file and catkin build again and then it should work.

## License BSD
If you want to use this package please contact: [me](https://simact.de/about_me).

Please also respect the License of [rviz_satellite](https://github.com/gareth-cross/rviz_satellite)