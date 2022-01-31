#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "aruco_msgs/MarkerArray.h"
#include "aruco_msgs/Marker.h"
#include "vector"
#include "tf/tf.h"
#include "boost/thread.hpp"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

class NAVIGATOR{
	public:
		NAVIGATOR();

        void aruco_cb(aruco_msgs::MarkerArray ar);
        void navigation();
        void run();
        void path_finished_cb(std_msgs::Float64 path_finished);
        void lidar_cb(sensor_msgs::LaserScan laser);


		
	private:

    ros::NodeHandle _nh;
	ros::Subscriber _ar_sub;
    ros::Publisher _curr_path_pub;
    ros::Publisher _id_path_pub;
    ros::Subscriber _path_finished_sub;
    ros::Subscriber _lidar_sub;
    ros::Publisher _obst_pub;
    std_msgs::Float64MultiArray _curr_path;
    std_msgs::Float64MultiArray _path0;
    std_msgs::Float64MultiArray _path1;
    std_msgs::Float64MultiArray _path2;
    std_msgs::Float64MultiArray _path3;
    std_msgs::Float64MultiArray _path4;
    std_msgs::Float64MultiArray _path5;
    std_msgs::Float64 _path_finished;
     std_msgs::Float64 _id_path;
    aruco_msgs::Marker _ar_marker;
    bool _first_nav;
    std_msgs::Bool _obst;

     
};
