#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64MultiArray.h"
#include "boost/thread.hpp"
#include "vector"
#include "geometry_msgs/Point.h"
using namespace std;

class MARKER{
    public:
        MARKER();
        void run();
        void marker_print();
        void obst_print();
        std_msgs::Float64MultiArray _my_obst;
        std_msgs::Float64MultiArray _my_path;
        std_msgs::Float64MultiArray _path0;
        std_msgs::Float64MultiArray _path1;
        std_msgs::Float64MultiArray _path2;
        std_msgs::Float64MultiArray _path3;
        std_msgs::Float64MultiArray _path4;
        std_msgs::Float64MultiArray _path5;
        


    private:
        ros::NodeHandle _nh;
        visualization_msgs::Marker _points, _line_strip, _obstPoints;

        ros::Publisher _marker_pub;
        ros::Publisher _obst_pub;
        
     


};
