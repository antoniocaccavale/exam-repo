#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include "boost/thread.hpp"
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

using namespace std;

class ODOM{
    public:
        ODOM();
        void run();
        void joint_states_cb(sensor_msgs::JointState joint_states);
        void odom ();
        void imu_cb(sensor_msgs::Imu msg);
        void model_states_cb(gazebo_msgs::ModelStates model_states);
        int getIndex(std::vector<std::string> v, std::string value);

    private:
        ros::NodeHandle _nh;
        long double _dt;
        long double _delta_phi_l;
        long double _delta_phi_r;
        long double _v;
        long double _w;
        float _wl;
        float _wr;
        long double _x;
        long double _y;
        double _th;
        bool _first_odom;
        geometry_msgs::Pose _gazebo_pose;
        ros::Subscriber _encoders_sub;
        ros::Subscriber _imu_sub;
        ros::Publisher _odom_pub;
        ros::Subscriber _model_states_sub;
        double _gaz_roll, _gaz_pitch, _gaz_yaw;
        std_msgs::Float64 _odom_norm_err;
        geometry_msgs::Pose _odom_err;
        ros::Publisher _odom_norm_err_pub;
        ros::Publisher _odom_err_pub;
        ros::Publisher _sim_pose_pub;


};
