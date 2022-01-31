#include "ros/ros.h"
#include "nav_msgs/Odometry.h" 
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "boost/thread.hpp"
#include "aruco_msgs/Marker.h"
#include "fstream"
#include "geometry_msgs/Twist.h"


using namespace std;


class CONTROLLER{
	public:
		CONTROLLER();
		void odometry_cb(nav_msgs::Odometry odom);	
		void path_cb(std_msgs::Float64MultiArray path);
		void id_path_cb(std_msgs::Float64 id_path);
		void run();
		void control();	
		void obst_sub(std_msgs::Bool obst);

	
	private:
		ros::NodeHandle _nh;  
		double _rho, _d;
   		float _v,_w;
   		float _b;
   		float _k1;
   		float _k2;
		float _kp;
		float _kd;
   		float _u1, _u2;
   		std_msgs::Float64 _wl, _wr;
   		geometry_msgs::Point _y;
   		ros::Subscriber _odom_sub;
   		ros::Publisher _wl_vel_pub;
   		ros::Publisher _wr_vel_pub;
		ros::Subscriber _path_sub;
		ros::Subscriber _id_path_sub;
		ros::Publisher _path_finished_pub;
   		geometry_msgs::Point _curr_p;
		ros::Publisher _lin_vel_pub;
		ros::Publisher _ang_vel_pub;
		ros::Publisher _track_err_pub;
		ros::Publisher _yaw_err_pub;
		ros::Subscriber _obst_sub;
   		double _curr_q_dot[2];
   		double _wp_pos_dot[2];
        double _wp_pos[2];
		double _wp_vel[2];
		double _wp_yaw;
		double _tresh_pos;
		double _tresh_yaw;
		double _wp_yaw_dot;
   		double _curr_yaw;
		double _curr_velyaw;
   		bool _first_odom;
		bool _goal_pos;
		int _goal_index;
		bool _goal_yaw;
		bool _path_read;
		std_msgs::Float64MultiArray _my_path;
		int _pathIndex;
		std_msgs::Float64 _path_finished;
		std_msgs::Float64 _id_path;
		std_msgs::Float64 _lin_vel;
		std_msgs::Float64 _ang_vel;
		std_msgs::Float64 _track_err;
		std_msgs::Float64 _yaw_err;
		double _w_rot;
		bool _obst;

	
};
