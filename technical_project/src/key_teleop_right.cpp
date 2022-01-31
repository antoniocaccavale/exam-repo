#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"

using namespace std;

#define WHEEL_VEL 2.0



class KEY_CTRL {
	public:
		KEY_CTRL();
		void key_input();
		void run();
		void vel_ctrl();

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _vel_pub;

		float _fv; //Forward velocity	

};


KEY_CTRL::KEY_CTRL() {	
	_vel_pub = _nh.advertise< std_msgs::Float64 >("/right_wheel_velocity_controller/command", 1);
	_fv = 0.0;
}


void KEY_CTRL::key_input() {

	string input;

	cout << "Keyboard Input: " << endl;
	cout << "[u]: Forward direction velocity" << endl;
	cout << "[m]: Backward direction velocity" << endl;
	cout << "[j]: stop the robot!" << endl;

	while (ros::ok()) {

		getline( cin, input);

		if( input == "u" ) 
			_fv = (_fv < 0.0 ) ? 0.0 : WHEEL_VEL;
		else if( input == "m" ) 
			_fv = (_fv > 0.0 ) ? 0.0 : -WHEEL_VEL;
		else if( input == "j" ) 
			_fv = 0.0;

		
	}
}


void KEY_CTRL::vel_ctrl() {
	
	ros::Rate r(10);
	std_msgs::Float64 command_right;

	while(ros::ok()) {

		command_right.data = _fv;
		_vel_pub.publish( command_right );

		r.sleep();
	}

}


void KEY_CTRL::run() {
	boost::thread key_input_t( &KEY_CTRL::key_input, this );
	boost::thread vel_ctrl_t( &KEY_CTRL::vel_ctrl, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "key_ctrl_right");

	KEY_CTRL kc;
	kc.run();

	return 0;
}
