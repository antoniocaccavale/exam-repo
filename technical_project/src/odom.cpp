#include "odom.h"

ODOM::ODOM(){
    _encoders_sub = _nh.subscribe("/joint_states", 1, &ODOM::joint_states_cb, this);
    _model_states_sub = _nh.subscribe("/gazebo/model_states", 1, &ODOM::model_states_cb, this);
    _imu_sub= _nh.subscribe("/imu", 1, &ODOM::imu_cb, this);
    _odom_pub = _nh.advertise<nav_msgs::Odometry>("/odom",1);

    //Publishers for rosbag
    _odom_norm_err_pub = _nh.advertise<std_msgs::Float64>("/odom_norm_err",1);
    _odom_err_pub = _nh.advertise<geometry_msgs::Pose>("/odom_err",1);
    _sim_pose_pub = _nh.advertise<geometry_msgs::Pose>("/sim_pose",1);

    _wl=0.0;
    _wr=0.0;
    _x=0.0;
    _y=0.0;
    _th=0.0;
    _first_odom=false;
}

void ODOM::joint_states_cb(sensor_msgs::JointState joint_states){
    if (joint_states.velocity.size() < 2) 
        return;
    _wl= joint_states.velocity[0];
    _wr=joint_states.velocity[1];

}


void ODOM::imu_cb(sensor_msgs::Imu msg){
    tf::Quaternion q (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _th);
    if (_th>2*M_PI)
        _th=_th-2.0*M_PI;
    else if(_th<0.0) 
        _th=_th+2.0*M_PI;
}

//I need a function to retrieve "diff_wheeled_robot" index
int ODOM::getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

//To retrieve _gazebo_pose for odometric error
void ODOM::model_states_cb(gazebo_msgs::ModelStates model_states){

     int index = getIndex(model_states.name, "diff_wheeled_robot");
    _gazebo_pose=model_states.pose[index];
    tf::Quaternion q(
    _gazebo_pose.orientation.x,
    _gazebo_pose.orientation.y,
    _gazebo_pose.orientation.z,
    _gazebo_pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(_gaz_roll, _gaz_pitch, _gaz_yaw);

}




void ODOM::odom(){

    ros::Rate r(200);
    tf::TransformBroadcaster odom_broadcaster;

    _dt=1/200.0;

    while(ros::ok()){

        _delta_phi_r= _wr*_dt;
        _delta_phi_l= _wl*_dt;
        _v= (0.133/2)*(_delta_phi_r+_delta_phi_l)/_dt;
        _w=(0.133/0.61)*(_delta_phi_r-_delta_phi_l)/_dt;


        //Runge-Kutta integration
        _x +=_v*_dt*cos(_th+(_w*_dt)/2.0);

        if(isnan(_x))
        _x=0;

        _y +=_v*_dt*sin(_th+(_w*_dt)/2.0);

        if(isnan(_y))
        _y=0;
       
        _odom_err.position.x = fabs(_x-_gazebo_pose.position.x);
        _odom_err.position.y = fabs(_y-_gazebo_pose.position.y);
        _odom_norm_err.data = sqrt(pow(_odom_err.position.x,2)+pow(_odom_err.position.y,2));
  
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_center";
        odom_trans.transform.translation.x = _x;
        odom_trans.transform.translation.y = _y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = _x;
        odom.pose.pose.position.y = _y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_center";
        odom.twist.twist.linear.x = _v*cos(_th);
        odom.twist.twist.linear.y = _v*sin(_th);
        odom.twist.twist.angular.z = _w;
        
        _odom_pub.publish(odom);
        _odom_norm_err_pub.publish(_odom_norm_err);
        _odom_err_pub.publish(_odom_err);
        _sim_pose_pub.publish(_gazebo_pose);
        r.sleep();

    }
}

void ODOM::run(){

    boost::thread odom_t( &ODOM::odom, this);

    ros::spin();
}

int main(int argc, char** argv){
 ros::init(argc,argv, "odom_node");

    ODOM odom;
    odom.run();


    return 0;
}
