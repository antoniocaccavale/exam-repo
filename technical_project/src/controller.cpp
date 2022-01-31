#include "controller.h"
using namespace std;

//constructor
CONTROLLER::CONTROLLER(){

	_odom_sub = _nh.subscribe("/odom", 1, &CONTROLLER::odometry_cb, this);
   _path_sub = _nh.subscribe("/curr_path", 1, &CONTROLLER::path_cb, this);
	_wl_vel_pub = _nh.advertise<std_msgs::Float64>("/left_wheel_velocity_controller/command", 1);
   _wr_vel_pub = _nh.advertise<std_msgs::Float64>("/right_wheel_velocity_controller/command",1);
   _path_finished_pub = _nh.advertise<std_msgs::Float64>("/path_finished",1);
   _id_path_sub = _nh.subscribe("/id_path", 1, &CONTROLLER::id_path_cb, this);
   _obst_sub = _nh.subscribe("/obst",1,&CONTROLLER::obst_sub,this);

   //Publishers for rosbag
   _lin_vel_pub  = _nh.advertise<std_msgs::Float64>("/lin_vel",1);
   _ang_vel_pub  = _nh.advertise<std_msgs::Float64>("/ang_vel",1);
   _track_err_pub = _nh.advertise<std_msgs::Float64>("/track_err",1);
   _yaw_err_pub = _nh.advertise<std_msgs::Float64>("/yaw_err",1);
	
	_rho=0.133;
	_d=0.61;
	_b=0.1;
	_k1=0.2;   
	_k2=0.2;   
   _kp=1.0;
   _kd=0.0;
   _tresh_pos=0.20;  
   _tresh_yaw=0.157; 
   _wp_yaw_dot = 0.0;
   _wp_pos_dot[0] = 0.0;
   _wp_pos_dot[1] = 0.0;
   _wp_pos[0] = 0.0;
   _wp_pos[1] = 0.0;
   _wp_yaw = 0.0;
   _goal_index=0;
   _pathIndex = 0;
	_first_odom=false;
   _goal_pos=false;
   _goal_yaw=false;
   _id_path.data=0;
   _path_finished.data=0;
   _w_rot=0.5;
	
}


void CONTROLLER::odometry_cb(nav_msgs::Odometry odom){


   _curr_p.x = odom.pose.pose.position.x;
   _curr_p.y = odom.pose.pose.position.y;

   tf::Quaternion q (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
   double roll, pitch;
   tf::Matrix3x3(q).getRPY(roll, pitch, _curr_yaw);

   if (_curr_yaw>2*M_PI)
      _curr_yaw=_curr_yaw-2.0*M_PI;
   else if(_curr_yaw<0.0) 
      _curr_yaw=_curr_yaw+2.0*M_PI;

   _curr_q_dot[0] = odom.twist.twist.linear.x;
   _curr_q_dot[1]= odom.twist.twist.linear.y;
   _curr_velyaw = odom.twist.twist.angular.z;
   _first_odom=true;    
}



void CONTROLLER::path_cb(std_msgs::Float64MultiArray path){

   
   _path_read = true;
   _my_path = path;

}


void CONTROLLER::id_path_cb(std_msgs::Float64 id_path){

   
   _id_path.data = id_path.data;

}

void CONTROLLER::obst_sub(std_msgs::Bool obst){

   _obst=obst.data;

}



void CONTROLLER::control(){

      ros::Rate r(100);
   	ros::Rate r_wait(10);
      double dt=1.0/100.0;
      bool finish = false;
      bool nav_finished = false;
      bool first=false;

   while(!_first_odom)
      r_wait.sleep(); 
   while(!_path_read)
      r_wait.sleep(); 
 

   while(ros::ok() && !nav_finished){

      //simulation end
      if( (_path_finished.data==3 && finish) | (_path_finished.data==2 && _id_path.data==5 && finish)| _obst){
         _wl.data=0.0;
         _wr.data=0.0;
         _wl_vel_pub.publish(_wl);
         _wr_vel_pub.publish(_wr);
         nav_finished = true;
      }

      //current path end
      if(finish == true){ 
          _pathIndex = 0;
         _goal_pos = false;
         _goal_yaw = false;
         finish=false;
         first=true;
       }

      float q_des[2];
      float pos_err[2];
      float u_bar[2];
      float sp_vel[2];
      float sp_pos[2];

      //position error
      pos_err[0]=_wp_pos[0]-_curr_p.x;
      pos_err[1]=_wp_pos[1]-_curr_p.y;

      //setopoints in position and velocity
      u_bar[0]=0.2*pos_err[0]-0.5*_curr_q_dot[0];
      u_bar[1]=0.2*pos_err[1]-0.5*_curr_q_dot[1];
      sp_vel[0]+=u_bar[0]*dt;
      sp_vel[1]+=u_bar[1]*dt;
      sp_pos[0]+=sp_vel[0]*dt;
      sp_pos[1]+=sp_vel[1]*dt;

      //traslation at b distance along the sagittal axis
      q_des[0]=sp_pos[0]+_b*cos(_curr_yaw);
      q_des[1]=sp_pos[1]+_b*sin(_curr_yaw);

      //point y coordinates
      _y.x=_curr_p.x+_b*cos(_curr_yaw);
      _y.y=_curr_p.y+_b*sin(_curr_yaw);

      //I/O feedback linearization
      _u1=sp_vel[0]+(_k1*(q_des[0]-_y.x));
      _u2=sp_vel[1]+(_k2*(q_des[1]-_y.y));    

      //Go to the next node of the path
      if(_goal_pos && _goal_yaw){
        _pathIndex=_pathIndex+2;
        _goal_pos=false;
        _goal_yaw=false;
        }

      //Set the waypoints
      _wp_pos[0] = _my_path.data[_pathIndex];
      _wp_pos[1] = _my_path.data[_pathIndex+1];
      _wp_yaw = _curr_yaw;


      if(!_goal_pos){

            //tracking control
            _v= cos(_curr_yaw)*_u1+sin(_curr_yaw)*_u2;
            _w=-sin(_curr_yaw)/_b*_u1+cos(_curr_yaw)/_b*_u2;

     
            _track_err.data=sqrt(pow(pos_err[0], 2) + pow(pos_err[1], 2));

            if (_track_err.data < _tresh_pos)
               _goal_pos=true;


      }else{ 

            //orientation references
            if( _pathIndex == _my_path.data.size()-2){

               if (_id_path.data==0)
                  _wp_yaw = 1.57; 
               if(_id_path.data == 1 | _id_path.data == 3)
                  _wp_yaw = 3.14; 
               if (_id_path.data==2|_id_path.data ==4 | _id_path.data == 5)
                  _wp_yaw = 6.28;
                         
            }

            _yaw_err.data=fabs(_wp_yaw-_curr_yaw);

            if(_yaw_err.data>_tresh_yaw){
               
               //orientation regulation
               _v=0;
               _w += (_kp*(_wp_yaw-_curr_yaw) + _kd*(_wp_yaw_dot-_curr_velyaw))*dt;

               if(_w>_w_rot)
                  _w=_w_rot;
               if(_w<-_w_rot)
               _w=-_w_rot;

            }else{

               _goal_yaw=true;
               _v=_v;
               _w=_w;


               if( _pathIndex == _my_path.data.size()-2){
                  finish=true;
                  _path_finished.data++;
                  
                  }
               }
               
            }

      _wl.data= (_v/_rho)-(_d*_w/(2*_rho));
      _wr.data= (_v/_rho)+(_d*_w/(2*_rho));

      if(isnan(_wl.data)) 
      _wl.data=0.0;
      if(isnan(_wr.data)) 
      _wr.data=0.0;

      _path_finished_pub.publish(_path_finished);
      _wr_vel_pub.publish(_wr);
      _wl_vel_pub.publish(_wl);      
      _lin_vel.data=_v;
      _lin_vel_pub.publish(_lin_vel);
      _ang_vel.data=_w;
      _ang_vel_pub.publish(_ang_vel);
      _track_err_pub.publish(_track_err);
      _yaw_err_pub.publish(_yaw_err);
      r.sleep();
   }

      _wl.data=0.0;
      _wr.data=0.0;
      _wl_vel_pub.publish(_wl);
      _wr_vel_pub.publish(_wr);

      cout<<"\n";
      cout << "Controller stops the robot. Navigation is concluded!" << endl;

}


void CONTROLLER::run(){
      boost::thread control_thr( &CONTROLLER::control, this);
      ros::spin();      
}



int main(int argc, char** argv){   
   ros::init(argc,argv, "controller_node");
   CONTROLLER cntr;

   cntr.run();
   return 0;
}








