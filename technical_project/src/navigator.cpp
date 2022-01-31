#include "navigator.h"



//constructor
NAVIGATOR::NAVIGATOR(){

     _ar_sub = _nh.subscribe("/aruco_marker_publisher/markers", 1, &NAVIGATOR::aruco_cb, this);
    _curr_path_pub= _nh.advertise<std_msgs::Float64MultiArray>("/curr_path",1);
    _id_path_pub = _nh.advertise<std_msgs::Float64>("/id_path",1);
    _path_finished_sub = _nh.subscribe("/path_finished", 1, &NAVIGATOR::path_finished_cb, this);
    _lidar_sub = _nh.subscribe("/laser/scan",1, &NAVIGATOR::lidar_cb, this);
    _obst_pub = _nh.advertise<std_msgs::Bool>("/obst",1);

    _ar_marker.id=0;
    _first_nav = true;
    _id_path.data=0;

    //access to "path.yaml" file
    vector<double> temp;
    _nh.getParam("path_0",temp);
    for(int i=0; i<temp.size(); i++)
        _path0.data.push_back(temp[i]);
    temp.clear();

    _nh.getParam("path_1",temp);
    for(int i=0; i<temp.size(); i++)
        _path1.data.push_back(temp[i]);
    temp.clear();

        _nh.getParam("path_2",temp);
    for(int i=0; i<temp.size(); i++)
        _path2.data.push_back(temp[i]);
    temp.clear();

        _nh.getParam("path_3",temp);
    for(int i=0; i<temp.size(); i++)
        _path3.data.push_back(temp[i]);
    temp.clear();

        _nh.getParam("path_4",temp);
    for(int i=0; i<temp.size(); i++)
        _path4.data.push_back(temp[i]);
    temp.clear();

        _nh.getParam("path_5",temp);
    for(int i=0; i<temp.size(); i++)
        _path5.data.push_back(temp[i]);

}



void NAVIGATOR::aruco_cb(aruco_msgs::MarkerArray ar_marker_array){

   _ar_marker = ar_marker_array.markers[0];
}


void NAVIGATOR::path_finished_cb(std_msgs::Float64 path_finished){
    
   _path_finished.data = path_finished.data;
   
}

void NAVIGATOR::lidar_cb(sensor_msgs::LaserScan laser){


   for(int i=0; i<laser.ranges.size(); i++){
      if(laser.ranges[i]<laser.range_min+0.02)
         _obst.data = true;
      
   }    
}




void NAVIGATOR::navigation(){

ros::Rate r(100);
ros::Rate r_wait(10);
bool room1, room2;
room1=false;
room2=false;
bool primo=true;
bool secondo=true;

    while(ros::ok){

        //initialization with motion to warehouse
        if(_first_nav && _path_finished.data==0){

                _curr_path=_path0;
                _id_path.data=0;
                _first_nav = false;
            }

        //next path choice after ArUco marker dectection        
        if(_path_finished.data==1){

               if(_ar_marker.id == 1){
                _curr_path = _path1;
                _id_path.data=1;
                    room1=true;
               }
   
             
              else if(_ar_marker.id == 2){
                _curr_path = _path3;
                _id_path.data=3;
                    room2=true;
               }

               else{
                   _curr_path = _path5;
                   _id_path.data=5;
               }
        }

        //next path choice after the reaching of the room
        else if(_path_finished.data==2){

                if(room1){
                    _curr_path = _path2;
                    _id_path.data=2;
                    room1=false;

                }
                if(room2){
                    _curr_path = _path4;
                    _id_path.data=4;
                    room2=false;
            
            }
        }


            _curr_path_pub.publish(_curr_path);
            _id_path_pub.publish(_id_path);
            _obst_pub.publish(_obst);
            r.sleep(); 
    }

}


void NAVIGATOR::run(){

    boost::thread navigation_thr(&NAVIGATOR::navigation, this);
    ros::spin();    

}


int main(int argc, char** argv){   
   ros::init(argc,argv, "navigator_node");
   NAVIGATOR nav;
    nav.run();
   return 0;
}