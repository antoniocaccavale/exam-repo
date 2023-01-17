#include "visualization.h"


MARKER::MARKER(){


	_marker_pub = _nh.advertise<visualization_msgs::Marker>( "visualization_points_and_lines", 0 );
	_obst_pub = _nh.advertise<visualization_msgs::Marker>( "visualization_obst", 0 );

	//reading from "path.yaml" "obstacles.yaml" files
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
	temp.clear();

		_nh.getParam("obstacles",temp);
	for(int i=0; i<temp.size(); i++)
		_my_obst.data.push_back(temp[i]);

}


void MARKER::marker_print(){
	ros::Rate loop_rate(0.1);
	
	int pathIndex=0;
	ros::Rate r_wait(10);
	ros::Rate r(1);

	while (ros::ok()){
	
		while(pathIndex<_my_path.data.size()){
			
			_points.header.frame_id = _line_strip.header.frame_id = "map";
			_points.header.stamp = _line_strip.header.stamp = ros::Time();
			_points.ns = _line_strip.ns = "points_and_lines";
			_points.id = 1;
			_line_strip.id = 2;
			_points.type = visualization_msgs::Marker::POINTS;
			_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
			_points.action = _line_strip.action = visualization_msgs::Marker::ADD;
			_points.scale.x = 0.05;
			_points.scale.y = 0.05;
			_points.color.a = 1.0;
			_points.color.r = 1.0;
			_line_strip.scale.x = 0.01;
			_line_strip.color.a = 1.0;
			_line_strip.color.g = 0.75;
			
			

			float x = _my_path.data[pathIndex];
			float y = _my_path.data[pathIndex+1];
			float z = 0.0;

			geometry_msgs::Point p;
			p.x = x;
			p.y= y;
			p.z = z;

			_points.points.push_back(p);
			_line_strip.points.push_back(p);
			pathIndex = pathIndex + 2;
			
		}
		
		_marker_pub.publish(_points);
		_marker_pub.publish(_line_strip);

		r.sleep();

	}
}



void MARKER::obst_print(){
	ros::Rate loop_rate(0.1);
	
	int obstIndex=0;
	ros::Rate r_wait(10);
	ros::Rate r(1);

	while (ros::ok()){
	
		while(obstIndex<_my_obst.data.size()){
			
			_obstPoints.header.frame_id = _line_strip.header.frame_id = "map";
			_obstPoints.header.stamp = _line_strip.header.stamp = ros::Time();
			_obstPoints.ns = _line_strip.ns = "obstacles";
			_obstPoints.id = 3;
			_obstPoints.type = visualization_msgs::Marker::POINTS;
			_obstPoints.action = visualization_msgs::Marker::ADD;
			_obstPoints.scale.x = 0.05;
			_obstPoints.scale.y = 0.05;
			_obstPoints.color.a = 1.0;
			_obstPoints.color.r = 1.0;

			float x = _my_obst.data[obstIndex];
			float y = _my_obst.data[obstIndex+1];
			float z = 0.0;

			geometry_msgs::Point p;
			p.x = x;
			p.y= y;
			p.z = z;

			_obstPoints.points.push_back(p);
			obstIndex = obstIndex + 2;
			
		}
		
		_obst_pub.publish(_obstPoints);
		r.sleep();

	}
}


void MARKER::run(){

    boost::thread points_and_lines_t( &MARKER::marker_print, this);
	boost::thread obst_t( &MARKER::obst_print, this);
    ros::spin();
}


int main(int argc, char** argv){

 	ros::init(argc,argv, "visualization_node");
    MARKER marker;
	cout<<"\n";
    cout<<"Digit 0 to choose path0"<<endl<<"Digit 1 to choose path1"<<endl<<"Digit 2 to choose path2"<<endl<<"Digit 3 to choose path3"<<endl<<"Digit 4 to choose path4"<<endl<<"Digit 5 to choose path5"<<endl<<endl;
   int choice;
   cout<<"Make your choice: ";
   cin>>choice;
   system("pause");

   if (choice==0){
	   	marker._my_path = marker._path0;
        marker.run();
   }
   if (choice==1){
	   marker._my_path = marker._path1;
        marker.run();
   }
    if (choice==2){
		marker._my_path = marker._path2;
        marker.run();
	}
	if (choice==3){
		marker._my_path = marker._path3;
        marker.run();
	}
	if (choice==4){
		marker._my_path = marker._path4;
		marker.run();
	}
	if (choice==5){
		marker._my_path = marker._path5;
		marker.run();
	}
    return 0;
}










