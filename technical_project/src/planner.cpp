#include "planner.h"



PLANNER::PLANNER(){


    _map_sub = _nh.subscribe("/map", 1, &PLANNER::map_cb, this);
    _read_map=false;
    _nPaths = 6;

    }


void PLANNER::map_cb(nav_msgs::OccupancyGrid map){

    _read_map = true;
    _map = map;
   
}



void PLANNER::readMapData(){
    int nrow = _map.info.height;
    int ncol = _map.info.width;
    int npixel = _map.data.size();
    int count=0;
    double occup_tresh = 0.65;
    double free_tresh = 0.196;
    double resolution = 0.05;
    int nblack = 0;
    vector<geometry_msgs::Point> blackPixel;
    vector<geometry_msgs::Point> blackStart;
    vector<geometry_msgs::Point> blackEnd;
    
    geometry_msgs::Point tempPoint;
    while(count<npixel){
        for(int i=0; i<nrow; i++){
            for(int j=0; j<ncol; j++){
                
                
                if(_map.data[count]>occup_tresh){
                    tempPoint.x=j;
                    tempPoint.y=i;
                    blackPixel.push_back(tempPoint);
                    nblack++;
                }
                    count++;
            }
        }
    }
    
    tempPoint.x=0;
    tempPoint.y=0;

     for(int i=0; i<blackPixel.size(); i++){

        tempPoint.x = _map.info.origin.position.x + blackPixel[i].x*resolution;
        tempPoint.y = _map.info.origin.position.y + blackPixel[i].y*resolution;
        blackStart.push_back(tempPoint);

        tempPoint.x = blackStart[i].x + resolution;
        tempPoint.y = blackStart[i].y + resolution;
        blackEnd.push_back(tempPoint);
    }

     tempPoint.x=0;
     tempPoint.y=0;

    for(int i=0; i<blackPixel.size(); i++){


        tempPoint.x = blackStart[i].x;
        tempPoint.y = blackStart[i].y;
        if(pointNotPresent(tempPoint, _blackPoints))
            _blackPoints.push_back(tempPoint);

        tempPoint.x = blackEnd[i].x;
        tempPoint.y = blackStart[i].y;
        if(pointNotPresent(tempPoint, _blackPoints))
            _blackPoints.push_back(tempPoint);

        tempPoint.x = blackStart[i].x;
        tempPoint.y = blackEnd[i].y;
        if(pointNotPresent(tempPoint, _blackPoints))
            _blackPoints.push_back(tempPoint);

        tempPoint.x = blackEnd[i].x;
        tempPoint.y = blackEnd[i].y;
        if(pointNotPresent(tempPoint, _blackPoints))
            _blackPoints.push_back(tempPoint);

    }

    ofstream myfile;
    myfile.open ("/home/user/ros_ws/src/technical_project/config/obstacles.yaml", ios::out | ios::app | ios::binary);
    string name;
    bool last = false;
    
    name = "obstacles: [";
        myfile << name;
     
    for(int i=0; i<_blackPoints.size(); i++){
        if(i==_blackPoints.size()-1)
            last=true;

        _obst.data.push_back(_blackPoints[i].x);
        _obst.data.push_back(_blackPoints[i].y);

        if(!last)
            myfile<<_blackPoints[i].x<<", "<<_blackPoints[i].y<<", \n";
        else
        myfile<<_blackPoints[i].x<<", "<<_blackPoints[i].y;
    }
    myfile<<"]\n";
    myfile.close();
   
} 



bool PLANNER::pointNotPresent(geometry_msgs::Point& point, vector<geometry_msgs::Point>& pointVec){
    for(int i=0; i<pointVec.size(); i++){
        if(pointVec[i]==point)
          return  false;
    }
        return true;

}


bool PLANNER::obstacleFound(PLANNER::rrtNode &Node){
    
  double delta=0.40; //soglia di distanza 40 cm

    for(int i=0; i<_blackPoints.size(); i++){
        if(getDistance(_blackPoints[i],Node)<delta)
                return true;
    }
    return false;
}


void PLANNER::initNode(PLANNER::rrtNode &Node, const geometry_msgs::PoseStamped& posNode)
{
    Node.posX=posNode.pose.position.x;
    Node.posY=posNode.pose.position.y;
    Node.parentID = 0;
    Node.nodeID = 0;
    Node.cost=0;

}





double PLANNER::getDistance(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2){

    double distance;
    distance = sqrt(pow(Node1.posX-Node2.posX,2)+pow(Node1.posY-Node2.posY,2));
    return distance;

}

double PLANNER::getDistance(geometry_msgs::Point& Point, PLANNER::rrtNode& Node){

    double distance;
    distance = sqrt(pow(Point.x-Node.posX,2)+pow(Point.y-Node.posY,2));
    return distance;

}

double PLANNER::getDistance(geometry_msgs::Point& Point1, geometry_msgs::Point& Point2){

    double distance;
    distance = sqrt(pow(Point1.x-Point2.x,2)+pow(Point1.y-Point2.y,2));
    return distance;

}



PLANNER::rrtNode PLANNER::setDistance(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2, double delta){

    PLANNER::rrtNode Node3;
    double alfa;
    alfa = atan((Node1.posY-Node2.posY)/(Node1.posX-Node2.posX));
    Node3.posX = Node2.posX + delta*sin(alfa);
    Node3.posY = Node2.posY + delta*cos(alfa);
    Node3.nodeID = Node1.nodeID;
    return Node3;

}

void PLANNER::swipe(geometry_msgs::PoseStamped& Node1, PLANNER::rrtNode& Node2){
    Node1.pose.position.x = Node2.posX;
    Node1.pose.position.y = Node2.posY;
}


PLANNER::rrtNode PLANNER::generateRandNode(){

    PLANNER::rrtNode Node;
    //limits
    double x_min = -10.0;
    double y_min = -10.0;
    double x_max = 10.0;
    double y_max = 10.0;

    double x = (double)rand() / RAND_MAX;
    x = x_min + x * ( x_max - x_min);
    Node.posX = x;

    double y = (double)rand() / RAND_MAX;
    y = y_min + y * ( y_max - y_min);
    Node.posY = y;
        
    return Node;

}

PLANNER::rrtNode PLANNER::checkNearNode(PLANNER::rrtNode& Node1, vector<PLANNER::rrtNode>& Tree){

    PLANNER::rrtNode Node2; 
    int indexNear = 0;
    double minDis = INF;
    double currDis = 0.0;
    for(int i=0; i<Tree.size(); i++){
        currDis=getDistance(Node1,Tree[i]);
        if(currDis<minDis){
            minDis = currDis;
            indexNear = i;
        }
    }
    Node2 = Tree[indexNear];
    return Node2;
}


PLANNER::rrtNode PLANNER::checkNewNode(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2){

    PLANNER::rrtNode Node3;
    double delta = 0.2;

    if(getDistance(Node1,Node2)<delta){
        
        Node3=Node1;
    
    }else
        Node3=setDistance(Node1, Node2, delta);
    
    Node3.parentID = Node2.nodeID;
    Node3.cost = Node2.cost + getDistance (Node2, Node3);

    return Node3;
}

bool PLANNER::goalNodeReached(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2){

    double tresh=0.5;
    if(fabs(Node1.posX-Node2.posX)<tresh && fabs(Node1.posY-Node2.posY)<tresh)
        return true;

    else
        return false;
}



vector<PLANNER::rrtNode> PLANNER::reverse(vector<PLANNER::rrtNode>& Tree1){

    vector<PLANNER::rrtNode> Tree2;
    
            for(int j=Tree1.size()-1; j>=0; j--){
                Tree2.push_back(Tree1[j]);
            
        }
    
    return Tree2;
}

vector<PLANNER::rrtNode> PLANNER::concatenate(vector<PLANNER::rrtNode>& Tree1, vector<PLANNER::rrtNode>& Tree2){

    vector<PLANNER::rrtNode> Tree3;
    Tree3=Tree1;
    
    for(int i=0; i<Tree2.size(); i++)
        Tree3.push_back(Tree2[i]);

    return Tree3;
}





vector<PLANNER::rrtNode> PLANNER::checkFinalPath(vector<PLANNER::rrtNode>& Tree){

    vector<PLANNER::rrtNode> Path;

    int finalId=Tree.size()-1;
     Path.push_back(Tree[finalId]);
        while(finalId!=0){

            for(int j=0; j<finalId; j++){
            if(Tree[finalId].parentID==Tree[j].nodeID){
                Path.push_back(Tree[j]);
                finalId=j;
            }
        }
    }
    return Path;
}





vector<PLANNER::rrtNode> PLANNER::checkFinalPathStar(vector<PLANNER::rrtNode>& Tree){

    
    double searchRad =0.3;
    bool neighFound=false;
    int currIndex;
    int minIndex;
    double currCost=0;
    double minCost=INF;
    

        vector<PLANNER::rrtNode> Path;
    //inizializzo con il nodo di arrivo
    int finalId=Tree.size()-1;
     Path.push_back(Tree[finalId]);


        while(finalId!=0){

        for(int j=0; j<finalId; j++){

            

            if(Tree[finalId].parentID==Tree[j].nodeID){

                for(int k=0; k<finalId; k++){
                    if(isaNeighbour(Tree[j],Tree[k],searchRad)){
                    
                        if((getDistance(Tree[k],Tree[finalId])+Tree[k].cost)<Tree[finalId].cost){

                           neighFound=true;
                            currIndex=k;
                            currCost=Tree[k].cost;

                                if(currCost<minCost){
                                    minCost=currCost;
                                    minIndex=currIndex;
                                    
                                }

                            }
                        }
                    }
                    if(neighFound){
                        Path.push_back(Tree[minIndex]);
                            finalId = minIndex;
                            neighFound=false;

                    }else{
                        Path.push_back(Tree[j]);
                            finalId = j;
                    }

                }
            }
        }

    return Path;    
}

bool PLANNER::isaNeighbour(PLANNER::rrtNode& Node1, PLANNER::rrtNode& Node2, double radius){


    if(getDistance(Node1,Node2)<radius)
        return true;
    return false;
}




void PLANNER::initParAr(){

    _partenza.resize(_nPaths);
    _arrivo.resize(_nPaths);
    _partenza[0].pose.position.x = 0.0;
    _partenza[0].pose.position.y = 0.0;
    _arrivo[0].pose.position.x = 5.0;
    _arrivo[0].pose.position.y = 6.0;

    _partenza[1].pose.position.x = 5.0;
    _partenza[1].pose.position.y = 6.0;
    _arrivo[1].pose.position.x = -5.5;
    _arrivo[1].pose.position.y = 5.5;

    _partenza[2].pose.position.x = -5.5;
    _partenza[2].pose.position.y = 5.5;
    _arrivo[2].pose.position.x = 0.0;
    _arrivo[2].pose.position.y = 0.0;

    _partenza[3].pose.position.x = 5.0;
    _partenza[3].pose.position.y = 6.0;
    _arrivo[3].pose.position.x = -5.5;
    _arrivo[3].pose.position.y = -5.0;

    _partenza[4].pose.position.x = -5.5;
    _partenza[4].pose.position.y = -5.0;
    _arrivo[4].pose.position.x = 0.0;
    _arrivo[4].pose.position.y = 0.0;

    _partenza[5].pose.position.x = 5.0;
    _partenza[5].pose.position.y = 6.0;
    _arrivo[5].pose.position.x = 0.0;
    _arrivo[5].pose.position.y = 0.0;
}

void PLANNER::makePlan()
{
    ros::Rate r(100000);
    ros::Rate r_wait(10);
    bool first;
    bool arrived;
    int Id;


    while(!_read_map)
      r_wait.sleep(); 

    //consider obstacles from occupancy grid map
    readMapData();

    //set starts and ends for each path
    initParAr();

    //generation of 6 paths
    for(int k=0; k<_nPaths; k++){
    
    arrived=false;
    first = true;
    Id=0;

    initNode(startNode, _partenza[k]);
    initNode(goalNode, _arrivo[k]);
    rrtTree.clear();

    
    while(ros::ok() && !arrived){
        
        if(first){
            first = false;
            
            randNode = startNode;
            randNode.nodeID = Id;
            Id++;
            rrtTree.push_back(randNode);
            
        }else
            randNode = generateRandNode();
            randNode.nodeID = Id;
            Id++;
            nearNode = checkNearNode(randNode, rrtTree);
            newNode = checkNewNode(randNode,nearNode);
//          cout<<"posX: "<<newNode.posX<<endl;
//          cout<<"posY: "<<newNode.posY<<endl;
//          cout<<"My ID is:  "<<newNode.nodeID<<endl;
//          cout<<"My parentID is: "<<newNode.parentID<<endl;
//          cout<<"My cost is: "<<newNode.cost<<endl;
            
            
        if(!obstacleFound(newNode))
            rrtTree.push_back(newNode);
//        else
//            cout<<"Obstacle Found!";
           
        
            cout<<"Count: "<<rrtTree.size()<<endl<<endl;

            if(goalNodeReached(newNode, goalNode))
                arrived = true;
        
        r.sleep(); 
    }
    cout<<"arrived!"<<endl;

    double sumDis=0.0;

    finalPath = checkFinalPath(rrtTree);
    finalPath = reverse(finalPath);

    cout<<"<--------------------->"<<endl;
    cout<<"Final Path: "<<endl<<endl;
    
    
     for(int i=0; i<finalPath.size(); i++){
         
                cout<<"finalPath.posX: "<<finalPath[i].posX<<"      "<<"finalPath.posY: "<<finalPath[i].posY<<endl;

        }

     for(int i=0; i<finalPath.size()-1; i++){
         
                sumDis+=getDistance(finalPath[i],finalPath[i+1]);   
        }
    cout<<"\n";
    cout<<"Path length: "<<sumDis<<endl;
    cout<<"Num. of iterations: "<<rrtTree.size()<<endl;

    for (int j=0; j<finalPath.size();j++){
            path.data.push_back(finalPath[j].posX);
            path.data.push_back(finalPath[j].posY);
    }


    _pathVec.push_back(path);
   
    }

    //writing in "path.yaml"
    ofstream myfile;
    myfile.open ("/home/user/ros_ws/src/technical_project/config/path.yaml", ios::out | ios::app | ios::binary);

    
    string name;
    bool last;
    int count=0;
    
    cout<<"<------- printing: -------->"<<endl;
    for(int i=0; i<_pathVec.size(); i++){
        
        last=false;

        name = "path_"+std::to_string(i)+": [";
        myfile << name;
        while(count<_pathVec[i].data.size()){
            
            if(count==_pathVec[i].data.size()-2)
                last=true;

            cout<<"pathVec[i].x: "<<_pathVec[i].data[count]<<"      "<<"pathVec[i].y: "<<_pathVec[i].data[count+1]<<endl;
            if(!last)
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<",\n";
            else
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<"\n";
            
            count=count+2;
        }

        myfile << "]\n";
        cout<<"     "<<endl<<endl;
    }

     myfile.close();

}


void PLANNER::makePlanBid()
{
    
   ros::Rate r(100000);
    ros::Rate r_wait(10);
    bool first;
    bool arrived;
    int IdDir;
    int IdRev;


    while(!_read_map)
      r_wait.sleep(); 

    //consider obstacles from occupancy grid map
    readMapData(); 

    //set starts end end for each path 
    initParAr();

    //generation of 6 paths
    for(int k=0; k<_nPaths; k++){

    initNode(startNode, _partenza[k]);
    initNode(goalNode, _arrivo[k]);
    rrtTreeDir.clear();
    rrtTreeRev.clear();
    int IdDir=0;
    int IdRev=0;
    arrived=false;
    first = true;

    while(ros::ok() && !arrived){
        
        if(first){
            first = false;
            
            randNodeDir = startNode;
            randNodeRev = goalNode;
            randNodeDir.nodeID = IdDir;
            randNodeRev.nodeID = IdRev;
            IdDir++;
            IdRev++;
            rrtTreeDir.push_back(randNodeDir);
            rrtTreeRev.push_back(randNodeRev);
            
        }else
            randNodeDir = generateRandNode();
            randNodeRev = generateRandNode();
            randNodeDir.nodeID = IdDir;
            randNodeRev.nodeID = IdRev;
            IdDir++;
            IdRev++;   
//          cout<<"randNodeDir.posX: "<<randNodeDir.posX<<"     "<<"randNodeRev.posX: "<<randNodeRev.posX<<endl;
//          cout<<"randNodeDir.posY: "<<randNodeDir.posY<<"     "<<"randNodeRev.posY: "<<randNodeRev.posY<<endl;
            nearNodeDir = checkNearNode(randNodeDir, rrtTreeDir);
            nearNodeRev = checkNearNode(randNodeRev, rrtTreeRev);
//          cout<<"nearNodeDir.posX: "<<nearNodeDir.posX<<"     "<<"nearNodeRev.posX: "<<nearNodeRev.posX<<endl;
//          cout<<"nearNodeDir.posY: "<<nearNodeDir.posY<<"     "<<"nearNodeRev.posY: "<<nearNodeRev.posY<<endl;
            newNodeDir = checkNewNode(randNodeDir,nearNodeDir);
            newNodeRev = checkNewNode(randNodeRev,nearNodeRev);
//          cout<<"newNodeDir.posX: "<<newNodeDir.posX<<"       "<<"newNodeRev.posX: "<<newNodeRev.posX<<endl;
//          cout<<"newNodeDir.posY: "<<newNodeDir.posY<<"       "<<"newNodeRev.posY: "<<newNodeRev.posY<<endl;
//          cout<<"newNodeDir.nodeId: "<<newNodeDir.nodeID<<"       "<<"newNodeRev.nodeId: "<<newNodeRev.nodeID<<endl;
//          cout<<"newNodeDir.parentId: "<<newNodeDir.parentID<<"       "<<"newNodeRev.parentId: "<<newNodeRev.parentID<<endl;
            
            
            if(!obstacleFound(newNodeDir))
                rrtTreeDir.push_back(newNodeDir);
 //           else
 //             cout<<"Obstacle in dir path found!"<<endl;

            if(!obstacleFound(newNodeRev))
                rrtTreeRev.push_back(newNodeRev);
 //           else
//              cout<<"Obstacle in rev path found!"<<endl;

            if(isaNeighbour(newNodeDir, newNodeRev, 0.3))
                arrived = true;
            
          cout<<"CountDir: "<<rrtTreeDir.size()<<"    "<<"CountRev: "<<rrtTreeRev.size()<<endl<<endl;
           
        r.sleep(); 
    }
    cout<<"arrived!"<<endl<<endl;
    finalPathDir = checkFinalPath(rrtTreeDir);
    finalPathRev = checkFinalPath(rrtTreeRev);
    finalPathRev.erase(finalPathRev.begin());
    finalPathDir = reverse(finalPathDir);
    finalPathBid = concatenate (finalPathDir, finalPathRev);
  

    cout<<"<--------------------->"<<endl;


  cout<<"Final Path Dir: "<<endl<<endl;
    for(int i=0; i<finalPathDir.size(); i++){
        cout<<"finalPathDir.posX: "<<finalPathDir[i].posX<<"      "<<"finalPathDir.posY: "<<finalPathDir[i].posY<<endl;
    }

    cout<<"Final Path Rev: "<<endl<<endl;
    for(int i=0; i<finalPathRev.size(); i++){
        cout<<"finalPathRev.posX: "<<finalPathRev[i].posX<<"      "<<"finalPathRev.posY: "<<finalPathRev[i].posY<<endl;
    }

    cout<<"Final Path Bid: "<<endl<<endl;
    for(int i=0; i<finalPathBid.size(); i++){
        cout<<"finalPathBid.posX: "<<finalPathBid[i].posX<<"      "<<"finalPathBid.posY: "<<finalPathBid[i].posY<<endl;
    }

    double sumDis = 0.0;

    for(int i=0; i<finalPathBid.size()-1; i++){
         
                sumDis+=getDistance(finalPathBid[i],finalPathBid[i+1]);   
        }
    
    cout<<"\n";
    cout<<"Path length: "<<sumDis<<endl;
    cout<<"Num. of iterations Dir: "<<rrtTreeDir.size()<<"     "<<"Num. of iterations Rev: "<<rrtTreeRev.size()<<endl;


    for (int j=0; j<finalPathBid.size();j++){
            path.data.push_back(finalPathBid[j].posX);
            path.data.push_back(finalPathBid[j].posY);
    }

        _pathVec.push_back(path);
    
    }

    //writing in "path.yaml" file
    ofstream myfile;
    myfile.open ("/home/user/ros_ws/src/technical_project/config/path.yaml", ios::out | ios::app | ios::binary);

    
    string name;
    bool last;
    int count=0;
    
    cout<<"<------- printing: -------->"<<endl;
    for(int i=0; i<_pathVec.size(); i++){
        
        last=false;

        name = "path_"+std::to_string(i)+": [";
        myfile << name;
        while(count<_pathVec[i].data.size()){
            
            if(count==_pathVec[i].data.size()-2)
                last=true;

            cout<<"pathVec[i].x: "<<_pathVec[i].data[count]<<"      "<<"pathVec[i].y: "<<_pathVec[i].data[count+1]<<endl;
            if(!last)
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<",\n";
            else
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<"\n";
            
            count=count+2;
        }

        myfile << "]\n";
        cout<<"     "<<endl<<endl;
    }

     myfile.close();

}


void PLANNER::makePlanStar()
{

    ros::Rate r(100000);
    ros::Rate r_wait(10);
    bool first;
    bool arrived;
    int Id;

    while(!_read_map)
      r_wait.sleep(); 

    //consider obstacles from occupancy grid map
    readMapData(); 

    //set starts and ends for each path
    initParAr();

    //generation of 6 paths
    for(int k=0; k<_nPaths; k++){

    initNode(startNode, _partenza[k]);
    initNode(goalNode, _arrivo[k]);

    rrtTree.clear();
    Id=0;
    arrived=false;
    first=true;

    while(ros::ok() && !arrived){
        
        if(first){
            first = false;
            
            randNode = startNode;
            randNode.nodeID = Id;
            Id++;
            rrtTree.push_back(randNode);
            
        }else
            randNode = generateRandNode();
            randNode.nodeID = Id;
            Id++;
            nearNode = checkNearNode(randNode, rrtTree);
            newNode = checkNewNode(randNode,nearNode);
        //    cout<<"posX: "<<newNode.posX<<endl;
        //    cout<<"posY: "<<newNode.posY<<endl;
        //    cout<<"My ID is:  "<<newNode.nodeID<<endl;
        //    cout<<"My parentID is: "<<newNode.parentID<<endl;
        //    cout<<"My cost is: "<<newNode.cost<<endl;
            
            
        if(!obstacleFound(newNode))
            rrtTree.push_back(newNode);
    //    else
    //        cout<<"Obstacle Found!";
        
            cout<<"Count: "<<rrtTree.size()<<endl<<endl;

            if(goalNodeReached(newNode, goalNode))
                arrived = true;
        
        r.sleep(); 
    }
    cout<<"arrived!"<<endl;

    double sumDis=0.0;

    finalPathStar = checkFinalPathStar(rrtTree);
    finalPathStar = reverse(finalPathStar);

    cout<<"<--------------------->"<<endl;
    cout<<"Final Path: "<<endl<<endl;
    
    
     for(int i=0; i<finalPathStar.size(); i++){
         
                cout<<"finalPathStar.posX: "<<finalPathStar[i].posX<<"      "<<"finalPathStar.posY: "<<finalPathStar[i].posY<<endl;
             // cout<<"finalPath.nodeId: "<<finalPath[i].nodeID<<"      "<<"finalPath.parentId: "<<finalPath[i].parentID<<endl; 
        }

     for(int i=0; i<finalPathStar.size()-1; i++){
         
                sumDis+=getDistance(finalPathStar[i],finalPathStar[i+1]);   
        }
    
    cout<<"\n";
    cout<<"Path length: "<<sumDis<<endl;
    cout<<"Num. of iterations: "<<rrtTree.size()<<endl;

    for (int j=0; j<finalPathStar.size();j++){
            path.data.push_back(finalPathStar[j].posX);
            path.data.push_back(finalPathStar[j].posY);
    }

    _pathVec.push_back(path);

    }

    //writing in "path.yaml" file
    ofstream myfile;
    myfile.open ("/home/user/ros_ws/src/technical_project/config/path.yaml", ios::out | ios::app | ios::binary);

    
    string name;
    bool last;
    int count=0;
    
    cout<<"<------- printing: -------->"<<endl;
    for(int i=0; i<_pathVec.size(); i++){
        last=false;

        name = "path_"+std::to_string(i)+": [";
        myfile << name;
        while(count<_pathVec[i].data.size()){
            
            if(count==_pathVec[i].data.size()-2)
                last=true;

            cout<<"pathVec[i].x: "<<_pathVec[i].data[count]<<"      "<<"pathVec[i].y: "<<_pathVec[i].data[count+1]<<endl;
            if(!last)
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<",\n";
            else
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<"\n";
            
            count=count+2;
        }

        myfile << "]\n";
        cout<<"     "<<endl<<endl;
    }

     myfile.close();

}
void PLANNER::makePlanBidStar()
{
    
   ros::Rate r(100000);
    ros::Rate r_wait(10);
    bool first;
    bool arrived;
    int IdDir;
    int IdRev;

    while(!_read_map)
      r_wait.sleep(); 

    //consider obstacles from occupancy grid map
    readMapData();

    //set starts and ends for each path
    initParAr();

    //generation of 6 paths
    for(int k=0; k<_nPaths; k++){
    initNode(startNode, _partenza[k]);
    initNode(goalNode, _arrivo[k]);
    rrtTreeDir.clear();
    rrtTreeRev.clear();
    IdDir=0;
    IdRev=0;
    arrived=false;
    first=true;

    while(ros::ok() && !arrived){
        
        if(first){
            first = false;
            
            randNodeDir = startNode;
            randNodeRev = goalNode;
            randNodeDir.nodeID = IdDir;
            randNodeRev.nodeID = IdRev;
            IdDir++;
            IdRev++;
            rrtTreeDir.push_back(randNodeDir);
            rrtTreeRev.push_back(randNodeRev);
            
        }else
            randNodeDir = generateRandNode();
            randNodeRev = generateRandNode();
            randNodeDir.nodeID = IdDir;
            randNodeRev.nodeID = IdRev;
            IdDir++;
            IdRev++;
 //         cout<<"randNodeDir.posX: "<<randNodeDir.posX<<"     "<<"randNodeRev.posX: "<<randNodeRev.posX<<endl;
 //         cout<<"randNodeDir.posY: "<<randNodeDir.posY<<"     "<<"randNodeRev.posY: "<<randNodeRev.posY<<endl;
            nearNodeDir = checkNearNode(randNodeDir, rrtTreeDir);
            nearNodeRev = checkNearNode(randNodeRev, rrtTreeRev);
//          cout<<"nearNodeDir.posX: "<<nearNodeDir.posX<<"     "<<"nearNodeRev.posX: "<<nearNodeRev.posX<<endl;
//          cout<<"nearNodeDir.posY: "<<nearNodeDir.posY<<"     "<<"nearNodeRev.posY: "<<nearNodeRev.posY<<endl;
            newNodeDir = checkNewNode(randNodeDir,nearNodeDir);
            newNodeRev = checkNewNode(randNodeRev,nearNodeRev);
//          cout<<"newNodeDir.posX: "<<newNodeDir.posX<<"       "<<"newNodeRev.posX: "<<newNodeRev.posX<<endl;
//          cout<<"newNodeDir.posY: "<<newNodeDir.posY<<"       "<<"newNodeRev.posY: "<<newNodeRev.posY<<endl;
//          cout<<"newNodeDir.nodeId: "<<newNodeDir.nodeID<<"       "<<"newNodeRev.nodeId: "<<newNodeRev.nodeID<<endl;
//          cout<<"newNodeDir.parentId: "<<newNodeDir.parentID<<"       "<<"newNodeRev.parentId: "<<newNodeRev.parentID<<endl;
            
            
            if(!obstacleFound(newNodeDir))
                rrtTreeDir.push_back(newNodeDir);
//            else
//                cout<<"Obstacle in dir path found!"<<endl;

            if(!obstacleFound(newNodeRev))
                rrtTreeRev.push_back(newNodeRev);
//            else
//                cout<<"Obstacle in rev path found!"<<endl;

            if(isaNeighbour(newNodeDir, newNodeRev, 0.3))
                arrived = true;
            
            cout<<"CountDir: "<<rrtTreeDir.size()<<"    "<<"CountRev: "<<rrtTreeRev.size()<<endl<<endl;
           
        r.sleep(); 
    }
    cout<<"arrived!"<<endl<<endl;
    finalPathDir = checkFinalPathStar(rrtTreeDir);
    finalPathRev = checkFinalPathStar(rrtTreeRev);
    finalPathRev.erase(finalPathRev.begin());
    finalPathDir = reverse(finalPathDir);
    finalPathBidStar = concatenate (finalPathDir, finalPathRev);
  

    cout<<"<--------------------->"<<endl;


  cout<<"Final Path Dir: "<<endl<<endl;
    for(int i=0; i<finalPathDir.size(); i++){
        cout<<"finalPathDir.posX: "<<finalPathDir[i].posX<<"      "<<"finalPathDir.posY: "<<finalPathDir[i].posY<<endl;
    }

    cout<<"Final Path Rev: "<<endl<<endl;
    for(int i=0; i<finalPathRev.size(); i++){
        cout<<"finalPathRev.posX: "<<finalPathRev[i].posX<<"      "<<"finalPathRev.posY: "<<finalPathRev[i].posY<<endl;
    }

    cout<<"Final Path Bid: "<<endl<<endl;
    for(int i=0; i<finalPathBidStar.size(); i++){
        cout<<"finalPathBid.posX: "<<finalPathBidStar[i].posX<<"      "<<"finalPathBid.posY: "<<finalPathBidStar[i].posY<<endl;
    }

    double sumDis = 0.0;

    for(int i=0; i<finalPathBidStar.size()-1; i++){
         
                sumDis+=getDistance(finalPathBidStar[i],finalPathBidStar[i+1]);   
        }
    
    cout<<"\n";
    cout<<"Path length: "<<sumDis<<endl;
    cout<<"Num. of iterations Dir: "<<rrtTreeDir.size()<<"     "<<"Num. of iterations Rev: "<<rrtTreeRev.size()<<endl;


    for (int j=0; j<finalPathBidStar.size();j++){
            path.data.push_back(finalPathBidStar[j].posX);
            path.data.push_back(finalPathBidStar[j].posY);
    }

    _pathVec.push_back(path);
    
    }

    //writing in "path.yaml" file
    ofstream myfile;
    myfile.open ("/home/user/ros_ws/src/technical_project/config/path.yaml", ios::out | ios::app | ios::binary);

    
    string name;
    bool last;
    int count=0;
    
    cout<<"<------- printing: -------->"<<endl;
    for(int i=0; i<_pathVec.size(); i++){
       
        last=false;

        name = "path_"+std::to_string(i)+": [";
        myfile << name;
        while(count<_pathVec[i].data.size()){
            
            if(count==_pathVec[i].data.size()-2)
                last=true;

            cout<<"pathVec[i].x: "<<_pathVec[i].data[count]<<"      "<<"pathVec[i].y: "<<_pathVec[i].data[count+1]<<endl;
            if(!last)
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<",\n";
            else
                myfile <<_pathVec[i].data[count]<<", "<<_pathVec[i].data[count+1]<<"\n";
            
            count=count+2;
        }

        myfile << "]\n";
        cout<<"     "<<endl<<endl;
    }

     myfile.close();

}


void PLANNER::run1(){
    
      boost::thread makePlan_thr( &PLANNER::makePlan, this);
      ros::spin();      
}


void PLANNER::run2(){
    
      boost::thread makePlanBid_thr( &PLANNER::makePlanBid, this);
      ros::spin();      
}



void PLANNER::run3(){
    
      boost::thread makePlanStar_thr( &PLANNER::makePlanStar, this);
      ros::spin();      
}

void PLANNER::run4(){
    
      boost::thread makePlanBidStar_thr( &PLANNER::makePlanBidStar, this);
      ros::spin();      
}



//Uncomment the lines of code if you want to test other algorithm versions
//Otherwise RRT* is selected by default

int main(int argc, char** argv){   
    
    int choice; 
   ros::init(argc,argv, "planner_node");
   PLANNER planner;
//   cout<<"Digit 1 to choice RRT algorithm."<<endl<<"Digit 2 to choice Bidirectional-RRT algorithm."<<endl<<"Digit 3 to choice RRT* algotithm."<<endl<<"Digit 4 to choice Bidirectional-RRT* algorithm: "<<endl<<endl;
//   cin>>choice;
//   system("pause");
//   if (choice==1)
//        planner.run1();
//   if (choice==2)
//        planner.run2();
//    if (choice==3)
        planner.run3();
//    if (choice==4)
//        planner.run4();

   return 0;
}
