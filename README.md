# exam-repo
Welcome to the repository of the final examinations for Robotics Lab and Field and Service Robotics courses (MSc in Automation and Robotics Engineering, University of Naples Federico II, A.Y. 2020/21).

The repository is composed by the **technical_project** folder of the ROS package and the **technical_project.ogv** video.

#
## Before starting
1. The **technical_project** package needs the installation of the following extra packages:

   <ul>
   <li><em>velocity_controllers</em></li>
   <li><em>aruco_ros</em></li>
   <li><em>map_server</em></li>
   <li><em>amcl</em></li>
   <li><em>gmapping</em></li>
   </ul>

2. Clone the repository in your ROS workspace.

3. Open a new terminal in your ROS workspace and compile it with:

   `catkin_make`

#
## To start a simulation

Open a terminal and type:

1. `roslaunch technical_project scenario.launch`
>To spawn the **scenario** and the **PowerBot** in Gazebo
 
2. `roslaunch technical_project simulation.launch`
>To run the **odom**, the **navigator** and the **controller**  

By default AR marker identifier is equal to **1** (*Simulation 1*) 

#
## To change simulation

Gain access to:

<em><strong>\technical_project\models\ar_marker\material\scripts\ar_marker.material</em></strong>

Change <em><strong>texture aruco-1.png</strong></em> with:

<ul>
<li><em><strong>texture aruco-2.png</strong></em></li>

>To set AR marker identifier equal to **2** (to change in *Simulation 2*)
 
 <li><em><strong>texture aruco-0.png</strong></em></li> (or any other thing)
 
>To set AR marker identifier equal to **0** (or in any case to change in *Simulation 3*)
 
</ul>

Now start a new simulation.


#
## Additional features




### If you want to replan the paths
 
Be sure that in 
<em><strong>\technical_project\config</em></strong> are not present <em><strong>path.yaml</em></strong> and <em><strong>obstacles.yaml</strong></em></strong> files.

Type:

`roslaunch technical_project planner.launch`
>To run the **planner** and load the **map**

**N.B. PLANNER MIGHT TAKE MORE THAN 1h 30m TO FINISH!**
 
For this reason final results in **path.yaml** and **obstacles.yaml** are already provided.

 
#
### If you want to visualize the paths
 
 Open a new terminal and type:
 
1. `roslaunch technical_project visualization.launch`

>To run **visualization**, **RViz** and load the **map**
 
2. In the same terminal type a number from **0** to **5** to select a path
 
3. In RViz add by topic:

   <ul>
   <li><em>\map Map</em></li>
   <li><em>\visualization_obst Marker</em></li>
   <li><em>\visualization_points_and_lines Marker</em></li>
   </ul>
 
 to visualize the selected path.
 


# 
### If you want to localize the robot during a simulation
 
 Open a new terminal and type:
 
`roslaunch technical_project amcl.launch`

>To run **amcl**, **RViz** and load the **map**
 

