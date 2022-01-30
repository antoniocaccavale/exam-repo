# exam-repo
Welcome to the repository related to the final examinations of Robotics Lab and Field and Service Robotics courses (a.y. 2020-21).

## To start a simulation

Open a terminal and type:

1.`roslaunch technical_project scenario.launch`
>To spawn the **scenario** and the **powerbot** in Gazebo
 
2.`roslaunch technical_project simulation.launch`
>To run the **odom**, the **navigator** and the **controller** simultainously  

By default AR marker identifier is equal to **1** (*Simulation 1*). 

## To change simulation

Gain access to:

*\technical_project\models\ar_marker\material\scripts\ar_marker.material*

Change  <em>texture aruco-1.png</em>  with:

<ul>
<li><em>texture aruco-2.png</em></li>

>To set AR marker identifier equal to **2** (*Simulation 2*)
 
 <li><em>texture aruco-0.png</em></li>
 
>To set AR marker identifier equal to **0** (*Simulation 3*)
 
</ul>

Start a new simulation.

## Additional features


<ul>
 
<li>If you are interested in re-planning the paths
 
Be sure that in

<em>\technical_project\config\ </em>

are not present any *path.yaml* or *obstacles.yaml* file.

Type:

`roslaunch technical_project planner.launch`
>To load the **map** and run the **planner**.


**N.B. PLANNER MIGHT TAKE MORE THAN 1h 30m TO FINISH!**
 
For this reason final results in:

*\technical_project\config\path.yaml*

*\technical_project\config\obstacles.yaml*

are already provided.</li>
 
 
<li> If you want to visualize the paths 
 
 Opne a new terminal and type:
 
`roslaunch technical_project visualization.launch`

>To run **visualization**,**RViz** and load the **map**
 
 In the same terminal type a number from 0 to 5 for the corresponding path.
 
 In RViz add by topic:
 
 - \map Map
 - \visualization_obst Marker
 - \visualization_points_and_lines Marker
 

</li>

<li> If you want to localize the robot during a simulation 
 
 Open a new terminal and type:
 
`roslaunch technical_project amcl.launch`

>To run **amcl**, **RViz** and load the **map**
 
 

</li>
</ul>
