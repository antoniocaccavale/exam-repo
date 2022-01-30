# exam-repo
Welcome to the repository related to the final examinations of Robotics Lab and Field and Service Robotics courses (a.y. 2020-21).

## To start a simulation

Open a terminal and type:

1.`roslaunch technical_project scenario.launch`
>To spawn the **scenario** and the **powerbot** in Gazebo
 
2.`roslaunch technical_project simulation.launch`
>To start a simulation

By default AR marker identifier is *1*. 

## To change simulation

Gain access to:

`\technical_project\models\ar_marker\material\scripts`

In `ar_marker.material` change 'texture aruco-1.png' in:

1.`texture aruco-2.png`
>To set AR marker identifier equal to *2*

1.`texture aruco-0.png`
>To set AR marker identifier equal to *0*
