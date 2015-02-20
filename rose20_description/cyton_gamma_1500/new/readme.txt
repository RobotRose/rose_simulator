Het nieuwe urdf bestand is met behulp van xacros uitgewerkt. 

- base
 *part 1: cilinder
 *part 2: cilinder 
- arm
 *joint 1 t/m 5 bestaande uit een cilinder en een box
- gripper
 *part 1 t/m 5 bestaande uit kubussen

In het bestand: properties.urdf.xacro staan alle properties van de robot.
Deze zijn individueel aan te passen zonder dat alle andere joints verschuiven. (zie afbeelding vb1.png in de map screenshots)

vragen? -> j.metzemaekers@student.fontys.nl
==================================================================================================
opstarten:
roslaunch cyton_gamma display.launch
==================================================================================================
