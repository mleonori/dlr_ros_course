To launch GMapping:

Launch a roscore
``` 
roscore
```

Open a new terminal and launch the DLR vessel gazebo simulation:
```
reset; mon launch dlr_vessel_description spawn_dlr_vessel.launch  
```

Open a new terminal and launch the DLR vessel gmapping:
```
reset; mon launch dlr_gmapping dlr_vessel.launch  
```

Open a new terminal and launch the rqt gui vessel gmapping perspective:
```
reset; mon launch dlr_gmapping dlr_vessel.launch  
```
rosrun rqt_gui rqt_gui --perspective-file src/dlr_ros_course/dlr_mapping/dlr_gmapping/config/gmapping.perspective