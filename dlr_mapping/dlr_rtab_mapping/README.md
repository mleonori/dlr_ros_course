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
reset; mon launch dlr_rtab_mapping dlr_vessel.launch  
```

Open a new terminal and launch the rqt gui vessel gmapping perspective:
```
rosrun rqt_gui rqt_gui --perspective-file src/dlr_ros_course/dlr_mapping/dlr_rtab_mapping/config/dlr_rtab_mapping.perspective 
```
