To regenerate the urdf model run: 
```rosrun xacro xacro bathyboat_custom.urdf.xacro > boat_generated.urdf```
The bathyboat_custom.urdf.xacro file represents the barebones model with just the base link
and both the tension and dynamics plugin.  
The bathywamv.urdf.xacro file represents the more commplete model with sensors and thrusters
that VRX uses for competition.  
