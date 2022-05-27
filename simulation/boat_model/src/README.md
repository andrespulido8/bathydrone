To regenerate the urdf model run: 
```rosrun xacro xacro bathyboat_custom.urdf.xacro > boat_generated.urdf```
The bathyboat_custom.urdf.xacro file represents the barebones model with just the base link
and both the tension and dynamics plugin.  
The ./bathyboat/bathyboat_base.urdf.xacro file represents the base link with the appropiate collision shapes and masses.  
