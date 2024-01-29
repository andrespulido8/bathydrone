## GNC scripts for Bathy-drone
`boat.py` is where we call the main simulation simulation loop and the controller.
`tethered_dynamics.py` or `open_loop_dynamics.py` is where we define the dynamics of the boat and the tether force applied to it. 
The `lqRRRT` module is the trajectory planning algorithm that we use to generate the path to follow.
`path_planner.py` is where we generate the vessel water coverage path planning

## NOTE
csv files of the field data testing needed for Bathy-drone simulation (boat.py) are in the [APRILab google drive](https://drive.google.com/drive/folders/1xcXnQYFzGNeY_nqtE-7GW_qQDqcfpFVx?usp=sharing)

csv files of the lake boundaries for path planning (path_planner2023.py) are in the [APRILab google drive](https://drive.google.com/drive/folders/16S4cXtVO6yilZiwSBgRCVeSEOzOF5Jrx?usp=drive_link)

Visit https://github.com/andrespulido8/lqRRT to properly install lqRRT and have it run

## Future ideas:
- Improve the tethered model. Make the tension force proportional to the distance instead of an on-off switch
- use an MPC controller to do trajectory optimization and reduce the error

## Debuggin
If getting a LaTeX error when plotting the sim, install LaTeX for ubuntu with `sudo apt-get install texlive-full`
