
# Project: APRILab FastSAM UI
Goal: Design a UI to integrate the path planning workflow for the [Bathydrone](https://aprilab.mae.ufl.edu/research/), using [FastSAM](https://github.com/CASIA-IVA-Lab/FastSAM) for water body detection.

## Setup and Installation (Ubuntu 20.04):
Clone the repo:
```console
$ git clone https://github.com/andrespulido8/bathydrone.git 
```

Once you have cloned and entered this directory in the repo, it is recommended to create a virtual environment using [venv](https://docs.python.org/3/tutorial/venv.html):
### 1. Install Python and venv
- (Verified on version 3.9. FastSAM relies on 3.7 or later; see https://github.com/CASIA-IVA-Lab/FastSAM#installation for more information.)
```console
$ sudo apt install python3.9
$ sudo apt install python3.9-venv
```
- (Optional): Add an alias for that specific Python version to your .bashrc:
```console
alias python3.9='/usr/bin/python3.9'
```
### 2. Create a virtual environment
- Make sure you use the Python version you just installed when creating the virtualenv!
```console
$ mkdir .venv && cd .venv
$ python3.9 -m venv fastsam-env # replace 'fastsam-env' throughout with name of your choosing
$ source fastsam-env/bin/activate
$ cd .. # back to the root directory of the repo
```
- Verify that the environment is activated before continuing! Your terminal should look like:
```console
(fastsam-env) user@user-host-machine:path/to/installation$ 
# e.g.,
(fastsam-env) blake@blake-laptop-ubu:~/Documents/GitHub/SAM_Testing$
```
- You can also confirm the version of Python (and corresponding version of pip):
```console
$ python --version
Python 3.9.5
$ pip --version
pip 20.0.2 from /home/blake/Documents/GitHub/SAM_Testing/.venv/fastsam-env/lib/python3.9/site-packages/pip (python 3.9)
```
### 3. install required packages
- First install PyTorch and TorchVision [here](https://pytorch.org/get-started/locally/). If you have an NVIDIA GPU, it is highly recommended to install the CUDA versions. Example below:
```console
$ pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```
- Install requirements from the file included in the repo:
```console
$ pip install wheel
$ pip install -r requirements.txt
```
- If you want to run the generate a path, you may need to also install the path planner dependencies
```console
$ cd ../gnc/
$ pip install -r requirements.txt
$ cd ../path_planning_ui/
```
- Install CLIP:
```console
$ pip install git+https://github.com/openai/CLIP.git
```

# Usage
Once finished setting up a virtual environment and installing the necessary packages, the UI can be opened with
```console
$ python april_fastsam_ui.py
```
After a few seconds, the UI should pop up as a separate window:
![image](docs/april_fastsam_ui.png)

# Important 
As of 10/7/24, before running the APRIL FASTSAM GUI, ensure that the following scripts are in the same directory as the APRIL FASTSAM GUI python script (currently named 'april_fastsam_ui_8_1_24.py'): 'mission_planner_code_python.py', 'adding_accel_decel_waypoints.py', and 'LatLongTurn_Integrated_Code.py'.

Once the GUI program is open, select the 'Select image file' widget located on the top left of the GUI. This will prompt the user to select an image. Select the 'MavicCitra' image. Once selected, the GUI should appear in a similar figure to that as Fig 1. 

![GUI_Image_1](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/GUI_Image_1.png?raw=true)
Fig 1). Screen shot of FASTSAM GUI appearance after the Lake Citra satellite image is selected. 


After the satellite image is imported into the GUI, select the ‘Segment Image’ widget located directly below the ‘Select image file’ to segment objects within the image into different colors. 

Once the image segmentation has taken place, click on the lake located within the image to create a contour of the lake. After the contour is created, select the ‘Generate Path’ widget located on the bottom of the left most column of the GUI. After both steps have been completed, the GUI should appear like Figure 2. 

![GUI_Image_2](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/GUI_Image_2.png?raw=true)
Fig 2).  Screen shot of FASTSAM GUI after the 'Segment Image' and 'Generate Path' widgets have been clicked. 


Next, check the ‘Lat-long points’ box located on the left most column. This will allow for the entries of each latitude and longitude box to be “un-greyed” allowing for the user to enter the corresponding latitude and longitude values associated with the top, bottom, left, and right most points of the lake. 

The following values shown in Figure 3 were inputted into the GUI for the bilinear interpolation calculation. 

![GUI_Image_3](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/GUI_Image_3.png?raw=true)

Fig 3). Screen shot of latitude and longitude values inserted into GUI. 

After the appropriate latitude and longitude values are inputted into the GUI, the user is to select the ‘Export Waypoint File’ widget. This will generate a text file that ends with ‘.waypoint’. The waypoint file can be located within the same directory that the FAST SAM GUI python program is located in. 

After the .waypoint file is generated, the user is to open Mission Planner. 
The interface for Mission Planner should be identical to that as shown in Figure 4.

![MissionPlannerImage1](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/Mission_Planner_Image1.png?raw=true)
Fig 4). Screen shot of Mission Planner user interface when it is initially opened. 


Once Mission Planner is open, the user is to select ‘Simulation’ (located on the topmost row) and then select ‘Multirotor’. A pop up will appear asking the user if they would prefer the “Dev” or “Stable” version to be run. The user is to select the “Stable” version. After, the Mission Planner interface should appear identical to that of Figure 5. 

![MissionPlannerImage2](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/Mission_Planner_Image2.png?raw=true)
Fig 5). Screen shot of Mission Planner interface after selecting the stimulation for a multirotor. 


Subsequently, the user presses ‘Plan’ (located on the topmost row of the user interface) and then the ‘Load File’ button to load the .waypoint file into Mission Planner. Once the .waypoint file has been imported, the user presses ‘Write’. The Mission Planner interface should appear identical to Figure 6. 

![MissionPlannerImage3](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/Mission_Planner_Image3.png?raw=true)
Fig 6).  Screen shot of Mission Planner interface after .waypoint file has been loaded and written into Mission Planner. 


Next, the user presses ‘Data’ (located on the top left corner of the interface) and then the ‘Actions’ tab located on the left of the interface (visual provided on Figure 7). 

Subsequentially, the user clicks the drop-down menu right next to the ‘Do Action’ button. The user selects the ‘Mission_Start’ option and then the ‘Arm/Disarm” button. 

Lastly, the user clicks the ‘Do Action’ button to initiate the simulation. 

![MissionPlannerImage3](https://github.com/andrespulido8/bathydrone/blob/GUI-testing/path_planning_ui/docs/Mission_Planner_Image4.png?raw=true)

Fig 8). Screen shot of Mission Planner displaying the 'Actions' tab after 'Data' is pressed. 
