# bathydrone
Code for the bathydrone project, the collaboration between Dr. Ifju and Dr. Shin's lab at the University of Florida

## Setup and Installation (Ubuntu 20.04):
Clone the repo:
```console
$ git clone https://github.com/andrespulido8/bathydrone.git
```

Once you have cloned and entered this directory in the repo, it is recommended to create a virtual environment using [venv](https://docs.python.org/3/tutorial/venv.html):

### 1. Install Python and venv
- Get python and virtual environment
```console
$ sudo apt install python3.9
$ sudo apt install python3.9-venv
```
(For GUI: Verified on version 3.9. FastSAM relies on 3.7 or later; see https://github.com/CASIA-IVA-Lab/FastSAM#installation for more information.)
- (Optional): Add an alias for that specific Python version to your .bashrc:
```console
alias python3.9='/usr/bin/python3.9'
```
### 2. Create a virtual environment
- Make sure you use the Python version you just installed when creating the virtualenv!
```console
$ mkdir .venv && cd .venv
$ python3.9 -m venv bathy-env # replace 'bathy-env' throughout with name of your choosing
$ source bathy-env/bin/activate
$ cd .. # back to the root directory of the repo
```
- Verify that the environment is activated before continuing! Your terminal should look like:
```console
(bathy-env) user@user-host-machine:path/to/installation$
# e.g.,
(bathy-env) blake@blake-laptop-ubu:~/Documents/GitHub/SAM_Testing$
```
- You can also confirm the version of Python (and corresponding version of pip):
```console
$ python --version
Python 3.9.5
$ pip --version
pip 20.0.2 from /home/blake/Documents/bathydrone/.venv/bathy-env/lib/python3.9/site-packages/pip (python 3.9)
```
### 3. Install required packages
- For GUI: First install PyTorch and TorchVision [here](https://pytorch.org/get-started/locally/). If you have an NVIDIA GPU, it is highly recommended to install the CUDA versions. Example below:
```console
$ pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```
- For GUI: Install requirements from the file included in the repo:
```console
$ cd path_planning_ui/
$ pip install wheel
$ pip install -r requirements.txt
```
- If you want to run the generate a path, you may need to also install the path planner dependencies
```console
$ cd gnc/
$ pip install -r requirements.txt
```
- Install CLIP:
```console
$ pip install git+https://github.com/openai/CLIP.git
```

### 4. Deactivate the virtual environment
To deactivate the virtual environment, simply run:
```console
$ deactivate
```

## Contributing Guide
To make changes to this repo, it is recommended to use the tool [pre-commit](https://pre-commit.com/).
To install it, run `pip3 install -r requirements.txt` inside this repo, and then install the hooks specified in the config file by doing `pre-commit install`. Now to run it ag    ainst all the files to check if it worked, run `pre-commit run --all-files`.

## NOTE FOR ROS
Force is being applied to the wamv/base_link
Force is magnitude of 1000000.0 N in the x direction to show noticeable movement in boat.
