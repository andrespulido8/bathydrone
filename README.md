# bathydrone
Code for the bathydrone project, the collaboration between Dr. Ifju and Dr. Shin's lab at the University of Florida

## Contributing Guide
To make changes to this repo, it is recommended to use the tool [pre-commit](https://pre-commit.com/).
To install it, run `pip3 install -r requirements.txt` inside this repo, and then install the hooks specified in the config file by doing `pre-commit install`. Now to run it ag    ainst all the files to check if it worked, run `pre-commit run --all-files`.

## NOTE FOR ROS
Force is being applied to the wamv/base_link
Force is magnitude of 1000000.0 N in the x direction to show noticeable movement in boat.
