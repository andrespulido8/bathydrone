![APRILAB Logo](GRIP_Figures/aprilab-logo.png)

# Grip OpenCV Reference
*Blake Sanders - 04 June 2023*


## Overview

This document provides a reference on how to use the [GRIP](https://wpiroboticsprojects.github.io/GRIP/#/) tool for generating OpenCV pipelines, along with an example relevant to the Bathydrone project.

Note that I used Ubuntu 20.04, Python 3.9.15, and GRIP 1.5.2 on an X86 PC (not a Mac) when working with this tool, so the process may be slightly different on other platforms/operating systems/etc.

Please also note that for my specific setup, I needed to slightly edit the pipeline (e.g., changing the number of values received in a function call) in order to make it work for my setup. Be aware of that when troubleshooting.

## Installation

- Locate the latest release [here](https://github.com/WPIRoboticsProjects/GRIP/releases). (Note that later releases may be marked as pre-releases. Consider scrolling down till you find one marked as a release. See figure below for reference.)

![GRIP Release Example](GRIP_Figures/grip-docs-2.png)

- Find the appropriate download in “Assets” for your system type and operating system. Mine was the -x64.deb (64-bit AMD/Intel Processor on a Debian-based distribution, such as Ubuntu) at the top of the list.

- Download the file and begin installation. In my case, I installed it using dpkg (Debian package manager) via the following terminal command:

	sudo dpkg -i ~/Downloads/grip-1.5.2-x64.deb

- I then searched for the executable in the GUI (Windows+S hotkey by default; type “grip” into the search bar) and selecting it.

![GRIP Search in Gnome desktop environment](GRIP_Figures/grip-docs-3.png)


- We are then greeted with the GRIP interface (see figure in next section).

## Usage

![GRIP Interface](GRIP_Figures/grip-docs-3.png)

- The “Preview” window in the top left shows the results of each stage of the pipeline when we make them visible (to be explained later). 
- “Operation Palette” shows all possible OpenCV operations, along with a search bar to find them.
- The user can drag and drop operations into the section to the right of “Sources”, where an input (e.g., an image) can be selected.
- When finished, Select “Tools -> Generate Code” (or use Ctrl+G) to begin generating code.



## Creating a Pipeline

OpenCV pipelines sequentially perform operations on an input (such as a video or image) to acquire some output (in our example, a contour–polygon–surrounding a body of water).
- First select an image (“Add Source -> Image(s)”, then navigate to the image file you want to use)

![Add a Source](GRIP_Figures/grip-docs-4.png)

- Click the eye icon to make it visible in the Preview:

![Make Visible in Preview](GRIP_Figures/grip-docs-5.png)


- Then begin adding operations to the pipeline. 

For each operation: search for, then click and drag, operations into the bottom-right section of the app. Connect the image source output to the input of the first operation, the output of that operation to the input of the second, and so on.


- We will start with a blur to soften edges:

![Box Blur](GRIP_Figures/grip-docs-6.png)


- Then an HSV (hue/saturation/value) threshold to filter for water colors:

![HSV Filter](GRIP_Figures/grip-docs-7.png)

- Then we will find contours:

![Find Contours](GRIP_Figures/grip-docs-8.png)

- Then filter them by minimum area:

![Filter Contours](GRIP_Figures/grip-docs-9.png)

The result of this pipeline is a contour, or effectively a list of points, surrounding the water body of interest, which can then be used as needed in the calling code.


## Generating Code

Select Tools -> Generate Code (or press Ctrl+G) to open the below prompt, selecting the appropriate language, save location, and name:

![Generate Code](GRIP_Figures/grip-docs-A.png)


Once it generates, you can exit GRIP. Save your GRIP project if you like.

The pipeline can then be called in another Python script by placing it in the same directory and importing it:

	import grip # replace "grip" with the name of the pipeline file you generated

Example code can be found in the [Bathydrone Project GitHub](https://github.com/andrespulido8/bathydrone/blob/maps_image_processing/maps_image_processing/water_body_demo.py).
