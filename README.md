# Virtual Whiteboard

This package allows one to use a TV as a virtual whiteboard for a robot. Rather than the robot actually holding a drawing implement, you can run the virtual_whiteboard.py node and turn the TV into a whiteboard.

## Installation
This package was developed on ROS noetic and Ubuntu 20.04. In addition to the python libraries that come bundled with ROS, the following libraries are required:
- [pygame](https://www.pygame.org/wiki/GettingStarted)

I haven't figured out how to use relative paths for the svg background files, so you'll need to change the path path for the "background" ros parameter  in whiteboard_params.yaml to match your system.

After dropping this package into your ROS workspace, don't forget to do the usual steps: `chmod +x *.py` in the root directory of this repository and `catkin_make` in the root of your ROS workspace.

## Usage
First, set the screen_w_px and screen_h_px parameters in whiteboard_params.yaml to match the width and height, in pixels, of the screen you'll be using as a whiteboard. Set screen_w_meters to the width of your screen, in meters.

Launch the node with virtual_whiteboard.launch, which just loads the parameter file into ROS parameter server and launches the virtual_whiteboard.py node.

To test that things are working without hooking up a real robot, you can run the spoof_marker.py node (`rosrun virtual_whiteboard spoof_marker.py`). This just publishes a line of points to the /marker_position topic. You should see a line drawn across your screen.

## Topic and Message format
virtual_whiteboard.py listens to the /marker_position topic to know where to draw. /marker_position is a custom msg type called Marker. The origin is the bottom right corner of the screen, x goes to the right if you're facing the screen, and y goes up.