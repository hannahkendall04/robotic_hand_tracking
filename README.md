# Gesture-Guided Robotics: Real-Time End Effector Teleoperation

## Overview 

This code base implements robotic teleoperation using visual hand tracking. The program uses MuJoCo's Robosuite module to run and render the robot simulation, and uses Google's MediaPipe Hand Tracking model and OpenCV to track the location of the user's hand in real-time.

## How to Run 

**Note:** The following instructions have been tested on a Windows machine only. If you are using a Mac or Linux machine, you may have to modify certain aspects of the program to get it to run. Based on what I have seen online, one common issue for Mac systems is the live webcam rendering, which would make running this project difficult. Thankfully, there are resources online that can be used to work around this issue if/when it occurs.

#### Package Installation and Set-Up

Run the following commands to install the necessary module prerequisites.

```shell
# Windows instructions
python -m venv .venv
.venv/Scripts/activate 
pip install -r requirements.txt

# Mac/Linux instructions
python -m venv .venv 
source .venv/bin/activate
pip install -r requirements.txt
```


If you run into issues installing or using Robosuite after running the above commands, refer to Robosuite's documentation <a href="https://robosuite.ai/docs/installation.html">here</a>.


#### Running the Program 

Run the following command to start the program:

```shell
python run_tracking.py
```

The default environment that is loaded is the 'Stack' environment using Robosuite's Panda robot. You can change these parameters by modifying the below section in `run_tracking.py`:

```python
env = suite.make(
    env_name="Stack",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    horizon=6000,
    control_freq=20
)
```

The Square Nut Assembly task is commented out in `run_tracking.py` for your convenience. For a full list of environments and robots supported by Robosuite, refer to their <a href="https://robosuite.ai/docs/overview.html">documentation</a>.


#### Tracking Parameters 

Using Google MediaPipe's Hand Tracking model, the program tracks the location of your right hand thumb, index finger, and middle finger in real-time.
- The position of the tip of your index finger is used to move the end effector of the robot. 
- The distance between the tip of your index finger and the tip of your thumb is used to determine whether the gripper should be open or closed. 
- Finally, the distance between the tip of your index finger and the tip of your middle finger is used to determine the current 'movement mode' of the system.
    - If your middle finger and index finger are close together, the movement mode will be set to 'depth', otherwise it will be set to 'lateral'. 
    
There are two movement modes, 'depth' and 'lateral'. 
- In the 'lateral' mode, the x-direction movement of your index finger will be mirrored in side-to-side/horizontal movement of the robot's end effector, and y-direction movement will be mirrored with up-down/vertical movement.  
- In 'depth' mode, horizontal movement of your index finger will result in forward/backwards movement of the end effector (right for forward movement and left for backward movement). 
    - Vertical movement of your finger will have no effect on the robot's end effector in this mode.

The goal of the default Stack environment is to pick up the red cube and place it on top of the green cube. Currently, this program does not support rotation of the end effector, so you may need to be creative to get the program to complete the task! The default environment will run 5 iterations of the task, moving on to the next iteration after successful stacking or after a timeout period of 5 minutes.


**Step-by-Step Guide**
1. Run the program using `python run_tracking.py`.
2. Resize the webcam and simulation windows so you can adequately see both simultaneously.
3. Hold your right hand in a backwards L shaped position, with your right thumb pointing to the left, your index finger pointing to the ceiling, and the rest of your fingers curled towards your palm.
4. Move the tip of your index finger to the green circle in the upper right section of your screen.
5. Once the position has been calibration you can either:
    1. Move the end effector side-to-side by keeping your 3rd-5th fingers curled and moving your index finger away from the green dot.
    2. Un-curl your middle finger and place it directly next to your index finger to activate 'depth' mode. Move your index finger to the right to bring the end effector forward and/or move your index finger to the left to move the end effector backwards.
6. Play around with movement combinations to complete the task!!

## Final Notes

As mentioned above, this program does not currently support end effector rotation, however that is something I hope to add in the future! While relatively straightforward, I think this program is a fun and hands-on intro to Computer Vision applications in robotics, and I hope you enjoy using this program as much as I do!