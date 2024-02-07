#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from matplotlib.figure import Figure
from matplotlib.pyplot import imread
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import numpy as np
import sys
from PIL import Image as PIL_Image
import rospy  # ROS Python library
from std_msgs.msg import String  # Import the ROS message type you want to use

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import Image as SensorImage # ROS Image message
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
import cv2 # OpenCV2 for saving an image


# Instantiate CvBridge
bridge_color = CvBridge()

# ROS Initialization
rospy.init_node("robot_control_gui")

def imgmsg_to_cv2_custom(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

# Callback function to handle incoming ROS messages
def ros_message_callback(data):
    message_text.set(data.data)  # Update the text in the text box with the ROS message content

def ros_image_callback(img_msg):
    np_img_orig = imgmsg_to_cv2_custom(img_msg)
    # Update the Matplotlib subplot with the new image
    #ax_rgb.clear()
    ax_rgb.imshow(np_img_orig)
    canvas2.draw()

# Create a subscriber to listen to ROS messages
rospy.Subscriber("/your_ros_topic", String, ros_message_callback)  # Replace with the actual ROS topic name and message type
rospy.Subscriber("/rgb", SensorImage, ros_image_callback)

# load action controller to sent goal
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()


map_image = PIL_Image.open('src/human_detection/map/generated_map.png').convert('LA')
w, h = map_image.size
print('size', w, h)
# Create the main application window
root = tk.Tk()
root.title("Robot Control GUI")

# Create a Frame for the map
map_frame = ttk.Frame(root)
map_frame.grid(row=0, column=0, padx=10, pady=10)

# Create a Matplotlib figure for the map
fig = Figure(figsize=(8, 6))
ax = fig.add_subplot(111)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Building Map")
ax.imshow(map_image)

# Create a canvas for Matplotlib figure
canvas = FigureCanvasTkAgg(fig, master=map_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Function to handle map click event
def on_map_click(event):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x=x*0.02-63.12
        y=y*0.02-10.00
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.x = y
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0
        # Sends the goal to the action server.
        client.send_goal(goal)
        messagebox.showinfo("Destination Selected", f"Robot will go to ({x:.2f}, {y:.2f})")

# Bind the map click event to the function
canvas.mpl_connect("button_press_event", on_map_click)

# Create an Exit button
exit_button = ttk.Button(root, text="Exit", command=root.quit)
exit_button.grid(row=1, column=0, padx=10, pady=10, sticky="e")

# Create a text box to display ROS messages
message_text = tk.StringVar()
message_box = ttk.Label(root, textvariable=message_text, wraplength=400)
message_box.grid(row=1, column=0, padx=10, pady=10)

video_frame = ttk.Frame(root)
video_frame.grid(row=2, column=0, padx=10, pady=10)

# Create a Matplotlib figure for the map
fig2 = Figure(figsize=(4, 3))
ax_rgb = fig2.add_subplot(111)
ax_rgb.set_title("RGB camera")

# Create a canvas for Matplotlib figure
canvas2 = FigureCanvasTkAgg(fig2, master=video_frame)
canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Start the GUI main loop
root.mainloop()

