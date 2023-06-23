#%%
#!/usr/bin/env python
import rospy
from pynput.keyboard import Listener, KeyCode
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
class Ellipse():
    def __init__(self):
        rospy.init_node('LfD', anonymous=True)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        self.end=False
        self.r=rospy.Rate(10)
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True           
    def visualize_ellispoid(self):
        while not(self.end):
            # Create a marker message
            marker = Marker()
            marker.header.frame_id = "panda_hand"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(0.5, 0.2, 0.2)  # Dimensions of the ellipsoid
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            # Set the position and orientation of the ellipsoid
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0


            # Publish the marker
            self.marker_pub.publish(marker)
            self.r.sleep()
#%%   
Ellipse=Ellipse()
#%%
Ellipse.visualize_ellispoid()






