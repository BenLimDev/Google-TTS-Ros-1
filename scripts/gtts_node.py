#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import String

class FestivalTTSNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("festival_tts_node", anonymous=True)
        
        # Subscribe to the /text_to_speech topic
        rospy.Subscriber("/text_to_speech", String, self.on_text_received)
        
        rospy.loginfo("Festival TTS node is running...")

    def on_text_received(self, msg):
        """Callback when text is received from the /text_to_speech topic"""
        text = msg.data.strip()
        if text:
            self.speak(text)

    def speak(self, text):
        """Generate speech using Festival"""
        subprocess.run(['festival', '--tts'], input=text.encode('utf-8'))
        rospy.loginfo(f"Speech generated for: {text}")


if __name__ == "__main__":
    try:
        node = FestivalTTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
