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
        rospy.sleep(2)  # Short sleep to ensure ROS node is ready

        # Speak a test sentence at startup
        self.speak("Preloaded phrases working, lets go team airost")

    def on_text_received(self, msg):
        """Callback when text is received from the /text_to_speech topic"""
        text = msg.data.strip()
        if text:
            self.speak(text)

    def speak(self, text):
        """Generate speech using Festival"""
        try:
            # Using subprocess with pipe to Festival
            process = subprocess.Popen(['festival', '--tts'], stdin=subprocess.PIPE)
            process.communicate(input=text.encode('utf-8'))  # Sending text to festival
            rospy.loginfo(f"Speech generated for: {text}")
        except Exception as e:
            rospy.logerr(f"Error while generating speech: {e}")


if __name__ == "__main__":
    try:
        node = FestivalTTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
