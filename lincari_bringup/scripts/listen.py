#!/usr/bin/env python3

import rospy
from rospy import AnyMsg
import os

def speech_callback(data):
    """
    Callback function for receiving speech data from the ROS topic.
    """
    rospy.loginfo("Received a message!")
    rospy.loginfo(data)
   # rospy.loginfo(f"Message type: {type(data)}")
   # rospy.loginfo(f"Message attributes: {dir(data)}") # List all attributes

    # Try to access fields directly as per rostopic echo output
    # This is the most common way AnyMsg works for simple messages
    incremental_text = getattr(data, 'incremental', None)
    final_text = getattr(data, 'final', None)
    confidence_value = getattr(data, 'confidence', None)
    header = getattr(data, 'header', None)


    if incremental_text is not None and incremental_text:
        rospy.loginfo(f"Incremental Speech: {incremental_text}")
    else:
        rospy.loginfo(f"Incremental field not found or empty. Value: {incremental_text}")

    if final_text is not None and final_text:
        rospy.loginfo(f"Final Speech: {final_text}")
        if confidence_value is not None:
            rospy.loginfo(f"Confidence: {confidence_value:.2f}")
        rospy.loginfo("-" * 30)
    else:
        rospy.loginfo(f"Final field not found or empty. Value: {final_text}")

    # If the above direct attribute access fails, the data might be in a different format.
    # Print the raw string representation of the message as a fallback:
    rospy.loginfo(f"Raw message data (repr): {repr(data)}")
    rospy.loginfo(f"Raw message data (str): {str(data)}")
    rospy.loginfo("--- End of message processing ---")


def speech_logger():
    rospy.init_node('anonymous_speaker_speech_logger', anonymous=True)
    rospy.Subscriber('/humans/voices/anonymous_speaker/speech', AnyMsg, speech_callback)
    rospy.loginfo("Speech logger node started. Listening for speech on /humans/voices/anonymous_speaker/speech...")
    rospy.spin()

if __name__ == '__main__':
    try:
        speech_logger()
    except rospy.ROSInterruptException:
        rospy.loginfo("Speech logger node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")