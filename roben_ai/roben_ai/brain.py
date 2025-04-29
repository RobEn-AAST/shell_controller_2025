#!/usr/bin/env python3
import rospy
from ai_src import make_carla_env

class Brain:
    def __init__(self):
        rospy.init_node('brain', anonymous=False)
        rospy.loginfo("Brain node started...")
        
        # Initialize CARLA Gym environment
        self.env = make_carla_env(eval=True)
        self.obs, _ = self.env.reset()
        rospy.loginfo("done resetting...")
        
        # Start the control loop
        self.run_loop()

    def run_loop(self):
        rospy.loginfo("Started playing loop...")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            action = [2, 1]  # Replace with your RL policy later
            obs, reward, terminated, truncated, _ = self.env.step(action)
            rospy.loginfo(f"Reward: {reward}")
            rate.sleep()

if __name__ == "__main__":
    try:
        Brain()  # Just instantiate the class; it handles everything
    except rospy.ROSInterruptException:
        rospy.logwarn("Brain node shutdown requested.")