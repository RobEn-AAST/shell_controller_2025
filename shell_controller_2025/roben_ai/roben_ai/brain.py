import rospy
from ai_src import make_carla_env

# pytorch stablebaseline gymansium
class Brain:
    def __init__(self):
        """
        The main controller of them all, the ultimate god of this body!
        Takes the image, and any other sensors we could find,
        sends them to an actor critic policy, and retreives the action we are asked to execute
        And executes it! voalla!!
        """
        rospy.init_node("brain")

        rospy.loginfo("Brain node started...")

        env = make_carla_env(eval=True)

        obs, _ = env.reset()
        while not rospy.is_shutdown():
            action = [2, 1]
            obs, r, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            rospy.loginfo(f'reward: {r}')
            if done:
                # obs = env.reset()
                rospy.loginfo('welp here went nothing')




def main(args=None):
    try:
        node = Brain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
