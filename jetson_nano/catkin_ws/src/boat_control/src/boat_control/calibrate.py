from boat_control import apm
import rospy
import random

if __name__ == "__main__":
    rospy.init_node('apm_calibrate', anonymous=True)

    apm.set_apm_mode("MANUAL")

    print("Channels before:")
    print(apm.get_rc_channels())

    apm.set_rc_channel(0, random.randint(800, 1500))

    print("Channels after:")
    print(apm.get_rc_channels())


# %%