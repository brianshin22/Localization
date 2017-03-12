import rospy
import can_sender as cs
import location_sender as ls
import data_interfacing as di
def auto():
    cs.can_sender()
    ls.location_sender()
    di.data_listener()
    di.output()

if __name__ == '__main__':
    try:
        auto() 
    except rospy.ROSInterruptException:
        pass
