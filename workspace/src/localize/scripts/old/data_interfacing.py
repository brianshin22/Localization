import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D

X=[]
Y=[]
Psi=[]
rt=[]

def callback1(location):
    global X,Y,Psi
    X.append(pose.x)
    Y.append(pose.y)
    Psi.append(pose.theta)

def callback2(runtime)
    global rt
    rt.append(runtime.data)

def output():
    global X,Y,Psi,rt
    x=0
    for eachRT in rt
	saveLine=eachRT+","+X[x]+","+Y[x]+","+Psi[x]+"\n"
        saveFile=open("PFdata.csv","a")
	saveFile.write(saveLine)
	x+=1
    saveFile.close()

def data_listener():
    rospy.init_node('data_interfacing', anonymous=True)
    rospy.Subscriber("location", Pose2D, callback1)
    rospy.Subscriber("runtime", Int32, callback2)

f __name__ == '__main__':
    try:
        data_listenser()
    except rospy.ROSInterruptException:
        pass
    output()

