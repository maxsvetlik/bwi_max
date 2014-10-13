import sys
import rospy
import time
from pcl_perception.srv import *

def detect_people_call():
    rospy.wait_for_service('segbot_pcl_person_detector/people_detection_service')
    try:
        detect_people = rospy.ServiceProxy('segbot_pcl_person_detector/people_detection_service', PeopleDetectionSrv)
        resp1 = detect_people('detect_people')
        print resp1
        return True
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
	time.sleep(3)
	print "Calling service to detect people..."
	detect_people_call()
	print "...done"
    
	
