#!/usr/bin/env python
import rospy
import os

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Header, ColorRGBA
import time
import Queue
import roslaunch
import threading 
import json
from rospy_message_converter import json_message_converter



from agv_central.srv import maps, mapsResponse
from agv_central.srv import waypoints, waypointsResponse

q_maker = Queue.Queue()
q_clear_marker = Queue.Queue()
q_sync = Queue.Queue()

q_wayponts = Queue.Queue()
q_wayponts_names = Queue.Queue()



robot_names = ['eru','bot1']

#robot_names = ['eru']

MAP_PATH = '/home/pi/linorobot_ws/src/linorobot/maps'



class markerSub:

    def __init__(self, topic, iid, name):
        self.sub = rospy.Subscriber(topic, Marker, self.callback)
        self.count = 0
        self.timer = threading.Timer(5,self.timeout) # If 5 seconds elapse, call timeout()
        self.timer.start()
        self.id = iid
        self.name = name

    def timeout(self):
        print("No message received for 5 seconds for " + self.name)
        q_clear_marker.put(self.name)           
        self.timer.cancel()
        self.timer = threading.Timer(5,self.timeout)
        self.timer.start()


        #q_maker.put(Marker(text=self.name, id=self.id))
        # Do something

        
    def callback(self,data):           
        self.timer.cancel()
        self.timer = threading.Timer(5,self.timeout)
        self.timer.start()

        
        self.count = self.count + 1
        if self.count == 5:
            self.count = 0
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.text) 


            q_maker.put(data)


def get_maps(mess):
    print "get map"
    file_names = []

    for r, d, f in os.walk(MAP_PATH):
        for file in f:
            if ".yaml" in file:
                filename, file_extension = os.path.splitext(file)
                file_names.append(filename)
                print file

    if mess.onezero==1:
        return  mapsResponse(file_names)
    else:
        return  mapsResponse(file_names)
        
def get_waypoints(mess):
    print "get waypoints"
    res = 'OK'
    
    print mess
   
    pose_a1 = []
    name_a1 = []
   
    with open(os.path.join(MAP_PATH, mess.mapname + '.json')) as json_file:
        data = json.load(json_file)

    for key, val in data.items():
        pose_w = json_message_converter.convert_json_to_ros_message('geometry_msgs/Pose', val)
        pose_a1.append(pose_w)
        name_a1.append(key)
    
    print pose_a1
    print name_a1

    q_wayponts.put(pose_a1)
    q_wayponts_names.put(name_a1)

    if 1==1:
        return  waypointsResponse('OK')
    else:
        return  waypointsResponse('OK')





def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('central', anonymous=True)
    rate = rospy.Rate(20.0)

    
    markers = {}
    markerArray = MarkerArray()
    markerArrayText = MarkerArray()

    markerArray2 = MarkerArray()
    markerArray3 = MarkerArray()
    
    agv_location_publisher = rospy.Publisher('visualization_marker_array', MarkerArray,  queue_size=5)
    agv_location_text_publisher = rospy.Publisher('visualization_marker_array_text', MarkerArray,  queue_size=5)

    waypoints_publisher = rospy.Publisher('waypoints_marker_array', MarkerArray,  queue_size=5)
    waypoints_publisher_text = rospy.Publisher('waypoints_marker_array_text', MarkerArray,  queue_size=5)



    get_map=rospy.Service('get_map', maps,  get_maps)
    get_waypoint=rospy.Service('get_waypoints', waypoints,  get_waypoints)

    isSyncRunning = False
    
    ppose = Pose()
    ppose.position.x = 0
    ppose.position.y = 0
    ppose.position.z = 0
    ppose.orientation.x = 0
    ppose.orientation.y = 0
    ppose.orientation.z = 0
    ppose.orientation.w = 0
    ii = 0
    for n in robot_names:
        topic = "/" + n+"/visualization_marker"
        markers[n] = Marker(text=n)
        markerArray.markers.append(Marker(text=n, id=ii))
        markerArrayText.markers.append(Marker(text=n, id=ii))
        obc = markerSub(topic = topic, name = n, iid=ii)
        ii = ii + 1
    #print(markerArray)
    

    while not rospy.is_shutdown():
        if not q_maker.empty():
            mmk =  q_maker.get()
            markers[mmk.text] = mmk
            
            idx = robot_names.index(mmk.text)
            mmk.id = idx
            #print(mmk)
            #print(idx)

            markerArray.markers[idx] = mmk
            #print(markerArray)
            agv_location_publisher.publish(markerArray)

            ik_pose = mmk.pose
            ik_pose.position.z = ik_pose.position.z + 0.1
 
            markerArrayText.markers[idx] = Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=idx,
                    pose=ik_pose,
                    scale=Vector3(0.3, 0.3, 0.3),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 0.1, 1.0, 1.0),
                    text=mmk.text)
            agv_location_text_publisher.publish(markerArrayText)
            #print markerArray

        if not q_clear_marker.empty():
            r_name = q_clear_marker.get()    
            print(r_name)
            
            
            idx = robot_names.index(r_name)
            markerArrayText.markers[idx] = Marker(type=Marker.TEXT_VIEW_FACING, id=idx)
            markerArray.markers[idx] = Marker(text=r_name, id=idx)
            agv_location_text_publisher.publish(markerArrayText)
            agv_location_publisher.publish(markerArray)


        if not q_wayponts.empty():
            wpts = q_wayponts.get()
            wp_names = q_wayponts_names.get()
            print "========>"
            print len(wpts)
            #print wpts
            mk_length = len(markerArray2.markers)
            id = 0
            id_t = len(markerArray2.markers) + 1
            i = 0
            del markerArray2.markers[:]
            del markerArray3.markers[:]
            for x in range(0, mk_length):
                #marker = 
                markerArray2.markers.append(Marker(
                    type=Marker.ARROW,
                    id=id,
                    action=2,
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 1.0, 0.0, 1.0),
                    text="AGV"))


                markerArray3.markers.append(Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=id_t,
                    action=2,
                    scale=Vector3(0.3, 0.3, 0.3),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 0.1, 1.0, 1.0),
                    text="null"))
                
                i = i + 1

                    
                id += 1
                id_t += 1


            waypoints_publisher.publish(markerArray2)
            waypoints_publisher_text.publish(markerArray3)
            print "clear========>"

            del markerArray2.markers[:]
            del markerArray3.markers[:]
            #markerArray2.markers = []
            #markerArray3.markers = []
            id = 0
            id_t = len(wpts) + 1
            i = 0
            for x in wpts:
                #marker = 
                markerArray2.markers.append(Marker(
                    type=Marker.ARROW,
                    id=id,
                    pose= x,
                    lifetime = rospy.Duration.from_sec(1),
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 1.0, 0.0, 1.0),
                    text="AGV"))
           
        
                
                ik_pose = Pose()
                ik_pose.position.x = x.position.x
                ik_pose.position.y = x.position.y
                ik_pose.position.z = x.position.z + 0.1
                ik_pose.orientation.x = x.orientation.x
                ik_pose.orientation.y = x.orientation.y
                ik_pose.orientation.z = x.orientation.z
                ik_pose.orientation.w = x.orientation.w
                markerArray3.markers.append(Marker(
                    type=Marker.TEXT_VIEW_FACING,
                    id=id_t,
                    pose=ik_pose,
                    scale=Vector3(0.3, 0.3, 0.3),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 0.1, 1.0, 1.0),
                    text=wp_names[i]))
                
                i = i + 1

                    
                id += 1
                id_t += 1

            p_waypoint_length = len(wpts)
            p_wpts = wpts
            
                
            
            #print markerArray3

            waypoints_publisher.publish(markerArray2)
            waypoints_publisher_text.publish(markerArray3)

    rate.sleep()

        


            

        

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()

