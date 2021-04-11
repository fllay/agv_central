# agv_central


To show the map we need a map server to publish a map

```rosrun map_server map_server /home/pi/agv/maps/house_2nd_floor.yaml```


To show waypoints on the map, there is a service in ```central.py``` to get waypoints

```rosservice call /get_waypoints "mapname: 'house_2nd_floor'"```

The detail of ```central.py``` node is in the following


```
pi@pi:~/catkin_ws/src/agv_central$ rosnode info central_4288_1618140417275
--------------------------------------------------------------------------------
Node [/central_4288_1618140417275]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /visualization_marker_array [visualization_msgs/MarkerArray]
 * /visualization_marker_array_text [visualization_msgs/MarkerArray]
 * /waypoints_marker_array [visualization_msgs/MarkerArray]
 * /waypoints_marker_array_text [visualization_msgs/MarkerArray]

Subscriptions: 
 * /bot1/visualization_marker [visualization_msgs/Marker]
 * /eru/visualization_marker [visualization_msgs/Marker]

Services: 
 * /central_4288_1618140417275/get_loggers
 * /central_4288_1618140417275/set_logger_level
 * /get_map
 * /get_waypoints


contacting node http://pi:33613/ ...
Pid: 4288
Connections:
 * topic: /waypoints_marker_array
    * to: /rosbridge_websocket
    * direction: outbound (35949 - 127.0.0.1:45330) [20]
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound (35949 - 127.0.0.1:45282) [9]
    * transport: TCPROS
 * topic: /waypoints_marker_array_text
    * to: /rosbridge_websocket
    * direction: outbound (35949 - 127.0.0.1:45340) [21]
    * transport: TCPROS
 * topic: /visualization_marker_array_text
    * to: /rosbridge_websocket
    * direction: outbound (35949 - 127.0.0.1:45328) [19]
    * transport: TCPROS
 * topic: /visualization_marker_array
    * to: /rosbridge_websocket
    * direction: outbound (35949 - 127.0.0.1:45318) [14]
    * transport: TCPROS
 * topic: /eru/visualization_marker
    * to: /mqtt_bridge (http://pi:42111/)
    * direction: inbound
    * transport: TCPROS
 * topic: /bot1/visualization_marker
    * to: /mqtt_bridge (http://pi:42111/)
    * direction: inbound
    * transport: TCPROS

```
