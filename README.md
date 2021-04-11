# agv_central


To show the map we need a map server to publish a map

```rosrun map_server map_server /home/pi/agv/maps/house_2nd_floor.yaml```


To show waypoints on the map, there is a service in central.py to get waypoints

```rosservice call /get_waypoints "mapname: 'house_2nd_floor'"```



