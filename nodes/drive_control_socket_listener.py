#!/usr/bin/env python
import rospy
#import socket
import json
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import numpy as np

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
 

# this node will be listening to the socket connection for drive messages and send those to the drive_control.py
# and subscribes to some ROS topics and send those messages to the client via the websocket as JSON messages

wss = []
class WSHandler(tornado.websocket.WebSocketHandler): # this is the websocket connection handler class for websocket events from Tornado
    def check_origin(self, origin):
        return True
    
    def open(self):
        print 'new connection'
        self.write_message("Connection established")
        if self not in wss:
            wss.append(self)
      
    def on_message(self, message):
        print 'message received %s' % message
        pub.publish(message) # send message to the ROS topic 
 
    def on_close(self):
        print 'connection closed'
        if self in wss:
            wss.remove(self)

def wsSend(message): # it sends a message to the websocket, see http://stackoverflow.com/questions/12479054/how-to-run-functions-outside-websocket-loop-in-python-tornado
    for ws in wss:
        if not ws.ws_connection.stream.socket:
            print "Web socket does not exist anymore!!!"
            wss.remove(ws)
        else:
            ws.write_message(message)

# initiate the websocket tornado server application under the /ws url
application = tornado.web.Application([
    (r'/ws', WSHandler),
])

# callback function on the ROS topics which will publish the messages coming from the ROS topics to the websocket
def speed_angle_msg(angle):
    angle = str(angle).split(' ') # the data arriving from ROS is in the format of "data: 72" and we need the number (72), so we split it and then take the number part
    angle = int(angle[1])
    message = json.dumps({"type": "speed_angle", "value": angle}) # build JSON, see https://docs.python.org/2/library/json.html
    wsSend(json.dumps(message))
    
def direction_angle_msg(angle):
    angle = str(angle).split(' ') # the data arriving from ROS is in the format of "data: 72" and we need the number (72), so we split it and then take the number part
    angle = int(angle[1])
    message = json.dumps({"type": "direction_angle", "value": angle})
    wsSend(message)

## ROS setup - publishers, subscriber and node
pub = rospy.Publisher('drive_control_publish', String, queue_size=10) # create the ROS publisher to which incoming messages will be published to
rospy.init_node('drive_control_publisher', anonymous=False) # initiate the ROS node
rospy.Subscriber("drive_control_speed_angle", Int32, speed_angle_msg)
rospy.Subscriber("drive_control_direction_angle", Int32, direction_angle_msg)
 
if __name__ == '__main__':
    try:
        http_server = tornado.httpserver.HTTPServer(application) # start the tornado server
        http_server.listen(8888)
        tornado.ioloop.IOLoop.instance().start()
    except rospy.ROSInterruptException:
        pass
    