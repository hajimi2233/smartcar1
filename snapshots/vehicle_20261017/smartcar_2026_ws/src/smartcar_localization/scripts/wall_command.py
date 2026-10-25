#!/usr/bin/env python
"""Acknowledged wall editor commands; fail visibly on timeout or rejected edits."""
from __future__ import print_function
import json
import sys
import time
import uuid
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('wall_command',anonymous=True)
    request_id=uuid.uuid4().hex;reply=[]
    def receive(msg):
        data=json.loads(msg.data)
        if data.get('request_id')==request_id: reply.append(data)
    subscriber=rospy.Subscriber('/wall_features/reply',String,receive,queue_size=10)
    publisher=rospy.Publisher('/wall_features/command',String,queue_size=1)
    deadline=time.time()+5.
    while not rospy.is_shutdown() and time.time()<deadline:
        if publisher.get_num_connections() and subscriber.get_num_connections(): break
        time.sleep(.05)
    if not publisher.get_num_connections():
        print('Wall editor not running. Start localization with wall_features:=true.');return 1
    publisher.publish(json.dumps(dict(request_id=request_id,args=rospy.myargv()[1:])))
    deadline=time.time()+5.
    while not rospy.is_shutdown() and not reply and time.time()<deadline: time.sleep(.05)
    if not reply: print('No acknowledgement; inspect editor_status before retrying.');return 1
    print(reply[0]['message']);return 0 if reply[0]['ok'] else 1


if __name__=='__main__': sys.exit(main())
