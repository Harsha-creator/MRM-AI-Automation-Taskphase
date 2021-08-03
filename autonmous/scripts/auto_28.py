#!/usr/bin/env python
import time
from numpy.core.numerictypes import english_lower
from numpy.lib.function_base import select
import rospy
import math
import pyproj

from geometry_msgs.msg import Twist, Point
from rospy.timer import sleep
from sensor_msgs.msg import NavSatFix,Imu
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class auto28:
    def __init__(self):
        rospy.init_node('Autonomous')
        self.yaw = 0.0
        self.for_az = 0.0
        self.back_az = 0.0
        self.d = 0.0
        self.pub = None
        self.la =    49.8999527017
        self.l = 8.9000662282
        self.a = -1.33332332268
        self.state = 1
        self.geod = pyproj.Geod(ellps='WGS84')
        self.yaw_precision = 0.9
        self.dist_precision = 1
        self.buff = 1.5
        self.dist = 0
        self.dir = 0
        self.flag = 0
        self.pub= rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/group_task2/laser/scan', LaserScan, self.laser)
        self.fix_sub = rospy.Subscriber('/group_task2/fix', NavSatFix, self.distance)
        self.imu_sub = rospy.Subscriber('/group_task2/imu', Imu , self.angle)
        self.rate = rospy.Rate(10)
        self.regions = {}
        self.rang = []
        rospy.spin()

    def angle(self,message):
        orientation_q = message.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        self.yaw = math.degrees(self.yaw)
        if( self.yaw < 0 ): 
            self.yaw += 360.0
        self.rate.sleep()
        if(self.state == 0):
            self.move(self.regions)
        elif(self.state ==1):
            self.fix_yaw()
        elif(self.state == 2):
            self.go_straight_ahead()
        else:
            self.done()

    def distance(self,message):
        self.for_az, self.back_az, self.d = self.geod.inv(message.longitude, message.latitude, self.l, self.la)
        self.for_az = -self.for_az
        if( self.for_az < 0 ): 
            self.for_az += 360.0


    def fix_yaw(self):
        #self.rate.sleep()
        twist_msg = Twist()
        ang = self.for_az - self.yaw
        if(ang < 0):
            if(math.fabs(ang) > 180):
                err_yaw = 360 + ang
                self.dir = -1
            else:
                err_yaw = ang
                self.dir = 1
        else:
            if(ang > 180):
                err_yaw = 360 - ang
                self.dir = 1
            else:
                err_yaw = ang
                self.dir = -1
        if(all(x > 1.5 for x in self.regions.values())):
            if math.fabs(err_yaw) > self.yaw_precision:
                print(self.yaw)
                print(self.for_az)
                print (err_yaw)
                print("--------------------------------")
                twist_msg.angular.z = 0.6* self.dir
                self.pub.publish(twist_msg)
        else:
            self.change_state(0)

        if math.fabs(err_yaw) <= self.yaw_precision:
            twist_msg.angular.z = 0
            self.pub.publish(twist_msg)
            print ('Yaw error: [%s]') % err_yaw

            self.flag = 0
            self.change_state(0)

       
    def go_straight_ahead(self):
        #self.rate.sleep()
        ang = self.for_az - self.yaw
        if(ang < 0):
            if(math.fabs(ang) > 180):
                err_yaw = 360 + ang
                self.dir = -1
            else:
                err_yaw = ang
                self.dir = 1
        else:
            if(ang > 180):
                err_yaw = 360 - ang
                self.dir = 1
            else:
                err_yaw = ang
                self.dir = -1
        
        if (self.d > self.dist_precision):
            print(self.d)
            twist_msg = Twist()
            twist_msg.linear.x = 0.9
            self.pub.publish(twist_msg)

        else:
            print ('Position error: [%s]') % self.d
            self.change_state(3)
            self.done()
        
        if(self.dist - self.d > 0.5 or all(x > 1.5 for x in self.regions.values())):
            if math.fabs(err_yaw) > self.yaw_precision:
                print ('Yaw error: [%s]') % err_yaw
                self.change_state(1)
            

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.pub.publish(twist_msg)

    def change_state(self,state1):
        self.state = state1
        print ('State changed to [%s]') % self.state

    def nanvalues_fix(self, ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        return ranges

 
    def average_range(self, ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        if (len(ranges)!=0):
            return ( sum(ranges) / float(len(ranges)) )
        else:
            return 0.0

    def laser(self , msg):
        #self.rate.sleep()
        msg.ranges = self.nanvalues_fix(msg.ranges)
        self.regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
        


    def move(self , regions):
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''


        if regions['front'] > self.buff and regions['fleft'] > self.buff and regions['fright'] > self.buff:
            state_description = 'case 1 - nothing'
            linear_x = 0
            angular_z = 0
            self.change_state(2)
            self.flag = 1
            self.dist = self.d

        elif regions['front'] < self.buff and regions['fleft'] > self.buff and regions['fright'] > self.buff:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = -0.6

        elif regions['front'] > self.buff and regions['fleft'] > self.buff and regions['fright'] < self.buff:
            state_description = 'case 3 - fright'
            linear_x = 0.5
            angular_z = -0.6

        elif regions['front'] > self.buff and regions['fleft'] < self.buff and regions['fright'] > self.buff:
            state_description = 'case 4 - fleft'
            linear_x = 0.5
            angular_z = 0.6

        elif regions['front'] < self.buff and regions['fleft'] > self.buff and regions['fright'] < self.buff:
            state_description = 'case 5 - front and fright'
            linear_x = 0.3
            angular_z = -0.6

        elif regions['front'] < self.buff and regions['fleft'] < self.buff and regions['fright'] > self.buff:
            state_description = 'case 6 - front and fleft'
            linear_x = 0.3
            angular_z = 0.6

        elif regions['front'] < self.buff and regions['fleft'] < self.buff and regions['fright'] < self.buff:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0
            angular_z = -0.6

        elif regions['front'] > self.buff and regions['fleft'] < self.buff and regions['fright'] < self.buff:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0.3
            angular_z = -0.6

        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

if __name__ == "__main__":
    auto28()