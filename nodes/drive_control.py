#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
from pyfirmata import Arduino

on_hardware = False # whether we are running this node on the actual car (so it can access the IO board)
wheelpin = 3 # to which pin on the IO sheld the whee pin got connected
drivepin = 5 # to which pin on the IO shield the drive cable got connected
if on_hardware == True: # running on hardware -- we need to set the board connection
    board = Arduino('/dev/ttyACM99', baudrate=57600)
    board.servo_config(wheelpin, min_pulse=1, max_pulse=20, angle=90) # set initial direction to straight forward
    board.servo_config(drivepin, min_pulse=1, max_pulse=20, angle=90) # set initial speed to natural

speed_current_angle = 90 # this variable will carry the actual speed at any time and will be used to determine direction of change (in case of decay or full stop)
speed_min_angle_reverse = 75 # this is the angle below which the car start moving in reverse
speed_min_angle_forward = 105 # this is the angle above which the car start moving forward
speed_max_angle_reverse = 0 # maximum angle allowed in reverse (which is actually a minimum mathematically, as the angle goes 0-90)
speed_max_angle_forward = 180 # maximum angle allowed in forward
speed_decay_angle = 1 # how much we decrease the angle when there is a decay request
speed_change_angle = 5 # when we receive a request to change the speed, this is the angle change we will do
speed_direction_change_delay = 2 # in sec - delay enforced between changing direction (forward-backward)
last_stop_timestamp = 0.0 # the last time we have reached the zero speed from a non-zero speed (used with the speed_direction_change_delay)

direction_natural = 90 # this is the natural (straight ahead) position of the wheel in angles
direction_current_angle = direction_natural # this variable will carry the actual direction angle at any time
direction_max_angle_left = 30 # maximum angle allowed when setting the direction to the left (which is actually a minimum mathematically, as the angle goes 0-90)
direction_max_angle_right = 150 # maximum angle allowed when setting the direction to the right
direction_decay_angle = 2 # how much we decrease the angle when there is a decay request
direction_change_angle = 3 # when we receive a request to change the direction, this is the angle change we will do

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard speed/direction message %s", data.data)
    speed_instructions(data.data)
    
def decay_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard decay message %s", data.data)
    speed_instructions(data.data)
    
def listener():
    #board = Arduino('/dev/ttyACM99', baudrate=57600)
    # using pyfirmata: https://github.com/tino/pyFirmata
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_control', anonymous=True)
    
    if on_hardware == True:
        rospy.loginfo("Running on hardware")
    else:
        rospy.loginfo("Not running on hardware (in simulation")
    
    ####
    # initialization
    # before starting we need to set the car to idle and wheels facing forward
    set_speed_angle(speed_current_angle) # sets the speed to the current angle, which has a default of 90 at start
    set_direction_angle(direction_current_angle) # sets the direction to the current angle, which has a default of 90 at start
    rospy.loginfo("Started.")
    
    ####
    # subscribe to the topics
    rospy.Subscriber("drive_control_publish", String, callback)
    rospy.Subscriber("drive_control_decay_publish", String, decay_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
# this function will set the speed of the car, based on the received instruction
def speed_instructions(instruction):
    global speed_current_angle # so we can change this global variable in this function
    
    if(instruction == 'decay'): # this was a decay request
        if(speed_current_angle <= speed_min_angle_reverse): # we are currently moving in reverse
            change_speed(speed_decay_angle)
        if(speed_current_angle >= speed_min_angle_forward): # we are currently moving forward
            change_speed(-1*speed_decay_angle)
        if(direction_current_angle < direction_natural + direction_decay_angle and direction_current_angle > direction_natural - direction_decay_angle):
            # we need to set the direction to the natural direction, because the current angle is within the decay range of natural
            change_direction(direction_natural)
        else: # we are not within a decay range from the natural direction, so we will decay the direction angle by the decay
            if(direction_current_angle < direction_natural): # current direction is to the left
                change_direction(direction_decay_angle)
            if(direction_current_angle > direction_natural): # current direction is to the right
                change_direction(-1*direction_decay_angle)
    
    if(instruction == 'U'): # this is a speed up request (~up button pressed)
        if(speed_current_angle > speed_min_angle_reverse and speed_current_angle < speed_min_angle_forward): # currently we are in the neutral zone
            speed_current_angle = speed_min_angle_forward
            
        if(speed_current_angle <= speed_min_angle_reverse): # we are currently moving in reverse
            change_speed(-1*speed_change_angle)
        if(speed_current_angle >= speed_min_angle_forward): # we are currently moving forward
            change_speed(speed_change_angle)
    
    if(instruction == 'D'): # this is a speed down request (~down button pressed)
        if(speed_current_angle > speed_min_angle_reverse and speed_current_angle < speed_min_angle_forward): # currently we are in the neutral zone
            speed_current_angle = speed_min_angle_reverse
            
        if(speed_current_angle <= speed_min_angle_reverse): # we are currently moving in reverse
            change_speed(speed_change_angle)
        if(speed_current_angle >= speed_min_angle_forward): # we are currently moving forward
            change_speed(-1*speed_change_angle)
    
    if(instruction == 'L'): # this is a turn left request (~left button pressed)
        change_direction(-1*direction_change_angle)
    
    if(instruction == 'R'): # this is a turn right request (~right button pressed)
        change_direction(direction_change_angle)

 
# this function is called with the angle change request and will change the current angle with the amount requested
def change_speed(angle_change):
    new_angle = speed_current_angle + angle_change
    set_speed_angle(new_angle)
    rospy.loginfo("Changed the speed by angle %i", angle_change)
 
# this function is called with the angle change request and will change the current angle with the amount requested
def change_direction(angle_change):
    new_angle = direction_current_angle + angle_change
    set_direction_angle(new_angle)
    rospy.loginfo("Changed the direction by angle %i", angle_change)
 
# sets the speed to the angle requested
def set_speed_angle(angle):
    global speed_current_angle # so we can change this global variable in this function
    global last_stop_timestamp # so we can set this global variable in this function
    movement_allowed = 'yes'
    
    #rospy.loginfo("Value of speed_current_angle is %i", speed_current_angle)
    #rospy.loginfo("Value of new angle to be set is %i", angle)
    
    if(angle < speed_max_angle_reverse or angle > speed_max_angle_forward):
        rospy.loginfo("Out of range angle was requested for speed: %i", angle)
    else:
        # the old (current) speed is NOT in the zero range but the new speed is in the zero range, then we need to set the last_stop_timestamp,
        # which later we will use to determine whether the speed_direction_change_delay has passed yet
        # but we only set this if hasn't been set already
        if((speed_current_angle <= speed_min_angle_reverse or speed_current_angle >= speed_min_angle_forward)
        and angle > speed_min_angle_reverse and angle < speed_min_angle_forward
        and last_stop_timestamp == 0.0):
            last_stop_timestamp = rospy.get_time() # populate the last_stop_timestamp with the unix timestamp (example: 1424637131.834309)
            rospy.loginfo("Last stop timestamp set %f", last_stop_timestamp)
            movement_allowed = 'yes'
        else:         
            # the old (current) speed is in the zero range but the new speed is NOT in the zero range, then we need to check the last_stop_timestamp,
            # whether the speed_direction_change_delay has passed already
            if(speed_current_angle >= speed_min_angle_reverse and speed_current_angle <= speed_min_angle_forward
            and (angle < speed_min_angle_reverse or angle > speed_min_angle_forward )):
                # if the speed_direction_change_delay already passed or there wasn one then we can start moving
                if(rospy.get_time() > (last_stop_timestamp + speed_direction_change_delay)):
                    movement_allowed = 'yes'
                    last_stop_timestamp = 0.0
                else:
                    movement_allowed = 'no'
                    rospy.loginfo("No movement allowed, because the speed_direction_change_delay hasn't passed yet!")
            else:
                movement_allowed = 'yes'
                last_stop_timestamp = 0.0
        
        if(movement_allowed == 'yes'):
            if on_hardware == True: # running on hardware -- we need to actually write this value to the PWM
                board.digital[drivepin].write(angle)
            speed_current_angle = angle # overwrite the global variable with the new value
            rospy.loginfo("Set the speed to anglee %i", angle)
    
# sets the direction to the angle requested
def set_direction_angle(angle):
    global direction_current_angle # so we can change this global variable in this function
    
    if(angle < direction_max_angle_left or angle > direction_max_angle_right):
        rospy.loginfo("Out of range angle was requested for direction: %i", angle)
    else:
        if on_hardware == True: # running on hardware -- we need to actually write this value to the PWM
            board.digital[wheelpin].write(angle)
        direction_current_angle = angle # overwrite the global variable with the new value
        rospy.loginfo("Set the direction to angle %i", angle)
    
 
if __name__ == '__main__':
    listener()
    
# start an iterator thread so
# serial buffer doesn't overflow
#iter8 = pyfirmata.util.Iterator(board)
#iter8.start()

"""
drivepin=6
board.servo_config(drivepin, min_pulse=15, max_pulse=25, angle=0) # set initial direction to straight forward

time.sleep(3)
board.digital[drivepin].write(0)
print('set 0')
time.sleep(3)
board.digital[drivepin].write(70)
print('set 20')
time.sleep(1)
board.digital[drivepin].write(0)
print('set 0')
time.sleep(2)
board.exit() # this will exit all servos"""