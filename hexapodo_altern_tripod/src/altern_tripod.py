#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from smach import CBState

### -- VARIABLES GLOBALES -- ###


### -- ESTADOS -- ###

### -- ON FLOOR -- ###
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished','failed'])
def stand_up_cb( user_data):
    rospy.loginfo('Stand Up')
    #takeoff_topic = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    rospy.sleep(1)
    #msg = Empty()
    #result = takeoff_topic.publish(msg)
    #if result == None:
    return 'finished'
    #else:
        #return 'failed'


### -- BUCLE PRINCIPAL -- ###
if __name__ == '__main__':

    rospy.init_node('altern_tripod_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])

    # Open the container
    with sm:
        # Stand UP state
        smach.StateMachine.add('ON_FLOOR', CBState(on_floor_cb),
                               {'finished': 'STOP', 'failed':'outcome'})

        # Stop state
        smach.StateMachine.add('STOP', CBState(stop_cb),
                               {'finished_1': 'ALTERN_TRIPOD', 'finished_2': 'TETRAPOD', 'finished_3': 'STAIR_CLIMB','failed':'outcome'})

        # Altern tripod state
        smach.StateMachine.add('ALTERN_TRIPOD', CBState(altern_tripod_cb),
                               {'finished': 'STOP', 'failed':'outcome'})

        # Tetrapod state
        smach.StateMachine.add('TETRAPOD', CBState(tetrapod_cb),
                               {'finished': 'STOP', 'failed':'outcome'})

        # Stair Climb state
        smach.StateMachine.add('STAIR_CLIMB', CBState(stair_climb_cb),
                               {'finished': 'STOP', 'failed':'outcome'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
