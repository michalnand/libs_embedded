from lx16a import *

import numpy
import time


def set_servo_id(required_id):

    for id_try in range(255):
        try:
            servo = LX16A(id_try)
            current_id = id_try
            break
        except:
            print("trying id ", id_try)

    print("current ID is ", id_try)

    
    print("setting ID to ", required_id)
    servo.set_id(required_id)

    '''
    print("checking ID")
    try:
        servo_test = LX16A(required_id)

        print("ID check ok, set to ", servo_test.get_id())
        
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
    '''




if __name__ == "__main__":

    LX16A.initialize("/dev/tty.usbserial-11130")

    id = 1

    #set_servo_id(id)

    servos_ids = [1, 2, 3, 4]
    servos = []


    angle_min       = 0
    angle_center    = 120
    angle_max       = 240

    for i in range(len(servos_ids)):
        try:
            servo = LX16A(servos_ids[i])
            servo.set_angle_limits(angle_min, angle_max)
            servos.append(servo)
            
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()


    for i in range(len(servos_ids)):
        servos[i].move(angle_center)
        time.sleep(0.3)

    print("centering done")

    
    
    t = 0.0

    while True:
        
        yr    = []
        count = len(servos_ids)
        for i in range(count):

            angle = (1.0 + numpy.sin(t + 2*numpy.pi*i/count))
            yr.append(angle_center + 60*angle)

        for i in range(count):
            servos[i].move(yr[i])


        print(t, yr)

        t+= 0.01
        time.sleep(0.01)
    