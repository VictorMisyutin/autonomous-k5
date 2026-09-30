from picarx import Picarx
import random
import time

SOFT_TOLERANCE = 50.0
HARD_TOLERANCE = 15.0

MINIMUM_ANGLE_TURN = 20.0

avoiding_obstacle = False
avoidance_direction = ""

if __name__ == "__main__":
    
    px = Picarx()
   
    # ----

    px.set_dir_servo_angle(0)
    distance_from_obj = px.get_distance()
    last_distance = distance_from_obj

    try:
        while True:
            # if sensor value > 12 drive forward
            # else, drive back, turn random and drive forward
        
            distance_from_obj = px.get_distance()
            print(f"distance: {distance_from_obj}")
            
            if distance_from_obj >= SOFT_TOLERANCE or (distance_from_obj == -2 and last_distance > 150):
                px.forward(100)
                avoiding_obstacle = False
            else:
                if distance_from_obj <= HARD_TOLERANCE:
                    # TODO: Move in direction opposite of chosen direction
                    if avoidance_direction == "Left":
                        random_angle = 35.0
                    else:
                        random_angle = -35.0
                    px.set_dir_servo_angle(random_angle)
                    px.backward(100)
                    avoiding_obstacle = True
                    time.sleep(1.2)
                    px.forward(0)
                    px.set_dir_servo_angle(0)
                else:    
                    # attempt to move slightly
                    random_angle = 0.0
                    while random_angle < MINIMUM_ANGLE_TURN and random_angle > -MINIMUM_ANGLE_TURN:
                        random_angle = random.randint(-45,45)

                    if random_angle < 0:
                        avoidance_direction = "Left"
                    else:
                        avoidance_direction = "Right"
                    
                    print(f"angle: {random_angle}")
                    print(f"direction: {avoidance_direction}")
                    px.set_dir_servo_angle(random_angle)
                    px.forward(80)
                    avoiding_obstacle = True
                    time.sleep(1.2)
                    px.set_dir_servo_angle(0)

            last_distance = distance_from_obj

    except Exception as e:
        print(e)
    finally:
        px.stop();

