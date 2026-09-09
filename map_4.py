from picarx import Picarx
import random
import time

TOLERANCE = 30.0

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
            
            if distance_from_obj >= TOLERANCE or (distance_from_obj == -2 and last_distance > 150):
                px.forward(100)
            else:
                px.backward(100)
                time.sleep(1.5)
                random_angle = 0.0
                while random_angle < 5.0 and random_angle > -5.0:
                    random_angle = random.randint(-35,35)
                
                print(f"angle: {random_angle}")

                px.set_dir_servo_angle(random_angle)
                px.forward(80) # TODO: inverse angle while backing up and the revert to normal angle and drive forward for sharper turns
                time.sleep(1.5)
                px.forward(0)
                px.set_dir_servo_angle(0)
            
            last_distance = distance_from_obj

    except Exception as e:
        print(e)
    finally:
        px.stop();

