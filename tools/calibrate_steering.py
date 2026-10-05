from picarx import Picarx
px = Picarx()
print("old offset:", px.dir_cali_val)
px.dir_servo_calibrate(px.dir_cali_val -3)
print("new offset:", px.dir_cali_val)
