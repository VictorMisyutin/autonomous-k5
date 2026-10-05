from picarx import Picarx

px = Picarx()
while True:
    a = px.get_grayscale_data() # returns list of length 3 -> [left, middle, right] ranging from 0 (bright) to 1400 (black)
    print(a)
