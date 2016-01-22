###### The available game elements are: ######
# Wall (W)
# Platform (P)
# Red Cube (R)
# Green Cube (G)
# Stack of Cubes (S)
# Start Location (L)

###### Each element has attribute data: ######

# Wall or Platform, initial X, initial Y, final X, final Y
# (W|P),x1,y1,x2,y2

# Stack, x, y, type bottom, type middle, type top
# S,x,y,(R|G),(R|G),(R|G)

# Start Location, x, y
# L,x,y

###### Example of map inputs w/ a few walls ######

# P,1,1,2,0
# S,5,3,R,R,G
# L,9,2
# W,3,5,3,6
# W,3,6,3,7
# W,3,7,3,8

def main():
  map_file = open("red_map.txt")
  print map_file.read()

#def convert_map_to_array(text_file):
  




if __name__ == "__main__":
  main()

