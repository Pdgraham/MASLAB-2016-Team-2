import math

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


#int[][] maze = new int[width][height]; // The maze
#boolean[][] wasHere = new boolean[width][height];
#boolean[][] correctPath = new boolean[width][height]; // The solution to the maze
#int startX, startY; // Starting X and Y values of maze
#int endX, endY;     // Ending X and Y values of maze


def main():
  # Initializes blank maze that is 20x20
  # This is used with the 6x7 board given, where each unit length is 2ft
  maze = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
          [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

  # Adds the walls, starting position, and stacks from the text file
  start_x = None
  start_y = None
  stacks = []
  map_file = open("red_map.txt")
#  map_file = open("simple_map.txt")

  map_line = map_file.readline()
  while (map_line != ""):
    if (map_line[0] == "W" or map_line[0] == "P"):
      add_wall(maze, map_line)
    elif (map_line[0] == "L"):
      add_starting_pos(maze, map_line)
      start_x = map_line[2]
      start_y = map_line[4]
    elif (map_line[0] == "S"):
      add_stack(maze, map_line)
      stacks.append((map_line[2], map_line[4], map_line[6], map_line[8], map_line[10]))

    map_line = map_file.readline()

  print "Maze Map:"
  for i in range(len(maze)):
    print maze[i]
  print "Starting Position: x=", start_x, ", y=", start_y
  print "Stacks are:", stacks

def add_wall(maze, text_line):
  start_x = int(text_line[2])
  start_y = int(text_line[4])
  end_x = int(text_line[6])
  end_y = int(text_line[8])

  if (start_x == end_x): # Vertical wall
    # Swaps the y's to make the wall in a positive direction
    if (end_y < start_y):
      temp = start_y
      start_y = end_y
      end_y = temp
    for i in range((end_y - start_y) * 2 + 1):
      maze[start_y*2 + i][start_x*2] = 2

  elif (start_y == end_y): # Horizontal wall
    # Swaps the x's to make the wall in the positive direction
    if (end_x < start_x):
      temp = start_x
      start_x = end_x
      end_x = temp
    for i in range((end_x - start_x) * 2 + 1):
      maze[start_y*2][start_x*2 + i] = 2

  else: # Diagonal wall
    dir_x = int(math.copysign(1, end_x - start_x))
    dir_y = int(math.copysign(1, end_y - start_y))
    for i in range(abs(end_x - start_x) * 2 + 1):
      maze[start_y*2 + i*dir_y][start_x*2 + i*dir_x] = 2

def add_starting_pos(maze, text_line):
  start_x = int(text_line[2])
  start_y = int(text_line[4])
  maze[start_y*2][start_x*2] = 5

def add_stack(maze, text_line):
  start_x = int(text_line[2])
  start_y = int(text_line[4])
  maze[start_y*2][start_x*2] = 9

if __name__ == "__main__":
  main()

