from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import DigitalOutput, Motor, Gyro, Encoder, AnalogInput, Servo, Color
import time
import threading
import sys
from collections import deque
from red_detector import CalculateBlocks
import numpy as np
import cv2

GOAL_COLOR = "RED"

class MyRobot(SyncedSketch):
  runtime = 180000 #ms
  DOOR_OPEN_POS = 40
  DOOR_CLOSE_POS = 144
  GRIPPER_OPEN_POS = 0
  GRIPPER_CLOSE_POS = 180
  GRIPPER_DOWN = 1
  GRIPPER_UP = 0

  def setup(self):
    # initialize sensors, settings, start timers, etc.
    self.gameTimer = Timer()

    self.motorLeft = Motor(self.tamp, 18, 21)#20)
    self.motorRight = Motor(self.tamp, 19, 23)
    self.motorGripper = Motor(self.tamp, 7, 22)#22)
    self.motorval = 0
    self.motorLeft.write(1,0)
    self.motorRight.write(1,0)
    self.motorGripper.write(1,0)
    self.currentGripperLevel = 2
    print "Motors connected."

    self.servoDoor = Servo(self.tamp, 20)
    self.servovalDoor = self.DOOR_CLOSE_POS
    self.servoDoor.write(self.DOOR_CLOSE_POS)
    self.timerDoor = Timer()
    self.servoGripper = Servo(self.tamp, 10)
    self.servovalGripper = self.GRIPPER_CLOSE_POS
    self.servoGripper.write(self.servovalGripper)
    self.timerGripper = Timer()
    print "Servos connected."

    left_pins = 6,5
    right_pins = 3,4
    # Encoder doesn't work when after gyro
    self.encoderLeft = Encoder(self.tamp, 6,5, continuous=False)
    self.encoderRight = Encoder(self.tamp, 3,4, continuous=True)
    print "Encoders connected." 
   # TODO: set encoder to 0
    self.timer = Timer()
    self.gyro = Gyro(self.tamp, 9)
    print "Gyro connected."
    self.theta = self.gyro.val    
    self.dT = .01

    self.cam = cv2.VideoCapture(0)

    print "Camera connected."

    # self.color = Color(self.tamp,
    #                integrationTime=Color.INTEGRATION_TIME_101MS,
    #                gain=Color.GAIN_1X)

    frontLeftIR_pin = 14
    self.frontLeftIR = AnalogInput(self.tamp, frontLeftIR_pin)
    frontRightIR_pin = 15
    self.frontRightIR = AnalogInput(self.tamp, frontRightIR_pin)
    leftIR_pin = 16 
    self.leftIR = AnalogInput(self.tamp, leftIR_pin)
    rightIR_pin = 17
    self.rightIR = AnalogInput(self.tamp, rightIR_pin)


    # Initialize PID Values
    self.P = 10
    self.I = 5
    self.D = 0
    self.last_diff = 0
    self.integral = 0
    self.desiredAngle = self.theta
    self.finishedCollectingBlock = False
    self.finishedDiscardingBlock = False

    self.timer = Timer()
    self.state = ExploreState()
    # self.state = CollectBlockState()

    self.blocksCollected = 0
    self.leftIRVals = deque([])
    self.rightIRVals = deque([])
    self.frontRightIRVals = deque([])
    self.frontLeftIRVals = deque([])

    # Starts the robot
    print "Robot setup complete."

  def loop(self):
    if self.timer.millis() > self.dT*1000:
      # print("self.gameTimer", self.gameTimer)
      # print(self.gameTimer.millis())
      if (self.gameTimer.millis() > self.runtime - 5000): # 5 seconds left in the game
        self.openDoor()
      inputs = self.readSensors()
      process = self.state.process(inputs)
      print "Process: " + process.__class__.__name__
      # print(self.gyro.val)
      self.state = process.get_next_state()
      self.processOutputs(process.get_outputs())
      self.timer.reset()

  def readSensors(self):
    # Calculate the distance traveled, change in theta, and then reset sensors
    distance_traveled = (self.encoderLeft.val + self.encoderRight.val) / 2.0
    #encoder_omega = self.encoderLeft.val - self.encoderRight.val
    # print('frontRightIR: ', self.frontRightIR.val)
    # print("frontLeftIR: ", self.frontLeftIR.val)
    # print("leftIR: ", self.leftIR.val)
    # print("rightIR: ", self.rightIR.val)

    # Camera
    ret, frame = self.cam.read()
    img = cv2.resize(frame,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)
    print("VideoFrame captured: ", ret)
    blocks = CalculateBlocks(img); #what should CalculateBlocks return?

    leftIR = self.leftIR.val
    rightIR = self.rightIR.val
    frontLeftIR = self.frontLeftIR.val
    frontRightIR = self.frontRightIR.val
    if len(self.leftIRVals) == 100:
      print("Reading from averaged values")
      self.leftIRVals.append(leftIR)
      self.leftIRVals.popleft()
      leftIR = sum(leftIRVals)/100
    if len(self.rightIRVals) == 100:
      self.rightIRVals.append(rightIR)
      self.rightIRVals.popleft()
      rightIR = sum(self.rightIRVals)/100
    if len(self.frontLeftIRVals) == 100:
      self.frontLeftIRVals.append(frontLeftIR)
      self.frontLeftIRVals.popleft()
      frontLeftIR = sum(self.frontLeftIRVals)/100
    if len(self.frontRightIRVals) == 100:
      self.frontRightIRVals.append(frontRightIR)
      self.frontRightIRVals.popleft()
      frontRightIR = sum(self.frontRightIRVals)/100

    return Inputs(distance_traveled, self.gyro.val, frontRightIR, frontLeftIR, leftIR, rightIR, self.finishedCollectingBlock, blocks)
      # self.leftIR.val, self.rightIR.val, self.color.r, self.color.g, self.color.b)
     
    # distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR

  def processOutputs(self, Outputs):
    # TODO Missing servo outputs
    if (Outputs.driving == True):
      self.motorval = 50 #25?
    else:
      self.motorval = 0
    if (Outputs.turning == True):
      # if we turn, then update self.desiredAngle
      self.desiredAngle = self.gyro.val
      if (Outputs.turn_clockwise == True):
        self.PID(self.desiredAngle + 5)
      else:
        self.PID(self.desiredAngle - 5)
    else:
      # self.PID(self.gyro.val)
      self.PID(self.desiredAngle)
    if Outputs.isCollectingBlock and not self.finishedCollectingBlock:
      # Gripper is at level 2 and closed
      if self.currentGripperLevel == 2 and self.servovalGripper == self.GRIPPER_CLOSE_POS:
        self.closeOrOpenGripper()
      # Gripper is at level 2 and open
      elif self.currentGripperLevel == 2 and self.servovalGripper == self.GRIPPER_OPEN_POS:
        self.moveGripper()
      # Gripper is at level 1 and open
      elif self.currentGripperLevel == 1 and self.servovalGripper == self.GRIPPER_OPEN_POS:
        self.closeOrOpenGripper()
      # Gripper is at level 1 and closed
      elif self.currentGripperLevel == 1 and self.servovalGripper == self.GRIPPER_CLOSE_POS:
        self.moveGripper()
        # self.finishedCollectingBlock = True
        time.sleep(2)
        self.blocksCollected += 1
        if (self.blocksCollected == 4):
          self.finishedCollectingBlock = True
    if Outputs.isDiscardingBlock and not self.finishedDiscardingBlock:
      # Gripper level 2, closed
      if self.currentGripperLevel == 2 and self.servovalGripper == self.GRIPPER_CLOSE_POS:
        self.moveGripper()
      # Gripper level 1, closed
      elif self.currentGripperLevel == 1 and self.servovalGripper == self.GRIPPER_CLOSE_POS:
        self.closeOrOpenGripper()
      # Gripper level 1, open
      elif self.currentGripperLevel == 1 and self.servovalGripper == self.GRIPPER_OPEN_POS:
        self.moveGripper()
      # Gripper level 2, open
      elif self.currentGripperLevel == 2 and self.servovalGripper == self.GRIPPER_OPEN_POS:
        self.closeOrOpenGripper()
        self.finishedDiscardingBlock = True

  def PID(self, desired_theta):
    # Set encoder to 0 after turning.
    # To turn in place, set bias (i.e. motorval to 0)
    estimated = self.gyro.val # TODO: calculate estimated with encoder
    # print(self.gyro.val)
    diff = desired_theta - estimated
    # print diff
    self.integral += diff * self.dT
    derivative = (diff - self.last_diff)/self.dT
    power = self.P*diff + self.I*self.integral + self.D*derivative # NOTE: Cap self.D*derivative, use as timeout
    # print("motorLeft: ", min(255, abs(self.motorval + power)))
    # print("motorRight: ", min(255, abs(self.motorval - power)))
    self.motorLeft.write((self.motorval + power)>0, min(255, abs(self.motorval + power)))
    self.motorRight.write((self.motorval - power)>0, min(255, abs(self.motorval - power)))
    # print "EncoderLeft: " + str(self.encoderLeft.val)
    # print "EncoderRight: " + str(self.encoderRight.val)

  def moveGripper(self):
    if self.currentGripperLevel == 1:
      self.motorGripper.write(self.GRIPPER_UP, 100)
      timeToSleep = 1.3
      if (self.blocksCollected == 1):
        timeToSleep = 1.4
      elif (self.blocksCollected == 2):
        timeToSleep = 1.55
      elif (self.blocksCollected == 3):
        timeToSleep = 1.7
    elif self.currentGripperLevel == 2:
      self.motorGripper.write(self.GRIPPER_DOWN, 100)
      timeToSleep = 0.9
    time.sleep(timeToSleep)
    self.motorGripper.write(1,0)
    if self.currentGripperLevel == 1:
      self.currentGripperLevel = 2 
    else:
      self.currentGripperLevel = 1       

  def closeOrOpenGripper(self):
    if (self.servovalGripper > self.GRIPPER_OPEN_POS):
      while(self.servovalGripper > self.GRIPPER_OPEN_POS):
        if (self.timerGripper.millis() > 1):
          self.timerGripper.reset()
          self.servovalGripper -= 1
          self.servoGripper.write(abs(self.servovalGripper))    
    elif (self.servovalGripper < self.GRIPPER_CLOSE_POS):
      while(self.servovalGripper < self.GRIPPER_CLOSE_POS):
        if (self.timerGripper.millis() > 1):
          self.timerGripper.reset()
          self.servovalGripper += 1
          self.servoGripper.write(abs(self.servovalGripper))
    time.sleep(.1)

  def openDoor(self):
    while(self.servovalDoor > self.DOOR_OPEN_POS):
      if (self.timerDoor.millis() > 10):
        self.timerDoor.reset()
        self.servovalDoor -= 1
        # print self.servovalDoor
        self.servoDoor.write(abs(self.servovalDoor))

######################## States ###########################
class ExploreState:
  found_block = False
  left_wall_following = False
  right_wall_following = False
  facing_wall = False

  def process(self, Inputs):
    WALL_IN_FRONT = 15000 # orig: 20000
    print len(Inputs.blocks)
    if (Inputs.rightIR >= WALL_IN_FRONT):
      right_wall_following = True # True
    else:
      right_wall_following = False # False
    if (Inputs.leftIR >= WALL_IN_FRONT):
      left_wall_following = False # False
    else:
      left_wall_following = True # True

    if (self.found_block == False):
      if Inputs.frontRightIR >= WALL_IN_FRONT and Inputs.rightIR >= WALL_IN_FRONT:
        return TurnFromWall(self)
      elif Inputs.frontLeftIR >= WALL_IN_FRONT and Inputs.leftIR >= WALL_IN_FRONT:
        return TurnFromWall(self)   
      elif (Inputs.frontRightIR >= WALL_IN_FRONT and Inputs.frontLeftIR >= WALL_IN_FRONT): #originall and
        facing_wall = True
        print("Wall in front.")
        return TurnFromWall(self)
      elif (Inputs.leftIR >= WALL_IN_FRONT or Inputs.rightIR >= WALL_IN_FRONT):
        return WallFollowing(self)
      else:
        return DrivingStraight(self)
    else:
      return FoundBlock()


class DriveToBlockState:
  def process(self, Inputs):
    IN_FRONT_OF_BLOCK = 80 
    PIXEL_MARGIN = 20
    
    # In Position to knock down tower/Pick up block
    if (len(Inputs.blocks) > 0):
      for block in Inputs.blocks:
        if (block.minY >= 40): 
          return ClosingInOnBlock(self)

    # Move to cube if the closest block is our Goal Color
    if (len(Inputs.blocks) > 0):
      closest_block = None
      closest_block_our_color = None
      for block in Inputs.blocks:
        if block.minY > closest_block.minY:
          closest_block = block
          if block.color == GOAL_COLOR and block.minY > closest_block_our_color.minY:
            closest_block_our_color = block
      if closest_block.color == GOAL_COLOR:
        if (closest_block.meanX >= IN_FRONT_OF_BLOCK-PIXEL_MARGIN and
            closest_block.meanX <= IN_FRONT_OF_BLOCK+PIXEL_MARGIN):
          return DrivingToGoalBlock()
        elif (closest_block.meanX > IN_FRONT_OF_BLOCK+PIXEL_MARGIN):
          return TurnToGoalBlockClockwise()
        elif (closest_block.meanX < IN_FRONT_OF_BLOCK-PIXEL_MARGIN):
          return TurnToGoalBlockCounterClockwise()
    # return BlockInPosition(self)
    # Lost Cube
    # pass

class CollectBlockState:
  def process(self, Inputs):
    #if (TODO color sensor is not ready):
    return MovingUpToBlock()
    if Inputs.finishedCollectingBlock:
      Inputs.finishedCollectingBlock = False
      # If block collected is the correct color
      return CollectedCorrectColorBlock()
      # else:
      # return CollectedWrongColorBlock()
    else:
      return CollectingBlock(self)

class DiscardBlockState:


  def process(self, Inputs):
    if Inputs.finishedDiscardingBlock:
      print("Inside DiscardBlockState")


class SetTowerState:
  def process(self, Inputs):
    # Turn to face wall then back away
    # Turn 180, drive forward, back away, turn 180 again
    
    pass

####################### Processes #########################
# --------------- ExploreState Processes -----------------#
class FoundBlock():
  # Change to state->Drive to block without moving
  def get_next_state(self):
    return DriveToBlockState()
  def get_outputs(self):
    driving = False
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class WallFollowing():
  def __init__(self, ExploreState): 
    self.state = ExploreState
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class TurnFromWall():
  def __init__(self, explore):
    self.state = explore
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = False
    turning = True
    if (self.state.left_wall_following):
      turn_clockwise = True # orig: True
    else:
      turn_clockwise = False # orig: False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class DrivingStraight():
  def __init__(self, explore):
    self.state = explore
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

# --------------- DriveToBlockState Processes -----------------#
class DrivingToGoalBlock():
  def get_next_state(self):
    return DriveToBlockState()
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class TurnToGoalBlockClockwise():
  def get_next_state(self):
    return DriveToBlockState()
  def get_outputs(self):
    driving = False
    turning = True
    turn_clockwise = True
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class TurnToGoalBlockCounterClockwise():
  def get_next_state(self):
    return DriveToBlockState()
  def get_outputs(self):
    driving = False
    turning = True
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class ClosingInOnBlock():
  def get_next_state(self):
    return CollectBlockState()
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

# class DrivingToNonGoalBlock():  #Might not matter because we should just drive to almost all blocks

#class BlockInPosition():
#  # Change to state->CollectBlock without moving
#  def get_next_state(self):
#    return CollectBlockState()
#  def get_outputs(self):
#    driving = False
#    turning = False
#    turn_clockwise = False
#    isCollectingBlock = False
#    isDiscardingBlock = False
#    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

# --------------- CollectBlockState Processes -----------------#
class MovingUpToBlock():
  def __init__(self, driveToBlockState):
    self.state = driveToBlockState
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    isCollectingBlock = True
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class CollectingBlock(): 
  def __init__(self,driveToBlockState):
    self.state = driveToBlockState
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = False
    turning = False
    turn_clockwise = False
    isCollectingBlock = True
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class CollectedCorrectColorBlock():
  # Change to state->Explore without moving
  def get_next_state(self):
    return ExploreState()
  def get_outputs(self):
    driving = False
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

class CollectedWrongColorBlock():
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = False
    turning = False
    turn_clockwise = False
    isCollectingBlock = False
    isDiscardingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

# --------------- DiscardBlockState Processes -----------------#
# class Discard_DriveBackwards():

# class Discard_Rotate90CounterClockwise():

# class Discard_DriveFoward():

# class 

################## Seperate Classes #######################

# Represents the inputs returned from the sensors (gyro, encoders, and webcam)
class Inputs:
  def __init__(self, distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR, finishedCollectingBlock, blocks):
  # def __init__(self, distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR, colorR, colorG, colorB):
    self.distance_traveled = distance_traveled
    self.theta = theta
    self.blocks = blocks
    self.frontLeftIR = frontLeftIR
    self.frontRightIR = frontRightIR
    self.leftIR = leftIR
    self.rightIR = rightIR
    self.finishedCollectingBlock = finishedCollectingBlock
    # self.img = img
    # self.colorR = colorR
    # self.colorG = colorG
    # self.colorB = colorB
  
  def get_distance_traveled():
    return self.distance_traveled
  def get_theta():
    return self.theta

class Outputs:
  def __init__(self, driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock):
    self.driving = driving
    self.turning = turning
    self.turn_clockwise = turn_clockwise
    self.isCollectingBlock = isCollectingBlock
    self.isDiscardingBlock = isDiscardingBlock

# These can be written as updateable positions on the map
# For now they must be generated from every image
class Block:
  def __init__(self, distance_from_bot, heading, color):
    self.distance = distance_from_bot
    self.heading = heading
    self.goal_color = (color == GOAL_COLOR)

if __name__ == "__main__":
  robot = MyRobot(12, -0.00001, 100)
  sys.setrecursionlimit(10000)
  robot.run()
