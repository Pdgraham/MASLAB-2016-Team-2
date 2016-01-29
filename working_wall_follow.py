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
  runtime = 300000 #ms

  def setup(self):
    # initialize sensors, settings, start timers, etc.
    self.gameTimer = Timer()

    self.motorLeft = Motor(self.tamp, 7, 22)
    self.motorRight = Motor(self.tamp, 0, 21)
    self.motorval = 0
    self.motorLeft.write(1,0)
    self.motorRight.write(1,0)
    print "Motors connected."

    self.timer = Timer()
    self.gyro = Gyro(self.tamp, 9)
    print "Gyro connected."
    self.theta = self.gyro.val    
    self.dT = .01

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
    self.I = 0 #5
    self.D = 0
    self.last_diff = 0
    self.integral = 0
    self.desiredAngle = self.theta
    self.finishedCollectingBlock = False
    self.finishedDiscardingBlock = False

    self.timer = Timer()
    self.state = ExploreState()

    self.blocksCollected = 0
    self.leftIRVals = deque([])
    self.rightIRVals = deque([])
    self.frontRightIRVals = deque([])
    self.frontLeftIRVals = deque([])

    # Starts the robot
    print "Robot setup complete."

  def loop(self):
    if self.timer.millis() > self.dT*1000:
      if (self.gameTimer.millis() > self.runtime - 5000): # 5 seconds left in the game
        self.openDoorAndBuildTower()
      inputs = self.readSensors()
      process = self.state.process(inputs)
      print "Process: " + process.__class__.__name__
      # print(self.gyro.val)
      self.state = process.get_next_state()
      self.processOutputs(process.get_outputs())
      self.timer.reset()

  def readSensors(self):
    # Calculate the distance traveled, change in theta, and then reset sensors
    distance_traveled = 0 #(self.encoderLeft.val + self.encoderRight.val) / 2.0

    leftIR = self.leftIR.val
    rightIR = self.rightIR.val
    frontLeftIR = self.frontLeftIR.val
    frontRightIR = self.frontRightIR.val

    return Inputs(distance_traveled, self.gyro.val, frontRightIR, frontLeftIR, leftIR, rightIR, self.finishedCollectingBlock)     

  def processOutputs(self, Outputs):
    # TODO Missing servo outputs
    if (Outputs.driving == True):
      self.motorval = 50 #25?
    else:
      self.motorval = 0
    if (Outputs.isDiscardingBlock == True):
      self.motorval = -50

    if (Outputs.turning == True):
      # if we turn, then update self.desiredAngle
      self.desiredAngle = self.gyro.val
      if (Outputs.turn_clockwise == True):
        self.PID(self.desiredAngle + 5)
      else:
        self.PID(self.desiredAngle - 5)
    else:
      self.PID(self.desiredAngle)

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
    # print("motorLeft: ", min(75, abs(self.motorval + power)))
    # print("motorRight: ", min(75, abs(self.motorval - power)))
    self.motorLeft.write((self.motorval + power)>0, min(75, abs(self.motorval + power)))
    self.motorRight.write((self.motorval - power)>0, min(75, abs(self.motorval - power)))
    # print "EncoderLeft: " + str(self.encoderLeft.val)
    # print "EncoderRight: " + str(self.encoderRight.val)

######################## States ###########################
class ExploreState:
  found_block = False
  left_wall_following = False
  right_wall_following = False
  facing_wall = False
  distance_from_wall = 0
  counter = 0

  def process(self, Inputs):
    WALL_IN_FRONT = 20000 # orig: 20000

    if (Inputs.rightIR > Inputs.leftIR):
      right_wall_following = True
      left_wall_following = False
      distance_from_wall = Inputs.rightIR
    else:
      left_wall_following = True
      right_wall_following = False
      distance_from_wall = Inputs.leftIR
    self.counter += 1
    if (self.counter > 550):
      self.counter = 0
    elif (self.counter >= 500 ):
      return BackUpAndTurn(self)
    #print(Inputs.frontRightIR, Inputs.frontLeftIR)
    #print(Inputs.rightIR, Inputs.leftIR)
    #if (self.found_block == False):
    if (Inputs.frontRightIR >= WALL_IN_FRONT or Inputs.frontLeftIR >= WALL_IN_FRONT):
      return TurnFromWall(self)
    elif (Inputs.leftIR >= WALL_IN_FRONT or Inputs.rightIR >= WALL_IN_FRONT):
     return WallFollowing(self)
    else:
      return DrivingStraight(self)

####################### Processes #########################
# --------------- ExploreState Processes -----------------#

class WallFollowing():
  def __init__(self, ExploreState): 
    self.state = ExploreState
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    # Turn towards wall
    print self.state.distance_from_wall

#    if self.state.distance_from_wall < 26000:
#      turning = True
#      if (self.state.left_wall_following):
#        turn_clockwise = True # orig: True
#      else:
#        turn_clockwise = False # orig: False

    # Turn away from wall
    if self.state.distance_from_wall > 20000:
      driving = False
      turning = True
      if (self.state.left_wall_following):
        turn_clockwise = True # orig: True
      else:
        turn_clockwise = False # orig: False
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

class BackUpAndTurn():
  def __init__(self, explore):
    self.state = explore
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = True
    if (self.state.left_wall_following):
      turn_clockwise = True # orig: True
    else:
      turn_clockwise = False # orig: False
    isDiscardingBlock = True
    isCollectingBlock = False
    return Outputs(driving, turning, turn_clockwise, isCollectingBlock, isDiscardingBlock)

################## Seperate Classes #######################

# Represents the inputs returned from the sensors (gyro, encoders, and webcam)
class Inputs:
  def __init__(self, distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR, finishedCollectingBlock): 
  # def __init__(self, distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR, colorR, colorG, colorB):
    self.distance_traveled = distance_traveled
    self.theta = theta
    self.frontLeftIR = frontLeftIR
    self.frontRightIR = frontRightIR
    self.leftIR = leftIR
    self.rightIR = rightIR
    self.finishedCollectingBlock = finishedCollectingBlock
    # self.img = img
  
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
