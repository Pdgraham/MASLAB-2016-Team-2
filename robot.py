from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import DigitalOutput, Motor, Gyro, Encoder, AnalogInput, Servo
import time
import threading
import sys

GOAL_COLOR = "RED"

class MyRobot(SyncedSketch):
  runtime = 180 #seconds
  DOOR_OPEN_POS = 40
  DOOR_CLOSE_POS = 144
  GRIPPER_OPEN_POS = 0
  GRIPPER_CLOSE_POS = 180
  GRIPPER_DOWN = 1
  GRIPPER_UP = 0

  def setup(self):
    # initialize sensors, settings, start timers, etc.
    self.motorLeft = Motor(self.tamp, 18, 21)#20)
    self.motorRight = Motor(self.tamp, 19, 23)
    self.motorGripper = Motor(self.tamp, 7, 22)#22)
    self.motorval = 0
    self.motorLeft.write(1,0)
    self.motorRight.write(1,0)
    self.motorGripper.write(1,0)
    self.currentGripperLevel = 1
    self.moveGripper(2)
    print "Motors connected."

    self.servoDoor = Servo(self.tamp, 20)
    self.servovalDoor = self.DOOR_CLOSE_POS
    self.servoDoor.write(self.DOOR_CLOSE_POS)
    self.timerDoor = Timer()
    self.servoGripper = Servo(self.tamp, 10)
    self.servovalGripper = self.GRIPPER_OPEN_POS
    self.servoGripper.write(self.GRIPPER_OPEN_POS)
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
    self.dT = .03

    self.encoderLeft.start_continuous()
    self.encoderRight.start_continuous()

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
    self.I = 0
    self.D = 0
    self.last_diff = 0
    self.integral = 0
    self.desired = self.theta

    self.timer = Timer()
    self.state = ExploreState()
    # Starts the robot
    print "Robot setup complete."
    #self.run()

    # self.closeOrOpenGripper("Close")
    self.moveGripper(2)

  def loop(self):
    if self.timer.millis() > self.dT*1000:
      inputs = self.readSensors()
      process = self.state.process(inputs)
      # print "Process: " + process.__class__.__name__
      # print(self.gyro.val)
      self.state = process.get_next_state()
      self.processOutputs(process.get_outputs())

  def readSensors(self):
    # Calculate the distance traveled, change in theta, and then reset sensors
    distance_traveled = (self.encoderLeft.val + self.encoderRight.val) / 2.0
    #encoder_omega = self.encoderLeft.val - self.encoderRight.val
    print('frontRightIR: ', self.frontRightIR.val)
    print("frontLeftIR: ", self.frontLeftIR.val)
    print("leftIR: ", self.leftIR.val)
    print("rightIR: ", self.rightIR.val)
    # blocks = CalculateBlocks(); #what should CalculateBlocks return?
    return Inputs(distance_traveled, self.gyro.val, self.frontRightIR.val, self.frontLeftIR.val, self.leftIR.val, self.rightIR.val)
    # distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR

  def processOutputs(self, Outputs):
    # TODO Missing servo outputs
    if (Outputs.driving == True):
      self.motorval = 0 #25?
    else:
      self.motorval = 0
    if (Outputs.turning == True):
      if (Outputs.turn_clockwise == True):
        self.PID(self.gyro.val + 5)
      else:
        self.PID(self.gyro.val - 5)
    else:
      self.PID(self.gyro.val)
    if Outputs.gripperLevel != self.currentGripperLevel:
      self.moveGripper(Outputs.gripperLevel)

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

  def moveGripper(self, gripperLevel):
    # print("Moving gripper")
    # if gripperLevel < self.currentGripperLevel:
      # self.motorGripper.write(self.GRIPPER_DOWN, 50)
    # else:
      self.motorGripper.write(self.GRIPPER_UP, 50)
    # t = threading.Timer(2.0, self.motorGripper.write, [1,0]) # seconds
    # t.start()
    # self.currentGripperLevel = gripperLevel

  def openDoor(self):
    while(self.servovalDoor > self.DOOR_OPEN_POS):
      if (self.timerDoor.millis() > 10):
        self.timerDoor.reset()
        self.servovalDoor -= 1
        print self.servovalDoor
        self.servoDoor.write(abs(self.servovalDoor))

  def closeOrOpenGripper(self, closeOrOpenMode):
    # if closeOrOpenMode == "Close":
    # if closeOrOpenMode == "Open":
    while(self.servovalGripper < self.GRIPPER_CLOSE_POS):
      if (self.timerGripper.millis() > 10):
        self.timerGripper.reset()
        self.servovalGripper -= 1
        print self.servovalGripper
        self.servoGripper.write(abs(self.servovalGripper))

######################## States ###########################

class ExploreState:
  found_block = False
  left_wall_following = False
  right_wall_following = False
  facing_wall = False

  def process(self, Inputs):
    WALL_IN_FRONT = 20000
    if (Inputs.rightIR >= WALL_IN_FRONT):
      right_wall_following = True
    else:
      right_wall_following = False
    if (Inputs.leftIR >= WALL_IN_FRONT):
      left_wall_following = True
    else:
      left_wall_following = False

    if (self.found_block == False):
      if (Inputs.frontRightIR >= WALL_IN_FRONT and Inputs.frontLeftIR >= WALL_IN_FRONT):
        facing_wall = True
        return TurnFromWall(self)
      elif (Inputs.leftIR >= WALL_IN_FRONT or Inputs.rightIR >= WALL_IN_FRONT):
        return WallFollowing(self)
      else:
        return DrivingStraight(self)
    else:
      return FoundBlock()


class DriveToBlockState:
  def process(self, Inputs):
    # Move to cube
    # In Position
    # Lost Cube
    pass

####################### Processes #########################
class FoundBlock():
  # Change to state->Drive to block without moving
  def get_next_state(self):
    return DriveToBlockState()
  def get_outputs(self):
    driving = False
    turning = False
    turn_clockwise = False
    gripperLevel = 2
    return Outputs(driving, turning, turn_clockwise, gripperLevel)

class WallFollowing():
  def __init__(self, ExploreState):
    self.state = ExploreState
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    gripperLevel = 2
    return Outputs(driving, turning, turn_clockwise, gripperLevel)

class TurnFromWall():
  def __init__(self, explore):
    self.state = explore
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = False
    turning = True
    if (self.state.left_wall_following):
      turn_clockwise = True
    else:
      turn_clockwise = False
    gripperLevel = 2
    return Outputs(driving, turning, turn_clockwise, gripperLevel)

class DrivingStraight():
  def __init__(self, explore):
    self.state = explore
  def get_next_state(self):
    return self.state
  def get_outputs(self):
    driving = True
    turning = False
    turn_clockwise = False
    gripperLevel = 2
    return Outputs(driving, turning, turn_clockwise, gripperLevel)

################## Seperate Classes #######################

# Represents the inputs returned from the sensors (gyro, encoders, and webcam)
class Inputs:
  def __init__(self, distance_traveled, theta, frontRightIR, frontLeftIR, leftIR, rightIR):
    self.distance_traveled = distance_traveled
    self.theta = theta
    #self.blocks = blocks
    self.frontLeftIR = frontLeftIR
    self.frontRightIR = frontRightIR
    self.leftIR = leftIR
    self.rightIR = rightIR
  
  def get_distance_traveled():
    return self.distance_traveled
  def get_theta():
    return self.theta

class Outputs:
  def __init__(self, driving, turning, turn_clockwise, gripperLevel):
    self.driving = driving
    self.turning = turning
    self.turn_clockwise = turn_clockwise
    self.gripperLevel = gripperLevel

# These can be written as updateable positions on the map
# For now they must be generated from every image
class Block:
  def __init__(self, distance_from_bot, heading, color):
    self.distance = distance_from_bot
    self.heading = heading
    self.goal_color = (color == GOAL_COLOR)

if __name__ == "__main__":
  robot = MyRobot(5, -0.00001, 100)
  sys.setrecursionlimit(10000)
  robot.run()
