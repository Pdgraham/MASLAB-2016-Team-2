from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import DigitalOutput, Motor, Gyro, Encoder

GOAL_COLOR = "RED"

class MyRobot(SyncedSketch):
  runtime = 30 #seconds

  def setup(self):
    # initialize sensors, settings, start timers, etc.
    self.motorLeft = Motor(self.tamp, 21, 20)
    self.motorRight = Motor(self.tamp, 23, 22)
    self.motorval = 0
    self.motorLeft.write(1,0)
    self.motorRight.write(1,0)

    left_pins = 6,5
    right_pins = 3,4
    # Encoder doesn't work when after gyro
    self.encoderLeft = Encoder(self.tamp, 6,5, continuous=False)
    self.encoderRight = Encoder(self.tamp, 3,4, continuous=True)
    # TODO: set encoder to 0
    self.timer = Timer()
    self.gyro = Gyro(self.tamp, 10)
    self.theta = self.gyro.val    
    self.dT = .03

    self.encoderLeft.start_continuous()
    self.encoderRight.start_continuous()

    # Initialize PID Values
    self.P = 10
    self.I = 0
    self.D = 0
    self.last_diff = 0
    self.integral = 0
    self.desired = desired_theta

    self.timer = Timer()
    self.state = ExploreState()
    # Starts the robot
    self.run()

  def loop(self):
    if self.timer.millis() > self.dT*1000:
      inputs = self.readSensors()
      process = self.state.process(inputs)
      this.state = process.get_next_state()
      this.processOutputs(process.get_outputs())

  def readSensors():
    # Calculate the distance traveled, change in theta, and then reset sensors
    distance_traveled = (self.encoderLeft.val + self.encoderRight.val) / 2.0
    #encoder_omega = self.encoderLeft.val - self.encoderRight.val
    return Inputs(distance_traveled, self.gyro.val)

  def processOuputs(Outputs):
    # TODO Missing servo outputs
    if (Outputs.driving == True):
      self.motorval = 50
    else:
      self.motorval = 0
    if (Outputs.turning == True):
      PID(Outputs.desired_theta)
    else:
      PID()

  def PID(desired_theta=self.gyro.val):

    # Set encoder to 0 after turning.
    # To turn in place, set bias (i.e. motorval to 0)
    estimated = gyroVal # TODO: calculate estimated with encoder
    diff = self.desired_theta - estimated
    self.integral += diff * self.dT
    derivative = (diff - self.last_diff)/self.dT
    power = self.P*diff + self.I*self.integral + self.D*derivative # NOTE: Cap self.D*derivative, use as timeout
    self.motorLeft.write(self.motorval>0, min(255, abs(self.motorval + power)))
    self.motorRight.write(self.motorval>0, min(255, abs(self.motorval - power)))
    # print "EncoderLeft: " + str(self.encoderLeft.val)
    # print "EncoderRight: " + str(self.encoderRight.val)


######################## States ###########################

class ExploreState:
  def process(Inputs):
    # Found a cube
    # No Cubes
    pass

  

class DriveToBlockState:
  def process(Inputs):
    # Move to cube
    # In Position
    # Lost Cube
    pass

####################### Processes #########################

################## Seperate Classes #######################

# Represents the inputs returned from the sensors (gyro, encoders, and webcam)
class Inputs:
  def __init__(self, distance_traveled, theta):
    self.distance_traveled = distance_traveled
    self.theta = theta
    #self.blocks = blocks
  
  def get_distance_traveled():
    return self.distance_traveled
  
  def get_theta():
    return self.theta

# These can be written as updateable positions on the map
# For now they must be generated from every image
class Block:
  def __init__(self, distance_from_bot, heading, color):
    self.distance = distance_from_bot
    self.heading = heading
    self.goal_color = (color == GOAL_COLOR)

# Taken from pyHost/utils
class Timer(object):
    def __init__(self):
        self.val = time()
    def millis(self):
        return round((time() - self.val)*1e3)
    def micros(self):
        return round((time() - self.val)*1e6)
    def set(self, new_value):
        self.val = new_value
    def reset(self):
        self.val = time()

if __name__ == "__main__":
  MyRobot(5, -0.00001, 100)
