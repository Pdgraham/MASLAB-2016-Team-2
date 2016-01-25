from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import DigitalOutput, Motor, Gyro, Encoder

class PID(SyncedSketch):
    def setup(self):
        #self.motorLeft = Motor(self.tamp, 21, 20)
        #self.motorRight = Motor(self.tamp, 23, 22)
        self.motorLeft = Motor(self.tamp, 20, 21)
        self.motorRight = Motor(self.tamp, 22, 23)
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

        self.P = 5
        self.I = 0 # should be negative
        self.D = 0
        self.dT = .03
        self.motorval = 25 #50
        self.last_diff = 0
        self.integral = 0
        self.desired = self.gyro.val #+ 45 # to drive in a straight line
        self.encoderLeft.start_continuous()
        self.encoderRight.start_continuous()

    def loop(self):
        # Set encoder to 0 after turning. 
        # To turn in place, set bias (i.e. motorval to 0)
        if self.timer.millis() > self.dT*1000:
            self.timer.reset()
            gyroVal = self.gyro.val
            print gyroVal, self.gyro.status
            # TODO: encoderVal
            estimated = gyroVal # TODO: calculate estimated with encoder
            diff = self.desired-estimated
            self.integral += diff*self.dT
            derivative = (diff - self.last_diff)/self.dT
            power = self.P*diff + self.I*self.integral + self.D*derivative # NOTE: Cap self.D*derivative, use as timeout
            power = min(40, power)
            self.motorLeft.write((self.motorval + power) > 0, min(255, abs(self.motorval + power)))
            self.motorRight.write((self.motorval - power) > 0, min(255, abs(self.motorval - power)))
            print "EncoderLeft: " + str(self.encoderLeft.val)
            print "EncoderRight: " + str(self.encoderRight.val)

    def stop(self):
        self.motorLeft.write(1, 0)
        self.motorRight.write(1, 0)
        # print self.motorval
        super(PID,self).stop()
        self.tamp.clear_devices();

if __name__ == "__main__":
    # change the first argument as necessary
    sketch = PID(5, -0.00001, 100)
    sketch.run()
