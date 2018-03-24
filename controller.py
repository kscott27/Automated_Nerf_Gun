# controller.py
# Kevin Scott, Kevin, Gasik, Caleb Barber

import Encoder
import utime

class Controller:
    ''' Class for performing closed-loop proportional control'''
    
    def __init__(self, kp, setpoint, encoder):
        '''Constructs the controller and sets the proportional gain and initial setpoint.
        @param kp The proportional gain
        @param setpoint The initial setpoint
        @param '''
        
        self.encoder = encoder
        self.kp = kp
        self.setpoint = setpoint
        self.times = []
        self.positions = []
        self.errorList = []
        self.position = self.encoder.get_position()
        self.error = self.position - self.setpoint
        self.offset = 0
        self.cur_time = []
        self.g = 9.81
        self.belt_ratio = 10
        self.gear_ratio = 31
        self.ticks_per_rev = 2000
       
        
    def run_control(self):
        ''' Runs the control algorithm repeatedly. It returns the actuation value (PWM)
        to be sent to the device (motor)'''
       

        self.position = self.encoder.get_position()
        self.error = self.position - self.setpoint
        actuation = self.kp * self.error
        
        if actuation > 20:
            actuation = 20
        elif actuation < -20:
            actuation = -20
        
        return actuation
    
    #def run_control(self):
        #''' Runs the control algorithm repeatedly. It returns the actuation value (PWM)
        #to be sent to the device (motor)'''
       
        
        #self.times.append(utime.ticks_ms())
        #self.offset = self.times[0]
        #self.cur_time.append(utime.ticks_ms() - self.offset)
        #self.position = self.encoder.get_position()
        #self.positions.append(self.position)
        #self.error = self.position - self.setpoint
        #self.errorList.append(self.error)
        #actuation = self.kp * self.error
        
        #return actuation
    
    def run_imu_control(self, imu_span, imu_tilt):
        ''' Runs the control algorithm repeatedly. It returns the actuation value (PWM)
        to be sent to the device (motor)'''
        
        
        self.setpoint 
        self.position = self.encoder.get_position()
        self.error = self.position - self.setpoint
        
        return actuation
        
        
    def set_setpoint(self, value):
        '''Sets the setpoint for the controller.'''
        self.setpoint = value
        self.error = self.position - self.setpoint
        
    def set_gain(self, k):
        '''Sets the gain for the controller.'''
        self.kp = k
    
    def return_results(self):
        '''Prints results of step response test.'''
        #for i in range(len(self.times)):
           
            #print('{}, {}'.format(self.cur_time[i], self.positions[i]))
        return(self.cur_time , self.positions)
        
    def reset_data(self): 
        self.times = []
        self.cur_time = []
        self.positions = []
           
            
    def get_response(self):
        return self.encoder.get_position()
