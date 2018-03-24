import pyb
from pyb import I2C
import MMA845X
import micropython
import gc
import math

import cotask
import task_share

import L6206MD
import CRSBotBrainMD

import encoder

import controller

import utime

# Virtual com port object for serial communication
vcp = pyb.USB_VCP()

# I2C object for sensor communication
i2c = I2C(1, I2C.MASTER, baudrate = 100000)

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
micropython.alloc_emergency_exception_buf (100)

GOING = const (0)
STOPPED = const (1)

def main_menu ():
    ''' Function that prints the main menu options to the serial interface.'''
    
    vcp.write('\r\nTHE FOLLOWING COMMANDS MUST BE UPPERCASE.\r\n'.encode())
    vcp.write('Press G to enter grid coordinates.\r\n'.encode())
    vcp.write('Press A to enable the dart motors.\r\n'.encode())
    vcp.write('Press Z to disable the dart motors.\r\n'.encode())
    vcp.write('Press F to fire a dart.\r\n'.encode())
    vcp.write('Press I to have the most recent IMU coordinates printed.\r\n'.encode())
    vcp.write('Press C to enter calibration transform data for the device.\r\n'.encode())
    vcp.write('Press S to directly operate the span motor.\r\n'.encode())
    vcp.write('Press T to directly operate the tilt motor.\r\n'.encode())
            
def task_span_motor_fun ():
    ''' Function that implements the Span Motor Task, which runs a closed-loop
    proportional controller for this motor.'''
    
    state = 0
    counter = 0
    controller_count = 0
    startTime = utime.ticks_ms()
    
    #setting up pinC6, C7 and time8 for encoder
    pinC6 = pyb.Pin(pyb.Pin.board.PC6)

    pinC7 = pyb.Pin(pyb.Pin.board.PC7)
    tim8 = pyb.Timer(8, period=0xFFFF, prescaler=0)

    enc = encoder.EncoderReader(pinC6, pinC7, tim8)
    enc.zero()
    
    #setting up enable pin, IN1 and IN2 pins, and timer for motor driver
    pinA10 = pyb.Pin(pyb.Pin.board.PA10, pyb.Pin.OUT_PP) #enable pin
        
    pinB4 = pyb.Pin (pyb.Pin.board.PB4, pyb.Pin.OUT_PP)                           
    pinB5 = pyb.Pin (pyb.Pin.board.PB5, pyb.Pin.OUT_PP)
    tim3 = pyb.Timer (3, freq=20000)
    
    md = L6206MD.L6206MD(pinA10, pinB4, pinB5, tim3)
    dir_op_duty_cycle = 10
    
    #setting up controller object
    control = controller.Controller(1, 0, enc)
    control.set_gain(0.01)
    
    while True: 
       
        if state == 0:

            if new_setpoint.get() > 0:
                print ("Span motor initially at " + str(enc.get_position()) + " ticks.")
                control.set_setpoint(span_setpoint.get())
                controller_count = 0
                state = 1
                
            elif span_mot_op.get() == 1:
                md.set_duty_cycle(dir_op_duty_cycle)
                state = 4
                
            elif span_mot_op.get() == 2:
                md.set_duty_cycle(-dir_op_duty_cycle)
                state = 5
                vcp.write('Span motor spinning CCW.'.encode())
                
            elif enc_zero.get() > 0:
                enc.zero()
                enc_zero.put(0)
                print (str(enc.get_position))
                
            elif imu_mode.get() > 0:
                state = 3

        elif state == 1:

            md.set_duty_cycle(control.run_control())
            controller_count = controller_count + 1
            if controller_count == 70:
                state = 0
                controller_count = 0
                md.set_duty_cycle(0)
                new_setpoint.put(0)
                print ('Span control loop terminated.')
                print ('Span motor: ' + str(enc.get_position()) + '\r\n')

        elif state == 2:
            md.set_duty_cycle(0)
            xdata,ydata = control.return_results()
            for i in range(len(xdata)):
                str_data = str(xdata[i]) + ',' + str(ydata[i]) + '\n'
                str_data = str_data.encode('UTF-8')
                vcp.write(str_data)
            
            control.reset_data()
            state = 0
            
        elif state == 3:
            
            if imu_mode.get() > 0:
                control.set_setpoint(span_setpoint.get())
                md.set_duty_cycle(control.run_control())
            else:
                span_setpoint.put(0)
                state = 0
                
        elif state == 4:
            if span_mot_op.get() == 0:
                md.set_duty_cycle(0)
                state = 0
            elif span_mot_op.get() == 2:
                md.set_duty_cycle(-dir_op_duty_cycle)
                state = 5
                
        elif state == 5:
            if span_mot_op.get() == 0:
                md.set_duty_cycle(0)
                state = 0
            elif span_mot_op.get() == 1:
                md.set_duty_cycle(dir_op_duty_cycle)
                state = 4

        yield (state)

        
        
def task_tilt_motor_fun ():
    ''' Function that implements the Tilt Motor Task, which runs a closed-loop
    proportional controller for this motor.'''
    
    state = 0
    counter = 0
    controller_count = 0
    startTime = utime.ticks_ms()
    
    #setting up pinB6, B7 and time4
    pinB6 = pyb.Pin(pyb.Pin.board.PB6)

    pinB7 = pyb.Pin(pyb.Pin.board.PB7)
    tim4 = pyb.Timer(4, period=0xFFFF, prescaler=0)

    enc = encoder.EncoderReader(pinB6, pinB7, tim4)
    enc.zero()
    
    
    #setting up enable pin, IN1 and IN2 pins, and timer for motor driver
    pinC1 = pyb.Pin(pyb.Pin.board.PC1, pyb.Pin.OUT_PP) #enable pin
        
    pinA0 = pyb.Pin (pyb.Pin.board.PA0, pyb.Pin.OUT_PP)                           
    pinA1 = pyb.Pin (pyb.Pin.board.PA1, pyb.Pin.OUT_PP)
    tim5 = pyb.Timer (5, freq=20000)
    
    md = L6206MD.L6206MD(pinC1, pinA0, pinA1, tim5)
    dir_op_duty_cycle = 10
    
    #setting up controller object
    control = controller.Controller(1, 0, enc)
    control.set_gain(0.01)
    
    
    while True: 
        
        if state == 0:

            if new_setpoint.get() > 0:
                print ("Tilt motor initially at " + str(enc.get_position()) + " ticks.")
                control.set_setpoint(tilt_setpoint.get())
                controller_count = 0
                state = 1
                
            elif tilt_mot_op.get() == 1:
                md.set_duty_cycle(dir_op_duty_cycle)
                state = 4
                
            elif tilt_mot_op.get() == 2:
                md.set_duty_cycle(-dir_op_duty_cycle)
                state = 5
                
            elif enc_zero.get() > 0:
                enc.zero()
                enc_zero.put(0)
                print (str(enc.get_position))
                
            elif imu_mode.get() > 0:
                state = 3

        elif state == 1:

            md.set_duty_cycle(control.run_control())
            #print (enc.get_position())
            controller_count = controller_count + 1
            if controller_count == 70:
                state = 0
                controller_count = 0
                new_setpoint.put(0)
                md.set_duty_cycle(0)
                print ("Tilt control loop terminated.")
                print ('Tilt motor: ' + str(enc.get_position()))

        elif state == 2:
            md.set_duty_cycle(0)
            xdata,ydata = control.return_results()
            for i in range(len(xdata)):
                str_data = str(xdata[i]) + ',' + str(ydata[i]) + '\n'
                vcp.write(str_data.encode('UTF-8'))
            
            control.reset_data()
            state = 0

        elif state == 3:
            if imu_mode.get() > 0:
                control.set_setpoint(tilt_setpoint.get())
                md.set_duty_cycle(control.run_control())
            else:
                tilt_setpoint.put(0)
                state = 0
                
        elif state == 4:
            if tilt_mot_op.get() == 0:
                md.set_duty_cycle(0)
                state = 0
            elif tilt_mot_op.get() == 2:
                md.set_duty_cycle(-dir_op_duty_cycle)
                state = 5
                
        elif state == 5:
            if tilt_mot_op.get() == 0:
                md.set_duty_cycle(0)
                state = 0
            elif tilt_mot_op.get() == 1:
                md.set_duty_cycle(dir_op_duty_cycle)
                state = 4
                
        elif state == 6:
            md.set_duty_cycle(control.run_control())
            controller_count = controller_count + 1
            if controller_count == 250:
                state = 0
                new_setpoint.put(0)
                run_calibration.put(0)
            
        yield (state)        
    
def task_interface_fun ():
    ''' Function that implements the Interface Task. This task will continually
    check the serial port for instructions sent from a PC about where to aim
    and when to fire.'''

    state = 0
    line = ''
    main_menu()
    char_in = None
    
    while True:
         
        if vcp.isconnected() == True and vcp.any():
            char_in = vcp.read()
            char_in = char_in.decode()
            line = line + char_in
            
            #The Interface is in the main menu of options.
            if state == 0:
                    
                if char_in == 'G':
                    
                    vcp.write('\r\nPlease enter coordinates:\r\n'.encode())
                    line = ''
                    state = 1
        
                elif char_in == 'F':
                    fire.put(1)
                    
                elif char_in == 'L':
                    vcp.write('Span motor turning left.'.encode())
                    span_mot_op.put(1)
                    
                elif char_in == 'R':
                    vcp.write('Span motor turning right.'.encode())
                    span_mot_op.put(2)
                    
                elif char_in == 'D':
                    print ("Tilt motor moving down.")
                    tilt_mot_op.put(1)
                    
                elif char_in == 'U':
                    print ("Tilt motor moving up.")
                    tilt_mot_op.put(2)
                    
                elif char_in == 'S':
                    vcp.write('Motors stopped.'.encode())
                    span_mot_op.put(0)
                    tilt_mot_op.put(0)
                    
                    
                elif char_in == 'Z':
                    enc_zero.put(1)

                elif char_in == '\x03':
                    run_RTOS.put (0)
                    
                    
                elif char_in == 'I':
                    
                    x_imu = imu_x.get()
                    y_imu = imu_y.get()
                    z_imu = imu_z.get()
                    
                    
                    vcp.write('\r\nax = '.encode())
                    vcp.send(str(x_imu).encode())
                    
                    vcp.write('\r\nay = '.encode())
                    vcp.write(str(y_imu).encode())
                    
                    vcp.write('\r\naz = '.encode())
                    vcp.write(str(z_imu).encode())
                
                    
                    vcp.write('\r\n'.encode())
                    
                elif char_in == 'J':
                    imu_mode.put(1)
                    state = 6
                    
                elif char_in == 'C':
                    print ('Please enter the following information in this format: A,B,C,D,E,F:\r\n')
                    line = ''
                    run_calibration.put(1)
                    state = 3
                
                elif char_in == 'H':
                    main_menu()
                    
                else:
                    print (char_in + " is not a valid command")
                
            #The interface is gathering which grid square to fire at.    
            elif state == 1:
                
                vcp.write(char_in)
                
                # If enter is pressed, make all of the characters that have been
                # entered onto the serial bus into a list.
                if char_in == '\r': 
                    vcp.write('\n'.encode())

                    coordinates = list(line)
                    x = coordinates[0]
                    y = coordinates[1]                   
                    
                    index_conv = {"A": 0,
                                  "B": 1,
                                  "C": 2,
                                  "D": 3,
                                  "E": 4}
                    
                    
                    x = index_conv[x]
                    y = int(y) - 1
                    dest_row.put (y)
                    dest_col.put (x)
                    calc_setpoint.put (1)

                    state = 0
                    
            
            # The interface is gathering calibration data from the user
            # which it will then send to the calculator.
            elif state == 3:
                
                vcp.write(char_in)
                
                if char_in == '\r': 
                    vcp.write('\n'.encode())

                    coordinates = line.split(',')

                    A = float(coordinates[0])
                    B = float(coordinates[1])
                    C = float(coordinates[2])
                    D = float(coordinates[3])
                    E = float(coordinates[4])
                    F = float(coordinates[5])
                    
                    calibration_data.put(A)
                    calibration_data.put(B)
                    calibration_data.put(C)
                    calibration_data.put(D)
                    calibration_data.put(E)
                    calibration_data.put(F)
                    
                    vcp.write('Calibrating...\r\n'.encode())
                    main_menu()
                    calibrate_calc.put(1)
                    state = 0
            
            # If any key is pressed, the interface exits IMU mode.
            elif state == 6:
                imu_mode.put(0)
                print ("Exiting IMU Mode.")
                state = 0
                line = ''
                    
            if char_in == '\r':
                    line = ''
        yield (state)
        

def task_dart_motor_fun ():
    ''' Function that provides power for the gun motor that propels
    the darts. '''
    
    #output pin
    pinC3 = pyb.Pin(pyb.Pin.board.PC3, pyb.Pin.OUT_PP)
    
    #Set output pin low initially
    pinC3.low()
    
    state = 0
    
    while True:
        
        if state == 0:
            if dart_motor.get() > 0:
                print ("Gun motors a'blazin!")
                pinC3.high()
                state = 1
                
        elif state == 1:
            if dart_motor.get() == 0:
                print ("Gun motors deactivated.")
                pinC3.low()
                state = 0
                
        yield (state)
        
        
def task_trigger_fun ():
    ''' Function that implements the Trigger Actuation Task. This task powers a servo
    motor which is used to pull the trigger of the gun in order to push the dart into
    the motor chamber.'''
    
    pinA5 = pyb.Pin (pyb.Pin.board.PA5, pyb.Pin.OUT_PP)
    tim2 = pyb.Timer (2, freq=50)
    
    md = CRSBotBrainMD.CRSBotBrainMD(pinA5, tim2)
    md.set_duty_cycle(10.5)
    state = 0
    
    while True:
        
        if state == 0:
            if fire.get() > 0:
                vcp.write('Dart fired!'.encode())
                md.set_duty_cycle(1.3)
                counter = 0
                state = 1
                        
        elif state == 1:
            counter = counter + 1
            if fire.get() == 0 or counter == 5:
                fire.put (0)
                state = 0
                md.set_duty_cycle(10.5)
              
            
        yield (state)
        
        
def task_imu_fun ():
    ''' Function that implements the IMU task. This task utilizes I2C functionality to continually
    check the orientation of the sensor, and store it in global shares to be accessed by other tasks.'''
    
    imu_addr = i2c.scan()
    imu_addr = imu_addr[0]
    
    imu = MMA845X.MMA845x(i2c, imu_addr)
    
    imu.active()
    
    state = 0
    
    while True:
            
        if state == 0:
            
            current_pos = imu.get_accels()
            state = 1                
            
        elif state == 1:
                
            x = float(current_pos[0])
            y = float(current_pos[1])
            z = float(current_pos[2])
                
            imu_x.put(x)
            imu_y.put(y)
            imu_z.put(z)
            
            state = 0
            
            
        yield (state)
        
        
def task_calculator_fun ():
    ''' Function that implements the calculator task. This task performs all of the math behind
    the calibration of the device, as well as the set point needed to hit certain grid squares.'''
    
    ticks_per_rev = 2000
    gear_ratio = 31
    belt_ratio = 10
    g = 2
    
    sqr_size = 8
    grid_dim = 5
    w = sqr_size * grid_dim
    h = sqr_size * grid_dim
    
    y = 10.5*12
    
    # Default transform values - no coordinates will be transformed with these
    # coefficients
    A = 1
    B = 0
    C = 0
    D = 0
    E = 1
    F = 0

    DET = A * E - B * D

    
    matrix = [[0 for x in range(2)] for y in range(2)]

    matrix[0][0] = E / DET
    matrix[0][1] = -B / DET
    matrix[1][0] = -D / DET
    matrix[1][1] = A / DET
    vector = [[0], [0]]
    position_vector = [[0], [0]]
    
    grid_locs = [[0 for x in range(grid_dim)] for y in range(grid_dim)]
    
    # Initial grid value coordinates to be used.
    for columns in range(grid_dim):
        x_range = (columns - (grid_dim-1)/2) * sqr_size
        
        for rows in range(grid_dim):
            z_range = (((grid_dim-1)/2) - rows) * sqr_size
            
            grid_locs[rows][columns] = [x_range, z_range]
    
    
    state = 0
    
    while True:
            
        # In State 0, the calculator is idle.
        if state == 0:
            
            if run_calibration.get() > 0:
                state = 1 
            elif calc_setpoint.get() > 0:
                state = 2
            elif imu_mode.get() > 0:
                state = 3
        
        # In State 1, the calculator calibrates the device.
        elif state == 1:

            A = calibration_data.get()
            B = calibration_data.get()
            C = calibration_data.get()
            D = calibration_data.get()
            E = calibration_data.get()
            F = calibration_data.get()
            
            
            # Transform coefficients
            DET = A * E - B * D
            matrix[0][0] = E / DET
            matrix[0][1] = -B / DET
            matrix[1][0] = -D / DET
            matrix[1][1] = A / DET

            calibrate_calc.put(0)
            run_calibration.put(0)
            state = 0
        
        # In State 2, the calculator determines the setpoint of the encoder
        # needed to hit the desired location.
        elif state == 2:
            
            print ("Calculating...\r\n")
            x, z = grid_locs[dest_row.get()][dest_col.get()]
            
            print ("OG x (inches): " + str(x))
            print ("OG z (inches): " + str(z))
            
            vector[0] = x - C
            vector[1] = z - F

            position_vector[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1]
            position_vector[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1]


            x = position_vector[0]
            z = position_vector[1]
            
            print ("Calibrated x (inches): " + str(x))
            print ("Calibrated z (inches): " + str(z))
            
            theta_span = math.atan2(x, y)
            theta_tilt = math.atan2(z, y)
            theta_span_deg = theta_span * 360 / 6.28
            theta_tilt_deg = theta_tilt * 360 / 6.28
            
            ticks_span = theta_span / 6.28 * belt_ratio * gear_ratio * ticks_per_rev
            ticks_tilt = theta_tilt / 6.28 * belt_ratio * gear_ratio * ticks_per_rev
            
            tilt_setpoint.put(ticks_tilt)
            span_setpoint.put(ticks_span)
            
            calc_setpoint.put(0)
            new_setpoint.put(1)
            print ('Span setpoint: ' + str(span_setpoint.get()) + '\r\n')
            print ('Tilt setpoint: ' + str(tilt_setpoint.get()) + '\r\n')
            print ('Span degrees: ' + str(theta_span_deg) + '\r\n')
            print ('Tilt degrees: ' + str(theta_tilt_deg) + '\r\n')
            
            state = 0
            
        # In State 3, the calculator is constantly re-adjusting the setpoints of the
        # motors based on the orientation of the IMU.
        elif state == 3:
            if imu_mode.get() > 0:
                theta_span = math.asin(imu_x.get()/(2))
                theta_tilt = math.asin(imu_z.get()/(2))
                
                if theta_span > .35:
                    theta_span = .35
                elif theta_span < -.35:
                    theta_span = -.35
                        
                if theta_tilt > .35:
                    theta_tilt = .35
                elif theta_tilt < -.35:
                    theta_tilt = -.35
                        
                
                ticks_span = theta_span / 6.28 * belt_ratio * gear_ratio * ticks_per_rev
                ticks_tilt = theta_tilt / 6.28 * belt_ratio * gear_ratio * ticks_per_rev
                
                print ("Span setpoint: " + str(ticks_span))
                print ("Tilt setpoint: " + str(ticks_tilt))
                
                span_setpoint.put(ticks_span)
                tilt_setpoint.put(ticks_tilt)
                
            else:
                
                state = 0
            
            
        yield (state)
        

# =============================================================================

if __name__ == "__main__":

    vcp.write('Welcome to the STM32 Real-Time Operating System.\r\n'.encode())
    
    import print_task
    
    # Shares for the interface to tell each motor where its destination is
    span_setpoint = task_share.Share ('f', thread_protect = True, name = "Span_Motor_Setpoint")
    tilt_setpoint = task_share.Share ('f', thread_protect = True, name = "Tilt_Motor_Setpoint")
    
    # Shares for directly operating both motors from the interface.
    span_mot_op = task_share.Share ('i', thread_protect = True, name = "Span_Motor_Operator")
    tilt_mot_op = task_share.Share ('i', thread_protect = True, name = "Tilt_Motor_Operator")
    
    # Share for the calculator to tell the motors that there are new setpoints.
    new_setpoint = task_share.Share ('i', thread_protect = True, name = "New_Setpoint")
    
    # Flag for zeroing both encoders
    enc_zero = task_share.Share ('i', thread_protect = True, name = "Encoder_Zero")
    
    # Share for the interface to tell the dart motor to turn on and off
    dart_motor = task_share.Share ('i', thread_protect = True, name = "Dart_Motor")
    
    # Share for the interface to tell the trigger actuator to fire a dart
    fire = task_share.Share ('i', thread_protect = True, name = "Fire")
    
    # Share for the interface to tell the scheduler to stop running
    run_RTOS = task_share.Share ('i', thread_protect = False, name = "Run_RTOS")
    
    # Share for the interface to tell the calculator to calculate the next encoder
    # set points.
    calc_setpoint = task_share.Share ('i', thread_protect = False, name = "Calculate_Setpoint")
    
    # Share for holding the destination row.
    dest_row = task_share.Share ('i', thread_protect = False, name = "Destination_Row")
    
    # Share for holding the destination column.
    dest_col = task_share.Share ('i', thread_protect = False, name = "Destination_Column")
    
    # Share for print IMU Data
    imu_print = task_share.Share ('i', thread_protect = False, name = "IMU_Data")
    
    # Share for the x data of the IMU
    imu_x = task_share.Share ('f', thread_protect = False, name = "IMU_X_Data")
    
    # Share for the y data of the IMU
    imu_y = task_share.Share ('f', thread_protect = False, name = "IMU_Y_Data")
    
    # Share for the z data of the IMU
    imu_z = task_share.Share ('f', thread_protect = False, name = "IMU_Z_Data")
    
    # Share for entering IMU mode
    imu_mode = task_share.Share ('i', thread_protect = False, name = "IMU_Mode")
    
    # Share for the interface to tell the calculator that it is time to run a calibration
    calibrate_calc = task_share.Share ('i', thread_protect = False, name = "Calibrate_Calculation")
    
    # Share for the interface to tell the motors to run a calibration procedure.
    run_calibration = task_share.Share ('i', thread_protect = False, name = "Run_Calibration")
    
    # Queue for storing calibration measurements used by the calculator.
    calibration_data = task_share.Queue ('f', 6, thread_protect = False, overwrite = False,
                                         name = "Calibration_Queue")

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    
    #task1 = cotask.Task (task1_fun, name = 'Task_1', priority = 1, 
                         #period = 1000, profile = True, trace = False)
    #task2 = cotask.Task (task2_fun, name = 'Task_2', priority = 2, 
                         #period = 100, profile = True, trace = False)
    span_motor_task = cotask.Task (task_span_motor_fun, name = 'Span_Motor_Task',
                               priority = 6, period = 10, profile = True,
                               trace = False)
    tilt_motor_task = cotask.Task (task_tilt_motor_fun, name = 'Tilt_Motor_Task',
                               priority = 6, period = 10, profile = True,
                               trace = False)
    interface_task = cotask.Task (task_interface_fun, name = 'Interface_Task',
                                  priority = 1, period = 100, profile = True,
                                  trace = False)
    dart_motor_task = cotask.Task (task_dart_motor_fun, name = 'Dart_Motor',
                              priority = 1, period = 250, profile = True,
                              trace = False)
    trigger_task = cotask.Task (task_trigger_fun, name = 'Trigger_Task',
                                priority = 4, period = 250, profile = True,
                                trace = False)
    imu_task = cotask.Task (task_imu_fun, name = 'IMU_Task',
                            priority = 3, period = 50, profile = True,
                            trace = True)
    calculator_task = cotask.Task (task_calculator_fun, name = 'Calculator_Task',
                                   priority = 3, period = 100, profile = True,
                                   trace = True)
    
    cotask.task_list.append (span_motor_task)
    cotask.task_list.append (tilt_motor_task)
    cotask.task_list.append (interface_task)
    cotask.task_list.append (dart_motor_task)
    cotask.task_list.append (trigger_task)
    cotask.task_list.append (imu_task)
    cotask.task_list.append (calculator_task)

    # A task which prints characters from a queue has automatically been
    # created in print_task.py; it is accessed by print_task.put_bytes()
  
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()

    # Run the scheduler with the chosen scheduling algorithm.
    run_RTOS.put (1)
    while run_RTOS.get():
        cotask.task_list.pri_sched ()

    vcp.read ()
    # Print a table of task data and a table of shared information data
    print ('\n' + str (cotask.task_list) + '\n')
    print (task_share.show_all ())
    print (task1.get_trace ())
    print (imu_task.get_trace())
    print ('\r\n')

