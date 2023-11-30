import cv2 
import matplotlib.pyplot as plt
from motor_controller import motor_controller
from qrcode import GetQRCode 
from pid_controller import pid_controller
from plots import Plot
from rotary_encoder_rpm import RotaryEncoder
import time


#Pin definitions
RPWM = 19;  # GPIO pin 19 to the RPWM on the BTS7960
LPWM = 13;  #GPIO pin 13 to the LPWM on the BTS7960
R_EN = 20;
L_EN = 21; # connect GPIO pin 21 to L_EN on the BTS7960

code = GetQRCode()
pid_controller = pid_controller()
control = motor_controller(RPWM, LPWM, L_EN, R_EN)
time_last_QR = time.time()

ppr = 500   #rotary encoder ppr, specification of the motor

error = []          #array with all error values, empty initialization
motor_input= []     #array with all motor input values, empty initialization
direc = []           #array with all direction values, empty initialization

start = False 
print("Press Enter to start or ESC to quit:")

#initialize variables for detection rate calculation
x_total = []
x_detection = []

try:
    while True: #infinite loop
        x = code.QRCodeDisplay() # in m
        #print('x offset', x)
        c = cv2.waitKey(10)
        
        
        if c == 13:
            #time_last_QR = time.time()                              #time initialization
            #print('First QR:', time_last_QR)
            time_interval = 0.5                                     #interval on which QR code is not detected and motor should stop
            time_out_duration = 5 
            
            start = True

        if start:
            if x is not None:                   #if QR code is being detected
                time_last_QR = time.time()
                direction, control_output = pid_controller.get_rotation_speed(x)
                error.append(x) #create an array with all error values over time
                motor_input.append(control_output) #create array with all control outputs over time
                direc.append(direction)

                if direction ==1:
                    control.motor_run_left(control_output)
                    print('Motor runs left with speed control output', control_output, 'and direction', direction)
                if direction == 0:
                    control.motor_run_right(control_output)
                    print('motor runs right with speed control output', control_output, 'and direction', direction)

            elif time.time()-time_last_QR > time_out_duration:
                print("Stopping execution: No QR code detected for a duration of:", time_out_duration, 'seconds')
                control_output = 0
                control.motorStop()
                break 
            
            elif x is None:
                print("QR code not detected")
                    
                if time.time()-time_last_QR > time_interval:
                        
                        control_output = 0
                        control.motor_run_left(control_output)
                        print("Motor stop, control_ouput = ", control_output)
                        
        x_total, x_detection, detection_rate = code.QR_detection_rate(x, x_total, x_detection)

        if c == 27:
            break
                                    
except KeyboardInterrupt:
    pass
       
finally: 
    control.motorStop()
    code.EndQRCode()
    

    print("Detection rate:", detection_rate)

    #creating a plot to get insight in the PID control
    plt.figure(1)
    Plot.plot_motor_input(error, motor_input, direc)    #creating a plot
    plt.show()
    