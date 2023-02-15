import curses
from itertools import count
import signal, sys
from math import pi
import numpy as np
import rtde_control
import rtde_receive
from threading import Thread, Lock


#Robot parameters
ROBOT_ADDRESS = "192.168.0.10"
JOINT_VELOCITY     = 2
JOINT_ACCELERATION = 1.5

#Home configuration
HOME = [0, -pi/2, 0, -pi/2, 0, 0]
#Configurations that realize the same EE pose
C  = np.array(( [1.0472, -1.2833, -0.7376, -2.6915, -1.5708,  3.1416],
                [1.0472, -1.9941,  0.7376,  2.8273, -1.5708,  3.1416],
                [1.0472, -1.5894, -0.5236,  0.5422,  1.5708,  0.0000],
                [1.0472, -2.0944,  0.5236,  0.0000,  1.5708,  0.0000],
                [2.7686, -1.0472, -0.5236,  3.1416, -1.5708,  1.4202],
                [2.7686, -1.5522,  0.5236,  2.5994, -1.5708,  1.4202],
                [2.7686, -1.1475, -0.7376,  0.3143,  1.5708, -1.7214],
                [2.7686, -1.8583,  0.7376, -0.4501,  1.5708, -1.7214]))
#Singular configuration
Q_SINGULAR1 = [1.6067, -1.1981, 0.0000, 5.3731, 1.3489, -4.6411]
Q_SINGULAR2 = [-1.0093, -2.3176, 0.0000, 5.3732, -4.9342, 4.0461]
Q_SINGULAR3 = [1.8014, -1.9925, -0.0000, -4.7123, 0.0001, 4.7632]
Q_SINGULAR4 = [1.6008, -1.5708, 0.0000, -5.3424, 4.2970, 5.0001]
Q_SINGULAR5 = [1.0106, -1.1150, -0.0000, -0.9100, 1.3488, -3.9721]

Q_SINGULAR  = np.array(( 
    [1.4698,   -0.5097,   -0.3219,    0.8790,    1.0712],
    [-1.9925,   -1.5971,   -1.6335,   -1.5708,   -0.9331],
    [0.0000,   -0.1456,   -0.0433,   -0.0000,   -0.0000],
    [2.2316,   -0.0011,   -0.4386,   -1.1793,   -0.9100],
    [1.7926,   -3.1361,   -0.0000,    0.5445,    1.3490],
    [-1.4991,   -0.1887,    0.2001,    0.4285,   -1.9221]))

#Variables used for the display
ROW_ITERATOR = (count(start = 0, step = 1))

#Initialize the terminal
def initTerminal(stdsrc):
    #Initialize the colors
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_WHITE)
    curses.init_pair(3, curses.COLOR_BLUE, curses.COLOR_WHITE)
    
    #Clear the terminal
    stdsrc.clear()

    #Display the title
    #TODO: Center the title
    stdsrc.addstr(next(ROW_ITERATOR), 0, "#"*curses.COLS)
    stdsrc.addstr(next(ROW_ITERATOR), 0, "#  _    _         _                                _   _____         _             _     _____                          ")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "# | |  | |       (_)                              | | |  __ \       | |           | |   |  __ \                         ")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "# | |  | | _ __   _ __   __ ___  _ __  ___   __ _ | | | |__) | ___  | |__    ___  | |_  | |  | |  ___  _ __ ___    ___  ")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "# | |  | || '_ \ | |\ \ / // _ \| '__|/ __| / _` || | |  _  / / _ \ | '_ \  / _ \ | __| | |  | | / _ \| '_ ` _ \  / _ \ ")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "# | |__| || | | || | \ V /|  __/| |   \__ \| (_| || | | | \ \| (_) || |_) || (_) || |_  | |__| ||  __/| | | | | || (_) |")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "#  \____/ |_| |_||_|  \_/  \___||_|   |___/ \__,_||_| |_|  \_\\\\___/ |_.__/  \___/  \__| |_____/  \___||_| |_| |_| \___/ ")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "#"*curses.COLS)

    #Display the available options
    stdsrc.addstr(next(ROW_ITERATOR), 0, "Press:")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "i) To see 6 unique IK solutions to a pose task;")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "s) To see a singular configuration of the robot;")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "m) To control the end effector position with the keyboard keys;")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "t) Teach mode.")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "h) Home the robot.")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "q) Exit the program.")

    #Display the joint values
    stdsrc.addstr(next(ROW_ITERATOR), 0, "********************")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "*Joint angles [rad]*")
    stdsrc.addstr(next(ROW_ITERATOR), 0, "********************")

    #Update the interface
    stdsrc.refresh()

#Update the terminal using the information from the robot
def updateTerminal(stdsrc):
    print("Update the terminal")


#Moves, in loop, the robot to the 8 configurations realizing the same E-E pose
def inverseKinematicsExample(stdsrc, rtde_c, stopTask):
    while True:
        for i in range(0, C.shape[0]):
            if stopTask():
                return
            rtde_c.moveJ(C[i, :], JOINT_VELOCITY, JOINT_ACCELERATION)

#Move the robot in a singular configuration
def singularConfigurationExample(stdsrc, rtde_c, rtde_r, stopTask):
    VEL = 1
    ACC = 0.8
    IN_SINGULARITY = False
    stdsrc.addstr(0, 0, "Moving in singularities...")
    stdsrc.refresh()
    while True:
        if stopTask():
            if IN_SINGULARITY is True:
                rtde_c.reconnect()
            return
        if IN_SINGULARITY is False:
            IN_SINGULARITY = True
            rtde_c.moveJ(Q_SINGULAR[:, 1], VEL, ACC)
            stdsrc.addstr(0, 0, "Setting teach mode...")
            stdsrc.refresh()
            rtde_c.disconnect()
            stdsrc.addstr(0, 0, "Teach mode: OK")
            stdsrc.refresh()

#Handle interruption or kill to close the communication with the robot
def killHandler(signum, frame):
    #stdsrc.addstr(next(ROW_ITERATOR), 0, "Exiting the program...")
    #stdsrc.refresh()

    #Close the communication
    print("Close the communication")
    sys.exit()

def main(stdsrc):
    #Initialize the interface
    initTerminal(stdsrc)

    #Connect to the robot
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_ADDRESS)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_ADDRESS)

    #Move the robot in the home configuration
    rtde_c.moveJ(HOME, JOINT_VELOCITY, JOINT_ACCELERATION)

    #Set blocking read for the commands
    stdsrc.timeout(-1)
    #Loop indefinetly and wait for a user input
    p = None
    stopTask = False
    while True:
        c = stdsrc.getch()
        #Given a new input, stop the current task
        if p is not None:
            if p.is_alive():
                stopTask = True
                p.join()
                #Prepare the variable for the next iteration
                stopTask = False
        #Process the user input
        if c == ord('i'):
            stdsrc.addstr(0, 0, "Move the robot in the IK")
            p = Thread(target=inverseKinematicsExample, args=(stdsrc, rtde_c, lambda: stopTask, ))
        elif c == ord('s'):
            stdsrc.addstr(0, 0, "See a singular configuration")
            p = Thread(target=singularConfigurationExample, args=(stdsrc, rtde_c, rtde_r, lambda: stopTask, ))
        elif c == ord('m'):
            stdsrc.addstr(0, 0, "Control the EE position")
            p = None
        elif c == ord('t'):
            stdsrc.addstr(0, 0, "Teach mode...")
            rtde_c.moveJ(HOME, JOINT_VELOCITY, JOINT_ACCELERATION)
            p = None
        elif c == ord('h'):
            rtde_c.moveJ(HOME, JOINT_VELOCITY, JOINT_ACCELERATION)
            p = None
        elif c == ord('q'):
            stdsrc.addstr(0, 0, "Move home and exit the program")
            stdsrc.refresh()
            #rtde_c.moveJ(HOME, JOINT_VELOCITY, JOINT_ACCELERATION)
            rtde_c.disconnect()
            p = None
            break
        #Start the new thread
        if p is not None:
            p.start()
        #Update the screen
        stdsrc.refresh()
    #Terminate the program
    sys.exit()
            

#Register killing and interrput signals
signal.signal(signal.SIGINT, killHandler)

mutex = Lock()

#Call the main method
curses.wrapper(main)
