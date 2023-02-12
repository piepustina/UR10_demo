import curses


from itertools import count
import signal, sys


#Robot parameters
ROBOT_ADDRESS = "192.168.0.10"



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

    #Run a separate thread to execute the user commands and monitor the robot status


    #Loop indefinetly and wait for a user input
    while True:
        c = stdsrc.getch()
        if c == ord('i'):
            stdsrc.addstr(0, 0, "Move the robot in the IK")
        elif c == ord('s'):
            stdsrc.addstr(0, 0, "See a singular configuration")
        elif c == ord('m'):
            stdsrc.addstr(0, 0, "Control the EE position")
        elif c == ord('q'):
            stdsrc.addstr(0, 0, "Exit the program")
            break
        #Update the screen
        stdsrc.refresh()
    
    #Terminate the program
    sys.exit()
            

#Register killing and interrput signals
signal.signal(signal.SIGINT, killHandler)


#Call the main method
curses.wrapper(main)
