import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial
import serial.tools.list_ports

#constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2
CENTER_TOLERANCE = 50
SERIAL_PORT_NAME = '/dev/ttyACM0'
BAUD_RATE = 9600

#color values

#dusty purple color values for cylinders
PURPLE_LOWER = np.array([125, 25, 30])
PURPLE_UPPER = np.array([145, 80, 200])

#bright yellow color for flag
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([35, 255, 255])

#pixel thresholds
MIN_AREA = 500
GRAB_AREA = 47500
DROP_AREA = 30000    

#new side of screen tracking values that I REALLY hope works
HOME_X_SWEET_SPOT = 580
HOME_X_TOLERANCE = 40


def wait_for_arduino():
    
    print("Waiting for Arduino to connect...")
    ser = None
    while ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT_NAME, BAUD_RATE, timeout=1)
            print(f"Connected to Arduino on {SERIAL_PORT_NAME}")
        except serial.SerialException:
            print(f"Failed to connect to {SERIAL_PORT_NAME}. Retrying...")
            time.sleep(1)
            
    print("Arduino connected waiting for 'READY' signal...")
    
    while True: #wait forever for "READY" from Arduino
        try:
            line = ser.readline().decode('utf-8').strip()
            if line == "READY":
                print("READY signal received")
                return ser #recieved
            elif line:
                print(f"Arduino says: {line}")
                
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(0.5)

def main():
    ser = None
    picam2 = None
    
    pi_state = "LISTENING"
    
    try:
        #wait for "handshake"
        ser = wait_for_arduino()
        
        #send 'I' wait for 'SEEK'
        print("PI: Sending 'I' command to Arduino.");
        ser.write(b'I')
        
        while pi_state == "LISTENING":
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Arduino says: {line}")
                        if line == "SEEK":
                            pi_state = "SEEKING"
                            break #exit
            except Exception as e:
                print(f"Error reading from serial: {e}")
            time.sleep(0.05)
        
        print("Deployment finished starting Pi systems.")
        
        #camera intialization
        print("Initializing Camera...")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)})
        picam2.configure(config)
        picam2.start()
        print("Camera started.")
        time.sleep(1.0) #give the camera a second

        #main logic loop
        print("Starting main CV loop...")
        last_signal = 'S' 

        while True:
            #listen for state changes from arduino
            try:
                if ser.in_waiting > 0:
                    arduino_msg = ser.readline().decode('utf-8').strip()
                    
                    if arduino_msg == "SEEK":
                        print("PI: Received SEEK looking for purple.")
                        pi_state = "SEEKING"
                    elif arduino_msg == "HOME":
                        print("PI: Received HOME looking for yellow.")
                        pi_state = "RETURNING_HOME"
                    elif arduino_msg:
                        print(f"Arduino DEBUG: {arduino_msg}")
                
            except Exception as e:
                print(f"Error reading serial message: {e}")

            #computer vision logic
            
            #spin right if it sees nothing
            signal_to_send = 'R' 
            
            if pi_state == "SEEKING":
                hsv = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2HSV)
                mask = cv2.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER)
                target_name = "Cylinder"
            elif pi_state == "RETURNING_HOME":
                hsv = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2HSV)
                mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
                target_name = "Home"
            else:
                signal_to_send = 'S' #just stop please OH MY WORD
                time.sleep(0.01)
                continue

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > MIN_AREA:
                    x, y, w, h = cv2.boundingRect(c)
                    target_x = x + (w // 2)
                    target_y = y + (h // 2) 
                    
                    #decision making
                    
                    if pi_state == "SEEKING":
                        #logic for grabbing
                        if target_x < (FRAME_CENTER_X - CENTER_TOLERANCE):
                            signal_to_send = 'L'
                        elif target_x > (FRAME_CENTER_X + CENTER_TOLERANCE):
                            signal_to_send = 'R'
                        else:
                            signal_to_send = 'F'
                            if area > GRAB_AREA:
                                print(f"PI: {target_name} is centered and close enough. Sending GRAB.")
                                signal_to_send = 'G'
                        
                    elif pi_state == "RETURNING_HOME":
                        #homing logic
                        home_x_min = HOME_X_SWEET_SPOT - HOME_X_TOLERANCE
                        home_x_max = HOME_X_SWEET_SPOT + HOME_X_TOLERANCE
                        
                        if target_x < home_x_min:
                            signal_to_send = 'R'
                        elif target_x > home_x_max:
                            signal_to_send = 'L'
                        else:
                            signal_to_send = 'F' #drive forward
                            if area > DROP_AREA:
                                print(f"PI: {target_name} aligned and close enough. Sending DROP.")
                                signal_to_send = 'D'

            #send serial command
            if signal_to_send != last_signal:
                ser.write(signal_to_send.encode())
                last_signal = signal_to_send
                print(f"Pi: Sending command '{signal_to_send}'")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")
    except Exception as e:
        print(f"AN UNEXPECTED ERROR OCCURRED: {e}")
    finally:
        #cleanup procedure
        print("\nShutting down.")
        if picam2:
            picam2.stop()
            print("Camera stopped.")
        if ser and ser.is_open:
            try:
                ser.write(b'S') 
            except:
                print("Could not send final STOP.")
            ser.close()
            print("Serial port closed.")
        print("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()