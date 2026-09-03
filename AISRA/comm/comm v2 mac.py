import serial
import serial.tools.list_ports
import time




class RobotComms:
   def __init__(self):
       self.ard1 = None
       self.ard2 = None
       self.connect_arduinos()


   def connect_arduinos(self):
       ports = serial.tools.list_ports.comports()
       for port in ports:
           # 1. MAC FIX: Skip Bluetooth and weird internal ports that freeze Python
           if "Bluetooth" in port.device or "wlan" in port.device or "debug" in port.device:
               continue


           try:
               # Open port at 115200 baud, non-blocking timeout
               s = serial.Serial(port.device, 115200, timeout=0.1)


               # Give it a short moment to physically reboot the board
               time.sleep(1.5)


               # 2. ACTIVE LISTENING FIX: Check for messages continuously for 3 seconds
               found = False
               start_time = time.time()


               while time.time() - start_time < 3.0:
                   if s.in_waiting > 0:
                       # Read one line at a time
                       line = s.readline().decode('utf-8', errors='ignore').strip()


                       if "ARDUINO_1_READY" in line:
                           self.ard1 = s
                           print(f"Connected Arduino 1 (Links 1-3) on {port.device}")
                           found = True
                           break
                       elif "ARDUINO_2_READY" in line:
                           self.ard2 = s
                           print(f"Connected Arduino 2 (Base) on {port.device}")
                           found = True
                           break


               # If 3 seconds pass and we didn't find our strings, close it
               if not found:
                   s.close()


           except Exception as e:
               pass  # Ignore ports that are busy/denied


       if not self.ard1 or not self.ard2:
           print("\nWARNING: Could not find both Arduinos!")
           print(
               "Please check your USB cables, ensure Arduino IDE Serial Monitors are closed, and verify the boards are flashed correctly.")


   def set_motor_state(self, state):
       """Sends 'ON' or 'OFF' to both Arduinos"""
       cmd = f"{state}\n".encode()
       if self.ard1:
           self.ard1.write(cmd)
       if self.ard2:
           self.ard2.write(cmd)
       print(f">>> Motor state commanded to: {state}")


   def send_motion(self, steps_0, steps_1, steps_2, steps_3, time_ms, speed_steps_0):
       # Send to Arduino 2 (Base/NEMA 34)
       if self.ard2 and steps_0 != 0:
           # Convert steps back to angle for the NEMA code
           deg_0 = abs(steps_0 * (360.0 / (6400)))
           direction = "CW" if steps_0 > 0 else "CCW"
           cmd2 = f"{deg_0} {speed_steps_0} {direction}\n"
           self.ard2.write(cmd2.encode())


       # Send to Arduino 1 (Links 1, 2, 3)
       if self.ard1 and (steps_1 != 0 or steps_2 != 0 or steps_3 != 0):
           cmd1 = f"{steps_1} {steps_2} {steps_3} {time_ms}\n"
           self.ard1.write(cmd1.encode())


   def wait_for_done(self):
       print("Waiting for motion to complete...")


       # Wait for Arduino 1 to finish
       if self.ard1:
           while True:
               if self.ard1.in_waiting:
                   msg = self.ard1.readline().decode('utf-8', errors='ignore').strip()
                   if msg == "DONE":
                       break


       # Wait for Arduino 2 to finish
       if self.ard2:
           while True:
               if self.ard2.in_waiting:
                   msg = self.ard2.readline().decode('utf-8', errors='ignore').strip()
                   if msg == "DONE":
                       break


       print("Motion complete.")

