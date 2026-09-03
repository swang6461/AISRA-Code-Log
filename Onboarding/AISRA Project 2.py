import serial
from serial.tools import list_ports
import time

firstConnection = True
tempSent = False
scoopSent = False
pingReceived = False
portWithArduino = ""
heatStatus = ""

while True:
# Discovery
    portFound = False
    availablePorts = serial.tools.list_ports.comports()
    # for port in availablePorts:
    #      print(port.device, "-", port.description)
    for port in availablePorts:
        if port.description.startswith("Arduino Uno"):
            portWithArduino = port.device
            portFound = True
            try:
                if firstConnection:
                    print("Port found")
                    serialcomm = serial.Serial(portWithArduino, 9600)
                    serialcomm.timeout = 1
                    time.sleep(2)
                    firstConnection = False
                if not pingReceived:
                    print("PINGING")
                    serialcomm.write("PING\n".encode())

# Tool Configuration
                toolIdentificationMessage = serialcomm.readline().decode("ascii").strip()
                print(toolIdentificationMessage)
                if "TOOL_ID" in toolIdentificationMessage:
                    pingReceived = True
                    parts = toolIdentificationMessage.split("|")
                    partID = int(parts[1].split(":")[1])
                    if partID == 1:
                        if not tempSent:
                            print("Sending Temp Data")
                            profile = "Scooping Mechanism"
                            time.sleep(2)
                            tempSent = True
                        serialcomm.write("SET_TEMP: -10C\n".encode())
                        heatingMessage = serialcomm.readline().decode("ascii")
                        print(heatingMessage)

# Readiness Check
                if heatStatus != "STATUS: READY":
                    print("Getting Heat Status")
                    serialcomm.write("GET_STATUS\n".encode())
                    time.sleep(2)
                    heatStatus = serialcomm.readline().decode("ascii").strip()
                    print(heatStatus)

# Execution and Completion
                if heatStatus == "STATUS: READY":
                    if not scoopSent:
                        print("Sending Scoop")
                        serialcomm.write("START_SCOOP\n".encode())
                        scoopStatus = serialcomm.readline().decode("ascii").strip()
                        print(scoopStatus)

                    if scoopStatus == "ACK|STATUS:SCOOPING":
                        scoopSent = True
                        print("getting status")
                        serialcomm.write("GET_STATUS\n".encode())
                        print(serialcomm.readline().decode("ascii"))

                time.sleep(2)
            except serial.SerialException:
                print("Arduino not connected yet")
                serialcomm.close()
                firstConnection = True
                time.sleep(2)
                continue
# Disconnect / Reset
    if not portFound:
        print("Checking for port with Arduino")
        firstConnection = True
        pingReceived = False
        tempSent = False
        scoopSent = False
        time.sleep(2)
