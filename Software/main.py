import time
import sys,os    # system related library
import numpy as np
import matplotlib.pyplot as plt
import cv2
import pyvisa as visa

ok_sdk_loc = "C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\Python\\x64"
ok_dll_loc = "C:\\Program Files\\Opal Kelly\\FrontPanelUSB\\API\\lib\\x64"

sys.path.append(ok_sdk_loc)   # add the path of the OK library
os.add_dll_directory(ok_dll_loc)

import ok
dev = ok.okCFrontPanel()
SerialStatus=dev.OpenBySerial("")
ConfigStatus=dev.ConfigureFPGA("U:\\ece437\\Final\\final.runs\\impl_1\\top.bit")

"""
MMIO REPRESENTATION {FPGA-Centric}


~~~~ Input (okWireIn) ~~~~


// CMV300 IMAGE SENSOR
0x00 - Signal line 0 : {PC_Control}     - {1'b}
0x01 - Address line  : {reg_addr, !R/W} - {7'b, 1'b}
0x02 - Data in line  : {data_to_write}  - {8'b}
0x03 - Signal line 1 : {PC_Image_Req}   - {1'b}


// Accelerometer + Magnetometer
0x04 - Signal line 2:  {PC_control_IMU} - {1'b}
0x05 - Signal line 3:  {data to write, # of bytes to read, R/W} - {8'b, 4'b, 1'b}
0x06 - Address line 1: {slave address, register address}


// PMOD
0x07 - Signal line 4:  {motor start, directionality, # of pulses} - {1'b, 1'b, 30'b}


~~~~ Output (okWireOut) ~~~~


// CMV300
0x20 - Data out line:    {*reg_addr}  - {8'b}


// Accelerometer + Magnetometer
0x21 - Data line 0 :    {reg_addr+0, reg_addr+1, reg_addr+2, reg_addr+3}
0x22 - Data line 1:    {reg_addr+4, reg_addr+5, reg_addr+6, reg_addr+7}


0xa0 - BTPipe

"""
SIGNAL_LINE_0   = 0x00
SIGNAL_LINE_1   = 0x03
SIGNAL_LINE_2   = 0x04
SIGNAL_LINE_3   = 0x05
SIGNAL_LINE_4   = 0x07


ADDRESS_LINE    = 0x01
ADDRESS_LINE_1  = 0x06


DATA_IN_LINE    = 0x02


DATA_OUT_LINE = 0x20


DATA_LINE_0   = 0x21
DATA_LINE_1   = 0x22


## Addresses & constants (acceleration/magnetic)
linear_acceleration_SAD = 0b0011001
magnetic_field_SAD      = 0b0011110


CTRL_REG1_A_addr        = 0x20
OUT_X_L_A               = 0x28
MR_REG_M_addr           = 0x02
OUT_X_H_M               = 0x03


CTRL_REG1_A_value       = 0b10010111    # Normal (1.344 kHz) / low-power mode (5.376 kHz)
MR_REG_M_val            = 0x00          # Continuous-conversion mode


IMAGE_CENTER_X, IMAGE_CENTER_Y = 324.0, 243.0
IMAGE_DIM_X, IMAGE_DIM_Y       = 648, 486
TOLERANCE_X, TOLERANCE_Y       = 80, 80


DELAY_0,DELAY_1,DELAY_2 = 0.1,0.5,0.1


## Power supply
MAX_VOLTAGE = 5
VOLTAGE_INCREMENT = 0.5


############################## Instruments and bitstream ##############################
def initBitstream():
    ## Initialization
    print("----------------------------------------------------")
    if SerialStatus == 0:
        print ("FrontPanel host interface was successfully initialized.")
    else:    
        print ("FrontPanel host interface not detected. The error code number is:" + str(int(SerialStatus)))
        print("Exiting the program.")
        sys.exit ()
    print("----------------------------------------------------")
    print("----------------------------------------------------")


    if ConfigStatus == 0:
        print ("Your bit file is successfully loaded in the FPGA.");
    else:
        print ("Your bit file did not load. The error code number is:" + str(int(ConfigStatus)))
        print ("Exiting the progam.")
        sys.exit ()
    print("----------------------------------------------------")


def configureInstruments():
    global power_supply
    device_manager = visa.ResourceManager()
    devices = device_manager.list_resources()
    number_of_device = len(devices)
    # print(devices)


    power_supply_id = -1;
    waveform_generator_id = -1;
    digital_multimeter_id = -1;
    oscilloscope_id = -1;


    # assumes only the DC power supply is connected
    for i in range (0, number_of_device):


    # check that it is actually the power supply
        try:
            device_temp = device_manager.open_resource(devices[i])
            print("Instrument connect on USB port number [" + str(i) + "] is " + device_temp.query("*IDN?"))
            if (device_temp.query("*IDN?") == 'HEWLETT-PACKARD,E3631A,0,3.2-6.0-2.0\r\n'):
                power_supply_id = i        
            if (device_temp.query("*IDN?") == 'HEWLETT-PACKARD,E3631A,0,3.0-6.0-2.0\r\n'):
                power_supply_id = i
            if (device_temp.query("*IDN?") == 'Agilent Technologies,33511B,MY52301259,3.03-1.19-2.00-52-00\n'):
                waveform_generator_id = i
            if (device_temp.query("*IDN?") == 'Agilent Technologies,34461A,MY53208021,A.01.10-02.25-01.10-00.35-01-01\n'):
                digital_multimeter_id = i
            if (device_temp.query("*IDN?") == 'Keysight Technologies,34461A,MY53212931,A.02.08-02.37-02.08-00.49-01-01\n'):
                digital_multimeter_id = i            
            if (device_temp.query("*IDN?") == 'KEYSIGHT TECHNOLOGIES,MSO-X 3024T,MY54440307,07.50.2021102830\n'):
                oscilloscope_id = i                        
            device_temp.close()
        except:
            print("Instrument on USB port number [" + str(i) + "] cannot be connected. The instrument might be powered of or you are trying to connect to a mouse or keyboard.\n")


    # Power supply
    if (power_supply_id == -1):
        print("Power supply instrument is not powered on or connected to the PC.")    
    else:
        print("Power supply is connected to the PC.\n")
        power_supply = device_manager.open_resource(devices[power_supply_id])


############################## ACCEL AND MAG FUNCTIONS ##############################
# takes in an arbitrary 16-bit value and converts into two's
# complement representation
def twos_comp(val):
    if (val & (1 << 15)) != 0:  #if negative (sign bit is 1)
        val = val - (1 << 16)


    return val


def sendStopSignal():
    dev.SetWireInValue(SIGNAL_LINE_2, 0)
    dev.UpdateWireIns()
    # time.sleep(DELAY_0)


# reads k byte(s) starting at `reg_addr` (idx=0) up to and including `reg_addr + k-1`.
# notes: 1 <= k <= 8
def getBytes(slave_addr, reg_addr, k):
    sendStopSignal()


    # Prepare and  parameters for read transaction
    dev.SetWireInValue(SIGNAL_LINE_2, 1) # 'go'
    dev.SetWireInValue(SIGNAL_LINE_3, (k << 1) + 1) # R/W = 1 for READ
    dev.SetWireInValue(ADDRESS_LINE_1, (slave_addr << 8) + reg_addr)
    dev.UpdateWireIns()
   
    # time.sleep(DELAY_1)
    sendStopSignal()


    # Read incoming data
    read_bytes = []
    dev.UpdateWireOuts()
    data_0, data_1 = int(dev.GetWireOutValue(DATA_LINE_0)), int(dev.GetWireOutValue(DATA_LINE_1))


    for i in range(k):
        if (i <= 3):
            read_bytes.append((data_0 >> (8*(3-i%4))) & 0xFF)   # extract ordered bytes
        else:
            read_bytes.append((data_1 >> (8*(3-i%4))) & 0xFF)
           
    return read_bytes


# writes a byte into `reg_addr`
def writeByte(slave_addr, reg_addr, byte_to_write):
    sendStopSignal()


    # Prepare parameters for write transaction
    dev.SetWireInValue(SIGNAL_LINE_2, 1) # 'go'
    dev.SetWireInValue(SIGNAL_LINE_3, (byte_to_write << 5) + 0) # R/W = 0 for WRITE
    dev.SetWireInValue(ADDRESS_LINE_1, (slave_addr << 8) + reg_addr)
    dev.UpdateWireIns()
   
    time.sleep(DELAY_1)


    sendStopSignal()


def getAcceleration():
    a_xL, a_xH, a_yL, a_yH, a_zL, a_zH = getBytes(linear_acceleration_SAD, OUT_X_L_A, 6)
    a_x, a_y, a_z = a_xL + (a_xH << 8), a_yL + (a_yH << 8), a_zL + (a_zH << 8)


    # FS[1:0] bit set to 00
    # Sensitivity: 1 [mg/LSB]
    # Range:       +/- 2g
    return [twos_comp(a) / 16000 for a in [a_x, a_y, a_z]]


def getMagnetic():
    b_xH, b_xL, b_zH, b_zL, b_yH, b_yL = getBytes(magnetic_field_SAD, OUT_X_H_M, 6)
    b_x, b_y, b_z = b_xL + (b_xH << 8), b_yL + (b_yH << 8), b_zL + (b_zH << 8)


    # GN[2:0] bit set to 001
    # Sensitivity: (X,Y) 1100 [LSB/gauss], (Z) 980 [LSB/gauss]
    # Range:       +/- 1.3 gauss
    return [twos_comp(b_x) / 1100, twos_comp(b_y) / 1100, twos_comp(b_z) / 980]
   
def configIMU():
    writeByte(linear_acceleration_SAD, CTRL_REG1_A_addr, CTRL_REG1_A_value)
    writeByte(magnetic_field_SAD, MR_REG_M_addr, MR_REG_M_val)


############################## PMOD/POWER SUPPLY FUNCTIONS ##############################    
def startMotor(pulses, direction):
    # print("startMotor(pulses=", pulses, ", direction=", direction, ")")
    direction, pulses = int(direction), int(pulses)
    dev.SetWireInValue(SIGNAL_LINE_4, (1 << 31) + (direction << 30) + pulses)
    dev.UpdateWireIns()
    time.sleep(0.15)


    dev.SetWireInValue(SIGNAL_LINE_4, 0)     # START=0
    dev.UpdateWireIns()


def printAccelMag():
    # while(True):
    a = getAcceleration()
    print("Acceleration X: ", a[0], " [g]")
    print("Acceleration Y: ", a[1], " [g]")
    print("Acceleration Z: ", a[2], " [g]")
    print ("\n")
   
    b = getMagnetic()
    print("Magnetic X: ", b[0], " [gauss]")
    print("Magnetic Y: ", b[1], " [gauss]")
    print("Magnetic Z: ", b[2], " [gauss]")
    print ("\n")



############################## CMV300 FUNCTIONS ##############################    
def sendStop():
    dev.SetWireInValue(SIGNAL_LINE_0, 0)
    dev.UpdateWireIns()
    time.sleep(DELAY_0)


# Reads in the data of the register at `reg_addr
# returns `*reg_addr` obtained from sensor
def readReg(reg_addr):
    sendStop()


    # Prepare parameters for read transaction
    dev.SetWireInValue(SIGNAL_LINE_0, 1) # 'go'
    dev.SetWireInValue(ADDRESS_LINE, (reg_addr << 1) + 0) # R/W = 0 for READ
    dev.UpdateWireIns()


    time.sleep(DELAY_0)
    sendStop()


    # Return incoming data
    dev.UpdateWireOuts()


    return int(dev.GetWireOutValue(DATA_OUT_LINE))


# Writes a byte into `reg_addr`
def writeReg(reg_addr, byte_to_write):
    sendStop()


    # Prepare parameters for write transaction
    dev.SetWireInValue(SIGNAL_LINE_0, 1) # 'go'
    dev.SetWireInValue(ADDRESS_LINE, (reg_addr << 1) + 1) # R/W = 1 for WRITE
    dev.SetWireInValue(DATA_IN_LINE, byte_to_write)
    dev.UpdateWireIns()
   
    time.sleep(DELAY_0)


    sendStop()


def configureSPI():
    for i in range(0, 128):
        # # Exp Time = {0b0000_0100 _1100_1001} = 1225 <== 10 us
        # # Exp Time = {0b_0111_0101}           = 117  <== 1 us
        if     i == 42:      # Exp_time[7:0]
            writeReg(i, 0b11001001)
        elif   i == 43:   # Exp_time[15:8]
            writeReg(i, 0b00000100);
        # elif   i == 44:   # Exp_time[23:16]
        #     writeReg(i, 0b00010010);
        elif i == 57:
            writeReg(i, 3)
        elif i == 58:
            writeReg(i, 44)
        elif i == 59:
            writeReg(i, 240)
        elif i == 60:
            writeReg(i, 10)
        elif i == 69:
            writeReg(i, 9)
        elif i == 80:
            writeReg(i, 2)
        elif i == 83:   # PLL_range
            writeReg(i, 187)
        elif i == 97:
            writeReg(i, 240)
        elif i == 98:
            writeReg(i, 10)
        elif i == 100:
            writeReg(i, 112)
        elif i == 101:
            writeReg(i, 98)
        elif i == 102:
            writeReg(i, 34)
        elif i == 103:
            writeReg(i, 64)
        elif i == 106:
            writeReg(i, 94)
        elif i == 107:
            writeReg(i, 110)
        elif i == 108:
            writeReg(i, 91)
        elif i == 109:
            writeReg(i, 82)
        elif i == 110:
            writeReg(i, 80)
        elif i == 117:
            writeReg(i, 91)


# Set 'FSM go/stop signal' for frame capture,
# based on the value in `new_req`
def setImageRequest(new_req):
    dev.SetWireInValue(SIGNAL_LINE_1, new_req)
    dev.UpdateWireIns()


IMAGE_CAPTURE = bytearray(308*1024)


# This function will set `PC_Image_Req` to 1 and then a 0,
# The falling edge of `PC_Image_Req` will initiate the capture image FSM sequence (within HWE):
#
# 1) Reset the FIFO: reset all counters and enable reset bits
# 2) Stall for some time for reset to process (inside hardware)
# 3) Send the `FRAME_REQ` pulse (3.10.1)
# 4) Wait out the exposure time, FOT          (inside hardware)
# 5) Start writing to the FIFO from imager data during 'read-out time'
# 6) Once FIFO has written 308*1024 pixels, return to an IDLE state
def getFrame():
    # Set `img_request` to 1 which causes a pulse in `FRAME_REQ`
    setImageRequest(1)
    setImageRequest(0) # Ensure only one `FRAME_REQ` will be sent


    dev.ReadFromBlockPipeOut(0xa0, 1024, IMAGE_CAPTURE)


def configCMV():
    ############## START-UP SEQUENCE ##############
    # Wait for SYS_RES_N to go HIGH
    time.sleep(DELAY_0)


    # SPI settings: Write required values to registers
    configureSPI()


    # Wait before requesting a frame
    time.sleep(DELAY_0)


############################# MAIN() #############################
configureInstruments()
configIMU()
configCMV()


plt.ion()
fig, ax = plt.subplots()

while True:
    printAccelMag()


    timeBefore = time.time()
    getFrame()
    timeAfter = time.time()


    # note: 648*486, 486 because we chose to exclude the last two lines in `IMAGE_CAPTURE`
    image_data = np.frombuffer(IMAGE_CAPTURE, dtype=np.uint8, count=648*486).reshape(486, 648)


    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(image_data,127,255,0)
   
    # calculate moments of binary image
    M = cv2.moments(thresh)
   
    # calculate x,y coordinate of center
    cX, cY = -1,-1
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])


    # move motor based on object's C.O.M
    if (0 <= cX <= IMAGE_CENTER_X - TOLERANCE_X):
        print("Moving motor to the LEFT!")
        startMotor(10, 1)  # Move left
    elif (IMAGE_CENTER_X + TOLERANCE_X <= cX <= IMAGE_DIM_X):
        print("Moving motor to the RIGHT!")
        startMotor(10, 0)  # Move right
   
    ax.clear()

    # Plot image data
    ax.imshow(image_data, cmap='gray')
    ax.axis('off')
    plt.draw()
   
    plt.pause(0.001)  # 0.1 second pause


dev.Close
