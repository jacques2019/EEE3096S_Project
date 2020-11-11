import busio
import ES2EEPROMUtils
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import threading
import time
import RPi.GPIO as GPIO

eeprom = ES2EEPROMUtils.ES2EEPROM()

temp_0volt = 0.4
temp_coefficient = 0.010
measure_flag = True

chan_temp = None
chan_LDR = None
btn_delay = 22
btn_measure = 27

start_time = 0
start_LDR_time = 0
delayTime = 10

readings = []

def read_thread():
    global chan_temp
    global start_time
    global delayTime
    global chan_LDR
    global readings

    # Setup thread at set time delay
    thread = threading.Timer(delayTime, read_thread)
    thread.daemon = True
    thread.start()

    # Set start time
    currentTime = int(round(time.time()))
    if (start_time == 0):
        start_time = currentTime

    print("Flag: {0}".format(measure_flag))
    print("start_time: {0}".format(start_time))
    print("delay_time: {0}".format(delayTime))

    if (measure_flag):
        # Read values from ADC
        temp_voltage = chan_temp.voltage
        temp_value = chan_temp.value
        
        LDR_value = chan_LDR.value
        LDR_voltage = chan_LDR.voltage

        # Change resisitor voltage to LDR voltage
        LDR_reading = (3.3 - LDR_voltage) / (LDR_voltage/1000)

        # Convert to Temp
        temp = (temp_voltage - temp_0volt)/temp_coefficient

        # Print temp readings
        now_time = time.strftime('%H:%M:%S', time.localtime())
        then_time = time.strftime('%H:%M:%S', time.localtime((currentTime - start_time)))
        print('System Time\tSys Timer\tLDR Resistance\tTemp')
        print('{0}\t{1}\t\t{2:.3f} Ohms\t{3:.3f} C'.format(now_time, then_time, LDR_reading, temp))
        
        # Save readings to EEPROM
        readings.append([temp_value, LDR_value])

        thread = threading.Thread(target=store_readings, args=(readings,))
        thread.daemon = True
        thread.start()

def store_readings(data):
    for i in range(0, 20):
        # Read block from eeprom
        current = [0,0,0,0]

        # Populate current with input data
        if (i < len(data)):
            current[2] = data[i][0]
            current[3] = data[i][1]

        # Ensures the last blok of data is written with zeroes to mark end of readings
        if (i > len(data)):
            break
        
        # Store temperature data
        current[0] = (current[2]&(0xFF00)) >> 8
        current[1] = (current[2]&(0b0000000011111111))
        
        # Store LDR data
        current[2] = (current[3]&(0xFF00)) >> 8
        current[3] = (current[3]&(0b0000000011111111))

def setup():
    global chan_temp
    global chan_LDR

    # create the spi bus
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

    # create the cs (chip select)
    cs = digitalio.DigitalInOut(board.D5)

    # create the mcp object
    mcp = MCP.MCP3008(spi, cs)

    # Create an analog input channel on pin 0 and 1
    chan_temp = AnalogIn(mcp, MCP.P1)
    chan_LDR = AnalogIn(mcp, MCP.P0)

    # Setup buttons for interupt and input
    GPIO.setup(btn_delay, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(btn_delay, GPIO.FALLING, callback=btn_delay_callback, bouncetime=250)
    
    GPIO.setup(btn_measure, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(btn_measure, GPIO.FALLING, callback=btn_measure_callback, bouncetime=250)
    
def btn_delay_callback(channel):
    global delayTime

    # Cycle delay 
    if (delayTime == 10):
        delayTime = 5
    elif (delayTime == 5):
        delayTime = 1
    elif (delayTime == 1):
        delayTime = 10

def btn_measure_callback(channel):
    global measure_flag

    measure_flag = not(measure_flag)

if __name__ == "__main__":
    setup()
    read_thread()

    # Run indefinitely
    while True:
        pass