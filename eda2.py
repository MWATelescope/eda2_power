#!/usr/bin/python

"""
Code to run on the Raspberry Pi connected to the prototype EDA2 power control unit.

Use as:

pi@raspberrypi:~ $ cd eda2_power/
pi@raspberrypi:~/eda2_power $ sudo python
Python 2.7.13 (default, Sep 26 2018, 18:42:22)
[GCC 6.3.0 20170516] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import eda2
>>> eda2.init()    # Initialise pins
>>> adcs = eda2.ADC_Set()   # Create an object to handle the 74X138 and the MCP3208 comms
>>> c1 = eda2.I2C_Control(instance=1)  # Create an object to handle I2C comms
>>> c1.read_environment()         # Reads the humidity and temperature
(50.872909290684895, 22.235075082407526)
>>> c1.turn_all_on()    # Turn on all of the 6416 outputs
True
>>> adcs.readADC(chipnum=4, channel=4)  # Read VB7
INFO: time 1546998908.272304 - Selected output 4 on 74X138
Sending:  [7, 0, 0]
INFO: time 1546998908.286555 - Disabled all outputs on 74X138
455
>>> adcs.readADC(chipnum=4, channel=5)  # Read IB7
INFO: time 1546998911.022609 - Selected output 4 on 74X138
Sending:  [7, 64, 0]
INFO: time 1546998911.036595 - Disabled all outputs on 74X138
1379
>>> adcs.readADC(chipnum=4, channel=6)  # Read VB8
INFO: time 1546998912.918866 - Selected output 4 on 74X138
Sending:  [7, 128, 0]
INFO: time 1546998912.932822 - Disabled all outputs on 74X138
456
>>> adcs.readADC(chipnum=4, channel=7)  # Read IB8
INFO: time 1546998914.782761 - Selected output 4 on 74X138
Sending:  [7, 192, 0]
INFO: time 1546998914.796718 - Disabled all outputs on 74X138
1388
>>> c1.turn_all_off()    # Turn off all of the 6416 outputs
True

"""

import atexit
import logging
from logging import handlers
import signal
import sys
import threading
import time
import traceback

import RPi.GPIO as GPIO
import smbus

import spidev

# set up the logging before importing Pyro4

LOGLEVEL_CONSOLE = logging.WARNING  # INFO and above will be printed to STDOUT as well as the logfile
LOGLEVEL_LOGFILE = logging.DEBUG  # All messages will be sent to the log file
LOGFILE = "/tmp/bfif.log"

ADCS = None    # When running, contains the ADCSet instance that handles all the ADC chips.
PC1 = None      # When running, contains an I2C_Control instance to control the first output control chip
PC2 = None      # When running, contains an I2C_Control instance to control the second output control chip
SMBUS = None    # When running, contains an smbus.SMBus instance to talk to the I2C bus

# When running, OUTPUTS contains a dictionary with 32 Antenna() instances, with the key being the
# output name - A1-A8, B1-B8, ... D1-D8.
OUTPUTS = {}

I2C_LOCK = threading.RLock()   # Used to prevent simultaneous use of the I2C bus by multiple threads.

# Key is antenna name, value is a tuple containing (chipselect, voltage_channel, current_channel)
CHIPMAP = {'A1':(7, 0, 1), 'A2':(7, 2, 3),
           'A3':(6, 0, 1), 'A4':(6, 2, 3),
           'A5':(5, 0, 1), 'A6':(5, 2, 3),
           'A7':(4, 0, 1), 'A8':(4, 2, 3),

           'B1':(7, 4, 5), 'B2':(7, 6, 7),
           'B3':(6, 4, 5), 'B4':(6, 6, 7),
           'B5':(5, 4, 5), 'B6':(5, 6, 7),
           'B7':(4, 4, 5), 'B8':(4, 6, 7),

           'C1':(0, 0, 1), 'C2':(0, 2, 3),
           'C3':(1, 0, 1), 'C4':(1, 2, 3),
           'C5':(2, 0, 1), 'C6':(2, 2, 3),
           'C7':(3, 0, 1), 'C8':(3, 2, 3),

           'D1':(0, 4, 5), 'D2':(0, 6, 7),
           'D3':(1, 4, 5), 'D4':(1, 6, 7),
           'D5':(2, 4, 5), 'D6':(2, 6, 7),
           'D7':(3, 4, 5), 'D8':(3, 6, 7),
           }


class MWALogFormatter(object):
    def format(self, record):
        return "%s: time %10.6f - %s" % (record.levelname, time.time(), record.getMessage())


mwalf = MWALogFormatter()

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

fh = handlers.RotatingFileHandler(LOGFILE, maxBytes=1e9, backupCount=5)  # 1 Gb per file, max of five old log files
fh.setLevel(LOGLEVEL_LOGFILE)
fh.setFormatter(mwalf)

ch = logging.StreamHandler()
ch.setLevel(LOGLEVEL_CONSOLE)
ch.setFormatter(mwalf)

# add the handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)

SIGNAL_HANDLERS = {}
CLEANUP_FUNCTION = None

# IO pin allocations on the Raspberry Pi GPIO header
CS_DECODE_A = 31         # Bit 0 of the address on the 74X138 3-to-8 decoder
CS_DECODE_B = 32         # Bit 1 of the address on the 74X138 3-to-8 decoder
CS_DECODE_C = 33         # Bit 2 of the address on the 74X138 3-to-8 decoder
CS_DECODE_ENABLE = 37    # Enable (active high) on the 74X138 3-to-8 decoder

# I2C device addresses for the PCA6416A IO control chip on the digital board. Note that these are _seven_ bit addresses,
# so 0x20 in seven bits corresponds to 0x40/0x41 when the r/w bit is appended as bit 0 of the address, and
# 0x21 corresponds to 0x42/0x43 with a r/w bit of 0.
ADDRESS7_PCA6416A_1 = 0x20
ADDRESS7_PCA6416A_2 = 0x21

ADDRESS_HIH7120 = 0x27   # Honeywell humidity/temperature sensor I2C address


def init():
    """
      Initialise IO pins, and create the global control object instances.
    """
    global ADCS, PC1, PC2, SMBUS
    GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
    GPIO.setwarnings(False)
    GPIO.setup(CS_DECODE_A, GPIO.OUT)
    GPIO.setup(CS_DECODE_B, GPIO.OUT)
    GPIO.setup(CS_DECODE_C, GPIO.OUT)
    GPIO.setup(CS_DECODE_ENABLE, GPIO.OUT)
    SMBUS = smbus.SMBus(1)
    ADCS = ADC_Set()
    PC1 = I2C_Control(instance=1)
    PC2 = I2C_Control(instance=2)
    for letter in ['A', 'B', 'C', 'D']:
        for number in range(1, 9):
            name = '%s%d' % (letter, number)
            OUTPUTS[name] = Antenna(name=name)


##################################################################################
#
# Functions to handle clean shutdown on exit
#
##################################################################################


def cleanup():
    """
      Called automatically on exit, either a normal program exit (eg, sys.exit()) or
      after trapping a signal. Note that 'kill -9' can NOT be trapped, in
      which case this cleanup function will NOT run.

      NOTE - GPIO.cleanup() is NOT called by this function, as that returns all GPIO pins
             to tristate mode, and in the prototype, this turns the fibre media converter
             (the Aux power supply) OFF, so we can't communicate with the Raspberry Pi.
    """
    # Turn off the charge FETs and all other BL233 output pins using the direct serial interface, in case of a deadlock on
    # the SPIU lock or some other problem with the SPIUHandler() code.

    try:
        if PC1 is not None:
            PC1.turn_all_off()
    except:
        logger.exception('cleanup() - FAILED to turn off PC1 outputs on cleanup. : %s', traceback.format_exc())

    try:
        if PC2 is not None:
            PC2.turn_all_off()
    except:
        logger.exception('cleanup() - FAILED to turn off PC2 outputs on cleanup. : %s', traceback.format_exc())

    try:
        GPIO.cleanup()
        if SMBUS is not None:
            SMBUS.close()
    except:
        logger.exception('cleanup() - FAILED to cleanup GPIO pins and close I2C bus. : %s', traceback.format_exc())


def SignalHandler(signum=None, frame=None):
    """
      Called when a signal is received thay would result in the programme exit, if the
      RegisterCleanup() function has been previously called to set the signal handlers and
      define an exit function using the 'atexit' module.

      Note that exit functions registered by atexit are NOT called when the programme exits due
      to a received signal, so we must trap signals where possible. The cleanup function will NOT
      be called when signal 9 (SIGKILL) is received, as this signal cannot be trapped.

      :param signum: Integer signal number, eg 15
      :param frame: current stack frame, not used here.
    """
    logger.critical("SignalHandler() - Signal %d received." % signum)
    sys.exit(-signum)  # Called by signal handler, so exit with a return code indicating the signal received, AFTER
    # calling the cleanup function registered by the atexit.register() call in RegisterCleanup()


def RegisterCleanup(func):
    """
      Traps a number of signals that would result in the program exit, to make sure that the
      function 'func' is called before exit. The calling process must define its own cleanup
      function - typically this would delete any facility controller objects, so that any
      processes they have started will be stopped.

      We don't need to trap signal 2 (SIGINT), because this is internally handled by the python
      interpreter, generating a KeyboardInterrupt exception - if this causes the process to exit,
      the function registered by atexit.register() will be called automatically.

      :param func: Function object to be called before exiting the program.
    """
    global SIGNAL_HANDLERS, CLEANUP_FUNCTION
    CLEANUP_FUNCTION = func
    for sig in [3, 15]:
        SIGNAL_HANDLERS[sig] = signal.signal(sig, SignalHandler)  # Register a signal handler
    SIGNAL_HANDLERS[1] = signal.signal(1, signal.SIG_IGN)
    # Register the passed CLEANUP_FUNCTION to be called on
    # on normal programme exit, with no arguments.
    atexit.register(CLEANUP_FUNCTION)


def read_environment():
    """
    Reads the the HIH7120 humidity/temperature sensor on address 0x27, and return the relative humidity as a percenteage,
    and the temperature in deg C.

    :return: a tuple of (humidity, temperature)
    """
    with I2C_LOCK:
        SMBUS.write_quick(0x27)
        time.sleep(0.11)  # Wait 110ms for conversion
        data = SMBUS.read_i2c_block_data(0x27, 0, 4)
    h_raw = (data[0] & 63) * 256 + data[1]
    humidity = h_raw / 16382.0 * 100.0
    t_raw = (data[0] * 256 + data[1]) / 4
    temperature = t_raw / 16382.0 * 165 - 40
    return humidity, temperature


class ADC_Set(object):
    """
    Handles communication with all eight of the MCP3208 A/D converter chips, and the 74X138 3-to-8 decoder chip
    that's used to select the correct MCP3208 chip to talk to.
    """
    def __init__(self):
        """
        Initialise the SPI bus, and create a lock object to control access to the 74X138 and the SPI bus.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.no_cs = True
        self.spi.max_speed_hz = 10000
        self.lock = threading.RLock()

    def _chip_select(self, number=None):
        """
        If number is a number between 0 and 7, send that address to the 74X138 decoder chip, and enable that
        (active low) output. If number is None, disable all outputs.

        The mapping is:

        number  signal name
        0       CS-X8
        1       CS-X6
        2       CS-X4
        3       CS-X2
        4       CS-X1
        5       CS-X3
        6       CS-X5
        7       CS-X7

        Note that the only currently populated 3208 chip, IC13, is on CS-X1 connected to output number 4.

        :param number: integer from 0-7 to enable an output on the 74X138, None to disable all outputs.
        :return: True for success, False if there was an error.
        """
        if number is None:
            with self.lock:
                GPIO.output(CS_DECODE_ENABLE, 0)
            logger.info('Disabled all outputs on 74X138')
            return True
        else:
            if type(number) == int:
                if (number >= 0) and (number <= 7):
                    with self.lock:
                        GPIO.output(CS_DECODE_ENABLE, 0)
                        numstr = '{0:03b}'.format(number)
                        a, b, c = int(numstr[-1]), int(numstr[-2]), int(numstr[-3])
                        GPIO.output(CS_DECODE_A, a)
                        GPIO.output(CS_DECODE_B, b)
                        GPIO.output(CS_DECODE_C, c)
                        GPIO.output(CS_DECODE_ENABLE, 1)
                    logger.info('Selected output %d on 74X138' % number)
                    return True
                else:
                    logger.error('Argument to chip_select must be None, or 0-7, not %d' % number)
                    return False
            else:
                logger.error('Argument to chip_select must be None, or 0-7, not %d' % number)
                return False

    def readADC(self, chipnum=0, channel=0):
        """
        Reads the 12 bit value from the given channel on the given MCP3208 chip. The chipnum is used to enable one of
        eight MCP3208 using a 74X138 3-to-8 decoder, and 'number' is the input port to read on that chip.

        Communications uses the SPI bus (GPIO pins 19, 21 and 23) to talk to the MCP3208, and three GPIO pins (31, 32, 33)
        to select the right chip using the 74X138.

        Note that this function doesn't currently return valid results, and needs further work.

        :param chipnum: number from 0-7 to send to the 74X138, to write the correct chip enable (see above)
        :param channel: input channel (0-7) number on the 3208 chip to read.
        :return: The raw 12 bit value
        """
        with self.lock:
            self._chip_select(number=chipnum)
            time.sleep(0.01)
            chanstr = '{0:03b}'.format(channel)
            d2, d1, d0 = int(chanstr[-3]), int(chanstr[-2]), int(chanstr[-1])
            cmd = [(0x6 | d2), (d1 << 7) | (d0 << 6), 0]
            r = self.spi.xfer2(cmd)   # Returns three bytes - the first is 0, the second and third are 0000XXXX, and XXXXXXXX
            self._chip_select(number=None)
        return 256 * (r[1] & 0b1111) + r[2]


class I2C_Control(object):
    """
    Class to handle the two PCA6416A chips on the I2C bus (GPIO pins 3 and 5) to do the output power control.
    """
    def __init__(self, instance=1):
        """
        Initialise the I2C bus, set the I2C address using the given chip instance number (1 or 2), and configure
        that chip to have all pins set to outputs, without polarity inversion, and all starting out switched off.

        Also initialises the 'portmap' attribute, which is a list of 16 zeroes or ones, containing the current output
        state for each of the 16 pins.

        Note - currently only chip 1 is populated.

        :param instance: Chip number to address - 1 or 2
        """
        if instance == 1:
            self.address = ADDRESS7_PCA6416A_1
        elif instance == 2:
            self.address = ADDRESS7_PCA6416A_2
        else:
            logger.error('Invalid 6416 instance (must be 1 or 2, not %d)' % instance)
            return

        self.portmap = [0] * 16  # Defaults to all outputs off.
        with I2C_LOCK:
            SMBUS.write_i2c_block_data(self.address, 2, [0, 0])  # Write 0,0 to output registers, to make sure outputs are off
            SMBUS.write_i2c_block_data(self.address, 4, [0, 0])  # Write 0,0 to polarity inversion register, for no inversion
            SMBUS.write_i2c_block_data(self.address, 6, [0, 0])  # Write 0,0 to configuration register, to set pins to outputs

    def _write_outputs(self, p1=0, p2=0):
        """
        Write 8 bits of output data to each of port1 and port 2 on the given PCA6416A chip, via i2c

        :param p1: port 1 output data (0-255)
        :param p2: port 2 output data (0-255)
        :return: True
        """
        with I2C_LOCK:
            SMBUS.write_i2c_block_data(self.address, 2, [p1, p2])  # Write 0,0 to output registers, to make sure outputs are off
        return True

    def turnon(self, channel=0):
        """
        Given a channel number from 1-16, change the value of that bit in self.portmap to 'on', and write the complete
        state (all bits) of self.portmap to the 6416 chip.

        :param channel: Channel number from 1-16
        :return: False if there was an error, True otherwise.
        """
        if type(channel) not in [int, long]:
            logger.error('Channel number must be an int from 1-16, not %s' % channel)
            return False
        if (channel < 1) or (channel > 16):
            logger.error('Channel number must be an int from 1-16, not %d' % channel)
            return False

        with I2C_LOCK:   # Get the lock before we start actually using the I2C bus, because we are changing instance data
            self.portmap[channel - 1] = 1
            p1 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[:8]), 2)
            p2 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[8:]), 2)
            self._write_outputs(p1=p1, p2=p2)
        return True

    def turnoff(self, channel=0):
        """
        Given a channel number from 1-16, change the value of that bit in self.portmap to 'off', and write the complete
        state (all bits) of self.portmap to the 6416 chip.

        :param channel: Channel number from 1-16
        :return: False if there was an error, True otherwise.
        """
        if type(channel) not in [int, long]:
            logger.error('Channel number must be an int from 1-16, not %s' % channel)
            return False
        if (channel < 1) or (channel > 16):
            logger.error('Channel number must be an int from 1-16, not %d' % channel)
            return False

        with I2C_LOCK:   # Get the lock before we start actually using the I2C bus, because we are changing instance data
            self.portmap[channel - 1] = 0
            p1 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[:8]), 2)
            p2 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[8:]), 2)
            self._write_outputs(p1=p1, p2=p2)
        return True

    def turn_all_on(self):
        """
        Turn ON all the outputs for this 6416 chip.

        :return: False if there was an error, True otherwise.
        """
        # TODO - loop over them all with a short delay, to reduce switching transients.
        with I2C_LOCK:    # Get the lock before we start actually using the I2C bus, because we are changing instance data
            self.portmap = [1] * 16
            p1 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[:8]), 2)
            p2 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[8:]), 2)
            self._write_outputs(p1=p1, p2=p2)
        return True

    def turn_all_off(self):
        """
        Turn OFF all the outputs for this 6416 chip.

        :return: False if there was an error, True otherwise.
        """
        # TODO - loop over them all with a short delay, to reduce switching transients.
        with I2C_LOCK:    # Get the lock before we start actually using the I2C bus, because we are changing instance data
            self.portmap = [0] * 16
            p1 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[:8]), 2)
            p2 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[8:]), 2)
            self._write_outputs(p1=p1, p2=p2)
        return True


class Antenna(object):
    """
    Class to represent one antenna power outlet. It can be turned on or off, and its voltage
    and current can be measured.

    Names are two characters, one of A1-A8, B1-B8, C1-C8 or D1-D8. The name is used to determine
    the chip select and ADC channel numbers to use for power control and sense measurements.
    """

    def __init__(self, name):
        assert (type(name) == str) and (len(name) == 2)
        assert name.upper() in CHIPMAP
        if name[0].upper() == 'A':
            self.pcontrol = PC1
            self.con_chan = (9 - int(name[1]))
        elif name[0].upper() == 'B':
            self.pcontrol = PC1
            self.con_chan = (17 - int(name[1]))
        elif name[0].upper() == 'C':
            self.pcontrol = PC2
            self.con_chan = (9 - int(name[1]))
        elif name[0].upper() == 'D':
            self.pcontrol = PC2
            self.con_chan = (17 - int(name[1]))
        self.chipnum, self.v_chan, self.i_chan = CHIPMAP[name]
        self._poweron = False

    def turnon(self):
        self.pcontrol.turnon(self.con_chan)
        self._poweron = True

    def turnoff(self):
        self.pcontrol.turnoff(self.con_chan)
        self._poweron = False

    def ison(self):
        return self._poweron

    def sense(self):
        """Returns a tuple of (voltage, current) in Volts and milliAmps respectively
        """
        v_raw = ADCS.readADC(chipnum=self.chipnum, channel=self.v_chan)
        i_raw = ADCS.readADC(chipnum=self.chipnum, channel=self.i_chan)
        # TODO - scale voltage and current
        return 60.0 * v_raw / 4096.0, i_raw / 4.096

    def __repr__(self):
        v, i = self.sense()
        if self.ison():
            return '<%s:  ON: %6.3f V, %5.3f mA>'
        else:
            return '<%s: OFF: %6.3f V, %5.3f mA>'


if __name__ == '__main__':
    init()
    logger.info('Main code starting.')
    for letter in 'ABCD':
        for number in '5678':
            name = '%s%s' % (letter, number)
            OUTPUTS[name].turnon()
            time.sleep(1.0)
            print OUTPUTS[name]
            OUTPUTS[name].turnoff()
            time.sleep(0.1)

    cleanup()
    # do stuff
    # RegisterCleanup(cleanup)             # Trap signals and register the cleanup() function to be run on exit.
