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
>>> c1.turn_all_off()    # Turn off all of the 6416 outputs
True
>>> adcs._chip_select(4)   # Enable the chip select line CS-X1 (active low) to IC13
INFO: time 1544422543.492386 - Selected output 4 on 74X138
True
>>> adcs._chip_select(None)   # Disable all the CS-X chip select lines
INFO: time 1544422551.228194 - Disabled all outputs on 74X138
True
>>>


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

LOGLEVEL_CONSOLE = logging.INFO  # INFO and above will be printed to STDOUT as well as the logfile
LOGLEVEL_LOGFILE = logging.DEBUG  # All messages will be sent to the log file
LOGLEVEL_REMOTE = logging.INFO  # INFO and above will be sent to the local syslog daemon (which can then formward them over the network)
LOGFILE = "/tmp/bfif.log"


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
      Initialise IO pins for power/enable control with all 8 beamformers,
      and create the global STATUS object.
    """
    GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
    GPIO.setwarnings(False)
    GPIO.setup(CS_DECODE_A, GPIO.OUT)
    GPIO.setup(CS_DECODE_B, GPIO.OUT)
    GPIO.setup(CS_DECODE_C, GPIO.OUT)
    GPIO.setup(CS_DECODE_ENABLE, GPIO.OUT)


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
        pass  # do cleanup stuff
    except:
        pass

    try:
        try:
            pass  # more cleanup stuff
        except:
            pass

    except:
        logger.exception('cleanup() - FAILED to do cleanup stuff. : %s', traceback.format_exc())

    logger.critical('cleanup() - finished.')


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
            chanstr = '{0:03b}'.format(channel)
            d2, d1, d0 = int(chanstr[-3]), int(chanstr[-2]), int(chanstr[-1])
            cmd = [(0x6 | d2), (d1 << 7) | (d0 << 6), 0]
            r = self.spi.xfer2(cmd)   # Returns three bytes - the first is 0, the second and third are 0000XXXX, and XXXXXXXX
            self._chip_select(number=None)
        return 256 * (r[1] & 0x1111) + r[2]


class I2C_Control(object):
    """
    Class to handle two devices on the I2C bus (GPIO pins 3 and 5) - the HIH7120 humidity/temperature sensor on address
    0x27, and two PCA6416A chips to do the output power control.
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
        self.lock = threading.RLock()
        self.bus = smbus.SMBus(1)
        if instance == 1:
            self.address = ADDRESS7_PCA6416A_1
        elif instance == 2:
            self.address = ADDRESS7_PCA6416A_2
        else:
            logger.error('Invalid 6416 instance (must be 1 or 2, not %d)' % instance)
            return

        self.portmap = [0] * 16  # Defaults to all outputs off.
        with self.lock:
            self.bus.write_i2c_block_data(self.address, 2, [0, 0])  # Write 0,0 to output registers, to make sure outputs are off
            self.bus.write_i2c_block_data(self.address, 4, [0, 0])  # Write 0,0 to polarity inversion register, for no inversion
            self.bus.write_i2c_block_data(self.address, 6, [0, 0])  # Write 0,0 to configuration register, to set pins to outputs

    def _write_outputs(self, p1=0, p2=0):
        """
        Write 8 bits of output data to each of port1 and port 2 on the given PCA6416A chip, via i2c

        :param p1: port 1 output data (0-255)
        :param p2: port 2 output data (0-255)
        :return: True
        """
        with self.lock:
            self.bus.write_i2c_block_data(self.address, 2, [p1, p2])  # Write 0,0 to output registers, to make sure outputs are off
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

        with self.lock:
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

        with self.lock:
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
        with self.lock:
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
        with self.lock:
            self.portmap = [0] * 16
            p1 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[:8]), 2)
            p2 = int('%d%d%d%d%d%d%d%d' % tuple(self.portmap[8:]), 2)
            self._write_outputs(p1=p1, p2=p2)
        return True

    # TODO - this will need to change, as there will be two of these instances for the two 6416 chips, but there's
    # only one HIH7120 chip.
    def read_environment(self):
        """
        Reads the HIH7120 humidity/temperature sensor, and return the relative humidity as a percenteage,
        and temperature in deg C.

        :return: a tuple of (humidity, temperature)
        """
        with self.lock:
            self.bus.write_quick(0x27)
            time.sleep(0.11)  # Wait 110ms for conversion
            data = self.bus.read_i2c_block_data(0x27, 0, 4)
        h_raw = (data[0] & 63) * 256 + data[1]
        humidity = h_raw / 16382.0 * 100.0
        t_raw = (data[0] * 256 + data[1]) / 4
        temperature = t_raw / 16382.0 * 165 - 40
        return humidity, temperature


if __name__ == '__main__':
    init()
    adcs = ADC_Set()
    c1 = I2C_Control(instance=1)
    #  c2 = I2C_Control(instance=2)   # Not yet populated
    logger.info('Main code starting.')
    # do stuff
    # RegisterCleanup(cleanup)             # Trap signals and register the cleanup() function to be run on exit.
