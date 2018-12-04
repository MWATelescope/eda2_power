#!/usr/bin/python

import atexit
import logging
from logging import handlers
import signal
import sys
import threading
import time
import traceback

import RPi.GPIO as GPIO

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
CS_DECODE_A = 6          # Bit 0 of the address on the 74X138 3-to-8 decoder
CS_DECODE_B = 12         # Bit 0 of the address on the 74X138 3-to-8 decoder
CS_DECODE_C = 13         # Bit 0 of the address on the 74X138 3-to-8 decoder
CS_DECODE_ENABLE = 26    # Enable (active high) on the 74X138 3-to-8 decoder


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


class ADCset(object):
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.lock = threading.RLock()

    def _chip_select(self, number=None):
        """
        If number is a number between 0 and 7, send that address to the 74X138 decoder chip, and enable that
        (active low) output. If number is None, disable all outputs.

        :param number: integer from 0-7 to enable an output on the 74X138, None to disable all outputs.
        :return: True for success, False if there was an error.
        """
        if number is None:
            GPIO.output(CS_DECODE_ENABLE, 0)
            logger.info('Disabled all outputs on 74X138')
            return True
        else:
            if type(number) == int:
                if (number >= 0) and (number <= 7):
                    GPIO.output(CS_DECODE_ENABLE, 0)
                    numstr = '{0:03b}'.format(number)
                    a, b, c = int(numstr[-1]), int(numstr[-2]), int(numstr[-3])
                    GPIO.output(CS_DECODE_A, a)
                    GPIO.output(CS_DECODE_B, b)
                    GPIO.output(CS_DECODE_C, c)
                    GPIO.output(CS_DECODE_ENABLE, 1)
                    logger.info('Selected output %d on 74X138')
                    return True
                else:
                    logger.error('Argument to chip_select must be None, or 0-7, not %d' % number)
                    return False
            else:
                logger.error('Argument to chip_select must be None, or 0-7, not %d' % number)
                return False

    def readADC(self, chipnum=0, channel=0):
        with self.lock:
            self._chip_select(number=chipnum)
            cmd = [1, 0x08 | channel, 0]
            r = self.spi.xfer2(cmd)   # Returns three bytes - the first is 0, the second and third are 0000XXXX, and XXXXXXXX
            self._chip_select(number=None)
            return 256 * (r[1] & 0x1111) + r[2]


if __name__ == '__main__':
    init()
    logger.info('Main code starting.')
    # do stuff
    # RegisterCleanup(cleanup)             # Trap signals and register the cleanup() function to be run on exit.
