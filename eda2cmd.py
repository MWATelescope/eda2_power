#!/usr/bin/env python

"""
Command line utility to call methods on the EDA2 (FNDH) power supply controllers remotely,
over the network, using Pyro4 RPC calls.

Usage information available using the --help argument.

Designed to be run from a symlink to this file, where the name of the symlink is equal to the
host name of the Raspberry Pi computer controlling the FNDH.

Written by Andrew Williams (Andrew.Williams@curtin.edu.au).


"""

import os
import sys
import warnings

import Pyro4
from Pyro4 import util
Pyro4.config.DETAILED_TRACEBACK = True

sys.excepthook = Pyro4.util.excepthook

warnings.simplefilter('ignore', UserWarning)

SLAVEPORT = 19999  # Network port for for Pyro4 server on the remote Raspberry Pi

USAGE = """
Usage: '%s <command> [<names>]' runs the specified command on host %s. 
        Some commands accept one or more output names - eg 'A2' or 3.

        The command can be:
            ping (check to see if we can communicate)
            
            turn_all_on (turn on all outputs)
            turn_all_off (turn off all outputs)
            
            ison <names> (query whether output <name> is turned on)
            turnon <names> (turn on output <name>)
            turnoff <names> (turn off output <name>)

            reboot (reboot the Raspberry Pi - DANGEROUS)
            shutdown (shutdown & HALT the Raspberry Pi - VERY DANGEROUS)
            
            status (print a status string for each output)
            read_env (print temperature and humidity inside the unit)
            version (print the software version number for each output)

       <names> is a list of one or more output names, each of which is an
       upper case letter (A, B, C or D) followed by a digit from 1-8, eg
       'A2' or 'D7' (without the quotes).
       
       You can also specify a single letter (A, B, C, or D) which will be
       expanded to all 8 outputs in that bank (eg B1, B2, ... B8), or the 
       word 'all', which will be expanded to all 32 outputs (A1 .. D8).
       
       Each name can also be a single number from 1-16, instead of a
       letter/number pair. In that case, the number is treated as the 
       'tile' number, and two outputs will be addressed. For example, 
       using '3' is the same as specifying 'A7 A8'.
""" % (sys.argv[0], sys.argv[0])

RWARNING = """
      DANGER! Only run the reboot command when absolutely necessary. If the
      Raspberry Pi fails to boot for some reason, someone will have to walk
      out to the FNDH with a keyboard and monitor to fix it.

"""

SWARNING = """
      DANGER! Only run the shutdown command when absolutely necessary. After
      running this command, the Raspberry Pi will be halted, ready to be powered
      down. The Raspberry Pi must be physically powered off, then on again, to 
      get it running again.

"""

NMAP = {1: ['C5', 'C6'],
        2: ['D5', 'D6'],
        3: ['A7', 'A8'],
        4: ['A3', 'A4'],
        5: ['A1', 'A2'],
        6: ['B7', 'B8'],
        7: ['C1', 'C2'],
        8: ['B3', 'B4'],
        9: ['B1', 'B2'],
        10:['C3', 'C4'],
        11:['D1', 'D2'],
        12:['C7', 'C8'],
        13:['D3', 'D4'],
        14:['D7', 'D8'],
        15:['A5', 'A6'],
        16:['B5', 'B6']}

lastoutput = ''  # Last line printed, to compare against new output line.

if __name__ == '__main__':
    cname = os.path.split(sys.argv[0])[-1]
    if cname == 'eda2cmd.py':
        print('Run this command symlinked to the name of the host to communicate with - eg fndh1 or fndh2')
        sys.exit(-1
                 )
    args = sys.argv[1:]
    tlist = []
    action = ''
    if (len(args) < 1) or (args[0].lower() in ['h', 'help', '-h', '-help', '--h', '--help', '?', '-?', '--?']):
        print(USAGE)
        sys.exit(0)

    if args[0] == 'reboot':
        print(RWARNING)
        answer = raw_input("Are you sure you want to continue? (y/n): ")
        if 'y' in answer.lower():
            action = 'reboot'
        else:
            print("Aborting reboot")
            sys.exit(-1)
    elif args[0] == 'shutdown':
        print(SWARNING)
        answer = raw_input("Are you sure you want to continue? (y/n): ")
        if 'y' in answer.lower():
            action = 'shutdown'
        else:
            print("Aborting shutdown")
            sys.exit(-1)
    elif args[0] in ['ping', 'turn_all_on', 'turn_all_off', 'ison', 'turnon',
                     'turnoff', 'status', 'read_env', 'version']:
        action = args[0]
    else:
        print(USAGE)
        sys.exit(-1)

    onames = []
    for arg in args[1:]:
        if arg.upper() in 'ABCD':
            for digit in '12345678':
                onames.append('%s%s' % (arg.upper(), digit))
        elif (len(arg) == 2) and (arg[0] in 'ABCDabcd') and (arg[1] in '12345678'):
            onames.append(arg.upper())
        elif arg.isdigit() and 1 <= int(arg) <= 16:   # Tile number, to be looked up in NMAP
            onames += NMAP[int(arg)]
        elif arg.upper() == 'ALL':
            onames = []
            for letter in 'ABCD':
                for digit in '12345678':
                    onames.append('%s%s' % (letter, digit))
            break
    onames.sort()

    proxy = Pyro4.Proxy('PYRO:eda2@%s.mwa128t.org:%d' % (cname, SLAVEPORT))

    result = None
    if action == 'ping':
        result = proxy.ping()
        if result:
            print("OK.")
    elif action == 'shutdown':
        result = proxy.shutdown()
        if result:
            print("OK.")
    elif action == 'turn_all_on':
        result = proxy.turn_all_on()
        if result:
            print('All ON')
        else:
            print('Error turning outputs on, output state unknown.')
    if action == 'turn_all_off':
        result = proxy.turn_all_off()
        if result:
            print('All OFF')
        else:
            print('Error turning outputs off, output state unknown.')
    elif action == 'ison':
        result = proxy.ison(onames)
        for i in range(len(onames)):
            print('%s: %s' % (onames[i], {False:'OFF', True:'ON'}[result[i]]))
    if action == 'turnon':
        result = proxy.turnon(onames)
        if False not in result:
            print('%s turned ON' % ', '.join(onames))
        else:
            print('Error turning output/s on, output state unknown. Result=%s' % zip(onames, result))
    elif action == 'turnoff':
        result = proxy.turnoff(onames)
        if False not in result:
            print('%s turned OFF' % ', '.join(onames))
        else:
            print('Error turning output/s off, output state unknown. Result=%s' % zip(onames, result))
    if action == 'status':
        result = proxy.get_powers()
        voltages = []
        currents = []
        for tid in range(1,17):
            onames = NMAP[tid]
            ostrings = []
            for oname in onames:
                if oname in result.keys():
                    if len(result[oname]) == 3:
                        pstate, v, i = result[oname]
                    else:
                        ison = proxy.ison(oname)
                        pstate = {False:'OFF', True:'ON'}[ison]
                        v, i = result[oname]
                    voltages.append(v)
                    currents.append(i)
                    ostrings.append('<%s %3s: %4.1f V, %3.0f mA>' % (oname, pstate, v, i))
                else:
                    ostrings.append('%s: Unknown.' % oname)
            print('Tile %2d:    %s\nTile %2d:    %s' % (tid, ostrings[0], tid, ostrings[1]))
        print('Voltages from %4.1f to %4.1f V, Currents from %3.0f to %3.0f mA' % (min(voltages),
                                                                            max(voltages),
                                                                            min(currents),
                                                                            max(currents)))
    elif action == 'version':
        result = proxy.version()
        print('Software Version running on %s is: %s' % (cname, result))
    if action == 'read_env':
        result = proxy.read_environment()
        if result is None:
            print('Temp/Humidity unknown.')
        else:
            humidity, temperature = result
            print('Temp=%4.1f degC, Humidity=%2.0f %%' % (temperature, humidity))
