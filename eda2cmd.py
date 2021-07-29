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
import pwd
import sys
import time
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
            
            ison <names> or 'all' (query whether output <name> is turned on)
            turnon <names> or 'all' (turn on output <name>)
            turnoff <names> or 'all' (turn off output <name>)

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
       
       If an output name is preceded by a minus sign (-), then that output
       will be excluded from the list to act on. For example, 
       
       ... turnon all -C   # Turn on everything except C1-C8
       ... turnon all -B3  # Turn on everything except B3
       ... turnon A B -A4 -A5   # Turn on A1-A3, A6-A8 and B1-B8
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

    if cname in ['fndh1', 'aavs2']:
        logfilename = '/var/log/fndh/aavs2'
    elif cname in ['fndh2', 'eda2']:
        logfilename = '/var/log/fndh/eda2'
    else:
        logfilename = '/var/log/fndh/eda2cmd'
    logfilename += '.%s.txt' % pwd.getpwuid(os.getuid())[0]
    logfile = open(logfilename, 'a')
    logfile.write('\n%s: Ran %s' % (time.ctime(), ' '.join(sys.argv)))

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

    includenames = []
    excludenames = []
    for arg in args[1:]:
        thesenames = []   # Expanded name list from just this argument
        subflag = False
        namespec = arg
        if arg.startswith('-'):
            subflag = True
            namespec = arg[1:]
        if namespec.upper() in 'ABCD':
            for digit in '12345678':
                thesenames.append('%s%s' % (namespec.upper(), digit))
        elif (len(namespec) == 2) and (namespec[0] in 'ABCDabcd') and (namespec[1] in '12345678'):
            thesenames.append(namespec.upper())
        elif namespec.isdigit() and 1 <= int(namespec) <= 16:   # Tile number, to be looked up in NMAP
            thesenames += NMAP[int(namespec)]
        elif namespec.upper() == 'ALL':
            for letter in 'ABCD':
                for digit in '12345678':
                    thesenames.append('%s%s' % (letter, digit))

        if subflag:
            excludenames += thesenames
        else:
            includenames += thesenames

    onames = [x for x in includenames if x not in excludenames]
    onames.sort()

    proxy = Pyro4.Proxy('PYRO:eda2@%s.mwa128t.org:%d' % (cname, SLAVEPORT))

    result = None
    if action == 'ping':
        result = proxy.ping()
        if result:
            print("OK.")
    elif action == 'shutdown':
        proxy.shutdown()
        logfile.write('%s: shutdown.' % time.time())
    elif action == 'turn_all_on':
        result = proxy.turn_all_on()
        if result:
            print('All ON')
            logfile.write('%s: All turned on.' % time.time())
        else:
            print('Error turning outputs on, output state unknown.')
            logfile.write('%s: Error turning all outputs on, output state unknown.' % time.time())
    if action == 'turn_all_off':
        result = proxy.turn_all_off()
        if result:
            print('All OFF')
            logfile.write('%s: All turned off.' % time.time())
        else:
            print('Error turning outputs off, output state unknown.')
            logfile.write('%s: Error turning all outputs off, output state unknown.' % time.time())
    elif action == 'ison':
        result = proxy.ison(onames)
        for i in range(len(onames)):
            print('%s: %s' % (onames[i], {False:'OFF', True:'ON'}[result[i]]))
    if action == 'turnon':
        result = proxy.turnon(onames)
        if False not in result:
            print('%s turned ON' % ', '.join(onames))
            logfile.write('%s: Output/s turned on: %s' % (time.time(), onames))
        else:
            print('Error turning output/s on, output state unknown. Result=%s' % zip(onames, result))
            logfile.write('%s: Error turning output/s on, output state unknown. Result=%s' % (time.time(), zip(onames, result)))
    elif action == 'turnoff':
        result = proxy.turnoff(onames)
        if False not in result:
            print('%s turned OFF' % ', '.join(onames))
            logfile.write('%s: Output/s turned of: %s' % (time.time(), onames))
        else:
            print('Error turning output/s off, output state unknown. Result=%s' % zip(onames, result))
            logfile.write('%s: Error turning output/s off, output state unknown. Result=%s' % (time.time(), zip(onames, result)))
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

    logfile.close()