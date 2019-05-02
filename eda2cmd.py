#!/usr/bin/python

"""Command line utility to call methods on the EDA2 (FNDH) power supply controllers using Pyro4.
"""

import os
import sys
import warnings

import Pyro4

sys.excepthook = Pyro4.util.excepthook

warnings.simplefilter('ignore', UserWarning)

SLAVEPORT = 19999  # Network port for for Pyro4 server on the remote Raspberry Pi

USAGE = """
Usage: '%s <command> [<name>]' runs the specified command. Some commands
        accept an output name - eg 'A2' or 'D7'.

        The command can be:
            ping (check to see if we can communicate)
            
            turn_all_on (turn on all outputs)
            turn_all_off (turn off all outputs)
            
            ison <name> (query whether output <name> is turned on)
            turnon <name> (turn on output <name>)
            turnoff <name> (turn off output <name>)

            reboot (reboot the Raspberry Pi - DANGEROUS)
            shutdown (shutdown & HALT the Raspberry Pi - VERY DANGEROUS)
            
            status (print a status string for each output)
            read_env (print the temperature and humidity inside the unit)
            version (print the software version number for each output)

       <name> is an upper case letter (A, B, C or D) followed by a digit 
       from 1-8, eg 'A2' or 'D7' (without the quotes).
""" % sys.argv[0]

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

lastoutput = ''  # Last line printed, to compare against new output line.

if __name__ == '__main__':
    cname = os.path.split(sys.argv[0])[-1]
    if cname == 'eda2cmd.py':
        print('Run this command symlinked to the name of the host to communicate with - fndh1 or fndh2')
        sys.exit(-1
                 )
    args = sys.argv[1:]
    tlist = []
    action = ''
    if (len(args) < 1) or (args[0].lower() in ['h', 'help', '-h', '-help', '--h', '--help', '?', '-?', '--?']):
        print USAGE
        sys.exit(0)

    if args[0] == 'reboot':
        print RWARNING
        answer = raw_input("Are you sure you want to continue? (y/n): ")
        if 'y' in answer.lower():
            action = 'reboot'
        else:
            print "Aborting reboot"
            sys.exit(-1)
    elif args[0] == 'shutdown':
        print SWARNING
        answer = raw_input("Are you sure you want to continue? (y/n): ")
        if 'y' in answer.lower():
            action = 'shutdown'
        else:
            print "Aborting shutdown"
            sys.exit(-1)
    elif args[0] in ['ping', 'turn_all_on', 'turn_all_off', 'ison', 'turnon',
                     'turnoff', 'status', 'read_env', 'version']:
        action = args[0]
    else:
        print USAGE
        sys.exit(-1)

    oname = ''
    if len(args) > 1:
        oname = args[1].upper()

    proxy = Pyro4.Proxy('PYRO:eda2@%s.mwa128t.org:%d' % (cname, SLAVEPORT))

    result = None
    if action == 'ping':
        result = proxy.ping()
        if result:
            print("OK.")
    elif action == 'turn_all_on':
        result = proxy.turn_all_on()
        print('All ON')
    if action == 'turn_all_off':
        result = proxy.turn_all_off()
        print('All OFF')
    elif action == 'ison':
        result = proxy.ison(oname)
        print({False:'OFF', True:'ON'}[result])
    if action == 'turnon':
        result = proxy.turnon(oname)
        print('%s turned ON' % oname)
    elif action == 'turnoff':
        result = proxy.turnoff(oname)
        print('%s turned OFF' % oname)
    if action == 'status':
        result = proxy.get_powers()
        for letter in 'ABCD':
            for number in range(1,9):
                name = '%s%d' % (letter, number)
                if name in result.keys():
                    if len(result) == 3:
                        pstate, v, i = result[name]
                    else:
                        ison = proxy.ison(name)
                        pstate = {False:'OFF', True:'ON'}[ison]
                        v, i = result[name]
                    print('<%s: %3s: %6.3f V, %6.3f mA>' % (name, pstate, v, i))
                else:
                    print('%s: Unknown.' % name)
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

