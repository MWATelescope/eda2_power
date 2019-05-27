#!/usr/bin/python

"""Nagios plugin to interrogate a EDA2/FNDH control host (the Raspberry Pi) and
   return formatted Nagios plugin output giving system health and other
   parameters.
"""

import sys

import Pyro4
from Pyro4 import util

sys.excepthook = Pyro4.util.excepthook
Pyro4.config.DETAILED_TRACEBACK = True


TEMP_CRIT = 80.0
TEMP_WARN = 70.0
HUMID_CRIT = 95.0
HUMID_WARN = 90.0

SLAVEPORT = 19999  # Network port for for Pyro4 server on the remote Raspberry Pi


def main(powers=None, env_data=None):
    p_res = 0
    p_error = ''
    oparams = []   # parameters to list in human-readable string
    pparams = []   # parameters to include in performance data for graphs
    if powers:
        for letter in 'ABCD':
            for digit in '12345678':
                name = '%s%s' % (letter, digit)
                if (name in powers) and powers[name]:
                    pstate, v, i = powers[name]
                    # On/off only human readable, voltage only in performance params, current in both
                    oparams.append('%s_state=%s' % (name, pstate))
                    pparams.append('%s_volts=%1.3f' % (name, v))
                    oparams.append('%s_mA=%1.3f' % (name, i))
                    pparams.append('%s_mA=%1.3f' % (name, i))
                else:
                    oparams.append('%s missing data')
                    p_error = "One or more channels missing power data"
                    p_res = 1
    else:
        p_error = 'No power data from device'
        p_res = 3

    t_res = 0
    t_error = ''
    h_res = 0
    h_error = ''
    if env_data:
        humidity, temperature = env_data
        # Show humidity and temperature in both human-readable string and performance data
        oparams.append('humidity=%1.0f' % humidity)
        pparams.append('humidity=%1.0f' % humidity)
        oparams.append('temperature=%1.1f' % temperature)
        pparams.append('temperature=%1.1f' % temperature)

        if temperature > TEMP_CRIT:
            t_res = 2
            t_error = 'Temperature critical'
        elif temperature > TEMP_WARN:
            t_res = 1
            t_error = 'Temperature warning'

        if humidity > HUMID_CRIT:
            h_res = 2
            h_error = 'Humidity critical'
        elif humidity > HUMID_WARN:
            h_res = 1
            h_error = 'Humidity warning'
    else:
        t_res = 3
        h_res = 3
        t_error = 'Missing temperature and humidity'
        h_error = ''

    if (p_res == 2) or (t_res == 2) or (h_res == 2):
        res = 2
        msg = 'CRITICAL: ' + (', '.join([p_error, t_error, h_error]))
    elif (p_res == 3) or (t_res == 3) or (h_res == 3):
        res = 3
        msg = 'UNKNOWN: ' + (', '.join([p_error, t_error, h_error]))
    elif (p_res == 1) or (t_res == 1) or (h_res == 1):
        res = 1
        msg = 'WARNING: ' + (', '.join([p_error, t_error, h_error]))
    else:
        res = 0
        msg = 'OK: '

    return res, "%s; %s " % (msg, ', '.join(oparams)), pparams


if __name__ == "__main__":
    if len(sys.argv) == 2:         # Eg 'bfif-nagios lbtile2002.mwa128t.org'
        fullhostname = sys.argv[1]
        hostname = fullhostname.split('.')[0]
    elif len(sys.argv) == 3 and sys.argv[1] == '-H':       # Eg 'bfif-nagios -H lba1'
        hostname = sys.argv[2].split('.')[0]
        fullhostname = hostname
    else:
        sys.exit()
    uri = 'PYRO:eda2@%s:%d' % (fullhostname, SLAVEPORT)
    p = Pyro4.Proxy(uri)
    powers = p.get_powers()
    env_data = p.read_environment()

    res, outstring, perfparams = main(powers=powers, env_data=env_data)
    print outstring + " | " + " ".join(perfparams) + " "

    sys.exit(res)
