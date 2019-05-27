# eda2_power

Engineering Development Array 2 power control software

Code to control a prototype 32-output 48V DC power supply at the MRO.

Files are:
  - eda2.py: Runs on the Raspberry Pi inside the controller.
  - eda2cmd.py: Runs on any host on the same network as the Raspberry Pi. This is a command line client.
  - eda2-nagios.py: (optional) Runs on an external host running Nagios or Icinga/Icing2. Interrogates the status of the power supply.

Written by Andrew Williams (Andrew.Williams@curtin.edu.au) 

