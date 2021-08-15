Copyright (c) 2010 by Blake Foster <blfoster@vassar.edu>

This file is free software; you can redistribute it and/or modify
it under the terms of either the GNU General Public License version 2
or the GNU Lesser General Public License version 2.1, both as
published by the Free Software Foundation.

# Ethernet-ICMP Ping

ICMP ping library for the Arduino

To use, copy the icmp_ping directory into the libraries directory of your Arduino folder.

e.g. cp ~/Arduino-Ethernet-Icmp/EthernetICMP /usr/share/arduino/libraries/EthernetICMP

Then restart the Arduino software if necessary, and EthernetICMP should be available under the libraries dropdown.

## Update

This is the modified version of Arduino-Ping from Blake Foster by Github User masterx1981 

Since the masterx1981 fixes is built in to Ethernet library, i tried to move the ICMP out of Ethernet Libray to make it easier to add

Synchronus Version is TESTED and ALREADY WORKED, but the asynchronus version return some weird result to serial monitor

I also added some example if you want to use some DHCP using the Ethernet Library. Feel free to ask, create issue, pull request or make fork repository if you want.

## Issue

- Some memory leakage warning found if using Ethernet Library Version 2.0.0, try using newest version of Ethernet Library on Github

## Github Related Issues Link

[https://github.com/arduino-libraries/Ethernet/issues/105](https://github.com/arduino-libraries/Ethernet/issues/105)

## Tested On

Synchronus Version Tested on 15 August 2021 With Ethernet Library Version 2.0.0 using Arduino Uno and Ethernet Shield W5100