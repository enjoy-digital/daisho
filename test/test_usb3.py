#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

# disable usb2 phy & core
wb.regs.usb2_control_phy_enable.write(0)
wb.regs.usb2_control_core_enable.write(0)

# disable usb3 phy & core
wb.regs.usb3_control_phy_enable.write(0)
wb.regs.usb3_control_core_enable.write(0)

# enable usb2 & usb3 phy
wb.regs.usb2_control_phy_enable.write(1)
wb.regs.usb3_control_phy_enable.write(1)

# enable usb2 & usb3 core
wb.regs.usb2_control_core_enable.write(1)
wb.regs.usb3_control_core_enable.write(1)

# # #

wb.close()
