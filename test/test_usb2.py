#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

# opt configuration
wb.regs.usb2_control_opt_disable_all.write(0)
wb.regs.usb2_control_opt_enable_hs.write(0)
wb.regs.usb2_control_opt_ignore_vbus.write(0)

# disable phy & core
wb.regs.usb2_control_phy_enable.write(0)
wb.regs.usb2_control_core_enable.write(0)

# enable phy
wb.regs.usb2_control_phy_enable.write(1)

# enable core
wb.regs.usb2_control_core_enable.write(1)

# # #

wb.close()
