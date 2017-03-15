#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

wb.regs.usb2_control_phy_enable.write(1)
wb.regs.usb2_control_core_enable.write(0)

analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
#analyzer.configure_trigger(cond={"soc_usb2_reset_n" : 0})
analyzer.configure_trigger(cond={"soc_dbg_state" : 20})
analyzer.configure_subsampler(1)
analyzer.run(offset=128, length=8192)

wb.regs.usb2_control_phy_enable.write(1)
wb.regs.usb2_control_core_enable.write(1)

while not analyzer.done():
    pass
analyzer.upload()
analyzer.save("dump.vcd")

wb.regs.usb2_control_phy_enable.write(0)
wb.regs.usb2_control_core_enable.write(0)

# # #

wb.close()
