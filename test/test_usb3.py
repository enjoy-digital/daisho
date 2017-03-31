#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

wb.regs.usb2_control_phy_enable.write(0)
wb.regs.usb2_control_core_enable.write(0)

# disable phy & core
wb.regs.usb3_control_phy_enable.write(0)
wb.regs.usb3_control_core_enable.write(0)

# trigger analyzer
analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
#analyzer.configure_trigger(cond={"usb_pipe_ctrl_phy_reset_n": 1})
#analyzer.configure_trigger(cond={"soc_usb3_reset_n": 1})
analyzer.configure_trigger(cond={"soc_phy_pipe_rx_datak": 0b0000})
#analyzer.configure_trigger(cond={"usb_ulpi_clk": 0b1})
analyzer.configure_subsampler(1)
analyzer.run(offset=2048-128, length=2048)

# enable phy
wb.regs.usb2_control_phy_enable.write(1)
wb.regs.usb3_control_phy_enable.write(1)

# enable core
wb.regs.usb2_control_core_enable.write(1)
wb.regs.usb3_control_core_enable.write(1)

# wait & dump analyzer
while not analyzer.done():
    pass
analyzer.upload()
analyzer.save("dump.vcd")


# # #

wb.close()
