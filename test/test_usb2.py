#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

# use opt configuration similar to daisho
wb.regs.usb2_control_opt_disable_all.write(0)
wb.regs.usb2_control_opt_enable_hs.write(0)
wb.regs.usb2_control_opt_ignore_vbus.write(0)

# disable phy & core
wb.regs.usb2_control_phy_enable.write(0)
wb.regs.usb2_control_core_enable.write(0)

# enable phy
wb.regs.usb2_control_phy_enable.write(1)

# trigger analyzer
analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
analyzer.configure_trigger(cond={"soc_dbg_ulpi_out_byte": 0xe9})
analyzer.configure_subsampler(1)
analyzer.run(offset=128, length=2048)

# enable core
wb.regs.usb2_control_core_enable.write(1)

# wait & dump analyzer
while not analyzer.done():
    pass
analyzer.upload()
analyzer.save("dump.vcd")

# # #

wb.close()
