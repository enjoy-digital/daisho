#!/usr/bin/env python3
import time

from litex.soc.tools.remote.comm_udp import CommUDP
from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
analyzer.configure_trigger(cond={"soc_reset_n_out" : 1})
analyzer.configure_subsampler(1)
analyzer.run(offset=128, length=2048)

wb.regs.usb2_control_enable.write(1)

while not analyzer.done():
    pass
analyzer.upload()
analyzer.save("dump.vcd")

time.sleep(2)

wb.regs.usb2_control_enable.write(0)

# # #

wb.close()
