#!/usr/bin/env python3

from litex.soc.tools.remote.comm_udp import CommUDP

wb = CommUDP("192.168.1.50", 1234, debug=False)
wb.open()

# # #

from litescope.software.driver.analyzer import LiteScopeAnalyzerDriver
analyzer = LiteScopeAnalyzerDriver(wb.regs, "analyzer", debug=True)
analyzer.configure_trigger(cond={})
analyzer.configure_subsampler(1)
analyzer.run(offset=128, length=256)
while not analyzer.done():
    pass
analyzer.upload()
analyzer.save("dump.vcd")

# # #

wb.close()
