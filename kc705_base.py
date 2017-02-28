#!/usr/bin/env python3

import argparse

from litex.gen import *
from litex.build.generic_platform import *

from litex.gen.genlib.io import CRG

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart.bridge import UARTWishboneBridge

from liteeth.common import convert_ip
from liteeth.phy import LiteEthPHY
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from kc705_platform import Platform


class BaseSoC(SoCCore):
    csr_map = {
        "eth_phy":  11,
        "eth_core": 12
    }
    csr_map.update(SoCCore.csr_map)
    def __init__(self, platform,
        mac_address=0x10e2d5000000,
        ip_address="192.168.1.50"):
        clk_freq = int(1e9/platform.default_clk_period)
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="Daisho USB3.0 Test Design",
            with_timer=False
        )
        self.submodules.crg = CRG(platform.request(platform.default_clk_name))

        # uart <--> wishbone
        self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"),
                                                  clk_freq, baudrate=115200))
        self.add_wb_master(self.cpu_or_bridge.wishbone)

        # ethernet PHY and UDP/IP stack
        self.submodules.eth_phy = LiteEthPHY(platform.request("eth_clocks"),
                                             platform.request("eth"),
                                             clk_freq=clk_freq)
        self.submodules.eth_core = LiteEthUDPIPCore(self.eth_phy,
                                                    mac_address,
                                                    convert_ip(ip_address),
                                                    clk_freq)

        # ethernet <--> wishbone
        self.submodules.etherbone = LiteEthEtherbone(self.eth_core.udp, 1234)
        self.add_wb_master(self.etherbone.master.bus)

        # timing constraints
        self.crg.cd_sys.clk.attr.add("keep")
        self.eth_phy.crg.cd_eth_rx.clk.attr.add("keep")
        self.eth_phy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, 6.0)
        self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_rx.clk, 8.0)
        self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_tx.clk, 8.0)

        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.eth_phy.crg.cd_eth_rx.clk,
            self.eth_phy.crg.cd_eth_tx.clk)


def main():
    platform = Platform()
    soc = BaseSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    builder.build()


if __name__ == "__main__":
    main()
