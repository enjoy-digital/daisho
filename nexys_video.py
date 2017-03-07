#!/usr/bin/env python3

from litex.gen import *
from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart.bridge import UARTWishboneBridge

from liteeth.common import convert_ip
from liteeth.phy.s7rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litex.boards.platforms import nexys_video


_usb3_io = [
    # HiTechGlobal USB3.0 FMC P1 connector
    ("usb_clkout", 0, Pins("LPC:LA00_CC_P"), IOStandard("LVCMOS25")),
    ("usb_reset_n", 0, Pins("LPC:LA32_N"), IOStandard("LVCMOS25")),
    ("usb_ulpi", 0,
        Subsignal("clk", Pins("LPC:HA01_CC_P")),
        Subsignal("data", Pins(
            "LPC:HA11_N", "LPC:HA13_N", "LPC:HA15_N", "LPC:HA12_N",
            "LPC:HA14_N", "LPC:HA15_P", "LPC:HA16_P", "LPC:HA12_P")),
        Subsignal("dir", Pins("LPC:HA14_P")),
        Subsignal("stp", Pins("LPC:HA16_N")),
        Subsignal("nxt", Pins("LPC:HA13_P")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_ctrl", 0,
        Subsignal("phy_reset_n", Pins("LPC:LA12_P")),
        Subsignal("tx_detrx_lpbk", Pins("LPC:LA29_N")),
        Subsignal("tx_elecidle", Pins("LPC:LA27_P")),
        Subsignal("power_down", Pins("LPC:LA12_N", "LPC:LA13_P")),
        Subsignal("tx_oneszeros", Pins("LPC:LA27_N")),
        Subsignal("tx_deemph", Pins("LPC:LA31_P", "LPC:LA28_N")),
        Subsignal("tx_margin", Pins("LPC:LA30_P", "LPC:LA30_N")),
        Subsignal("tx_swing", Pins("LPC:LA29_P")),
        Subsignal("rx_polarity", Pins("LPC:LA16_N")),
        Subsignal("rx_termination", Pins("LPC:LA13_N")),
        Subsignal("rate", Pins("LPC:LA28_P")),
        Subsignal("elas_buf_mode", Pins("LPC:LA15_N")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_status", 0,
        Subsignal("rx_elecidle", Pins("LPC:LA11_N")),
        Subsignal("rx_status", Pins("LPC:LA14_P", "LPC:LA15_P", "LPC:LA14_N")),
        Subsignal("phy_status", Pins("LPC:LA16_P")),
        Subsignal("pwr_present", Pins("LPC:LA32_P")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_data", 0,
        Subsignal("rx_clk", Pins("LPC:LA01_CC_P")),
        Subsignal("rx_valid", Pins("LPC:LA11_P")),
        Subsignal("rx_data", Pins(
            "LPC:LA10_N", "LPC:LA10_P", "LPC:LA09_N", "LPC:LA09_P",
            "LPC:LA07_N", "LPC:LA08_N", "LPC:LA05_N", "LPC:LA03_N",
            "LPC:LA06_N", "LPC:LA02_P", "LPC:LA06_P", "LPC:LA04_P",
            "LPC:LA03_P", "LPC:LA08_P", "LPC:LA07_P", "LPC:LA04_N")),
        Subsignal("rx_datak", Pins("LPC:LA02_N", "LPC:LA05_P")),

        Subsignal("tx_clk", Pins("LPC:LA20_P")),
        Subsignal("tx_data", Pins(
            "LPC:LA24_N", "LPC:LA26_N", "LPC:LA24_P", "LPC:LA26_P",
            "LPC:LA25_N", "LPC:LA21_N", "LPC:LA25_P", "LPC:LA21_P",
            "LPC:LA22_P", "LPC:LA22_N", "LPC:LA19_N", "LPC:LA23_N",
            "LPC:LA23_P", "LPC:LA18_CC_P", "LPC:LA19_P", "LPC:LA20_N")),
        Subsignal("tx_datak", Pins("LPC:LA18_CC_N", "LPC:LA17_CC_P")),
        IOStandard("LVCMOS25"),
    ),
]


class Platform(nexys_video.Platform):
    def __init__(self, *args, **kwargs):
        nexys_video.Platform.__init__(self, *args, **kwargs)
        self.add_extension(_usb3_io)


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
        self.submodules.eth_phy = LiteEthPHYRGMII(self.platform.request("eth_clocks"),
                                                 self.platform.request("eth"))
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


class USBSoC(BaseSoC):
    def __init__(self, platform):
        BaseSoC.__init__(self, platform)

        # usb ios
        usb_clkout = platform.request("usb_clkout")
        usb_reset_n = platform.request("usb_reset_n")
        usb_pipe_ctrl = platform.request("usb_pipe_ctrl")
        usb_pipe_status = platform.request("usb_pipe_status")
        usb_pipe_data = platform.request("usb_pipe_data")

        # TODO: - add ddr on usb_pipe_data
        #       - others signals to drive?
        self.comb += [
            usb_reset_n.eq(1),
        ]

        # phy pipe pll
        phy_pipe_pll_locked = Signal()
        phy_pipe_pll_fb = Signal()

        phy_pipe_half_clk_pll = Signal()
        phy_pipe_half_clk_phase_pll = Signal()
        phy_pipe_quarter_clk_pll = Signal()
        phy_pipe_tx_clk_pll = Signal()
        phy_pipe_tx_clk_phase_pll = Signal()

        phy_pipe_half_clk = Signal()
        phy_pipe_half_clk_phase = Signal()
        phy_pipe_quarter_clk = Signal()
        phy_pipe_tx_clk = Signal()
        phy_pipe_tx_clk_phase = Signal()

        self.specials += [
            Instance("PLLE2_BASE",
                p_STARTUP_WAIT="FALSE", o_LOCKED=phy_pipe_pll_locked,

                # VCO @ 1GHz
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=4.0,
                p_CLKFBOUT_MULT=4, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=usb_pipe_data.rx_clk, i_CLKFBIN=phy_pipe_pll_fb,
                o_CLKFBOUT=phy_pipe_pll_fb,

                # 125MHz: 1/2 PCLK
                p_CLKOUT0_DIVIDE=8, p_CLKOUT0_PHASE=0.0,
                o_CLKOUT0=phy_pipe_half_clk_pll,

                # 125MHz: 1/2 PCLK, phase shift 90
                p_CLKOUT1_DIVIDE=8, p_CLKOUT1_PHASE=90.0,
                o_CLKOUT1=phy_pipe_half_clk_phase_pll,

                # 62.5MHz: 1/4 PCLK
                p_CLKOUT2_DIVIDE=16, p_CLKOUT2_PHASE=0.0,
                o_CLKOUT2=phy_pipe_quarter_clk_pll,

                # 250Mhz: TX CLK
                p_CLKOUT3_DIVIDE=4, p_CLKOUT3_PHASE=0.0,
                o_CLKOUT3=phy_pipe_tx_clk_pll,

                # 250Mhz: TX CLK, phase shift 90
                p_CLKOUT4_DIVIDE=4, p_CLKOUT4_PHASE=90.0,
                o_CLKOUT4=phy_pipe_tx_clk_phase_pll
            ),
            Instance("BUFG", i_I=phy_pipe_half_clk_pll, o_O=phy_pipe_half_clk),
            Instance("BUFG", i_I=phy_pipe_half_clk_phase_pll, o_O=phy_pipe_half_clk_phase),
            Instance("BUFG", i_I=phy_pipe_quarter_clk_pll, o_O=phy_pipe_quarter_clk),
            Instance("BUFG", i_I=phy_pipe_tx_clk_pll, o_O=phy_pipe_tx_clk),
            Instance("BUFG", i_I=phy_pipe_tx_clk_phase_pll, o_O=phy_pipe_tx_clk_phase),
            Instance("IDELAYCTRL", i_REFCLK=ClockSignal(), i_RST=0) # FIXME
        ]

        # usb2 core
        # ulpi interface use HA FMC signals only available on HPC
        # so no usb2 core on this platform

        # usb3 core

        phy_pipe_rx_data = Signal(32)
        phy_pipe_rx_datak = Signal(4)
        phy_pipe_rx_valid = Signal(2)

        phy_pipe_tx_data = Signal(32)
        phy_pipe_tx_datak = Signal(4)


        self.specials += Instance("usb3_top",
            i_ext_clk=usb_clkout,
            i_reset_n=1,

            i_phy_pipe_pclk=usb_pipe_data.rx_clk,
            i_phy_pipe_half_clk=phy_pipe_half_clk,
            i_phy_pipe_half_clk_phase=phy_pipe_half_clk_phase,
            i_phy_pipe_quarter_clk=phy_pipe_quarter_clk,
            i_phy_pipe_rx_data=phy_pipe_rx_data,
            i_phy_pipe_rx_datak=phy_pipe_rx_datak,
            i_phy_pipe_rx_valid=phy_pipe_rx_valid,
            i_phy_pipe_tx_clk=phy_pipe_tx_clk,
            i_phy_pipe_tx_clk_phase=phy_pipe_tx_clk_phase,
            o_phy_pipe_tx_data=phy_pipe_tx_data,
            o_phy_pipe_tx_datak=phy_pipe_tx_datak,

            o_phy_reset_n=usb_pipe_ctrl.phy_reset_n,
            #o_phy_out_enable=, # TODO
            #o_phy_phy_reset_n=, # TODO
            o_phy_tx_detrx_lpbk=usb_pipe_ctrl.tx_detrx_lpbk,
            o_phy_tx_elecidle=usb_pipe_ctrl.tx_elecidle,
            io_phy_rx_elecidle=usb_pipe_status.rx_elecidle,
            i_phy_rx_status=usb_pipe_status.rx_status, # FIXME DDR
            o_phy_power_down=usb_pipe_ctrl.power_down,
            io_phy_phy_status=usb_pipe_status.phy_status, # FIXME DDR
            i_phy_pwrpresent=usb_pipe_status.pwr_present,

            o_phy_tx_oneszeros=usb_pipe_ctrl.tx_oneszeros,
            o_phy_tx_deemph=usb_pipe_ctrl.tx_deemph,
            o_phy_tx_margin=usb_pipe_ctrl.tx_margin,
            o_phy_tx_swing=usb_pipe_ctrl.tx_swing,
            o_phy_rx_polarity=usb_pipe_ctrl.rx_polarity,
            o_phy_rx_termination=usb_pipe_ctrl.rx_termination,
            o_phy_rate=usb_pipe_ctrl.rate,
            o_phy_elas_buf_mode=usb_pipe_ctrl.elas_buf_mode,

            i_buf_in_addr=0, # TODO
            i_buf_in_data=0, # TODO
            i_buf_in_wren=0, # TODO
            #o_buf_in_request=, # TODO
            #o_buf_in_ready=, # TODO
            i_buf_in_commit=0, # TODO
            i_buf_in_commit_len=0, # TODO
            #o_buf_in_commit_ack=, # TODO

            i_buf_out_addr=0, # TODO
            #o_buf_out_q=, # TODO
            #o_buf_out_len=, # TODO
            #o_buf_out_hasdata=, # TODO
            i_buf_out_arm=0, # TODO
            #o_buf_out_arm_ack=, # TODO

            #o_vend_req_act=, # TODO
            #o_vend_req_request=, # TODO
            #o_vend_req_val=, # TODO
        )
        platform.add_verilog_include_path(os.path.join("core"))
        platform.add_verilog_include_path(os.path.join("core", "usb3"))
        platform.add_source_dir(os.path.join("core", "usb3"))

        # ddr inputs
        self.specials += Instance("IDDR",
            p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
            i_C=ClockSignal(), i_CE=1, i_S=0, i_R=0, # FIXME (clock)
            i_D=usb_pipe_data.rx_valid, o_Q1=phy_pipe_rx_valid[0], o_Q2=phy_pipe_rx_valid[1],
        )
        for i in range(16):
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal(), i_CE=1, i_S=0, i_R=0, # FIXME (clock)
                i_D=usb_pipe_data.rx_data[i], o_Q1=phy_pipe_rx_data[i], o_Q2=phy_pipe_rx_data[16+i],
            )
        for i in range(2):
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal(), i_CE=1, i_S=0, i_R=0, # FIXME (clock)
                i_D=usb_pipe_data.rx_datak[i], o_Q1=phy_pipe_rx_datak[i], o_Q2=phy_pipe_rx_datak[2+i],
            )

        # ddr outputs
        for i in range(16):
            self.specials += Instance("ODDR",
                p_DDR_CLK_EDGE="SAME_EDGE",
                i_C=ClockSignal(), i_CE=1, i_S=0, i_R=0,
                i_D1=phy_pipe_tx_data[i], i_D2=phy_pipe_tx_data[16+i], o_Q=usb_pipe_data.tx_data[i],
            )
        for i in range(2):
            self.specials += Instance("ODDR",
                p_DDR_CLK_EDGE="SAME_EDGE",
                i_C=ClockSignal(), i_CE=1, i_S=0, i_R=0,
                i_D1=phy_pipe_tx_datak[i], i_D2=phy_pipe_tx_datak[2+i], o_Q=usb_pipe_data.tx_datak[i],
            )


def main():
    platform = Platform(toolchain="vivado")
    soc = USBSoC(platform)
    builder = Builder(soc, output_dir="build")
    vns = builder.build()
    soc.do_exit(vns)


if __name__ == "__main__":
    main()
