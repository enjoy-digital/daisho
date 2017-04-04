#!/usr/bin/env python3

from litex.gen import *
from litex.gen.genlib.resetsync import AsyncResetSynchronizer
from litex.gen.fhdl.specials import Tristate
from litex.build.generic_platform import *

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart.bridge import UARTWishboneBridge

from liteeth.common import convert_ip
from liteeth.phy.s7rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litex.boards.platforms import nexys_video

from litescope import LiteScopeAnalyzer


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


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_clk200 = ClockDomain()

        clk100 = platform.request("clk100")
        rst = platform.request("cpu_reset")

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys = Signal()
        pll_clk200 = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                     p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                     # VCO @ 1600 MHz
                     p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                     p_CLKFBOUT_MULT=16, p_DIVCLK_DIVIDE=1,
                     i_CLKIN1=clk100, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                     # 133 MHz
                     p_CLKOUT0_DIVIDE=12, p_CLKOUT0_PHASE=0.0,
                     o_CLKOUT0=pll_sys,

                     # 200 MHz
                     p_CLKOUT1_DIVIDE=8, p_CLKOUT1_PHASE=0.0,
                     o_CLKOUT1=pll_clk200,
            ),
            Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk),
            Instance("BUFG", i_I=pll_clk200, o_O=self.cd_clk200.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked | ~rst),
            AsyncResetSynchronizer(self.cd_clk200, ~pll_locked |  ~rst),
        ]

        reset_counter = Signal(4, reset=15)
        ic_reset = Signal(reset=1)
        self.sync.clk200 += \
            If(reset_counter != 0,
                reset_counter.eq(reset_counter - 1)
            ).Else(
                ic_reset.eq(0)
            )
        self.specials += Instance("IDELAYCTRL", i_REFCLK=ClockSignal("clk200"), i_RST=ic_reset)


class BaseSoC(SoCCore):
    csr_map = {
        "eth_phy":  11,
        "eth_core": 12
    }
    csr_map.update(SoCCore.csr_map)
    def __init__(self, platform,
        with_ethernet=True,
        mac_address=0x10e2d5000000,
        ip_address="192.168.1.50"):
        clk_freq = int(133e6)
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="Daisho USB3.0 Test Design",
            with_timer=False
        )
        self.submodules.crg = _CRG(platform)

        # uart <--> wishbone
        self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"),
                                                  clk_freq, baudrate=115200))
        self.add_wb_master(self.cpu_or_bridge.wishbone)

        self.crg.cd_sys.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, 7.5)

        # ethernet PHY and UDP/IP stack
        if with_ethernet:
            self.submodules.eth_phy = LiteEthPHYRGMII(self.platform.request("eth_clocks"),
                                                      self.platform.request("eth"))
            self.submodules.eth_core = LiteEthUDPIPCore(self.eth_phy,
                                                        mac_address,
                                                        convert_ip(ip_address),
                                                        clk_freq)

            # ethernet <--> wishbone
            self.submodules.etherbone = LiteEthEtherbone(self.eth_core.udp, 1234)
            self.add_wb_master(self.etherbone.wishbone.bus)

            # timing constraints
            self.eth_phy.crg.cd_eth_rx.clk.attr.add("keep")
            self.eth_phy.crg.cd_eth_tx.clk.attr.add("keep")
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_rx.clk, 8.0)
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_tx.clk, 8.0)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.eth_phy.crg.cd_eth_rx.clk,
                self.eth_phy.crg.cd_eth_tx.clk)


class USBSoC(BaseSoC):
    csr_map = {
        "analyzer": 20,
        "usb2_control": 21,
        "usb3_control": 22
    }
    csr_map.update(BaseSoC.csr_map)
    def __init__(self, platform, usb_connector=0,
        with_usb3=True, with_usb3_analyzer=False):
        BaseSoC.__init__(self, platform)

        # fmc vadj (2.5V)
        self.comb += platform.request("vadj").eq(0b10)

        # usb ios
        usb_reset_n = platform.request("usb_reset_n", usb_connector)
        if with_usb3:
            usb_clkout = platform.request("usb_clkout", usb_connector)
            usb_pipe_ctrl = platform.request("usb_pipe_ctrl", usb_connector)
            usb_pipe_status = platform.request("usb_pipe_status", usb_connector)
            usb_pipe_data = platform.request("usb_pipe_data", usb_connector)

        usb3_reset_n = Signal(reset=1)
        self.comb += usb_reset_n.eq(usb3_reset_n)


        # usb3 core
        if with_usb3:
            class USB3Control(Module, AutoCSR):
                def __init__(self):
                    self._phy_enable = CSRStorage()
                    self._core_enable = CSRStorage()

                    # # #

                    self.phy_enable = self._phy_enable.storage
                    self.core_enable = self._core_enable.storage


            self.submodules.usb3_control = USB3Control()

            phy_pipe_pll_locked = Signal()
            phy_pipe_pll_fb = Signal()

            phy_pipe_half_clk_pll = Signal()
            phy_pipe_half_clk_phase_pll = Signal()
            phy_pipe_quarter_clk_pll = Signal()
            phy_pipe_tx_clk_phase_pll = Signal()

            phy_pipe_half_clk = Signal()
            phy_pipe_half_clk_phase = Signal()
            phy_pipe_quarter_clk = Signal()
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

                    # 250Mhz: TX CLK, phase shift 90
                    p_CLKOUT3_DIVIDE=4, p_CLKOUT3_PHASE=90.0,
                    o_CLKOUT3=phy_pipe_tx_clk_phase_pll
                ),
                Instance("BUFG", i_I=phy_pipe_half_clk_pll, o_O=phy_pipe_half_clk),
                Instance("BUFG", i_I=phy_pipe_half_clk_phase_pll, o_O=phy_pipe_half_clk_phase),
                Instance("BUFG", i_I=phy_pipe_quarter_clk_pll, o_O=phy_pipe_quarter_clk),
                Instance("BUFG", i_I=phy_pipe_tx_clk_phase_pll, o_O=phy_pipe_tx_clk_phase),
                #Instance("IDELAYCTRL", i_REFCLK=ClockSignal(), i_RST=ResetSignal()) # not used
            ]

            self.clock_domains.cd_phy_pipe_half = ClockDomain()
            self.clock_domains.cd_phy_pipe_half_phase = ClockDomain()
            self.clock_domains.cd_phy_pipe_quarter = ClockDomain()
            self.clock_domains.cd_phy_pipe_tx_phase = ClockDomain()
            self.comb += [
                self.cd_phy_pipe_half.clk.eq(phy_pipe_half_clk),
                self.cd_phy_pipe_half_phase.clk.eq(phy_pipe_half_clk_phase),
                self.cd_phy_pipe_quarter.clk.eq(phy_pipe_quarter_clk),
                self.cd_phy_pipe_tx_phase.clk.eq(phy_pipe_tx_clk_phase)
            ]
            self.specials += [
                AsyncResetSynchronizer(self.cd_phy_pipe_half, ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_half_phase, ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_quarter, ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_tx_phase, ~phy_pipe_pll_locked)
            ]
            self.cd_phy_pipe_half.clk.attr.add("keep")
            self.cd_phy_pipe_half_phase.clk.attr.add("keep")
            self.cd_phy_pipe_quarter.clk.attr.add("keep")
            self.cd_phy_pipe_tx_phase.clk.attr.add("keep")
            self.platform.add_period_constraint(self.cd_phy_pipe_half.clk, 8.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_half_phase.clk, 8.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_quarter.clk, 16.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_tx_phase.clk, 4.0)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.cd_phy_pipe_half.clk,
                self.cd_phy_pipe_half_phase.clk,
                self.cd_phy_pipe_quarter.clk,
                self.cd_phy_pipe_tx_phase.clk)


            phy_pipe_rx_data = Signal(32)
            phy_pipe_rx_datak = Signal(4)
            phy_pipe_rx_valid = Signal(2)

            phy_pipe_tx_data = Signal(32)
            phy_pipe_tx_datak = Signal(4)

            phy_rx_status = Signal(6)
            phy_phy_status = Signal(2)

            dbg_pipe_state = Signal(6)
            dbg_ltssm_state = Signal(5)

            usb_pipe_status_phy_status = Signal()
            self.specials += Tristate(usb_pipe_status.phy_status, 0, ~usb3_reset_n, usb_pipe_status_phy_status)

            self.comb += usb3_reset_n.eq(self.usb3_control.phy_enable | platform.request("user_sw", 0))
            self.specials += Instance("usb3_top",
                i_ext_clk=ClockSignal(),
                i_reset_n=self.usb3_control.core_enable | platform.request("user_sw", 1),

                i_phy_pipe_half_clk=ClockSignal("phy_pipe_half"),
                i_phy_pipe_half_clk_phase=ClockSignal("phy_pipe_half_phase"),
                i_phy_pipe_quarter_clk=ClockSignal("phy_pipe_quarter"),

                i_phy_pipe_rx_data=phy_pipe_rx_data,
                i_phy_pipe_rx_datak=phy_pipe_rx_datak,
                i_phy_pipe_rx_valid=phy_pipe_rx_valid,
                o_phy_pipe_tx_data=phy_pipe_tx_data,
                o_phy_pipe_tx_datak=phy_pipe_tx_datak,

                #o_phy_reset_n=,
                #o_phy_out_enable=,
                o_phy_phy_reset_n=usb_pipe_ctrl.phy_reset_n,
                o_phy_tx_detrx_lpbk=usb_pipe_ctrl.tx_detrx_lpbk,
                o_phy_tx_elecidle=usb_pipe_ctrl.tx_elecidle,
                io_phy_rx_elecidle=usb_pipe_status.rx_elecidle,
                i_phy_rx_status=phy_rx_status,
                o_phy_power_down=usb_pipe_ctrl.power_down,
                i_phy_phy_status_i=phy_phy_status,
                #o_phy_phy_status_o=,
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

                o_dbg_pipe_state=dbg_pipe_state,
                o_dbg_ltssm_state=dbg_ltssm_state
            )
            platform.add_verilog_include_path(os.path.join("core"))
            platform.add_verilog_include_path(os.path.join("core", "usb3"))
            platform.add_source_dir(os.path.join("core", "usb3"))

            # ddr inputs
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                i_D=usb_pipe_data.rx_valid, o_Q1=phy_pipe_rx_valid[0], o_Q2=phy_pipe_rx_valid[1],
            )
            for i in range(16):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_data.rx_data[i], o_Q1=phy_pipe_rx_data[i], o_Q2=phy_pipe_rx_data[16+i],
                )
            for i in range(2):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_data.rx_datak[i], o_Q1=phy_pipe_rx_datak[i], o_Q2=phy_pipe_rx_datak[2+i],
                )
            for i in range(3):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_status.rx_status[i], o_Q1=phy_rx_status[i], o_Q2=phy_rx_status[3+i],
                )
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                i_D=usb_pipe_status_phy_status, o_Q1=phy_phy_status[0], o_Q2=phy_phy_status[1],
            )

            # ddr outputs
            self.specials += Instance("ODDR",
                p_DDR_CLK_EDGE="SAME_EDGE",
                i_C=ClockSignal("phy_pipe_tx_phase"), i_CE=1, i_S=0, i_R=0,
                i_D1=1, i_D2=0, o_Q=usb_pipe_data.tx_clk,
            )
            for i in range(16):
                self.specials += Instance("ODDR",
                    p_DDR_CLK_EDGE="SAME_EDGE",
                    i_C=ClockSignal("phy_pipe_half_phase"), i_CE=1, i_S=0, i_R=0,
                    i_D1=phy_pipe_tx_data[i], i_D2=phy_pipe_tx_data[16+i], o_Q=usb_pipe_data.tx_data[i],
                )
            for i in range(2):
                self.specials += Instance("ODDR",
                    p_DDR_CLK_EDGE="SAME_EDGE",
                    i_C=ClockSignal("phy_pipe_half_phase"), i_CE=1, i_S=0, i_R=0,
                    i_D1=phy_pipe_tx_datak[i], i_D2=phy_pipe_tx_datak[2+i], o_Q=usb_pipe_data.tx_datak[i],
                )

            # usb3 debug
            if with_usb3_analyzer:
                analyzer_signals = [
                    dbg_pipe_state,
                    dbg_ltssm_state,

                    usb_pipe_ctrl.tx_detrx_lpbk,
                    usb_pipe_ctrl.tx_elecidle,
                    usb_pipe_status.rx_elecidle,
                    usb_pipe_ctrl.power_down,
                    usb_pipe_status.phy_status,
                    usb_pipe_status.pwr_present,

                    usb_pipe_ctrl.tx_oneszeros,
                    usb_pipe_ctrl.tx_deemph,
                    usb_pipe_ctrl.tx_margin,
                    usb_pipe_ctrl.tx_swing,
                    usb_pipe_ctrl.rx_polarity,
                    usb_pipe_ctrl.rx_termination,
                    usb_pipe_ctrl.rate,
                    usb_pipe_ctrl.elas_buf_mode,

                    phy_pipe_rx_valid,
                    phy_pipe_rx_data,
                    phy_pipe_rx_datak,

                    phy_pipe_tx_data,
                    phy_pipe_tx_datak
                ]
                self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 2048, cd="phy_pipe_half")

    def do_exit(self, vns):
        if hasattr(self, "analyzer"):
            self.analyzer.export_csv(vns, "test/analyzer.csv")


def main():
    platform = Platform(toolchain="vivado")
    soc = USBSoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="test/csr.csv")
    vns = builder.build()
    soc.do_exit(vns)


if __name__ == "__main__":
    main()
