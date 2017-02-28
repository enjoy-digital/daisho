#!/usr/bin/env python3

import argparse

from litex.gen import *
from litex.build.generic_platform import *

from litex.soc.integration.builder import *
from litex.soc.interconnect.csr import *

from litex.gen.genlib.io import DifferentialInput, DifferentialOutput

from kc705_platform import Platform
from kc705_base import BaseSoC


class USBSoC(BaseSoC):
    def __init__(self, platform):
        BaseSoC.__init__(self, platform)

        # usb ios
        usb_clkout = platform.request("usb_clkout")
        usb_reset_n = platform.request("usb_reset_n")
        usb_ulpi = platform.request("usb_ulpi")
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
        self.specials += Instance("usb2_top",
            i_ext_clk=usb_clkout, # TODO
            i_reset_n=1, # TODO
            #o_reset_n_out=,

            i_phy_ulpi_clk=usb_ulpi.clk,
            io_phy_ulpi_d=usb_ulpi.data,
            i_phy_ulpi_dir=usb_ulpi.dir,
            o_phy_ulpi_stp=usb_ulpi.stp,
            i_phy_ulpi_nxt=usb_ulpi.nxt,

            i_opt_disable_all=0, # TODO
            i_opt_enable_hs=0, # TODO
            i_opt_ignore_vbus=0, # TODO
            #o_stat_connected=, # TODO
            #o_stat_fs=, # TODO
            #o_stat_hs=, # TODO
            #o_stat_configured=, # TODO

            i_buf_in_addr=0, # TODO
            i_buf_in_data=0, # TODO
            i_buf_in_wren=0, # TODO
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

            #o_err_crc_pid=, # TODO
            #o_err_crc_tok=, # TODO
            #o_err_crc_pkt=, # TODO
            #o_err_pid_out_of_seq=, # TODO
            #o_err_setup_pkt=, # TODO

            #o_dbg_frame_num=, # TODO
            #o_dbg_linestate= # TODO
        )
        platform.add_verilog_include_path(os.path.join("core"))
        platform.add_verilog_include_path(os.path.join("core", "usb2"))
        platform.add_source_dir(os.path.join("core", "usb2"))

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
