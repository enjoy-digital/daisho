from litex.build.generic_platform import *
from litex.boards.platforms import kc705

_usb3_io = [
    # HiTechGlobal USB3.0 FMC P1 connector
    ("usb_clkout", 0, Pins("HPC:LA00_CC_P"), IOStandard("LVCMOS25")),
    ("usb_reset_n", 0, Pins("HPC:LA32_N"), IOStandard("LVCMOS25")),
    ("usb_ulpi", 0,
        Subsignal("clk", Pins("HPC:HA01_CC_P")),
        Subsignal("data", Pins(
            "HPC:HA11_N", "HPC:HA13_N", "HPC:HA15_N", "HPC:HA12_N",
            "HPC:HA14_N", "HPC:HA15_P", "HPC:HA16_P", "HPC:HA12_P")),
        Subsignal("dir", Pins("HPC:HA14_P")),
        Subsignal("stp", Pins("HPC:HA16_N")),
        Subsignal("nxt", Pins("HPC:HA13_P")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_ctrl", 0,
        Subsignal("phy_reset_n", Pins("HPC:LA12_P")),
        Subsignal("tx_detrx_lpbk", Pins("HPC:LA29_N")),
        Subsignal("tx_elecidle", Pins("HPC:LA27_P")),
        Subsignal("power_down", Pins("HPC:LA12_N", "HPC:LA13_P")),
        Subsignal("tx_oneszeros", Pins("HPC:LA27_N")),
        Subsignal("tx_deemph", Pins("HPC:LA31_P", "HPC:LA28_N")),
        Subsignal("tx_margin", Pins("HPC:LA30_P", "HPC:LA30_N")),
        Subsignal("tx_swing", Pins("HPC:LA29_P")),
        Subsignal("rx_polarity", Pins("HPC:LA16_N")),
        Subsignal("rx_termination", Pins("HPC:LA13_N")),
        Subsignal("rate", Pins("HPC:LA28_P")),
        Subsignal("elas_buf_mode", Pins("HPC:LA15_N")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_status", 0,
        Subsignal("rx_elecidle", Pins("HPC:LA11_N")),
        Subsignal("rx_status", Pins("HPC:LA14_P", "HPC:LA15_P", "HPC:LA14_N")),
        Subsignal("phy_status", Pins("HPC:LA16_P")),
        Subsignal("pwr_present", Pins("HPC:LA32_P")),
        IOStandard("LVCMOS25"),
    ),
    ("usb_pipe_data", 0,
        Subsignal("rx_clk", Pins("HPC:LA01_CC_P")),
        Subsignal("rx_valid", Pins("HPC:LA11_P")),
        Subsignal("rx_data", Pins(
            "HPC:LA10_N", "HPC:LA10_P", "HPC:LA09_N", "HPC:LA09_P",
            "HPC:LA07_N", "HPC:LA08_N", "HPC:LA05_N", "HPC:LA03_N",
            "HPC:LA06_N", "HPC:LA02_P", "HPC:LA06_P", "HPC:LA04_P",
            "HPC:LA03_P", "HPC:LA08_P", "HPC:LA07_P", "HPC:LA04_N")),
        Subsignal("rx_datak", Pins("HPC:LA02_N", "HPC:LA05_P")),

        Subsignal("tx_clk", Pins("HPC:LA20_P")),
        Subsignal("tx_data", Pins(
            "HPC:LA24_N", "HPC:LA26_N", "HPC:LA24_P", "HPC:LA26_P",
            "HPC:LA25_N", "HPC:LA21_N", "HPC:LA25_P", "HPC:LA21_P",
            "HPC:LA22_P", "HPC:LA22_N", "HPC:LA19_N", "HPC:LA23_N",
            "HPC:LA23_P", "HPC:LA18_CC_P", "HPC:LA19_P", "HPC:LA20_N")),
        Subsignal("tx_datak", Pins("HPC:LA18_CC_N", "HPC:LA17_CC_P")),
        IOStandard("LVCMOS25"),
    ),
]


class Platform(kc705.Platform):
    def __init__(self, *args, **kwargs):
        kc705.Platform.__init__(self, *args, **kwargs)
        self.add_extension(_usb3_io)

        self.add_platform_command("""
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 2.5 [current_design]
""")

