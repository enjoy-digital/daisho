from litex.build.generic_platform import *
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
