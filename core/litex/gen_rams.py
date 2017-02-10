from litex.gen import *
from litex.gen.fhdl import verilog

# usb2_ep0in
class USB2EP0In(Module):
    def __init__(self):
        self.specials.mem = Memory(8, 64)
        write = self.mem.get_port(write_capable=True)
        read = self.mem.get_port()
        self.specials += write, read
        self.ios = {write.adr, write.dat_w, read.adr, read.dat_r}

usb2_ep0in = USB2EP0In()
conv_output = verilog.convert(usb2_ep0in, usb2_ep0in.ios, name="usb2_ep0in")
conv_output.write("usb2_ep0in.v")

# usb2_ep
class USB2EP(Module):
    def __init__(self):
        self.specials.mem = Memory(8, 1024)
        write = self.mem.get_port(write_capable=True, clock_domain="wr")
        read = self.mem.get_port(clock_domain="rd")
        self.specials += write, read
        self.ios = {write.adr, write.dat_w, read.adr, read.dat_r}

usb2_ep = USB2EP()
conv_output = verilog.convert(usb2_ep, usb2_ep.ios, name="usb2_ep")
conv_output.write("usb2_ep.v")

# usb3_ep0in
class USB3EP0In(Module):
    def __init__(self):
        self.specials.mem = Memory(32, 16)
        write = self.mem.get_port(write_capable=True)
        read = self.mem.get_port()
        self.specials += write, read
        self.ios = {write.adr, write.dat_w, read.adr, read.dat_r}

usb3_ep0in = USB3EP0In()
conv_output = verilog.convert(usb3_ep0in, usb3_ep0in.ios, name="usb3_ep0in")
conv_output.write("usb3_ep0in.v")

# usb3_ep
class USB3EP(Module):
    def __init__(self):
        self.specials.mem = Memory(32, 1024)
        write = self.mem.get_port(write_capable=True, clock_domain="wr")
        read = self.mem.get_port(clock_domain="rd")
        self.specials += write, read
        self.ios = {write.adr, write.dat_w, read.adr, read.dat_r}

usb3_ep = USB3EP()
conv_output = verilog.convert(usb3_ep, usb3_ep.ios, name="usb3_ep")
conv_output.write("usb3_ep.v")
