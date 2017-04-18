Test of the USB3 IP Core from Daisho on a Xilinx device

In this repository we are testing USB3 IP Core from Daisho on a Xilinx device.

 * USB2 / ULPI working :) (vendor agnostic)
 * UDB3 / PIPE working :) (IDDR/ODDR and PLL specific to Xilinx)

This work was supported by TimVideos.us (https://code.timvideos.us) and with a generous loan of a USB3.0 protocol analyzer from the Daisho project.

The design currently uses a TUSB1310A for interfacing with the USB3.0 connector. Future plans include replacing this part using the high speed transceivers (GTPs) found in Artix-7/Kintex-7 FPGAs.

The current work is being done with;

 * Kintex 7 work - [kc705 - Xilinx Kintex-7 FPGA KC705 Evaluation Kit](https://www.xilinx.com/products/boards-and-kits/ek-k7-kc705-g.html)
 * Artix 7 work[1] - [Nexys Video - Nexys Video Artix-7 FPGA: Trainer Board for Multimedia Applications](http://store.digilentinc.com/nexys-video-artix-7-fpga-trainer-board-for-multimedia-applications/)

 * USB3.0 work - [HiTech Gloabl - 3-Port USB 3 FMC Module](https://hitechglobal.us/index.php?route=product/product&path=18&product_id=233).

The FMC module has 3 x USB3.0 ports. Two are connected via TUSB1310A ICs and the third is connected directly to the high speed transceivers.

[1]: As the Nexys Video has only a [LPC FMC connector](https://en.wikipedia.org/wiki/FPGA_Mezzanine_Card#LPC_vs._HPC), only a limited amount set of the functionality is avalible, but it is enough to prove the design also works on the Artix-7 FPGA.
