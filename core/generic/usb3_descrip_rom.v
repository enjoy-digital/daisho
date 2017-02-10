module usb3_descrip_rom(
	input clk,
	input [6:0] adr,
	output [31:0] dat_r
);



reg [31:0] mem[0:127];
reg [6:0] memadr;
always @(posedge clk) begin
	memadr <= adr;
end

assign dat_r = mem[memadr];

initial begin
	$readmemh("usb3_descrip_rom.init", mem);
end

endmodule
