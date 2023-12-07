`timescale 1ns / 1ps




module tb;

reg clk;
reg rst;


localparam CYCLE = 10;

always begin
    #(CYCLE/2) clk = ~clk;
end

initial begin
    clk = 0;
    rst = 0;
    #(CYCLE);
    rst = 1;
    #(CYCLE);
    rst = 0;
end

localparam DATA_WIDTH = 64;
localparam ADDR_WIDTH = 5;

reg [ADDR_WIDTH-1:0] write_addr;
reg [DATA_WIDTH-1:0] write_data;
reg write_delete, write_en;
wire write_busy;


reg [DATA_WIDTH-1:0] cmp_data;
wire [ADDR_WIDTH-1:0] match_addr;
wire match;

reg enable;
reg [DATA_WIDTH-1:0] data;

initial begin
    #(10*CYCLE+CYCLE/2);

    write_addr <= 10;
    write_data <= 10;
    write_en <= 1;
    write_delete <= 0;
    #(CYCLE)
    write_addr <= 0;
    write_data <= 0;
    write_en <= 0;
    write_delete <= 0;
    #(20*CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    write_addr <= 14;
    write_data <= 14;
    write_en <= 1;
    write_delete <= 0;
    #(CYCLE)
    write_addr <= 0;
    write_data <= 0;
    write_en <= 0;
    write_delete <= 0;
    #(20*CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    cmp_data <= 10;
    #(CYCLE)
    cmp_data <= 0;
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    cmp_data <= 14;
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    data <= 1;
    #(CYCLE)
    #(CYCLE)
    enable <= data;
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE)
    #(CYCLE);
end



cam_srl #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .SLICE_WIDTH(4)
) cam_ins (
    .clk (clk),
    .rst (rst),
    .write_addr     (write_addr),
    .write_data     (write_data),
    .write_delete   (write_delete),
    .write_enable   (write_en),
    .write_busy     (write_busy),

    .compare_data   (cmp_data),
    .match_many     (),
    .match_single   (),
    .match_addr     (match_addr),
    .match          (match)
);


endmodule

