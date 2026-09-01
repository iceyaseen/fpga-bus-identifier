// ============================================================
//  fifo.v - plain-flip-flop FIFO between edge_capture and the UART.
//
//  DEPTH=64 x WIDTH=40 = 2560 flip-flops, which leaves headroom
//  inside the GW1NR-9C's ~6480-FF budget alongside the rest of the
//  design (256-deep would need 10240 FFs - more than the whole chip
//  has, before counting anything else). Swap for BSRAM-backed
//  storage later to go deeper without spending general-purpose FFs.
//
//  PTR_W is set by hand to log2(DEPTH), same convention as the
//  hand-picked counter widths elsewhere in this project (see
//  debounce's 19-bit count) - if you change DEPTH, update PTR_W
//  to match (PTR_W = 6 for 64, 7 for 128, etc).
//
//  wr_en/wr_data must already be valid the cycle wr_en is high -
//  there's no separate 'ready' handshake. Pushing while full sets
//  the sticky overflow flag instead of corrupting the queue.
// ============================================================
module fifo_ff #(
    parameter DEPTH = 64,
    parameter WIDTH = 40,
    parameter PTR_W = 6
) (
    input  wire              clk,
    input  wire              ovf_clear,      // one-cycle pulse: clears overflow + high_water

    input  wire              wr_en,
    input  wire [WIDTH-1:0]  wr_data,
    output wire               full,

    input  wire               rd_en,
    output reg  [WIDTH-1:0]   rd_data,
    output wire                empty,

    output reg                 overflow,      // sticky until ovf_clear
    output reg  [PTR_W:0]      high_water     // largest occupancy seen, 0..DEPTH
);

    (* ram_style = "registers" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_W-1:0] wr_ptr;
    reg [PTR_W-1:0] rd_ptr;
    reg [PTR_W:0]   count;          // 0..DEPTH

    initial begin
        wr_ptr     = {PTR_W{1'b0}};
        rd_ptr     = {PTR_W{1'b0}};
        count      = {(PTR_W+1){1'b0}};
        rd_data    = {WIDTH{1'b0}};
        overflow   = 1'b0;
        high_water = {(PTR_W+1){1'b0}};
    end

    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    wire do_push = wr_en && !full;
    wire do_pop  = rd_en && !empty;

    // computed combinationally so high_water reflects this cycle's
    // actual resulting occupancy, not last cycle's stale count
    wire [PTR_W:0] next_count = (do_push && !do_pop) ? (count + 1'b1) :
                                 (do_pop && !do_push) ? (count - 1'b1) :
                                 count;

    always @(posedge clk) begin
        if (do_push) begin
            mem[wr_ptr] <= wr_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end

        if (do_pop) begin
            rd_data <= mem[rd_ptr];
            rd_ptr  <= rd_ptr + 1'b1;
        end

        count <= next_count;

        if (ovf_clear) begin
            overflow   <= 1'b0;
            high_water <= {(PTR_W+1){1'b0}};
        end else begin
            if (wr_en && full)
                overflow <= 1'b1;
            if (next_count > high_water)
                high_water <= next_count;
        end
    end

endmodule
