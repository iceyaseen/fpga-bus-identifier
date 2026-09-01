// ============================================================
//  fifo.v - plain-flip-flop FIFO between edge_capture and the UART.
//
//  DEPTH is a real top-level parameter now (see top.v's FIFO_DEPTH).
//  At WIDTH=40, DEPTH x 40 flip-flops get spent on this FIFO alone -
//  128 is 5120 FFs, comfortably inside the GW1NR-9C's ~6480-FF total
//  budget alongside the rest of the design (~90 FFs for everything
//  else, last measured). 256 would need 10240 FFs, more than the
//  whole chip has - swap for BSRAM-backed storage to go that deep.
//
//  PTR_W used to be a separate hand-set parameter (matching this
//  project's usual "explicit, hand-picked widths" convention - see
//  debounce's 19-bit count), but that's an easy thing to forget to
//  update when DEPTH changes, and DEPTH is now something you're meant
//  to actually adjust. So PTR_W defaults to $clog2(DEPTH) instead -
//  still a plain `parameter` (Icarus's default elaboration mode
//  rejects a `localparam` inside the port list, "requires
//  SystemVerilog"), just never overridden at instantiation, so it
//  can't drift out of sync with DEPTH.
//
//  DEPTH must be a power of 2: wr_ptr/rd_ptr wrap via plain binary
//  overflow, not an explicit "wrap at DEPTH" reset, so a non-power-of-2
//  DEPTH would silently address entries beyond DEPTH-1 without ever
//  reusing them correctly.
//
//  wr_en/wr_data must already be valid the cycle wr_en is high -
//  there's no separate 'ready' handshake. Pushing while full sets
//  the sticky overflow flag instead of corrupting the queue.
// ============================================================
module fifo_ff #(
    parameter DEPTH = 128,
    parameter WIDTH = 40,
    parameter PTR_W = $clog2(DEPTH)
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
