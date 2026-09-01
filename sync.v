// ============================================================
//  sync.v - two-flip-flop synchroniser.
//  The probe signals come from another chip with no shared clock,
//  so every bit must cross two flops before any logic in this
//  clock domain looks at it, or metastability can propagate into
//  the edge detector and corrupt the capture.
// ============================================================
module sync #(
    parameter WIDTH = 2
) (
    input  wire             clk,
    input  wire [WIDTH-1:0] async_in,
    output reg  [WIDTH-1:0] sync_out
);

    reg [WIDTH-1:0] stage0;

    initial begin
        stage0   = {WIDTH{1'b0}};
        sync_out = {WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        stage0   <= async_in;
        sync_out <= stage0;
    end

endmodule
