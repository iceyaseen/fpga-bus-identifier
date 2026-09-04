// ============================================================
//  UART receiver - 8N1, LSB first.
//  Synchronises rx (it comes from an external chip, not this clock
//  domain), finds the falling edge of the start bit, waits half a
//  bit period to land in the middle of it, then samples every
//  following bit in the middle of its period - never at the edges,
//  where a real signal is still slewing.
// ============================================================
module uart_rx #(
    // 27_000_000 / 921_600 = 29.296875, rounded to 29 -> actual baud
    // 931_034 (27_000_000 / 29), +1.02% off the nominal 921600. Inside
    // the ~2% tolerance a UART needs, so no alternate rate required.
    parameter [15:0] CLKS_PER_BIT = 16'd29,
    parameter [15:0] HALF_BIT     = 16'd14     // CLKS_PER_BIT / 2, constant-folded, not a runtime divide
)(
    input  wire       clk,
    input  wire       rx,
    output reg  [7:0] data  = 8'h00,
    output reg        valid = 1'b0
);

    // 2-flop synchroniser - rx is asynchronous to clk
    reg rx_sync0 = 1'b1;
    reg rx_sync1 = 1'b1;
    reg rx_prev  = 1'b1;

    localparam RX_IDLE  = 2'd0,
               RX_START = 2'd1,
               RX_DATA  = 2'd2,
               RX_STOP  = 2'd3;

    reg [1:0]  rstate   = RX_IDLE;
    reg [15:0] clk_cnt  = 16'd0;
    reg [2:0]  bit_cnt  = 3'd0;
    reg [7:0]  rx_shift = 8'h00;

    always @(posedge clk) begin
        rx_sync0 <= rx;
        rx_sync1 <= rx_sync0;
        rx_prev  <= rx_sync1;

        valid <= 1'b0;      // one-cycle pulse by default

        case (rstate)

        // watch for the falling edge of the start bit
        RX_IDLE: begin
            if (rx_prev && !rx_sync1) begin
                rstate  <= RX_START;
                clk_cnt <= 16'd0;
            end
        end

        // wait half a bit period so later samples land mid-bit
        RX_START: begin
            if (clk_cnt == HALF_BIT - 1'b1) begin
                clk_cnt <= 16'd0;
                bit_cnt <= 3'd0;
                rstate  <= RX_DATA;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end

        // sample 8 data bits, one full bit period apart, LSB first
        RX_DATA: begin
            if (clk_cnt == CLKS_PER_BIT - 1'b1) begin
                clk_cnt  <= 16'd0;
                rx_shift <= {rx_sync1, rx_shift[7:1]};   // new bit into MSB, shift down
                if (bit_cnt == 3'd7) begin
                    rstate <= RX_STOP;
                end else begin
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end

        // land in the middle of the stop bit, then publish the byte
        RX_STOP: begin
            if (clk_cnt == CLKS_PER_BIT - 1'b1) begin
                clk_cnt <= 16'd0;
                data    <= rx_shift;
                valid   <= 1'b1;
                rstate  <= RX_IDLE;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end

        default: rstate <= RX_IDLE;

        endcase
    end

endmodule
