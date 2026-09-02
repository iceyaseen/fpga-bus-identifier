// ============================================================
//  UART transmitter - 8N1, LSB first, TX idles high.
//  Reusable: give it a byte and a one-cycle `start` pulse, it
//  shifts out start bit (0) + data[7:0] LSB-first + stop bit (1).
// ============================================================
module uart_tx #(
    parameter [15:0] CLKS_PER_BIT = 16'd234   // 27_000_000 / 115_200 = 234.375 -> 234
)(
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx   = 1'b1,     // idle high - a permanent 0 here looks like a break
    output wire       busy
);

    // frame = {stop, data[7:0], start} = {1, data7..data0, 0}, 10 bits.
    // shifted out LSB-first (bit0 first), so bit0=start, bit1=data0, ...
    // bit8=data7, bit9=stop - exactly the order a UART frame needs.
    reg  [9:0]  shift_reg = 10'b11_1111_1111;
    reg  [3:0]  bit_idx   = 4'd0;      // which of the 10 frame bits is on the line
    reg  [15:0] clk_cnt   = 16'd0;
    reg         busy_r    = 1'b0;

    // busy must go high in the SAME cycle as start, otherwise the caller
    // looks one cycle too early, sees busy=0, and drops the byte
    // (the exact bug that cost hours in the SPI module - same fix here)
    assign busy = busy_r | start;

    always @(posedge clk) begin
        if (!busy_r) begin
            if (start) begin
                shift_reg <= {1'b1, data, 1'b0};
                tx        <= 1'b0;          // drive the start bit immediately
                bit_idx   <= 4'd0;
                clk_cnt   <= 16'd0;
                busy_r    <= 1'b1;
            end
        end else begin
            if (clk_cnt == CLKS_PER_BIT - 1'b1) begin
                clk_cnt <= 16'd0;
                if (bit_idx == 4'd9) begin
                    busy_r <= 1'b0;          // stop bit's period just finished
                end else begin
                    bit_idx   <= bit_idx + 1'b1;
                    tx        <= shift_reg[1];             // next bit to send
                    shift_reg <= {1'b1, shift_reg[9:1]};    // shift right, pad with idle
                end
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end
    end

endmodule
