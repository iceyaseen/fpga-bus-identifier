`timescale 1ns/1ps
// ============================================================
//  Testbench for uart_tx / uart_rx.
//
//  Wires uart_tx straight into uart_rx (loopback) and pushes a set
//  of test bytes through, including the edge cases 0x00, 0xFF, 0x55
//  and 0xAA. Also decodes the raw tx line itself, independently of
//  uart_rx, so bit order/framing bugs in uart_rx can't hide behind
//  a matching bug in uart_tx.
//
//  CLKS_PER_BIT is shrunk to speed up simulation - only the ratio
//  between bit periods matters for this test, not the real baud rate.
// ============================================================
module uart_tb;

    localparam CLKS_PER_BIT = 16'd20;
    localparam HALF_BIT     = 16'd10;

    reg clk = 1'b0;
    always #1 clk = ~clk;

    reg        start = 1'b0;
    reg  [7:0] tx_data = 8'h00;
    wire       tx_line;
    wire       tx_busy;

    wire [7:0] rx_data;
    wire       rx_valid;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) dut_tx (
        .clk   (clk),
        .start (start),
        .data  (tx_data),
        .tx    (tx_line),
        .busy  (tx_busy)
    );

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT), .HALF_BIT(HALF_BIT)) dut_rx (
        .clk   (clk),
        .rx    (tx_line),
        .data  (rx_data),
        .valid (rx_valid)
    );

    // ---- independent decoder of the raw tx line, bypassing uart_rx ----
    reg [7:0] line_byte  = 8'h00;
    reg       line_ready = 1'b0;

    task decode_line;
        integer i;
        reg bit_val;
        begin
            line_ready = 1'b0;
            @(negedge tx_line);                            // start bit begins
            repeat (CLKS_PER_BIT * 3 / 2) @(posedge clk);   // land mid data-bit0 (half start-bit + one full bit)
            for (i = 0; i < 8; i = i + 1) begin
                bit_val = tx_line;
                line_byte[i] = bit_val;          // LSB first -> bit i of the byte
                repeat (CLKS_PER_BIT) @(posedge clk);       // one bit period to the next bit's middle
            end
            line_ready = 1'b1;
        end
    endtask

    // ---- test sequence ----
    reg [7:0] test_bytes [0:5];
    integer   idx;
    integer   errors = 0;

    initial begin
        test_bytes[0] = 8'h00;
        test_bytes[1] = 8'hFF;
        test_bytes[2] = 8'h55;
        test_bytes[3] = 8'hAA;
        test_bytes[4] = 8'h41;   // 'A'
        test_bytes[5] = 8'h0D;   // '\r'

        for (idx = 0; idx < 6; idx = idx + 1) begin
            fork
                decode_line;
                begin
                    @(posedge clk);
                    tx_data = test_bytes[idx];
                    start   = 1'b1;
                    @(posedge clk);
                    start   = 1'b0;
                end
            join

            wait (rx_valid == 1'b1);
            #1;
            $display("byte %0d: sent=0x%02h  uart_rx got=0x%02h  raw-line decode=0x%02h  %s",
                      idx, test_bytes[idx], rx_data, line_byte,
                      (rx_data == test_bytes[idx] && line_byte == test_bytes[idx]) ? "OK" : "MISMATCH");

            if (rx_data !== test_bytes[idx]) begin
                errors = errors + 1;
                $display("    ERROR: uart_rx output did not match what was sent");
            end
            if (line_byte !== test_bytes[idx]) begin
                errors = errors + 1;
                $display("    ERROR: raw tx line decode did not match what was sent");
            end

            wait (tx_busy == 1'b0);
            repeat (CLKS_PER_BIT * 3) @(posedge clk);   // idle gap between frames
        end

        if (errors == 0) begin
            $display("---- all %0d bytes round-tripped correctly ----", 6);
        end else begin
            $display("---- %0d error(s) found ----", errors);
        end
        $finish;
    end

    // ---- watchdog ----
    initial begin
        #200_000;
        $display("---- WATCHDOG TIMEOUT ----");
        $finish;
    end

endmodule
