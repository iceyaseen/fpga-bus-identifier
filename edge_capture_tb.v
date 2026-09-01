`timescale 1ns/1ps
// ============================================================
//  Testbench for the edge capture engine (sync + edge_capture +
//  fifo_ff + top.v's record streamer/arbiter), end to end.
//
//  - Instantiates top unmodified (its RTL, not a re-implementation).
//  - Bit-bangs 'R' then 'S' into uart_rx, exactly like a host would.
//  - Drives the probe pins with a known pattern: an I2C-like burst
//    (SCL-style toggling on probe[0], SDA-style data changes on
//    probe[1] while SCL is low) followed by a slow square wave on
//    both channels together. Every transition is recorded as ground
//    truth (channel state + the exact clock-edge count it happened on).
//  - Independently bit-bangs the uart_tx line back into bytes, scans
//    for the 0xA5 record marker (ignoring the periodic hello text and
//    anything else on the wire, same as host.py has to), and decodes
//    each 6-byte frame into a timestamp + state.
//  - Checks: same number of records as transitions driven, states
//    match in order, and consecutive timestamp deltas exactly match
//    the real clock-edge deltas between the driving events. The fixed
//    pipeline latency (2-stage sync + capture register) is constant,
//    so it cancels out of every delta - this proves cycle-accurate
//    capture without having to hand-derive that latency.
// ============================================================
module edge_capture_tb;

    localparam CLKS_PER_BIT_TB = 234;   // matches uart_tx/uart_rx's default

    reg clk = 1'b0;
    always #1 clk = ~clk;

    reg btn1 = 1'b1;
    reg btn2 = 1'b1;

    reg  uart_rx_line = 1'b1;           // testbench acts as the host, driving into the DUT
    wire uart_tx_line;                  // DUT's tx, decoded by the testbench

    reg  [1:0] probe_drv = 2'b00;
    wire [1:0] probe;
    assign probe = probe_drv;           // DUT only ever drives 'z' onto this net in this version

    wire lcd_rst, lcd_dc, lcd_cs, lcd_sclk, lcd_mosi;
    wire [5:0] led;

    initial begin
        $dumpfile("edge_capture_tb.vcd");
        $dumpvars(0, uart_tx_line, probe, led);
    end

    top #(.CYCLES_PER_MS(4)) dut (
        .clk      (clk),
        .btn1     (btn1),
        .btn2     (btn2),
        .uart_rx  (uart_rx_line),
        .uart_tx  (uart_tx_line),
        .lcd_rst  (lcd_rst),
        .lcd_dc   (lcd_dc),
        .lcd_cs   (lcd_cs),
        .lcd_sclk (lcd_sclk),
        .lcd_mosi (lcd_mosi),
        .led      (led),
        .probe    (probe)
    );

    // ---- bit-bang a byte into the DUT's uart_rx, host-side format ----
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            uart_rx_line = 1'b0;                         // start bit
            repeat (CLKS_PER_BIT_TB) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_line = data[i];
                repeat (CLKS_PER_BIT_TB) @(posedge clk);
            end
            uart_rx_line = 1'b1;                          // stop bit
            repeat (CLKS_PER_BIT_TB) @(posedge clk);
        end
    endtask

    // ---- bit-bang one byte back off the DUT's uart_tx line ----
    task get_uart_byte(output [7:0] b);
        integer i;
        begin
            @(negedge uart_tx_line);
            repeat (CLKS_PER_BIT_TB * 3 / 2) @(posedge clk);   // half start-bit + one full bit -> mid data-bit0
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = uart_tx_line;
                repeat (CLKS_PER_BIT_TB) @(posedge clk);
            end
        end
    endtask

    // ---- background receiver: continuously decodes uart_tx_line,
    //      frames 0xA5 records (6 bytes) and 0xA6 status replies
    //      (4 bytes), and discards everything else (hello text, echo) ----
    localparam REC_MARKER  = 8'hA5;
    localparam STAT_MARKER = 8'hA6;

    reg [31:0] dec_ts    [0:63];
    reg [7:0]  dec_state [0:63];
    integer    dec_count;

    reg [7:0] rb;
    reg [7:0] fb0, fb1, fb2, fb3, fb4;

    initial begin
        dec_count = 0;
        forever begin
            get_uart_byte(rb);
            if (rb == REC_MARKER) begin
                get_uart_byte(fb0);
                get_uart_byte(fb1);
                get_uart_byte(fb2);
                get_uart_byte(fb3);
                get_uart_byte(fb4);
                dec_ts[dec_count]    = {fb0, fb1, fb2, fb3};
                dec_state[dec_count] = fb4;
                dec_count = dec_count + 1;
            end else if (rb == STAT_MARKER) begin
                get_uart_byte(fb0);
                get_uart_byte(fb1);
                get_uart_byte(fb2);
            end
            // else: plain hello/echo text byte - ignore
        end
    end

    // ---- stimulus: drive the probe pins, remember ground truth ----
    reg [31:0] exp_clk   [0:63];
    reg [7:0]  exp_state [0:63];
    integer    exp_count;
    integer    total_cycles;

    task drive_probe(input integer wait_cycles, input [1:0] newval);
        begin
            repeat (wait_cycles) @(posedge clk);
            @(posedge clk);
            total_cycles         = total_cycles + wait_cycles + 1;
            probe_drv             = newval;
            exp_clk[exp_count]    = total_cycles;
            exp_state[exp_count]  = {6'b0, newval};
            exp_count             = exp_count + 1;
        end
    endtask

    integer i, errors;
    reg [31:0] exp_delta, dec_delta;

    initial begin
        exp_count    = 0;
        total_cycles = 0;
        errors       = 0;

        // let power-on/reset settle
        repeat (2000) @(posedge clk);

        // 'R' then 'S' - reset the timestamp counter/overflow, start capturing
        send_uart_byte(8'h52);   // 'R'
        send_uart_byte(8'h53);   // 'S'
        repeat (100) @(posedge clk);

        // ---- I2C-like burst: probe[0]=SCL-style clock, probe[1]=SDA-style
        //      data (data only changes while SCL is low, like real I2C) ----
        drive_probe( 40, 2'b01);   // wake toggle - probe_drv starts at 00, so the
                                    // very first driven value must differ from that
                                    // to actually be a transition
        drive_probe( 30, 2'b10);   // SDA=1 (start condition setup)
        drive_probe( 30, 2'b11);   // SCL=1 SDA=1
        drive_probe( 30, 2'b01);   // SCL=0
        drive_probe( 30, 2'b00);   // SDA=0 while SCL low (data bit)
        drive_probe( 30, 2'b01);   // SCL=1
        drive_probe( 30, 2'b00);   // SCL=0
        drive_probe( 30, 2'b10);   // SDA=1 while SCL low (data bit)
        drive_probe( 30, 2'b11);   // SCL=1
        drive_probe( 30, 2'b01);   // SCL=0
        drive_probe( 30, 2'b00);   // SDA=0 while SCL low (data bit)
        drive_probe( 30, 2'b01);   // SCL=1 (stop condition setup)
        drive_probe( 30, 2'b11);   // SDA=1 -> stop condition

        // idle gap between bursts
        drive_probe(500, 2'b00);

        // ---- slow square wave on both channels together ----
        drive_probe(300, 2'b11);
        drive_probe(300, 2'b00);
        drive_probe(300, 2'b11);
        drive_probe(300, 2'b00);
        drive_probe(300, 2'b11);
        drive_probe(300, 2'b00);

        // let the FIFO fully drain over the real UART before checking -
        // 20 records x 6 bytes x 10 bits x 234 clocks/bit is already
        // ~280k clocks of serialised UART time by itself, so poll
        // instead of guessing a fixed wait
        i = 0;
        while (dec_count < exp_count && i < 2_000_000) begin
            @(posedge clk);
            i = i + 1;
        end

        // ---- compare decoded records against ground truth ----
        if (dec_count != exp_count) begin
            $display("FAIL: expected %0d records, decoded %0d", exp_count, dec_count);
            errors = errors + 1;
        end

        for (i = 0; i < exp_count && i < dec_count; i = i + 1) begin
            if (dec_state[i] !== exp_state[i]) begin
                $display("FAIL: record %0d state mismatch: expected %b got %b",
                          i, exp_state[i], dec_state[i]);
                errors = errors + 1;
            end
            if (i > 0) begin
                exp_delta = exp_clk[i] - exp_clk[i-1];
                dec_delta = dec_ts[i]  - dec_ts[i-1];
                if (exp_delta !== dec_delta) begin
                    $display("FAIL: record %0d timing mismatch: expected delta %0d clocks, got %0d",
                              i, exp_delta, dec_delta);
                    errors = errors + 1;
                end
            end
        end

        if (errors == 0)
            $display("---- all %0d records matched expected state and timing ----", exp_count);
        else
            $display("---- %0d error(s) found ----", errors);

        $finish;
    end

    // ---- watchdog ----
    initial begin
        #50_000_000;
        $display("---- WATCHDOG TIMEOUT: decoded %0d of %0d expected records ----", dec_count, exp_count);
        $finish;
    end

endmodule
