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
//  - Independently bit-bangs the uart_tx line back into bytes, frames
//    all four message types by marker (EDGE_RECORD/STATUS_REPLY/ACK/
//    ERROR), validates every checksum, and decodes each 7-byte
//    EDGE_RECORD frame into a timestamp + state.
//  - Checks: same number of records as transitions driven, states
//    match in order, and consecutive timestamp deltas exactly match
//    the real clock-edge deltas between the driving events. The fixed
//    pipeline latency (2-stage sync + capture register) is constant,
//    so it cancels out of every delta - this proves cycle-accurate
//    capture without having to hand-derive that latency.
// ============================================================
module edge_capture_tb;

    localparam CLKS_PER_BIT_TB = 29;    // matches uart_tx/uart_rx's default (921600 baud)
    localparam ACK_ROUND_TRIP  = 10_000; // > one 3-byte ACK frame's transmit time (870 clocks)

    reg clk = 1'b0;
    always #1 clk = ~clk;

    reg btn1 = 1'b1;
    reg btn2 = 1'b1;

    reg  uart_rx_line = 1'b1;           // testbench acts as the host, driving into the DUT
    wire uart_tx_line;                  // DUT's tx, decoded by the testbench

    reg  [1:0] probe_drv = 2'b00;
    wire [1:0] probe;
    assign probe = probe_drv;           // DUT only ever drives 'z' onto this net in this version

    wire ctrl_a, ctrl_b;
    wire [5:0] led;

    initial begin
        $dumpfile("edge_capture_tb.vcd");
        $dumpvars(0, uart_tx_line, probe, led);
    end

    top dut (
        .clk      (clk),
        .btn1     (btn1),
        .btn2     (btn2),
        .uart_rx  (uart_rx_line),
        .uart_tx  (uart_tx_line),
        .ctrl_a   (ctrl_a),
        .ctrl_b   (ctrl_b),
        .led      (led),
        .probe_a  (probe[0]),
        .probe_b  (probe[1])
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
    //      frames all four message types by marker, and validates the
    //      checksum on every one - a bad checksum here means a real
    //      bug (RTL checksum computation vs. what actually got
    //      latched), not just "unlucky corruption", since this is a
    //      clean wire straight from the DUT with nothing injected. ----
    localparam REC_MARKER  = 8'hA5;
    localparam STAT_MARKER = 8'hA6;
    localparam ACK_MARKER  = 8'hA7;
    localparam ERR_MARKER  = 8'hA8;

    reg [31:0] dec_ts    [0:63];
    reg [7:0]  dec_state [0:63];
    integer    dec_count;
    integer    checksum_errors;
    integer    ack_count;
    integer    err_count;
    reg [7:0]  last_err_code;
    reg [7:0]  last_err_bad;

    reg [7:0] rb;
    reg [7:0] fb0, fb1, fb2, fb3, fb4, fb5;
    reg [7:0] csum;

    initial begin
        dec_count       = 0;
        checksum_errors = 0;
        ack_count       = 0;
        err_count       = 0;
        forever begin
            get_uart_byte(rb);
            if (rb == REC_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                get_uart_byte(fb3); get_uart_byte(fb4); get_uart_byte(fb5);
                csum = REC_MARKER + fb0 + fb1 + fb2 + fb3 + fb4;
                if (csum !== fb5) begin
                    $display("FAIL: EDGE_RECORD checksum mismatch: computed %02h got %02h", csum, fb5);
                    checksum_errors = checksum_errors + 1;
                end else begin
                    dec_ts[dec_count]    = {fb0, fb1, fb2, fb3};
                    dec_state[dec_count] = fb4;
                    dec_count = dec_count + 1;
                end
            end else if (rb == STAT_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                get_uart_byte(fb3); get_uart_byte(fb4); get_uart_byte(fb5);
                csum = STAT_MARKER + fb0 + fb1 + fb2 + fb3 + fb4;
                if (csum !== fb5) begin
                    $display("FAIL: STATUS_REPLY checksum mismatch: computed %02h got %02h", csum, fb5);
                    checksum_errors = checksum_errors + 1;
                end
            end else if (rb == ACK_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1);
                csum = ACK_MARKER + fb0;
                if (csum !== fb1) begin
                    $display("FAIL: ACK checksum mismatch: computed %02h got %02h", csum, fb1);
                    checksum_errors = checksum_errors + 1;
                end
                ack_count = ack_count + 1;
            end else if (rb == ERR_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                csum = ERR_MARKER + fb0 + fb1;
                if (csum !== fb2) begin
                    $display("FAIL: ERROR checksum mismatch: computed %02h got %02h", csum, fb2);
                    checksum_errors = checksum_errors + 1;
                end
                err_count     = err_count + 1;
                last_err_code = fb0;
                last_err_bad  = fb1;
            end
            // else: a stray byte with no matching marker - shouldn't
            // happen on this clean, uninjected wire, and since nothing
            // else is on it any more (no hello, no echo) there is
            // nothing legitimate left for it to be
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

        // 'R' then 'S' - reset the timestamp counter/overflow, start capturing.
        // Real usage (host.py's REPL) sends one command, waits to see the
        // ACK, then sends the next - a human or script pacing commands,
        // not blasting bytes back to back. ACK_ROUND_TRIP is generous
        // headroom for one 3-byte ACK frame (3*10*29 clocks) to fully
        // arrive before the next command goes out, so the "won't get a
        // separate ACK if the previous one is still in flight" tradeoff
        // (documented at the ack_req/ack_pending guard in top.v) never
        // actually triggers here - it's testing realistic usage, not
        // artificially defeating its own documented assumption.
        send_uart_byte(8'h52);   // 'R'
        repeat (ACK_ROUND_TRIP) @(posedge clk);
        send_uart_byte(8'h53);   // 'S'
        repeat (ACK_ROUND_TRIP) @(posedge clk);

        // unrecognised command byte - expect a real ERROR frame, not a
        // silent drop or an echo of 'Z' back
        send_uart_byte(8'h5A);   // 'Z'
        repeat (ACK_ROUND_TRIP) @(posedge clk);

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
        // 20 records x 6 bytes x 10 bits x 29 clocks/bit is already
        // ~35k clocks of serialised UART time by itself, so poll
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

        if (checksum_errors > 0) begin
            $display("FAIL: %0d checksum mismatch(es) on an uninjected wire", checksum_errors);
            errors = errors + checksum_errors;
        end

        // R then S were sent - expect exactly 2 ACKs, 0 errors
        if (ack_count != 2) begin
            $display("FAIL: expected 2 ACKs (for R and S), got %0d", ack_count);
            errors = errors + 1;
        end
        if (err_count != 1) begin
            $display("FAIL: expected 1 ERROR frame (for the bad 'Z' byte), got %0d", err_count);
            errors = errors + 1;
        end else begin
            if (last_err_code !== 8'h01) begin
                $display("FAIL: ERROR code: expected 01 got %02h", last_err_code);
                errors = errors + 1;
            end
            if (last_err_bad !== 8'h5A) begin
                $display("FAIL: ERROR bad_byte: expected 5A got %02h", last_err_bad);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("---- all %0d records matched expected state and timing, checksums clean, %0d ACKs, %0d ERROR frame(s) all correct ----",
                      exp_count, ack_count, err_count);
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
