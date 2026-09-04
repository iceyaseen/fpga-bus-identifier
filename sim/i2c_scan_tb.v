`timescale 1ns/1ps
// ============================================================
//  Testbench for the I2C master + address scanner (Part B4).
//
//  - Instantiates top unmodified (its RTL, not a re-implementation),
//    with a NARROWED address range (0x08-0x10) so the sweep finishes
//    in a reasonable simulation time - the scanning MECHANISM is what
//    this proves, not the specific address value, and 9 addresses
//    exercises it exactly as thoroughly as the full 112 would.
//  - A simulated I2C slave sits on the two probe pins: it ACKs
//    exactly one configured address (0x0C, safely inside the
//    narrowed range) and NACKs every other address it sees.
//  - A `pullup()` primitive on each probe pin stands in for the
//    PCB's external 4.7k pull-up (itself out of scope for a digital
//    testbench) - both the FPGA and the simulated slave can pull
//    either line low; whichever releases first, the pullup brings it
//    back to a defined high, exactly like the real open-drain bus.
//  - Independently decodes the master's own SDA/SCL output for
//    START/STOP timing (SDA falling/rising while SCL is high) - not
//    just trusting the scan's own reported result - and separately
//    decodes every ADDR_HIT frame off the UART line to confirm
//    EXACTLY one address (0x0C) was reported found.
// ============================================================
module i2c_scan_tb;

    localparam CLKS_PER_BIT_TB = 29;    // matches uart_tx/uart_rx's default (921600 baud)

    reg clk = 1'b0;
    always #1 clk = ~clk;

    reg btn1 = 1'b1;
    reg btn2 = 1'b1;

    reg  uart_rx_line = 1'b1;           // testbench acts as the host, driving into the DUT
    wire uart_tx_line;                  // DUT's tx, decoded by the testbench

    wire ctrl_a, ctrl_b;
    wire [5:0] led;

    // ---- the open-drain I2C bus itself: probe_a=SDA, probe_b=SCL
    //      (default, unswapped mapping) ----
    wire probe_a, probe_b;
    pullup(probe_a);
    pullup(probe_b);

    initial begin
        $dumpfile("i2c_scan_tb.vcd");
        $dumpvars(0, uart_tx_line, probe_a, probe_b, led);
    end

    top #(
        .I2C_ADDR_FIRST (8'h08),
        .I2C_ADDR_LAST  (8'h10)
    ) dut (
        .clk      (clk),
        .btn1     (btn1),
        .btn2     (btn2),
        .uart_rx  (uart_rx_line),
        .uart_tx  (uart_tx_line),
        .ctrl_a   (ctrl_a),
        .ctrl_b   (ctrl_b),
        .led      (led),
        .probe_a  (probe_a),
        .probe_b  (probe_b)
    );

    // ============================================================
    //  Simulated I2C slave: ACKs exactly SLAVE_ACK_ADDRESS, NACKs
    //  (releases SDA, does nothing) every other address.
    // ============================================================
    localparam [6:0] SLAVE_ACK_ADDRESS = 7'h0C;

    reg slave_sda_drive_low = 1'b0;
    assign probe_a = slave_sda_drive_low ? 1'b0 : 1'bz;

    integer i;
    reg [7:0] rx_byte;
    reg [6:0] rx_addr;

    initial begin
        slave_sda_drive_low = 1'b0;
        forever begin
            @(negedge probe_a);
            if (probe_b !== 1'b1) begin
                // SCL wasn't high - not a real START, just a data bit
                // changing during some other transaction's low phase
                // (e.g. our own STOP sequence's SDA-low step); ignore
            end else begin
                rx_byte = 8'h00;
                for (i = 0; i < 8; i = i + 1) begin
                    @(posedge probe_b);
                    rx_byte = {rx_byte[6:0], probe_a};
                end
                rx_addr = rx_byte[7:1];

                @(negedge probe_b);   // master released SDA for the ACK bit, SCL now low
                if (rx_addr == SLAVE_ACK_ADDRESS) begin
                    slave_sda_drive_low = 1'b1;
                end
                @(posedge probe_b);   // master samples ACK/NACK during this high phase
                @(negedge probe_b);   // master drops SCL again, ACK phase done
                slave_sda_drive_low = 1'b0;
            end
        end
    end

    // ============================================================
    //  Independent protocol-timing checks on the master's own SDA/
    //  SCL output - decoded exactly the way a logic analyser would,
    //  not trusting the scan's own reported result alone.
    // ============================================================
    integer start_count   = 0;
    integer stop_count    = 0;

    // START: SDA falls while SCL is high
    always @(negedge probe_a) begin
        if (probe_b === 1'b1) begin
            start_count = start_count + 1;
        end
    end

    // STOP: SDA rises while SCL is high
    always @(posedge probe_a) begin
        if (probe_b === 1'b1) begin
            stop_count = stop_count + 1;
        end
    end

    // ---- bit-bang a byte into the DUT's uart_rx, host-side format ----
    task send_uart_byte(input [7:0] data);
        integer j;
        begin
            uart_rx_line = 1'b0;                         // start bit
            repeat (CLKS_PER_BIT_TB) @(posedge clk);
            for (j = 0; j < 8; j = j + 1) begin
                uart_rx_line = data[j];
                repeat (CLKS_PER_BIT_TB) @(posedge clk);
            end
            uart_rx_line = 1'b1;                          // stop bit
            repeat (CLKS_PER_BIT_TB) @(posedge clk);
        end
    endtask

    // ---- bit-bang one byte back off the DUT's uart_tx line ----
    task get_uart_byte(output [7:0] b);
        integer j;
        begin
            @(negedge uart_tx_line);
            repeat (CLKS_PER_BIT_TB * 3 / 2) @(posedge clk);   // half start-bit + one full bit -> mid data-bit0
            for (j = 0; j < 8; j = j + 1) begin
                b[j] = uart_tx_line;
                repeat (CLKS_PER_BIT_TB) @(posedge clk);
            end
        end
    endtask

    // ============================================================
    //  Background receiver: decodes uart_tx_line, frames every
    //  message type by marker, validates every checksum, and records
    //  ADDR_HIT/PRESCAN_RESULT/ACK contents for the checks below.
    // ============================================================
    localparam REC_MARKER      = 8'hA5;
    localparam STAT_MARKER     = 8'hA6;
    localparam ACK_MARKER      = 8'hA7;
    localparam ERR_MARKER      = 8'hA8;
    localparam PRESCAN_MARKER  = 8'hA9;
    localparam ADDRHIT_MARKER  = 8'hAA;

    integer checksum_errors = 0;
    integer ack_count       = 0;
    integer err_count       = 0;
    integer addrhit_count   = 0;
    integer prescan_count   = 0;
    reg [7:0] found_addrs [0:15];
    reg [7:0] last_ack_cmd;

    reg [7:0] rb;
    reg [7:0] fb0, fb1, fb2, fb3, fb4, fb5;
    reg [7:0] csum;

    initial begin
        forever begin
            get_uart_byte(rb);
            if (rb == REC_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                get_uart_byte(fb3); get_uart_byte(fb4); get_uart_byte(fb5);
                csum = REC_MARKER + fb0 + fb1 + fb2 + fb3 + fb4;
                if (csum !== fb5) begin
                    $display("FAIL: EDGE_RECORD checksum mismatch: computed %02h got %02h", csum, fb5);
                    checksum_errors = checksum_errors + 1;
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
                last_ack_cmd = fb0;
                ack_count    = ack_count + 1;
            end else if (rb == ERR_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                csum = ERR_MARKER + fb0 + fb1;
                if (csum !== fb2) begin
                    $display("FAIL: ERROR checksum mismatch: computed %02h got %02h", csum, fb2);
                    checksum_errors = checksum_errors + 1;
                end
                err_count = err_count + 1;
            end else if (rb == PRESCAN_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1); get_uart_byte(fb2);
                csum = PRESCAN_MARKER + fb0 + fb1;
                if (csum !== fb2) begin
                    $display("FAIL: PRESCAN_RESULT checksum mismatch: computed %02h got %02h", csum, fb2);
                    checksum_errors = checksum_errors + 1;
                end
                $display("t=%0t: PRESCAN_RESULT channel=%0d category=%0d", $time, fb0, fb1);
                prescan_count = prescan_count + 1;
            end else if (rb == ADDRHIT_MARKER) begin
                get_uart_byte(fb0); get_uart_byte(fb1);
                csum = ADDRHIT_MARKER + fb0;
                if (csum !== fb1) begin
                    $display("FAIL: ADDR_HIT checksum mismatch: computed %02h got %02h", csum, fb1);
                    checksum_errors = checksum_errors + 1;
                end else begin
                    $display("t=%0t: ADDR_HIT 0x%02h", $time, fb0);
                    found_addrs[addrhit_count] = fb0;
                    addrhit_count = addrhit_count + 1;
                end
            end
            // else: a stray byte with no matching marker - shouldn't
            // happen on this clean, uninjected wire
        end
    end

    // ============================================================
    //  Stimulus: reset, enable pull-ups, run a pre-scan, then run
    //  the address scan, then check everything.
    // ============================================================
    integer errors = 0;
    integer wait_i;   // separate from the slave model's own `i` (line ~76) - they run
                       // as concurrent processes and must not share a loop counter

    initial begin
        // let power-on/reset settle
        repeat (2000) @(posedge clk);

        // the pullup() primitives' very first x->1 settle at t=0 (before
        // anything is actually driven) can look like a spurious edge to
        // the START/STOP detectors above - discard whatever they saw
        // during power-on so only real, meaningful transitions count
        start_count = 0;
        stop_count  = 0;

        send_uart_byte(8'h52);   // 'R' - reset timestamp/overflow
        wait (ack_count == 1);

        send_uart_byte(8'h53);   // 'S' - start capture, so the scan's own
        wait (ack_count == 2);   // traffic gets captured too (Part 6)

        send_uart_byte(8'h45);   // 'E' - run the electrical pre-scan
        // pre-scan takes ~2*(2700+1350)+27000 =~ 34100 cycles; wait
        // generously, then confirm its own deferred ACK and 2 result
        // frames arrived
        wait_i = 0;
        while (ack_count < 3 && wait_i < 200_000) begin
            @(posedge clk);
            wait_i = wait_i + 1;
        end
        if (ack_count != 3) begin
            $display("FAIL: pre-scan's deferred ACK never arrived (ack_count=%0d)", ack_count);
            errors = errors + 1;
        end
        if (prescan_count != 2) begin
            $display("FAIL: expected 2 PRESCAN_RESULT frames, got %0d", prescan_count);
            errors = errors + 1;
        end

        send_uart_byte(8'h49);   // 'I' - run the address scan (0x08-0x10, 9 addresses)
        // wait for the scan's deferred ACK (4th ACK overall) - generous
        // margin over the ~9 * (~2970 txn + 60000 delay) =~ 567000
        // cycles this narrowed sweep should take (INTER_ADDR_DELAY is
        // now sized to drain the FIFO between addresses, not just let
        // a device settle - see i2c_scanner.v)
        wait_i = 0;
        while (ack_count < 4 && wait_i < 700_000) begin
            @(posedge clk);
            wait_i = wait_i + 1;
        end

        if (ack_count != 4) begin
            $display("FAIL: scan's deferred ACK never arrived (ack_count=%0d)", ack_count);
            errors = errors + 1;
        end
        if (last_ack_cmd !== 8'h49) begin
            $display("FAIL: last ACK should be for 'I' (0x49), got 0x%02h", last_ack_cmd);
            errors = errors + 1;
        end

        // ---- the actual verification the task asked for ----
        if (addrhit_count != 1) begin
            $display("FAIL: expected exactly 1 address found, got %0d", addrhit_count);
            errors = errors + 1;
        end else if (found_addrs[0] !== {1'b0, SLAVE_ACK_ADDRESS}) begin
            $display("FAIL: found address 0x%02h does not match the slave's 0x%02h",
                      found_addrs[0], SLAVE_ACK_ADDRESS);
            errors = errors + 1;
        end else begin
            $display("PASS: scan found exactly the one address the slave ACKs: 0x%02h", found_addrs[0]);
        end

        // one START and one STOP per address probed (9 addresses in
        // the narrowed 0x08-0x10 range)
        if (start_count != 9) begin
            $display("FAIL: expected 9 START conditions, observed %0d", start_count);
            errors = errors + 1;
        end
        if (stop_count != 9) begin
            $display("FAIL: expected 9 STOP conditions, observed %0d", stop_count);
            errors = errors + 1;
        end
        if (start_count == stop_count && start_count == 9) begin
            $display("PASS: %0d clean START/STOP condition pairs observed on the master's own SDA/SCL output",
                      start_count);
        end

        if (checksum_errors > 0) begin
            $display("FAIL: %0d checksum mismatch(es) on an uninjected wire", checksum_errors);
            errors = errors + checksum_errors;
        end
        if (err_count != 0) begin
            $display("FAIL: expected 0 ERROR frames, got %0d", err_count);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("---- all I2C scan checks passed: exactly one address found, timing verified ----");
        else
            $display("---- %0d error(s) found ----", errors);

        $finish;
    end

    // ---- watchdog ----
    initial begin
        #4_000_000;
        $display("---- WATCHDOG TIMEOUT: addrhit_count=%0d ack_count=%0d ----", addrhit_count, ack_count);
        $finish;
    end

endmodule
