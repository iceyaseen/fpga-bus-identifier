// ============================================================
//  top.v - FPGA communication checker
//  Board: Sipeed Tang Nano 9K (27 MHz clock)
//
//  Captures edges on two probe channels (probe_a/probe_b), timestamps
//  them against a free-running counter, and streams them to a host PC
//  over UART as framed, checksummed records. Also drives probe_a/
//  probe_b as an I2C master to scan for devices (Part B4) - see
//  i2c_master.v/i2c_prescan.v/i2c_scanner.v. See host.py for the
//  matching wire protocol and tangnano9k.cst for the pin mapping.
//
//  File layout follows strict declare-before-use order throughout:
//  Icarus's default elaboration mode rejects forward references (an
//  identifier used in an expression or port connection before any
//  `reg`/`wire` declaration of it has appeared earlier in the file),
//  so every block below is ordered so nothing it references was
//  declared later. Where two things are genuinely interdependent
//  (e.g. the TX arbiter's own state needs to exist before the I2C
//  submodules that report_ack/hit_ack feed back into it), the
//  declaration is pulled up near the top and the actual driving
//  logic stays lower down, next to the states it walks through.
// ============================================================

module top #(
    parameter NUM_CHANNELS  = 2,        // probe channel count - see the probe_driver instances below
    parameter FIFO_DEPTH    = 256,      // must be a power of 2 - see fifo.v; BRAM-backed at this depth
    // I2C address sweep range - real hardware always gets the full valid
    // 7-bit range (default); a testbench can narrow this for a fast,
    // still fully representative simulation. See i2c_scanner.v.
    parameter [7:0] I2C_ADDR_FIRST = 8'h08,
    parameter [7:0] I2C_ADDR_LAST  = 8'h77
) (
    input  wire       clk,        // 27 MHz, pin 52

    input  wire        btn1,       // active low, pin 3
    input  wire        btn2,       // active low, pin 4

    input  wire       uart_rx,    // from host (USB-serial TXD), pin 18
    output wire       uart_tx,    // to host (USB-serial RXD), pin 17

    output wire        ctrl_a,     // P-MOSFET gate, channel A pull-up - active low, pin 55
    output wire        ctrl_b,     // P-MOSFET gate, channel B pull-up - active low, pin 54

    output wire [5:0] led,

    inout  wire        probe_a,    // logic-analyser probe, channel A - pin 56
    inout  wire        probe_b     // logic-analyser probe, channel B - pin 57
);

    // ---- buttons: debounce + rising-edge detect --------------
    // Not consumed by anything yet (the display logic that used to read
    // these has been removed) - kept wired up for a future feature.
    wire b1_clean, b2_clean;

    debounce d1 (.clk(clk), .noisy(~btn1), .clean(b1_clean));
    debounce d2 (.clk(clk), .noisy(~btn2), .clean(b2_clean));

    reg b1_prev = 1'b0;
    reg b2_prev = 1'b0;
    wire b1_edge = b1_clean & ~b1_prev;
    wire b2_edge = b2_clean & ~b2_prev;

    always @(posedge clk) begin
        b1_prev <= b1_clean;
        b2_prev <= b2_clean;
    end

    // ---- UART: framed protocol only - no housekeeping text, no byte
    //      echo. Commands get a real ACK/ERROR/status reply instead ----
    wire       uart_tx_busy;
    reg        uart_tx_start = 1'b0;
    reg  [7:0] uart_tx_data  = 8'h00;
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;

    uart_tx tx_unit (
        .clk   (clk),
        .start (uart_tx_start),
        .data  (uart_tx_data),
        .tx    (uart_tx),
        .busy  (uart_tx_busy)
    );

    uart_rx rx_unit (
        .clk   (clk),
        .rx    (uart_rx),
        .data  (uart_rx_data),
        .valid (uart_rx_valid)
    );

    // low 6 bits of the last byte received, shown on the LEDs
    reg [7:0] rx_byte_latch = 8'h00;
    always @(posedge clk) begin
        if (uart_rx_valid) rx_byte_latch <= uart_rx_data;
    end

    // ---- TX arbiter state names + the regs it owns, declared up here
    // (ahead of everything below that needs to reference tx_state, or
    // needs report_ack/hit_ack, which are themselves computed from
    // tx_state) so every reference in this file is to an already-
    // declared symbol. The arbiter's actual always block (the logic
    // that drives uart_tx_data/tx_state) still lives much further
    // down, where it reads naturally alongside the states it walks
    // through and the *_pending flags it arbitrates between.
    //
    // 13 states now (IDLE + 2 each for REC/STAT/ACK/ERR/PRESCAN/ADDRHIT)
    // need 4 bits.
    localparam TX_IDLE          = 4'd0,
               TX_REC            = 4'd1,
               TX_REC_WAIT       = 4'd2,
               TX_STAT           = 4'd3,
               TX_STAT_WAIT      = 4'd4,
               TX_ACK            = 4'd5,
               TX_ACK_WAIT       = 4'd6,
               TX_ERR            = 4'd7,
               TX_ERR_WAIT       = 4'd8,
               TX_PRESCAN        = 4'd9,
               TX_PRESCAN_WAIT   = 4'd10,
               TX_ADDRHIT        = 4'd11,
               TX_ADDRHIT_WAIT   = 4'd12;

    reg [3:0] tx_state          = TX_IDLE;
    reg [2:0] rec_byte_idx      = 3'd0;   // EDGE_RECORD is 7 bytes: idx 0..6
    reg [2:0] stat_byte_idx     = 3'd0;   // STATUS_REPLY is 7 bytes: idx 0..6
    reg [1:0] ack_byte_idx      = 2'd0;   // ACK is 3 bytes: idx 0..2
    reg [1:0] err_byte_idx      = 2'd0;   // ERROR is 4 bytes: idx 0..3
    reg [1:0] prescan_byte_idx  = 2'd0;   // PRESCAN_RESULT is 4 bytes: idx 0..3
    reg [1:0] addrhit_byte_idx  = 2'd0;   // ADDR_HIT is 3 bytes: idx 0..2

    // combinational, computed straight from the TX arbiter's own state
    // (same pattern as rec_tx_done further down): the SOLE way
    // i2c_prescan/i2c_scanner learn "your frame was fully sent, go
    // ahead and clear your own pending flag" - top.v never touches
    // report_pending/hit_pending directly, for the same reason
    // rec_pending has exactly one driver (see the popper's comment).
    wire report_ack = (tx_state == TX_PRESCAN_WAIT) && !uart_tx_busy && (prescan_byte_idx == 2'd3);
    wire hit_ack    = (tx_state == TX_ADDRHIT_WAIT) && !uart_tx_busy && (addrhit_byte_idx == 2'd2);

    // ---- pull-up control (P-MOSFET gates, active low) --------
    // A P-channel MOSFET turns ON when its gate is pulled LOW, so driving
    // ctrl_a/ctrl_b LOW switches the corresponding channel's 4.7k pull-up
    // ON. Idling them HIGH keeps both pull-ups OFF, so probe_a/probe_b
    // stay high-impedance by default - a 3-pin device leaves one socket
    // position empty, and a fixed-on pull-up would leave that pin
    // floating into a driven state instead of a clean high-Z read.
    //
    // Two sources of control, muxed: the host's manual 'A'/'a'/'B'/'b'
    // commands (Part 2), and the electrical pre-scan (Part 3), which
    // needs exclusive control of both pull-ups while it runs and leaves
    // them ON when it finishes (what an address scan needs next). The
    // pre-scan wins the mux whenever it's busy; manual commands still
    // update their own register underneath it, so whatever the user last
    // asked for takes effect again the moment the pre-scan finishes.
    reg manual_pu_a_on = 1'b0;
    reg manual_pu_b_on = 1'b0;

    wire prescan_pu_a_on, prescan_pu_b_on;
    wire prescan_busy;

    // declared here (ahead of the i2c_scanner_unit/i2c_prescan_unit
    // instances below that connect to them, and the command-decode
    // always block further down that actually drives them) purely so
    // every reference in this file is to an already-declared symbol.
    reg start_prescan = 1'b0;   // one-cycle pulse
    reg start_scan    = 1'b0;   // one-cycle pulse

    wire eff_pu_a_on = prescan_busy ? prescan_pu_a_on : manual_pu_a_on;
    wire eff_pu_b_on = prescan_busy ? prescan_pu_b_on : manual_pu_b_on;

    assign ctrl_a = ~eff_pu_a_on;
    assign ctrl_b = ~eff_pu_b_on;

    // ---- probe drivers + I2C master --------------------------
    // probe_a/probe_b are inout: each gets its own probe_driver
    // instance (open-drain drive-low-or-release + synchronised
    // read-back - see probe_driver.v for why real builds need Gowin's
    // named IOBUF primitive there instead of a plain inout assign).
    //
    // Two named channels, not a generic NUM_CHANNELS-wide port: the
    // PCB has exactly two probe sockets wired to fixed pins (see
    // tangnano9k.cst). chan_a_sync/chan_b_sync pack into probe_in
    // below as {chan_b_sync, chan_a_sync} so channel indexing (bit 0 =
    // A, bit 1 = B) matches host.py's CH0/CH1 labelling, same as before.
    //
    // Passive by default: chan_a_drive_low/chan_b_drive_low are only
    // ever non-zero while the I2C master is actually running a
    // transaction (i2c_master_busy) - otherwise both channels stay
    // released, exactly the old always-passive behaviour, so capture
    // and waveform features are completely unaffected when the I2C
    // side of this design is never used.
    wire chan_a_sync, chan_b_sync;
    wire chan_a_drive_low, chan_b_drive_low;

    probe_driver u_probe_a (.clk(clk), .drive_low(chan_a_drive_low), .pin_sync(chan_a_sync), .pin(probe_a));
    probe_driver u_probe_b (.clk(clk), .drive_low(chan_b_drive_low), .pin_sync(chan_b_sync), .pin(probe_b));

    // already synchronised - each probe_driver instance does its own
    // 2-flop sync internally, so no separate sync stage is needed here
    wire [NUM_CHANNELS-1:0] probe_in = {chan_b_sync, chan_a_sync};

    // which physical channel is SDA and which is SCL is host-selectable
    // (Part 4's "which pin is which" question) rather than auto-detected:
    // simpler and more transparent to implement and debug, and the host
    // UI just re-runs the scan with the other mapping if the first one
    // finds nothing - see host.py and the README for the full reasoning.
    // Default (swap_i2c_pins=0): channel A = SDA, channel B = SCL.
    reg swap_i2c_pins = 1'b0;

    wire i2c_master_busy;
    wire i2c_scl_drive_low, i2c_sda_drive_low;
    wire i2c_scl_sync = swap_i2c_pins ? chan_a_sync : chan_b_sync;
    wire i2c_sda_sync = swap_i2c_pins ? chan_b_sync : chan_a_sync;

    assign chan_a_drive_low = i2c_master_busy ? (swap_i2c_pins ? i2c_scl_drive_low : i2c_sda_drive_low) : 1'b0;
    assign chan_b_drive_low = i2c_master_busy ? (swap_i2c_pins ? i2c_sda_drive_low : i2c_scl_drive_low) : 1'b0;

    wire        i2c_master_start;
    wire [7:0]  i2c_master_tx_byte;
    wire        i2c_master_done;
    wire        i2c_master_ack;

    i2c_master i2c_master_unit (
        .clk           (clk),
        .start         (i2c_master_start),
        .tx_byte       (i2c_master_tx_byte),
        .busy          (i2c_master_busy),
        .done          (i2c_master_done),
        .ack_received  (i2c_master_ack),
        .scl_drive_low (i2c_scl_drive_low),
        .sda_drive_low (i2c_sda_drive_low),
        .scl_sync      (i2c_scl_sync),
        .sda_sync      (i2c_sda_sync)
    );

    wire       scan_busy;
    wire       hit_pending;
    wire [7:0] hit_address;

    i2c_scanner #(.ADDR_FIRST(I2C_ADDR_FIRST), .ADDR_LAST(I2C_ADDR_LAST)) i2c_scanner_unit (
        .clk            (clk),
        .start          (start_scan),
        .busy           (scan_busy),
        .master_start   (i2c_master_start),
        .master_tx_byte (i2c_master_tx_byte),
        .master_done    (i2c_master_done),
        .master_ack     (i2c_master_ack),
        .hit_pending    (hit_pending),
        .hit_address    (hit_address),
        .hit_ack        (hit_ack)
    );

    wire        report_pending;
    wire        report_channel;
    wire [1:0]  report_category;

    i2c_prescan i2c_prescan_unit (
        .clk             (clk),
        .start           (start_prescan),
        .busy            (prescan_busy),
        .pu_a_on         (prescan_pu_a_on),
        .pu_b_on         (prescan_pu_b_on),
        .pin_a_sync      (chan_a_sync),
        .pin_b_sync      (chan_b_sync),
        .report_pending  (report_pending),
        .report_channel  (report_channel),
        .report_category (report_category),
        .report_ack      (report_ack)
    );

    // 'S'/'X'/'R'/'V' plus the new Part 2/4/5 commands. Any other byte
    // is unrecognised and gets a real ERROR reply, not a silent drop
    // or an echo of the byte back - and 'E'/'I' get one too if they
    // arrive while a pre-scan/scan is already running, distinguished
    // by ERR_BUSY from a genuinely unknown command (ERR_UNKNOWN_CMD).
    //
    //   'A'/'a' - channel A pull-up on/off      'N' - normal SDA/SCL mapping (A=SDA,B=SCL)
    //   'B'/'b' - channel B pull-up on/off      'W' - swapped mapping (A=SCL,B=SDA)
    //   'E'     - run the electrical pre-scan (Part 3)
    //   'I'     - run the I2C address scan (Part 5)
    //
    // 'E'/'I' are long-running, so unlike every other command here
    // their ACK is deferred (see the end of this same always block)
    // until the operation they kicked off actually finishes, not when
    // the command byte arrived - unblocking the host exactly when the
    // pre-scan/scan result frames are actually done arriving.
    reg capture_en = 1'b0;
    reg ts_reset   = 1'b0;   // one-cycle pulse: zeroes the timestamp counter + clears FIFO overflow
    reg status_req = 1'b0;   // one-cycle pulse: latch+queue a 'V' status reply
    reg ack_req    = 1'b0;   // one-cycle pulse: latch+queue an ACK
    reg [7:0] ack_cmd = 8'h00;
    reg err_req       = 1'b0;   // one-cycle pulse: latch+queue an ERROR reply
    reg [7:0] err_bad_byte = 8'h00;
    reg err_is_busy   = 1'b0;   // selects ERR_BUSY instead of ERR_UNKNOWN_CMD for the latched error

    reg deferred_ack_arm     = 1'b0;
    reg [7:0] deferred_ack_cmd = 8'h00;
    reg prescan_busy_prev    = 1'b0;
    reg scan_busy_prev       = 1'b0;

    always @(posedge clk) begin
        ts_reset      <= 1'b0;
        status_req    <= 1'b0;
        ack_req       <= 1'b0;
        err_req       <= 1'b0;
        start_prescan <= 1'b0;
        start_scan    <= 1'b0;

        prescan_busy_prev <= prescan_busy;
        scan_busy_prev    <= scan_busy;

        if (uart_rx_valid) begin
            case (uart_rx_data)
                8'h53: begin capture_en     <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'S'
                8'h58: begin capture_en     <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'X'
                8'h52: begin ts_reset       <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'R'
                8'h56: status_req <= 1'b1;                                                          // 'V'
                8'h41: begin manual_pu_a_on <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'A'
                8'h61: begin manual_pu_a_on <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'a'
                8'h42: begin manual_pu_b_on <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'B'
                8'h62: begin manual_pu_b_on <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'b'
                8'h4E: begin swap_i2c_pins  <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'N'
                8'h57: begin swap_i2c_pins  <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end  // 'W'
                8'h45: begin                                                                        // 'E'
                    if (!prescan_busy && !scan_busy) begin
                        start_prescan    <= 1'b1;
                        deferred_ack_cmd <= uart_rx_data;
                        deferred_ack_arm <= 1'b1;
                    end else begin
                        err_req      <= 1'b1;
                        err_bad_byte <= uart_rx_data;
                        err_is_busy  <= 1'b1;
                    end
                end
                8'h49: begin                                                                        // 'I'
                    if (!prescan_busy && !scan_busy) begin
                        start_scan       <= 1'b1;
                        deferred_ack_cmd <= uart_rx_data;
                        deferred_ack_arm <= 1'b1;
                    end else begin
                        err_req      <= 1'b1;
                        err_bad_byte <= uart_rx_data;
                        err_is_busy  <= 1'b1;
                    end
                end
                default: begin
                    err_req      <= 1'b1;
                    err_bad_byte <= uart_rx_data;
                    err_is_busy  <= 1'b0;
                end
            endcase
        end

        // fires once the pre-scan/scan this armed actually finishes -
        // watching for either busy signal's falling edge, never both
        // busy at once by construction (see the !prescan_busy &&
        // !scan_busy guard above)
        if (deferred_ack_arm && ((prescan_busy_prev && !prescan_busy) || (scan_busy_prev && !scan_busy))) begin
            ack_req          <= 1'b1;
            ack_cmd          <= deferred_ack_cmd;
            deferred_ack_arm <= 1'b0;
        end
    end

    wire        rec_valid;
    wire [31:0] rec_ts;
    wire [7:0]  rec_st;

    edge_capture #(.NUM_CHANNELS(NUM_CHANNELS)) capture_unit (
        .clk           (clk),
        .capture_en    (capture_en),
        .ts_reset      (ts_reset),
        .chan_sync     (probe_in),
        .rec_timestamp (rec_ts),
        .rec_state     (rec_st),
        .rec_valid     (rec_valid)
    );

    wire        fifo_full;
    wire        fifo_empty;
    wire        fifo_overflow;
    // width tracks FIFO_DEPTH automatically (fifo.v's own PTR_W does the
    // same $clog2 derivation) so this can't drift out of sync either
    wire [$clog2(FIFO_DEPTH):0] fifo_high_water;
    wire [39:0] fifo_rd_data;
    wire        fifo_rd_en;

    fifo_ff #(.DEPTH(FIFO_DEPTH), .WIDTH(40)) record_fifo (
        .clk        (clk),
        .ovf_clear  (ts_reset),
        .wr_en      (rec_valid),
        .wr_data    ({rec_ts, rec_st}),
        .full       (fifo_full),
        .rd_en      (fifo_rd_en),
        .rd_data    (fifo_rd_data),
        .empty      (fifo_empty),
        .overflow   (fifo_overflow),
        .high_water (fifo_high_water)
    );

    // ---- record popper: pulls one record out of the FIFO, latches
    //      it (rd_data is only valid one cycle after rd_en), then
    //      hands it to the TX arbiter below as rec_pending ----
    localparam POP_IDLE = 2'd0,
               POP_LATCH= 2'd1,
               POP_HOLD = 2'd2;

    reg [1:0]  pop_state   = POP_IDLE;
    reg [39:0] rec_latch   = 40'd0;
    reg        rec_pending = 1'b0;

    // combinational, not registered: if this were a reg (rd_en <= 1 in
    // POP_IDLE), the FIFO wouldn't see rd_en=1 until the cycle after
    // POP_IDLE, and POP_LATCH would then read rd_data one cycle too
    // early - a stale read that silently corrupts every record
    assign fifo_rd_en = (pop_state == POP_IDLE) && !fifo_empty && !rec_pending;

    // combinational flag: "the TX arbiter just sent this record's last
    // byte". rec_pending must have exactly ONE always-block driver: this
    // used to be written from here AND from the TX arbiter, which Yosys
    // correctly rejected as a multi-driver conflict and silently
    // resolved by tying it to a constant - so no status/record ever
    // actually got sent on real hardware even though it simulated fine
    // in Icarus. This wire lets the popper stay the sole driver while
    // still reacting to the arbiter finishing.
    wire rec_tx_done = (tx_state == TX_REC_WAIT) && !uart_tx_busy && (rec_byte_idx == 3'd6);

    always @(posedge clk) begin
        case (pop_state)
            POP_IDLE: begin
                if (fifo_rd_en) pop_state <= POP_LATCH;
            end
            POP_LATCH: begin
                rec_latch   <= fifo_rd_data;
                rec_pending <= 1'b1;
                pop_state   <= POP_HOLD;
            end
            POP_HOLD: begin
                if (rec_tx_done) begin
                    rec_pending <= 1'b0;
                    pop_state   <= POP_IDLE;
                end
            end
            default: pop_state <= POP_IDLE;
        endcase
    end

    // ---- reply state for status/ack/error. The actual set/clear of
    // each *_pending lives in the TX arbiter's always block below
    // (set at its tail, cleared in that message's own *_WAIT state),
    // never in a separate always block - see the big comment up at the
    // popper for why a second driver on the same reg is dangerous, not
    // just untidy: it's the exact bug that silently killed the status
    // reply the first time this project had one. ----
    reg        stat_pending    = 1'b0;
    reg        stat_overflow   = 1'b0;
    reg [15:0] stat_high_water = 16'd0;

    reg        ack_pending   = 1'b0;
    reg [7:0]  ack_cmd_latch = 8'h00;

    reg        err_pending    = 1'b0;
    reg [7:0]  err_bad_latch  = 8'h00;
    reg [7:0]  err_code_latch = 8'h01;   // ERR_UNKNOWN_CMD's value - that localparam isn't declared
                                          // until further down, and this file's own established
                                          // convention (see the TX arbiter state comment) is to
                                          // avoid forward references entirely

    // depth never changes at runtime, so no need to latch it - just
    // feed FIFO_DEPTH straight into the status byte lookup below
    localparam [15:0] STATUS_DEPTH = FIFO_DEPTH;

    // ---- framed protocol: [MARKER][PAYLOAD...][CHECKSUM]. The marker
    // byte alone tells the host both "this is a frame start" and,
    // since payload length is fixed per marker, exactly how many more
    // bytes to expect - no separate length byte needed. CHECKSUM is
    // the 8-bit sum (mod 256, via plain truncation on assignment to an
    // 8-bit function return - no multiply/divide anywhere) of the
    // marker plus every payload byte. If a host's checksum check fails
    // - false marker match, or real corruption - it should resync by
    // advancing exactly one byte and rescanning, not by skipping this
    // frame's whole claimed length. ----
    localparam [7:0] MARKER_EDGE    = 8'hA5;
    localparam [7:0] MARKER_STAT    = 8'hA6;
    localparam [7:0] MARKER_ACK     = 8'hA7;
    localparam [7:0] MARKER_ERR     = 8'hA8;
    localparam [7:0] MARKER_PRESCAN = 8'hA9;   // PRESCAN_RESULT (Part 3): channel, category
    localparam [7:0] MARKER_ADDRHIT = 8'hAA;   // ADDR_HIT (Part 5): one address that ACKed

    localparam [7:0] ERR_UNKNOWN_CMD = 8'h01;
    localparam [7:0] ERR_BUSY        = 8'h02;   // 'E'/'I' sent while a pre-scan/scan is already running

    // EDGE_RECORD - 7 bytes: marker, ts[31:24..7:0], state, checksum
    function [7:0] rec_checksum(input [39:0] rec);
        reg [7:0] sum;
        begin
            sum = MARKER_EDGE + rec[39:32] + rec[31:24] + rec[23:16] + rec[15:8] + rec[7:0];
            rec_checksum = sum;
        end
    endfunction

    function [7:0] rec_byte_lookup(input [39:0] rec, input [2:0] idx);
        case (idx)
            3'd0: rec_byte_lookup = MARKER_EDGE;
            3'd1: rec_byte_lookup = rec[39:32];    // timestamp[31:24]
            3'd2: rec_byte_lookup = rec[31:24];    // timestamp[23:16]
            3'd3: rec_byte_lookup = rec[23:16];    // timestamp[15:8]
            3'd4: rec_byte_lookup = rec[15:8];     // timestamp[7:0]
            3'd5: rec_byte_lookup = rec[7:0];      // channel state
            3'd6: rec_byte_lookup = rec_checksum(rec);
            default: rec_byte_lookup = 8'h00;
        endcase
    endfunction

    // STATUS_REPLY - 7 bytes: marker, overflow, high_water(2B), depth(2B), checksum
    function [7:0] stat_checksum(input ovf, input [15:0] hw, input [15:0] depth);
        reg [7:0] sum;
        begin
            sum = MARKER_STAT + {7'd0, ovf} + hw[15:8] + hw[7:0] + depth[15:8] + depth[7:0];
            stat_checksum = sum;
        end
    endfunction

    function [7:0] stat_byte_lookup(input ovf, input [15:0] hw, input [15:0] depth, input [2:0] idx);
        case (idx)
            3'd0: stat_byte_lookup = MARKER_STAT;
            3'd1: stat_byte_lookup = {7'd0, ovf};
            3'd2: stat_byte_lookup = hw[15:8];
            3'd3: stat_byte_lookup = hw[7:0];
            3'd4: stat_byte_lookup = depth[15:8];
            3'd5: stat_byte_lookup = depth[7:0];
            3'd6: stat_byte_lookup = stat_checksum(ovf, hw, depth);
            default: stat_byte_lookup = 8'h00;
        endcase
    endfunction

    // ACK - 3 bytes: marker, command byte, checksum
    function [7:0] ack_byte_lookup(input [7:0] cmd, input [1:0] idx);
        case (idx)
            2'd0: ack_byte_lookup = MARKER_ACK;
            2'd1: ack_byte_lookup = cmd;
            2'd2: ack_byte_lookup = MARKER_ACK + cmd;   // checksum
            default: ack_byte_lookup = 8'h00;
        endcase
    endfunction

    // PRESCAN_RESULT - 4 bytes: marker, channel, category, checksum
    function [7:0] prescan_byte_lookup(input chan, input [1:0] cat, input [1:0] idx);
        case (idx)
            2'd0: prescan_byte_lookup = MARKER_PRESCAN;
            2'd1: prescan_byte_lookup = {7'd0, chan};
            2'd2: prescan_byte_lookup = {6'd0, cat};
            2'd3: prescan_byte_lookup = MARKER_PRESCAN + {7'd0, chan} + {6'd0, cat};   // checksum
            default: prescan_byte_lookup = 8'h00;
        endcase
    endfunction

    // ADDR_HIT - 3 bytes: marker, address, checksum
    function [7:0] addrhit_byte_lookup(input [7:0] addr, input [1:0] idx);
        case (idx)
            2'd0: addrhit_byte_lookup = MARKER_ADDRHIT;
            2'd1: addrhit_byte_lookup = addr;
            2'd2: addrhit_byte_lookup = MARKER_ADDRHIT + addr;   // checksum
            default: addrhit_byte_lookup = 8'h00;
        endcase
    endfunction

    // ERROR - 4 bytes: marker, error code, offending byte, checksum
    function [7:0] err_byte_lookup(input [7:0] code, input [7:0] bad, input [1:0] idx);
        case (idx)
            2'd0: err_byte_lookup = MARKER_ERR;
            2'd1: err_byte_lookup = code;
            2'd2: err_byte_lookup = bad;
            2'd3: err_byte_lookup = MARKER_ERR + code + bad;   // checksum
            default: err_byte_lookup = 8'h00;
        endcase
    endfunction

    // single arbiter owns the one physical TX line. Priority, highest
    // first: capture records (losing these would silently corrupt a
    // capture), address-scan hits and pre-scan results (the actual
    // data the user is waiting on - losing one would silently drop a
    // found device or a channel's classification), error replies (you
    // want to know immediately if the host sent something
    // unrecognised), command acks, then status replies (already sent
    // only on request, fine to wait a beat).
    always @(posedge clk) begin
        uart_tx_start <= 1'b0;

        case (tx_state)
        TX_IDLE: begin
            if (rec_pending) begin
                rec_byte_idx <= 3'd0;
                tx_state     <= TX_REC;
            end else if (hit_pending) begin
                addrhit_byte_idx <= 2'd0;
                tx_state         <= TX_ADDRHIT;
            end else if (report_pending) begin
                prescan_byte_idx <= 2'd0;
                tx_state         <= TX_PRESCAN;
            end else if (err_pending) begin
                err_byte_idx <= 2'd0;
                tx_state     <= TX_ERR;
            end else if (ack_pending) begin
                ack_byte_idx <= 2'd0;
                tx_state     <= TX_ACK;
            end else if (stat_pending) begin
                stat_byte_idx <= 3'd0;
                tx_state      <= TX_STAT;
            end
        end

        TX_REC: begin
            uart_tx_data  <= rec_byte_lookup(rec_latch, rec_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_REC_WAIT;
        end

        TX_REC_WAIT: begin
            if (!uart_tx_busy) begin
                if (rec_byte_idx == 3'd6) begin
                    // rec_pending is cleared by the popper's own always
                    // block (it watches rec_tx_done, computed from
                    // tx_state/rec_byte_idx right here) - see the big
                    // comment up at the popper for why this can't also
                    // write rec_pending directly
                    tx_state <= TX_IDLE;
                end else begin
                    rec_byte_idx <= rec_byte_idx + 1'b1;
                    tx_state     <= TX_REC;
                end
            end
        end

        TX_STAT: begin
            uart_tx_data  <= stat_byte_lookup(stat_overflow, stat_high_water, STATUS_DEPTH, stat_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_STAT_WAIT;
        end

        TX_STAT_WAIT: begin
            if (!uart_tx_busy) begin
                if (stat_byte_idx == 3'd6) begin
                    stat_pending <= 1'b0;
                    tx_state     <= TX_IDLE;
                end else begin
                    stat_byte_idx <= stat_byte_idx + 1'b1;
                    tx_state      <= TX_STAT;
                end
            end
        end

        TX_ACK: begin
            uart_tx_data  <= ack_byte_lookup(ack_cmd_latch, ack_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_ACK_WAIT;
        end

        TX_ACK_WAIT: begin
            if (!uart_tx_busy) begin
                if (ack_byte_idx == 2'd2) begin
                    ack_pending <= 1'b0;
                    tx_state    <= TX_IDLE;
                end else begin
                    ack_byte_idx <= ack_byte_idx + 1'b1;
                    tx_state     <= TX_ACK;
                end
            end
        end

        TX_ERR: begin
            uart_tx_data  <= err_byte_lookup(err_code_latch, err_bad_latch, err_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_ERR_WAIT;
        end

        TX_ERR_WAIT: begin
            if (!uart_tx_busy) begin
                if (err_byte_idx == 2'd3) begin
                    err_pending <= 1'b0;
                    tx_state    <= TX_IDLE;
                end else begin
                    err_byte_idx <= err_byte_idx + 1'b1;
                    tx_state     <= TX_ERR;
                end
            end
        end

        TX_PRESCAN: begin
            uart_tx_data  <= prescan_byte_lookup(report_channel, report_category, prescan_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_PRESCAN_WAIT;
        end

        TX_PRESCAN_WAIT: begin
            if (!uart_tx_busy) begin
                if (prescan_byte_idx == 2'd3) begin
                    // report_pending is cleared by i2c_prescan itself
                    // (it watches report_ack, computed from tx_state/
                    // prescan_byte_idx right here) - same reasoning as
                    // rec_pending/rec_tx_done above
                    tx_state <= TX_IDLE;
                end else begin
                    prescan_byte_idx <= prescan_byte_idx + 1'b1;
                    tx_state         <= TX_PRESCAN;
                end
            end
        end

        TX_ADDRHIT: begin
            uart_tx_data  <= addrhit_byte_lookup(hit_address, addrhit_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_ADDRHIT_WAIT;
        end

        TX_ADDRHIT_WAIT: begin
            if (!uart_tx_busy) begin
                if (addrhit_byte_idx == 2'd2) begin
                    // hit_pending is cleared by i2c_scanner itself (it
                    // watches hit_ack, computed from tx_state/
                    // addrhit_byte_idx right here) - same reasoning as
                    // rec_pending/rec_tx_done above
                    tx_state <= TX_IDLE;
                end else begin
                    addrhit_byte_idx <= addrhit_byte_idx + 1'b1;
                    tx_state         <= TX_ADDRHIT;
                end
            end
        end

        default: tx_state <= TX_IDLE;
        endcase

        // set logic for stat/ack/err pending lives here (not in
        // separate always blocks) so each has exactly one driver - see
        // the big comment up at the popper for why. ack/err additionally
        // guard on "not already pending": if a second S/X/R (or a
        // second bad byte) arrives before the first ack/error finished
        // sending, it still takes effect immediately (capture_en etc.
        // are set unconditionally in the command decode block above),
        // it just won't get its own separate ack/error frame - in
        // practice these are ~260us round-trips and commands don't
        // arrive faster than a human or script can send them.
        if (status_req) begin
            stat_overflow   <= fifo_overflow;
            // zero-pad fifo_high_water (width tracks FIFO_DEPTH, see its
            // declaration above) out to the fixed 16-bit wire field -
            // a hardcoded pad count here would silently truncate the
            // top bit if FIFO_DEPTH ever needed a wider pointer
            stat_high_water <= {{(15 - $clog2(FIFO_DEPTH)){1'b0}}, fifo_high_water};
            stat_pending    <= 1'b1;
        end
        if (ack_req && !ack_pending) begin
            ack_cmd_latch <= ack_cmd;
            ack_pending   <= 1'b1;
        end
        if (err_req && !err_pending) begin
            err_bad_latch  <= err_bad_byte;
            err_code_latch <= err_is_busy ? ERR_BUSY : ERR_UNKNOWN_CMD;
            err_pending    <= 1'b1;
        end
    end

    // LEDs (active low): led[0] lit while capturing, led[1] lit (sticky)
    // once the FIFO has overflowed, led[5:2] still show the low nibble
    // of the last byte received over UART, as before
    assign led[0]   = ~capture_en;
    assign led[1]   = ~fifo_overflow;
    assign led[5:2] = ~rx_byte_latch[3:0];

endmodule


// ============================================================
//  ~10 ms debounce - same pattern as the earlier button test
// ============================================================
module debounce (
    input  wire clk,
    input  wire noisy,
    output reg  clean = 1'b0
);
    reg [18:0] count = 19'd0;      // ~10 ms at 27 MHz (270_000 needs 19 bits, not 18)
    reg        last  = 1'b0;

    always @(posedge clk) begin
        if (noisy != last) begin
            last  <= noisy;
            count <= 19'd0;
        end else if (count == 19'd270_000) begin
            clean <= last;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule
