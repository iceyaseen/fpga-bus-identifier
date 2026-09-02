// ============================================================
//  top.v - FPGA communication checker
//  Board: Sipeed Tang Nano 9K (27 MHz clock)
//
//  Captures edges on two probe channels (probe_a/probe_b), timestamps
//  them against a free-running counter, and streams them to a host PC
//  over UART as framed, checksummed records. See host.py for the
//  matching wire protocol and tangnano9k.cst for the pin mapping.
// ============================================================

module top #(
    parameter NUM_CHANNELS  = 2,        // probe channel count - see the probe_a/probe_b IOBUFs below
    parameter FIFO_DEPTH    = 128       // must be a power of 2 - see fifo.v
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

    // ---- pull-up control (P-MOSFET gates, active low) --------
    // A P-channel MOSFET turns ON when its gate is pulled LOW, so driving
    // ctrl_a/ctrl_b LOW switches the corresponding channel's 4.7k pull-up
    // ON. Idling them HIGH (the default here) keeps both pull-ups OFF, so
    // probe_a/probe_b stay high-impedance - a 3-pin device leaves one
    // socket position empty, and a fixed-on pull-up would leave that pin
    // floating into a driven state instead of a clean high-Z read. No
    // dynamic pull-up switching logic exists yet (that's part of the
    // still-to-be-built identification logic), so these are hardwired off
    // for now.
    assign ctrl_a = 1'b1;
    assign ctrl_b = 1'b1;

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

    // ---- edge capture engine -------------------------------
    // probe_a/probe_b are inout so active probing can be added later;
    // nothing drives them in this version.
    //
    // This pin's read side used to be `wire probe_in = probe;`, reading
    // straight off the inout port. That is legal Verilog, and simulates
    // fine (edge_capture_tb.v passed), but real synthesis broke it: with
    // synth_gowin, since probe_drive_en never varies (nothing drives
    // it), Yosys constant-folded probe's own drive expression at
    // compile time and used THAT as the value of probe_in too - i.e. it
    // treated "what we're (not) driving" as "what the pin reads",
    // instead of wiring the read back to the IOBUF's real O pin. Traced
    // in the synthesized netlist: probe_in ended up tied to a literal
    // constant 'z', completely disconnected from the actual pad voltage
    // - so no amount of external toggling on the probe pins could ever
    // have registered, with no error or warning pointing at it directly.
    //
    // Fix: instantiate Gowin's IOBUF primitive by name for real builds.
    // That primitive's O pin is a genuine hardware read-back path, not
    // something Yosys can fold away. `SYNTHESIS` is defined
    // automatically by `read_verilog` (confirmed: `yosys -h read_verilog`
    // says so, and the actual build command this project's toolchain
    // runs never passes -nosynthesis), so this reliably picks the real
    // primitive for every real build and the simple behavioral version
    // for iverilog simulation, which doesn't know about Gowin's IOBUF.
    //
    // Two named channels (probe_a/probe_b), not a generic NUM_CHANNELS-wide
    // port: the PCB has exactly two probe sockets wired to fixed pins (see
    // tangnano9k.cst). probe_in packs them as {probe_b, probe_a} so channel
    // indexing (bit 0 = A, bit 1 = B) matches host.py's CH0/CH1 labelling.
    reg                     probe_drive_en  = 1'b0;
    reg  [NUM_CHANNELS-1:0] probe_drive_val = {NUM_CHANNELS{1'b0}};
    wire [NUM_CHANNELS-1:0] probe_in;

`ifdef SYNTHESIS
    IOBUF u_probe_a_iobuf (
        .O   (probe_in[0]),
        .IO  (probe_a),
        .I   (probe_drive_val[0]),
        .OEN (~probe_drive_en)   // OEN=1 -> tri-stated, per Gowin's IOBUF model
    );
    IOBUF u_probe_b_iobuf (
        .O   (probe_in[1]),
        .IO  (probe_b),
        .I   (probe_drive_val[1]),
        .OEN (~probe_drive_en)
    );
`else
    assign probe_a  = probe_drive_en ? probe_drive_val[0] : 1'bz;
    assign probe_b  = probe_drive_en ? probe_drive_val[1] : 1'bz;
    assign probe_in = {probe_b, probe_a};
`endif

    // 'S'/'X'/'R'/'V' commands from the host. Any other byte is
    // unrecognised and gets a real ERROR reply, not a silent drop or
    // an echo of the byte back.
    reg capture_en = 1'b0;
    reg ts_reset   = 1'b0;   // one-cycle pulse: zeroes the timestamp counter + clears FIFO overflow
    reg status_req = 1'b0;   // one-cycle pulse: latch+queue a 'V' status reply
    reg ack_req    = 1'b0;   // one-cycle pulse: latch+queue an ACK for S/X/R
    reg [7:0] ack_cmd = 8'h00;
    reg err_req       = 1'b0;   // one-cycle pulse: latch+queue an ERROR reply
    reg [7:0] err_bad_byte = 8'h00;

    always @(posedge clk) begin
        ts_reset   <= 1'b0;
        status_req <= 1'b0;
        ack_req    <= 1'b0;
        err_req    <= 1'b0;
        if (uart_rx_valid) begin
            case (uart_rx_data)
                8'h53: begin capture_en <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'S'
                8'h58: begin capture_en <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'X'
                8'h52: begin ts_reset   <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'R'
                8'h56: status_req <= 1'b1;                                                       // 'V'
                default: begin
                    err_req      <= 1'b1;
                    err_bad_byte <= uart_rx_data;
                end
            endcase
        end
    end

    // TX arbiter state names + the regs it owns, declared up here (ahead
    // of the popper below, which needs TX_REC_WAIT/tx_state/rec_byte_idx
    // for rec_tx_done) so every reference in this file is to an
    // already-declared symbol - Icarus's default elaboration mode
    // rejects forward references even though plain Verilog allows them,
    // and depending on forward references made the multi-driver bug
    // below easy to miss. The arbiter's actual always block (the logic
    // that drives these regs) still lives further down, where it reads
    // naturally alongside the states it walks through.
    //
    // 9 states now (IDLE + 2 each for REC/STAT/ACK/ERR) need 4 bits.
    localparam TX_IDLE      = 4'd0,
               TX_REC       = 4'd1,
               TX_REC_WAIT  = 4'd2,
               TX_STAT      = 4'd3,
               TX_STAT_WAIT = 4'd4,
               TX_ACK       = 4'd5,
               TX_ACK_WAIT  = 4'd6,
               TX_ERR       = 4'd7,
               TX_ERR_WAIT  = 4'd8;

    reg [3:0] tx_state      = TX_IDLE;
    reg [2:0] rec_byte_idx  = 3'd0;   // EDGE_RECORD is 7 bytes: idx 0..6
    reg [2:0] stat_byte_idx = 3'd0;   // STATUS_REPLY is 7 bytes: idx 0..6
    reg [1:0] ack_byte_idx  = 2'd0;   // ACK is 3 bytes: idx 0..2
    reg [1:0] err_byte_idx  = 2'd0;   // ERROR is 4 bytes: idx 0..3

    wire [NUM_CHANNELS-1:0] probe_sync;
    sync #(.WIDTH(NUM_CHANNELS)) probe_sync_unit (
        .clk      (clk),
        .async_in (probe_in),
        .sync_out (probe_sync)
    );

    wire        rec_valid;
    wire [31:0] rec_ts;
    wire [7:0]  rec_st;

    edge_capture #(.NUM_CHANNELS(NUM_CHANNELS)) capture_unit (
        .clk           (clk),
        .capture_en    (capture_en),
        .ts_reset      (ts_reset),
        .chan_sync     (probe_sync),
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

    reg        err_pending   = 1'b0;
    reg [7:0]  err_bad_latch = 8'h00;

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
    localparam [7:0] MARKER_EDGE = 8'hA5;
    localparam [7:0] MARKER_STAT = 8'hA6;
    localparam [7:0] MARKER_ACK  = 8'hA7;
    localparam [7:0] MARKER_ERR  = 8'hA8;

    localparam [7:0] ERR_UNKNOWN_CMD = 8'h01;

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
    // capture), error replies (you want to know immediately if the
    // host sent something unrecognised), command acks, then status
    // replies (already sent only on request, fine to wait a beat).
    always @(posedge clk) begin
        uart_tx_start <= 1'b0;

        case (tx_state)
        TX_IDLE: begin
            if (rec_pending) begin
                rec_byte_idx <= 3'd0;
                tx_state     <= TX_REC;
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
            uart_tx_data  <= err_byte_lookup(ERR_UNKNOWN_CMD, err_bad_latch, err_byte_idx);
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
            err_bad_latch <= err_bad_byte;
            err_pending   <= 1'b1;
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
