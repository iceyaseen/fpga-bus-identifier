// ============================================================
//  i2c_prescan.v - electrical pre-scan: before driving anything as an
//  I2C master, characterise each channel purely by watching pull-up
//  behaviour - no bus traffic at all.
//
//  Per channel, reports one of:
//    2'b00 HAS_OWN_PULLUP - reads high with OUR pull-up off
//    2'b01 NEEDS_OURS     - reads low with ours off, but a clean,
//                            stable high once ours is switched on
//    2'b10 HELD_LOW       - still reads low at some point even with
//                            our pull-up on (a device driving it, or
//                            a short)
//    2'b11 FLOATING       - genuinely unstable with no pull-up at all
//                            (repeated samples disagree) - nothing
//                            connected there, distinct from a line
//                            that reads a clean, stable low
//
//  Takes exclusive control of both pull-ups while busy (see top.v's
//  mux between this and the manual A/a/B/b pull-up commands) and
//  leaves them BOTH ON when done, since that is what an address scan
//  needs next.
//
//  report_pending/report_ack follow the same single-slot handshake as
//  top.v's rec_pending/rec_tx_done: this module is the SOLE driver of
//  report_pending/report_channel/report_category, top.v only ever
//  computes report_ack combinationally from its own TX arbiter state.
//  Two drivers on one pending reg is the exact bug that silently
//  killed an earlier version of this project's status reply - see
//  top.v's own comment on rec_pending for the full story.
// ============================================================
module i2c_prescan (
    input  wire clk,

    input  wire start,             // one-cycle pulse
    output reg  busy,

    output reg  pu_a_on,
    output reg  pu_b_on,
    input  wire pin_a_sync,
    input  wire pin_b_sync,

    output reg        report_pending,
    output reg        report_channel,   // 0 = A, 1 = B
    output reg  [1:0] report_category,
    input  wire        report_ack
);

    localparam [1:0] CAT_HAS_OWN_PULLUP = 2'd0,
                      CAT_NEEDS_OURS     = 2'd1,
                      CAT_HELD_LOW       = 2'd2,
                      CAT_FLOATING       = 2'd3;

    localparam [15:0] SETTLE_CYCLES        = 16'd2700;    // ~100us at 27MHz - RC settle after a pull-up change
    localparam [15:0] STABILITY_SAMPLE_GAP = 16'd270;     // ~10us between step-1 stability samples
    localparam [3:0]  STABILITY_SAMPLES    = 4'd5;
    localparam [15:0] WATCH_CYCLES         = 16'd27000;   // ~1ms held-low watch window with our pull-up on

    localparam [3:0] S_IDLE          = 4'd0,
                      S_PU_OFF_SETTLE = 4'd1,
                      S_PU_OFF_SAMPLE = 4'd2,
                      S_PU_ON_SETTLE  = 4'd3,
                      S_PU_ON_WATCH   = 4'd4,
                      S_CLASSIFY      = 4'd5,
                      S_REPORT_A      = 4'd6,
                      S_REPORT_B      = 4'd7;

    reg [3:0]  state      = S_IDLE;
    reg [15:0] cnt        = 16'd0;
    reg [3:0]  sample_idx = 4'd0;

    reg first_sample_a = 1'b0, first_sample_b = 1'b0;   // step-1's very first sample: high = has own pull-up
    reg stable_a       = 1'b1, stable_b       = 1'b1;   // stays 1 unless a step-1 sample disagrees with the last
    reg last_a         = 1'b0, last_b         = 1'b0;
    reg ever_low_a     = 1'b0, ever_low_b     = 1'b0;   // set if step-2's watch window ever sees a low sample

    initial begin
        busy            = 1'b0;
        pu_a_on         = 1'b0;
        pu_b_on         = 1'b0;
        report_pending  = 1'b0;
        report_channel  = 1'b0;
        report_category = 2'd0;
    end

    always @(posedge clk) begin
        case (state)
        S_IDLE: begin
            if (start) begin
                busy    <= 1'b1;
                pu_a_on <= 1'b0;
                pu_b_on <= 1'b0;
                cnt     <= 16'd0;
                state   <= S_PU_OFF_SETTLE;
            end
        end

        S_PU_OFF_SETTLE: begin
            if (cnt == SETTLE_CYCLES - 1'b1) begin
                cnt            <= 16'd0;
                sample_idx     <= 4'd0;
                stable_a       <= 1'b1;
                stable_b       <= 1'b1;
                first_sample_a <= pin_a_sync;
                first_sample_b <= pin_b_sync;
                last_a         <= pin_a_sync;
                last_b         <= pin_b_sync;
                state          <= S_PU_OFF_SAMPLE;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end

        // sample STABILITY_SAMPLES times, STABILITY_SAMPLE_GAP cycles
        // apart, with pull-ups still off; any disagreement between
        // consecutive samples means this channel is genuinely floating
        S_PU_OFF_SAMPLE: begin
            if (cnt == STABILITY_SAMPLE_GAP - 1'b1) begin
                cnt <= 16'd0;
                if (pin_a_sync != last_a) stable_a <= 1'b0;
                if (pin_b_sync != last_b) stable_b <= 1'b0;
                last_a <= pin_a_sync;
                last_b <= pin_b_sync;
                if (sample_idx == STABILITY_SAMPLES - 1'b1) begin
                    pu_a_on    <= 1'b1;
                    pu_b_on    <= 1'b1;
                    ever_low_a <= 1'b0;
                    ever_low_b <= 1'b0;
                    cnt        <= 16'd0;
                    state      <= S_PU_ON_SETTLE;
                end else begin
                    sample_idx <= sample_idx + 1'b1;
                end
            end else begin
                cnt <= cnt + 1'b1;
            end
        end

        S_PU_ON_SETTLE: begin
            if (cnt == SETTLE_CYCLES - 1'b1) begin
                cnt   <= 16'd0;
                state <= S_PU_ON_WATCH;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end

        // watch for held-low with our pull-up on: any low sample
        // during this window means something is holding the line
        S_PU_ON_WATCH: begin
            if (!pin_a_sync) ever_low_a <= 1'b1;
            if (!pin_b_sync) ever_low_b <= 1'b1;
            if (cnt == WATCH_CYCLES - 1'b1) begin
                cnt   <= 16'd0;
                state <= S_CLASSIFY;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end

        S_CLASSIFY: begin
            report_channel <= 1'b0;
            if (!stable_a)
                report_category <= CAT_FLOATING;
            else if (first_sample_a)
                report_category <= CAT_HAS_OWN_PULLUP;
            else if (ever_low_a)
                report_category <= CAT_HELD_LOW;
            else
                report_category <= CAT_NEEDS_OURS;
            report_pending <= 1'b1;
            state          <= S_REPORT_A;
        end

        S_REPORT_A: begin
            if (report_ack) begin
                report_channel <= 1'b1;
                if (!stable_b)
                    report_category <= CAT_FLOATING;
                else if (first_sample_b)
                    report_category <= CAT_HAS_OWN_PULLUP;
                else if (ever_low_b)
                    report_category <= CAT_HELD_LOW;
                else
                    report_category <= CAT_NEEDS_OURS;
                report_pending <= 1'b1;
                state          <= S_REPORT_B;
            end
        end

        S_REPORT_B: begin
            if (report_ack) begin
                report_pending <= 1'b0;
                busy           <= 1'b0;
                state          <= S_IDLE;
            end
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
