// ============================================================
//  probe_driver.v - open-drain tristate driver + synchronised reader
//  for one probe channel. I2C is open-drain: this module can only
//  drive LOW or release to high impedance - it can never drive HIGH
//  (that would fight the device and the external pull-up). One
//  instance per channel (see top.v).
//
//  Same IOBUF-vs-behavioural split as the inline probe logic this
//  replaces used to have: Yosys constant-folds a plain
//  `assign pin = ...; wire in = pin;` pattern when synthesizing,
//  disconnecting the read-back from the real pad voltage (traced and
//  documented the hard way - see git history / top.v's older
//  comments), so real builds instantiate Gowin's IOBUF primitive by
//  name, whose O pin is a genuine hardware read-back path. Icarus
//  doesn't know that primitive, so simulation uses the simple
//  behavioural version instead. `SYNTHESIS` is defined automatically
//  by `read_verilog` for every real build.
// ============================================================
module probe_driver (
    input  wire clk,
    input  wire drive_low,   // 1 = actively pull the pin low; 0 = release (Hi-Z)
    output wire pin_sync,    // synchronised (2-flop) read of the pin level
    inout  wire pin
);

    wire raw_in;

`ifdef SYNTHESIS
    IOBUF u_iobuf (
        .O   (raw_in),
        .IO  (pin),
        .I   (1'b0),          // never used to drive high - OEN gates it off whenever released
        .OEN (~drive_low)     // OEN=1 -> tri-stated (per Gowin's IOBUF model), OEN=0 -> drives I (=0)
    );
`else
    assign pin    = drive_low ? 1'b0 : 1'bz;
    assign raw_in = pin;
`endif

    sync #(.WIDTH(1)) u_sync (
        .clk      (clk),
        .async_in (raw_in),
        .sync_out (pin_sync)
    );

endmodule
