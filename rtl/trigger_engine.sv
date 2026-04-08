// trigger_engine.sv — combinational trigger evaluation
//
// Compares probe_data against configured conditions each clock cycle.
// This module is PURELY COMBINATIONAL — the caller registers the output
// if needed, and maintains prev_data as a registered copy of the previous
// probe sample.
//
// Trigger modes:
//   2'b00  IMMEDIATE  — fires every cycle (arm-and-immediately-trigger)
//   2'b01  EQUALITY   — fires when (probe_data & mask) == (value & mask)
//   2'b10  RISING     — fires on 0→1 transition on any masked bit
//   2'b11  FALLING    — fires on 1→0 transition on any masked bit
//
// For RISING/FALLING, trigger_fire asserts on the cycle the transition
// is seen.  prev_data must hold the sample from the previous cycle.
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module trigger_engine #(
    parameter int PROBE_WIDTH = 16
) (
    input  logic [PROBE_WIDTH-1:0] probe_data,   // current sample
    input  logic [PROBE_WIDTH-1:0] prev_data,    // previous sample (registered externally)
    input  logic [1:0]             trig_mode,    // 00=IMM, 01=EQ, 10=RISE, 11=FALL
    input  logic [PROBE_WIDTH-1:0] trig_mask,    // which bits participate in trigger
    input  logic [PROBE_WIDTH-1:0] trig_value,   // reference value for EQ mode
    output logic                   trigger_fire  // combinational output
);

    // Masked versions
    logic [PROBE_WIDTH-1:0] curr_masked, prev_masked, val_masked;
    assign curr_masked = probe_data & trig_mask;
    assign prev_masked = prev_data  & trig_mask;
    assign val_masked  = trig_value & trig_mask;

    // Rising-edge bits: bits that were 0 in prev and are 1 in curr, within mask
    logic [PROBE_WIDTH-1:0] rising_bits;
    assign rising_bits = (~prev_data & probe_data) & trig_mask;

    // Falling-edge bits: bits that were 1 in prev and are 0 in curr, within mask
    logic [PROBE_WIDTH-1:0] falling_bits;
    assign falling_bits = (prev_data & ~probe_data) & trig_mask;

    always_comb begin
        case (trig_mode)
            2'b00:   trigger_fire = 1'b1;                        // IMMEDIATE
            2'b01:   trigger_fire = (curr_masked == val_masked);  // EQUALITY
            2'b10:   trigger_fire = |rising_bits;                 // RISING edge
            2'b11:   trigger_fire = |falling_bits;                // FALLING edge
            default: trigger_fire = 1'b0;
        endcase
    end

endmodule

`default_nettype wire
