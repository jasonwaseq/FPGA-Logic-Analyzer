// config_regs.sv — configuration register bank
//
// Simple write-only register file accessed by uart_cmd_parser via
// (wr_addr, wr_data, wr_en).  Outputs are registered and held until
// the next write.  Reset restores all defaults.
//
// Register map (8-bit address):
//   0x00  TRIG_MODE       [1:0]   Trigger mode: 0=IMM,1=EQ,2=RISE,3=FALL
//   0x01  TRIG_MASK       [15:0]  Trigger channel mask
//   0x02  TRIG_VALUE      [15:0]  Trigger equality reference value
//   0x03  PRE_TRIG_DEPTH  [9:0]   Pre-trigger sample count (default 256)
//   0x04  POST_TRIG_DEPTH [9:0]   Post-trigger sample count (default 768)
//   0x05  RLE_EN          [0]     0=raw output, 1=RLE output
//   0x06  PROBE_MASK      [15:0]  Active probe channels (host display hint)
//
// Unknown addresses: write is silently ignored, read returns 0xDEAD_BEEF.
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module config_regs #(
    parameter int PROBE_WIDTH = 16,
    parameter int ADDR_BITS   = 10,    // matches BUFFER_DEPTH = 2^10 = 1024
    // Default depths
    parameter int DEF_PRE     = 256,
    parameter int DEF_POST    = 768
) (
    input  logic        clk,
    input  logic        rst_n,

    // Write port (from uart_cmd_parser)
    input  logic [7:0]  wr_addr,
    input  logic [31:0] wr_data,
    input  logic        wr_en,

    // Read port (for GET_REG command response)
    input  logic [7:0]  rd_addr,
    output logic [31:0] rd_data,

    // Decoded configuration outputs (to rest of design)
    output logic [1:0]              trig_mode,
    output logic [PROBE_WIDTH-1:0]  trig_mask,
    output logic [PROBE_WIDTH-1:0]  trig_value,
    output logic [ADDR_BITS-1:0]    pre_trig_depth,
    output logic [ADDR_BITS-1:0]    post_trig_depth,
    output logic                    rle_en,
    output logic [PROBE_WIDTH-1:0]  probe_mask
);

    // -----------------------------------------------------------------------
    // Register storage
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trig_mode       <= 2'b00;                    // IMMEDIATE
            trig_mask       <= {PROBE_WIDTH{1'b1}};       // all bits
            trig_value      <= {PROBE_WIDTH{1'b0}};
            pre_trig_depth  <= ADDR_BITS'(DEF_PRE);
            post_trig_depth <= ADDR_BITS'(DEF_POST);
            rle_en          <= 1'b0;                     // raw mode
            probe_mask      <= {PROBE_WIDTH{1'b1}};
        end else if (wr_en) begin
            case (wr_addr)
                8'h00: trig_mode       <= wr_data[1:0];
                8'h01: trig_mask       <= wr_data[PROBE_WIDTH-1:0];
                8'h02: trig_value      <= wr_data[PROBE_WIDTH-1:0];
                8'h03: pre_trig_depth  <= wr_data[ADDR_BITS-1:0];
                8'h04: post_trig_depth <= wr_data[ADDR_BITS-1:0];
                8'h05: rle_en          <= wr_data[0];
                8'h06: probe_mask      <= wr_data[PROBE_WIDTH-1:0];
                default: ; // unknown address: ignore
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Read port (combinational — registered caller-side for timing if needed)
    // -----------------------------------------------------------------------
    always_comb begin
        case (rd_addr)
            8'h00:   rd_data = 32'(trig_mode);
            8'h01:   rd_data = 32'(trig_mask);
            8'h02:   rd_data = 32'(trig_value);
            8'h03:   rd_data = 32'(pre_trig_depth);
            8'h04:   rd_data = 32'(post_trig_depth);
            8'h05:   rd_data = 32'(rle_en);
            8'h06:   rd_data = 32'(probe_mask);
            default: rd_data = 32'hDEAD_BEEF;
        endcase
    end

endmodule

`default_nettype wire
