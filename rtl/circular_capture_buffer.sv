// circular_capture_buffer.sv — synchronous dual-port BRAM wrapper
//
// Pure memory array.  All circular-addressing and pointer arithmetic
// lives in capture_controller_fsm — this module is intentionally a
// thin wrapper so yosys can map it cleanly onto iCE40 EBRs.
//
// For DEPTH=1024, WIDTH=16:  1024×16 = 16 Kbits = 4 iCE40 EBRs.
//
// Read output is REGISTERED (1-cycle latency).  The FSM must present
// rd_addr one cycle before it needs rd_data.
//
// Write and read may address the same location simultaneously (write-first
// behaviour is implementation-defined for EBRs; the FSM avoids this).
//
// Synthesis: yosys infers iCE40 SB_RAM40_4K from this pattern.

`default_nettype none

module circular_capture_buffer #(
    parameter int DEPTH     = 1024,
    parameter int WIDTH     = 16,
    parameter int ADDR_BITS = $clog2(DEPTH)
) (
    input  logic                  clk,

    // Write port (capture_controller_fsm)
    input  logic [ADDR_BITS-1:0]  wr_addr,
    input  logic [WIDTH-1:0]      wr_data,
    input  logic                  wr_en,

    // Read port  (capture_controller_fsm during READOUT)
    input  logic [ADDR_BITS-1:0]  rd_addr,
    output logic [WIDTH-1:0]      rd_data,    // registered; 1-cycle latency
    input  logic                  rd_en
);

    // The memory array — yosys maps this to EBRs
    (* ram_style = "block" *)
    logic [WIDTH-1:0] mem [0:DEPTH-1];

    // Write port: synchronous
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Read port: synchronous (registered output)
    always_ff @(posedge clk) begin
        if (rd_en)
            rd_data <= mem[rd_addr];
    end

endmodule

`default_nettype wire
