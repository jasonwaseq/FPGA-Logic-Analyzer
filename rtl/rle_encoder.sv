// rle_encoder.sv — Run-Length Encoding encoder for capture readout
//
// When en=0: passthrough mode.  Each 16-bit raw sample is output as
//   two bytes: [sample[15:8], sample[7:0]].
//
// When en=1: RLE encoding.
//   Literal token  (3 bytes): 0x00, sample[15:8], sample[7:0]
//   Run token      (1 byte):  0x80 | count[6:0]   (count = 1..127 extra repeats)
//
// A "run" token encodes COUNT additional repetitions of the PREVIOUS literal.
// For runs >127 the encoder emits multiple run tokens.
//
// Output is a byte stream via (enc_byte, enc_valid, enc_ready) ready/valid
// handshake.  The encoder stalls (raw_ready=0) when its internal output
// queue is full.
//
// Input side: (raw_data, raw_valid, raw_ready) valid/ready handshake.
// raw_valid must stay asserted until raw_ready goes high.
//
// End-of-stream: the caller asserts stream_end=1 for one cycle after
// the last raw_valid.  This causes any pending run token to be flushed.
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module rle_encoder #(
    parameter int PROBE_WIDTH = 16
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    en,          // 0=passthrough, 1=RLE

    // Raw sample input (valid/ready)
    input  logic [PROBE_WIDTH-1:0]  raw_data,
    input  logic                    raw_valid,
    output logic                    raw_ready,
    input  logic                    stream_end,  // pulse after last raw_valid

    // Encoded byte output (valid/ready)
    output logic [7:0]              enc_byte,
    output logic                    enc_valid,
    input  logic                    enc_ready
);

    // -----------------------------------------------------------------------
    // Output FIFO (4-entry)
    // -----------------------------------------------------------------------
    localparam int FIFO_DEPTH = 4;
    logic [7:0]  fifo [0:FIFO_DEPTH-1];
    logic [2:0]  fifo_wr_ptr;
    logic [2:0]  fifo_rd_ptr;
    logic [2:0]  fifo_count;

    wire fifo_full  = (fifo_count == 3'(FIFO_DEPTH));
    wire fifo_empty = (fifo_count == '0);

    // drain=1 when consumer takes a byte this cycle
    wire drain = enc_ready && !fifo_empty;

    // free slots after drain (accounts for simultaneous push+pop)
    wire [2:0] free_slots = 3'(FIFO_DEPTH) - fifo_count + {2'b0, drain};

    assign enc_byte  = fifo[fifo_rd_ptr[1:0]];
    assign enc_valid = !fifo_empty;

    // -----------------------------------------------------------------------
    // RLE state
    // -----------------------------------------------------------------------
    logic [PROBE_WIDTH-1:0] prev_sample;
    logic [6:0]             run_count;
    logic                   has_prev;

    typedef enum logic [1:0] {
        ENC_IDLE = 2'd0
    } enc_state_t;
    enc_state_t enc_state;

    // -----------------------------------------------------------------------
    // raw_ready: combinationally guard the handshake so that when raw_ready=1
    // AND raw_valid=1 the encoder is GUARANTEED to accept the sample.
    // We look at raw_data vs prev_sample to determine required FIFO slots.
    // -----------------------------------------------------------------------
    logic [2:0] required_slots;
    always_comb begin
        if (!en) begin
            required_slots = 3'd2;   // passthrough: 2 bytes
        end else if (!has_prev) begin
            required_slots = 3'd3;   // first literal: 3 bytes
        end else if (raw_data == prev_sample) begin
            // run: if overflow needs 1 byte for run token, else 0
            required_slots = (run_count == 7'd127) ? 3'd1 : 3'd0;
        end else begin
            // value change: run+literal (4) or literal only (3)
            required_slots = (run_count > 0) ? 3'd4 : 3'd3;
        end
    end

    assign raw_ready = (enc_state == ENC_IDLE) && (free_slots >= required_slots);

    // -----------------------------------------------------------------------
    // Main always_ff
    //
    // fifo_count is updated ONCE per cycle in each branch:
    //   push N bytes:  fifo_count + N - drain
    //   no push:       fifo_count - drain
    // The drain's rd_ptr update happens independently.
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wr_ptr <= '0;
            fifo_rd_ptr <= '0;
            fifo_count  <= '0;
            prev_sample <= '0;
            run_count   <= '0;
            has_prev    <= 1'b0;
            enc_state   <= ENC_IDLE;
            for (int i = 0; i < FIFO_DEPTH; i++) fifo[i] <= '0;
        end else begin

            // FIFO drain: advance rd_ptr (count handled per-branch below)
            if (drain)
                fifo_rd_ptr <= fifo_rd_ptr + 1'b1;

            case (enc_state)

                ENC_IDLE: begin
                    if (raw_valid && raw_ready) begin
                        // raw_ready guarantees free_slots >= required_slots
                        if (!en) begin
                            // Passthrough: 2 bytes
                            fifo[fifo_wr_ptr[1:0]]          <= (PROBE_WIDTH > 8) ? raw_data[PROBE_WIDTH-1:PROBE_WIDTH-8] : 8'h00;
                            fifo[(fifo_wr_ptr+1'b1) & 2'h3] <= raw_data[7:0];
                            fifo_wr_ptr <= fifo_wr_ptr + 2'd2;
                            fifo_count  <= fifo_count + 3'd2 - {2'b0, drain};

                        end else if (!has_prev) begin
                            // First literal: 3 bytes
                            fifo[fifo_wr_ptr[1:0]]       <= 8'h00;
                            fifo[(fifo_wr_ptr+1) & 2'h3] <= (PROBE_WIDTH > 8) ? raw_data[PROBE_WIDTH-1:PROBE_WIDTH-8] : 8'h00;
                            fifo[(fifo_wr_ptr+2) & 2'h3] <= raw_data[7:0];
                            fifo_wr_ptr <= fifo_wr_ptr + 2'd3;
                            fifo_count  <= fifo_count + 3'd3 - {2'b0, drain};
                            prev_sample <= raw_data;
                            has_prev    <= 1'b1;
                            run_count   <= '0;

                        end else if (raw_data == prev_sample) begin
                            if (run_count == 7'd127) begin
                                // Flush run(127), reset count to 1
                                fifo[fifo_wr_ptr[1:0]] <= 8'h80 | 7'd127;
                                fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                                fifo_count  <= fifo_count + 3'd1 - {2'b0, drain};
                                run_count   <= 7'd1;
                            end else begin
                                // Just accumulate
                                run_count  <= run_count + 1'b1;
                                fifo_count <= fifo_count - {2'b0, drain};
                            end

                        end else begin
                            // Value changed
                            if (run_count > 0) begin
                                // Emit run + literal: 4 bytes
                                fifo[fifo_wr_ptr[1:0]]       <= 8'h80 | run_count;
                                fifo[(fifo_wr_ptr+1) & 2'h3] <= 8'h00;
                                fifo[(fifo_wr_ptr+2) & 2'h3] <= (PROBE_WIDTH > 8) ? raw_data[PROBE_WIDTH-1:PROBE_WIDTH-8] : 8'h00;
                                fifo[(fifo_wr_ptr+3) & 2'h3] <= raw_data[7:0];
                                fifo_wr_ptr <= fifo_wr_ptr + 3'd4;
                                fifo_count  <= fifo_count + 3'd4 - {2'b0, drain};
                                prev_sample <= raw_data;
                                run_count   <= '0;
                            end else begin
                                // Emit literal: 3 bytes
                                fifo[fifo_wr_ptr[1:0]]       <= 8'h00;
                                fifo[(fifo_wr_ptr+1) & 2'h3] <= (PROBE_WIDTH > 8) ? raw_data[PROBE_WIDTH-1:PROBE_WIDTH-8] : 8'h00;
                                fifo[(fifo_wr_ptr+2) & 2'h3] <= raw_data[7:0];
                                fifo_wr_ptr <= fifo_wr_ptr + 2'd3;
                                fifo_count  <= fifo_count + 3'd3 - {2'b0, drain};
                                prev_sample <= raw_data;
                            end
                        end

                    end else if (stream_end && has_prev && run_count > 0 && en) begin
                        // Flush pending run token
                        fifo[fifo_wr_ptr[1:0]] <= 8'h80 | run_count;
                        fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
                        fifo_count  <= fifo_count + 3'd1 - {2'b0, drain};
                        run_count   <= '0;

                    end else if (stream_end) begin
                        has_prev   <= 1'b0;
                        run_count  <= '0;
                        fifo_count <= fifo_count - {2'b0, drain};

                    end else begin
                        // Idle: drain only
                        fifo_count <= fifo_count - {2'b0, drain};
                    end
                end

                default: enc_state <= ENC_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
