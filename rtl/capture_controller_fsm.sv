// capture_controller_fsm.sv — capture state machine
//
// This is the heart of the logic analyzer.  It:
//   • Continuously samples probe_data into the circular BRAM while ARMED
//   • Evaluates the combinational trigger_fire signal each cycle
//   • After trigger: captures post_trig_depth more samples, then asserts complete
//   • During READOUT: walks the BRAM in linearised order and streams samples
//     to uart_cmd_parser via the rd_next handshake
//
// State encoding (one-hot, 6 bits):
//   IDLE         — waiting for arm command
//   ARMED        — sampling into circular buffer, checking trigger
//   TRIGGERED    — one-cycle latch state (captures trig_ptr, computes start_addr)
//   POST_CAPTURE — capturing post-trigger samples
//   COMPLETE     — capture done, awaiting readout or re-arm
//   READOUT      — streaming BRAM contents to cmd_parser
//
// BRAM read latency: circular_capture_buffer has 1-cycle registered output.
// In READOUT the FSM asserts rd_en/rd_addr one cycle before rd_sample_valid.
//
// Buffer linearisation (power-of-2 BUFFER_DEPTH, so & mask == mod):
//   start_addr  = (trig_ptr - actual_pre) & (DEPTH-1)
//   trig_pos    = actual_pre
//   total       = actual_pre + 1 + post_trig_depth
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module capture_controller_fsm #(
    parameter int PROBE_WIDTH  = 16,
    parameter int BUFFER_DEPTH = 1024,
    parameter int ADDR_BITS    = $clog2(BUFFER_DEPTH)
) (
    input  logic                      clk,
    input  logic                      rst_n,

    // Probe data (registered at top-level IOB)
    input  logic [PROBE_WIDTH-1:0]    probe_data,

    // Previous sample output — fed to trigger_engine
    output logic [PROBE_WIDTH-1:0]    prev_data,

    // Trigger
    input  logic                      trigger_fire,

    // Configuration (from config_regs; latched at ARM time)
    input  logic [ADDR_BITS-1:0]      pre_trig_depth,
    input  logic [ADDR_BITS-1:0]      post_trig_depth,

    // Control
    input  logic                      arm,      // pulse: begin acquisition
    input  logic                      abort,    // pulse: return to IDLE

    // Status
    output logic                      o_armed,
    output logic                      o_triggered,
    output logic                      o_complete,
    output logic [2:0]                state_out,  // raw state for CMD_STATUS

    // BRAM write port
    output logic [ADDR_BITS-1:0]      wr_addr,
    output logic [PROBE_WIDTH-1:0]    wr_data,
    output logic                      wr_en,

    // BRAM read port
    output logic [ADDR_BITS-1:0]      rd_addr,
    output logic                      rd_en,
    input  logic [PROBE_WIDTH-1:0]    rd_data,

    // Readout streaming interface (to uart_cmd_parser)
    input  logic                      rd_next,         // advance to next sample
    output logic [PROBE_WIDTH-1:0]    rd_sample_out,   // sample value
    output logic                      rd_sample_valid,  // sample valid this cycle
    output logic                      rd_done,          // all samples streamed

    // Trigger position and total samples (for CMD_STATUS and READ_DATA framing)
    output logic [ADDR_BITS-1:0]      trig_position,
    output logic [ADDR_BITS:0]        total_samples    // ADDR_BITS+1 bits
);

    // -----------------------------------------------------------------------
    // State encoding
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {
        IDLE         = 3'd0,
        ARMED        = 3'd1,
        TRIGGERED    = 3'd2,
        POST_CAPTURE = 3'd3,
        COMPLETE     = 3'd4,
        READOUT      = 3'd5
    } state_t;

    state_t state;

    // -----------------------------------------------------------------------
    // Internal registers
    // -----------------------------------------------------------------------
    logic [ADDR_BITS-1:0]  write_ptr;       // next write address
    logic [ADDR_BITS-1:0]  pre_count;       // saturating count of pre-trigger samples
    logic [ADDR_BITS-1:0]  post_count;      // post-trigger samples captured
    logic [ADDR_BITS-1:0]  trig_ptr;        // buffer address of trigger sample
    logic [ADDR_BITS-1:0]  actual_pre;      // min(pre_count, pre_trig_depth)
    logic [ADDR_BITS-1:0]  start_addr;      // linearised readout start
    logic [ADDR_BITS-1:0]  rd_cursor;       // current readout pointer
    logic [ADDR_BITS:0]    rd_count;        // samples read so far

    // Latched config (latched at ARMED entry so mid-capture changes don't corrupt)
    logic [ADDR_BITS-1:0]  lat_pre_depth;
    logic [ADDR_BITS-1:0]  lat_post_depth;

    // -----------------------------------------------------------------------
    // Status outputs
    // -----------------------------------------------------------------------
    assign state_out    = state[2:0];
    assign o_armed      = (state == ARMED) | (state == POST_CAPTURE) | (state == TRIGGERED);
    assign o_triggered  = (state == POST_CAPTURE);
    assign o_complete   = (state == COMPLETE) | (state == READOUT);

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            write_ptr      <= '0;
            pre_count      <= '0;
            post_count     <= '0;
            trig_ptr       <= '0;
            actual_pre     <= '0;
            start_addr     <= '0;
            rd_cursor      <= '0;
            rd_count       <= '0;
            trig_position  <= '0;
            total_samples  <= '0;
            lat_pre_depth  <= '0;
            lat_post_depth <= '0;
            prev_data      <= '0;
            wr_addr        <= '0;
            wr_data        <= '0;
            wr_en          <= 1'b0;
            rd_addr        <= '0;
            rd_en          <= 1'b0;
            rd_sample_out  <= '0;
            rd_sample_valid<= 1'b0;
            rd_done        <= 1'b0;
        end else begin
            // Default: clear pulse outputs
            wr_en          <= 1'b0;
            rd_en          <= 1'b0;
            rd_sample_valid<= 1'b0;
            rd_done        <= 1'b0;

            case (state)
                // -----------------------------------------------------------
                IDLE: begin
                    if (arm) begin
                        write_ptr     <= '0;
                        pre_count     <= '0;
                        post_count    <= '0;
                        lat_pre_depth <= pre_trig_depth;
                        lat_post_depth<= post_trig_depth;
                        state         <= ARMED;
                    end
                end

                // -----------------------------------------------------------
                // Sample every cycle, check trigger.
                // write_ptr points to the address being written this cycle.
                // After clock edge: write_ptr has been incremented for next cycle.
                // -----------------------------------------------------------
                ARMED: begin
                    // Write current sample to buffer
                    wr_en    <= 1'b1;
                    wr_addr  <= write_ptr;
                    wr_data  <= probe_data;

                    // Update previous sample for edge detection
                    prev_data <= probe_data;

                    // Advance write pointer (wraps naturally at ADDR_BITS)
                    write_ptr <= write_ptr + 1'b1;

                    // Track pre-trigger sample count (saturate at lat_pre_depth).
                    // Do NOT increment on the trigger cycle itself — the trigger
                    // sample is not a pre-trigger sample.
                    if (pre_count < lat_pre_depth && !trigger_fire)
                        pre_count <= pre_count + 1'b1;

                    // Evaluate trigger
                    if (trigger_fire) begin
                        // trig_ptr = address we just wrote
                        trig_ptr <= write_ptr;
                        state    <= TRIGGERED;
                    end else if (abort) begin
                        state <= IDLE;
                    end
                end

                // -----------------------------------------------------------
                // One-cycle latch state.  Compute start_addr and actual_pre.
                // No new write this cycle.
                // -----------------------------------------------------------
                TRIGGERED: begin
                    // actual_pre is the smaller of: samples accumulated vs configured
                    actual_pre    <= (pre_count < lat_pre_depth) ? pre_count : lat_pre_depth;
                    post_count    <= '0;
                    // start_addr computed in COMPLETE after actual_pre is latched;
                    // no write this cycle
                    state <= POST_CAPTURE;
                end

                // -----------------------------------------------------------
                POST_CAPTURE: begin
                    // Continue sampling
                    wr_en    <= 1'b1;
                    wr_addr  <= write_ptr;
                    wr_data  <= probe_data;
                    prev_data <= probe_data;
                    write_ptr <= write_ptr + 1'b1;

                    post_count <= post_count + 1'b1;

                    if (post_count == (lat_post_depth - 1'b1)) begin
                        // Last post-trigger sample written; transition to COMPLETE
                        // Compute linearisation parameters
                        start_addr    <= (trig_ptr - actual_pre) & ADDR_BITS'(BUFFER_DEPTH - 1);
                        trig_position <= actual_pre;
                        total_samples <= {1'b0, actual_pre} + {1'b0, lat_post_depth} + 1;
                        rd_count      <= '0;
                        state         <= COMPLETE;
                    end else if (abort) begin
                        state <= IDLE;
                    end
                end

                // -----------------------------------------------------------
                COMPLETE: begin
                    if (abort) begin
                        state <= IDLE;
                    end else if (arm) begin
                        // Re-arm: discard current capture
                        write_ptr     <= '0;
                        pre_count     <= '0;
                        post_count    <= '0;
                        lat_pre_depth <= pre_trig_depth;
                        lat_post_depth<= post_trig_depth;
                        state         <= ARMED;
                    end else if (rd_next) begin
                        // Begin readout: issue first BRAM read
                        rd_cursor <= start_addr;
                        rd_addr   <= start_addr;
                        rd_en     <= 1'b1;
                        rd_count  <= '0;
                        state     <= READOUT;
                    end
                end

                // -----------------------------------------------------------
                // Stream total_samples samples from BRAM in linear order.
                // BRAM has 1-cycle read latency:
                //   Cycle N:   present rd_addr, assert rd_en
                //   Cycle N+1: rd_data is valid → drive rd_sample_out, rd_sample_valid
                // The rd_next handshake is used to advance each sample.
                // On entry (from COMPLETE, first rd_next): we already issued rd_addr.
                // -----------------------------------------------------------
                READOUT: begin
                    if (rd_next) begin
                        // rd_data from the address issued last cycle is now valid
                        rd_sample_out   <= rd_data;
                        rd_sample_valid <= 1'b1;
                        rd_count        <= rd_count + 1'b1;

                        if (rd_count + 1 == total_samples) begin
                            // Last sample delivered
                            rd_done <= 1'b1;
                            state   <= COMPLETE;
                        end else begin
                            // Issue next BRAM address (pipeline ahead)
                            rd_cursor <= rd_cursor + 1'b1;
                            rd_addr   <= (rd_cursor + 1'b1) & ADDR_BITS'(BUFFER_DEPTH - 1);
                            rd_en     <= 1'b1;
                        end
                    end

                    if (abort) begin
                        state <= IDLE;
                    end else if (arm) begin
                        write_ptr     <= '0;
                        pre_count     <= '0;
                        post_count    <= '0;
                        lat_pre_depth <= pre_trig_depth;
                        lat_post_depth<= post_trig_depth;
                        state         <= ARMED;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
