// uart_cmd_parser.sv — UART command protocol handler
//
// Frame format: 0xAA CMD LEN PAYLOAD[0..LEN-1] CHK
// Checksum:     XOR of CMD, LEN, and all PAYLOAD bytes
//
// RX parser: IDLE→CMD→LEN→PAYLOAD→CHECKSUM→DISPATCH
// TX responder: T_IDLE→T_STREAM→(T_DATA_RAW|T_DATA_RLE)→T_END_RLE
//
// KEY DESIGN NOTE: All stage[] and stage_len assignments use only
// non-blocking assignments (<=) at every dispatch site.  The previous
// load_stage() automatic task with blocking (=) assignments caused Yosys
// to create a multi-driver mux network with incorrect priority, producing
// wrong stage_len values on hardware (stage_len=1 instead of 5, so only
// the 0xAA magic byte was transmitted per frame).
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module uart_cmd_parser #(
    parameter int PROBE_WIDTH  = 16,
    parameter int BUFFER_DEPTH = 1024,
    parameter int ADDR_BITS    = $clog2(BUFFER_DEPTH),
    parameter int TIMEOUT_CLKS = 120_000
) (
    input  logic        clk,
    input  logic        rst_n,

    // From uart_rx (rx_valid is pre-gated by top-level holdoff)
    input  logic [7:0]  rx_data,
    input  logic        rx_valid,

    // To uart_tx
    output logic [7:0]  tx_data,
    output logic        tx_valid,
    input  logic        tx_ready,

    // Config register interface
    output logic [7:0]  reg_wr_addr,
    output logic [31:0] reg_wr_data,
    output logic        reg_wr_en,
    input  logic [7:0]  reg_rd_addr_i,
    output logic [7:0]  reg_rd_addr,
    input  logic [31:0] reg_rd_data,

    // Capture controller interface
    output logic        do_arm,
    output logic        do_abort,
    input  logic [2:0]  fsm_state,
    input  logic [ADDR_BITS-1:0]  trig_position,
    input  logic [ADDR_BITS:0]    total_samples,
    input  logic        rle_en,

    // Readout streaming
    output logic        rd_next,
    input  logic [PROBE_WIDTH-1:0] rd_sample,
    input  logic        rd_sample_valid,
    input  logic        rd_done,

    // RLE encoder interface
    output logic [PROBE_WIDTH-1:0] rle_raw_data,
    output logic        rle_raw_valid,
    input  logic        rle_raw_ready,
    output logic        rle_stream_end,
    input  logic [7:0]  rle_enc_byte,
    input  logic        rle_enc_valid,
    output logic        rle_enc_ready
);

    // -----------------------------------------------------------------------
    // Command / status codes and constants
    // -----------------------------------------------------------------------
    localparam logic [7:0] CMD_PING      = 8'h01;
    localparam logic [7:0] CMD_SET_REG   = 8'h02;
    localparam logic [7:0] CMD_GET_REG   = 8'h03;
    localparam logic [7:0] CMD_ARM       = 8'h04;
    localparam logic [7:0] CMD_ABORT_CMD = 8'h05;
    localparam logic [7:0] CMD_STATUS    = 8'h06;
    localparam logic [7:0] CMD_READ_DATA = 8'h07;

    localparam logic [7:0] STS_OK            = 8'h00;
    localparam logic [7:0] STS_ALREADY_ARMED = 8'h01;
    localparam logic [7:0] STS_NOT_COMPLETE  = 8'h02;

    localparam logic [7:0] MAGIC         = 8'hAA;
    localparam logic [7:0] MAGIC_END_RLE = 8'h87;

    localparam int MAX_PAYLOAD  = 8;
    localparam int STAGE_DEPTH  = 20;

    // -----------------------------------------------------------------------
    // RX parser FSM
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {
        P_IDLE     = 3'd0,
        P_CMD      = 3'd1,
        P_LEN      = 3'd2,
        P_PAYLOAD  = 3'd3,
        P_CHECKSUM = 3'd4,
        P_DISPATCH = 3'd5
    } parser_state_t;

    parser_state_t  p_state;
    logic [7:0]     cmd_reg;
    logic [7:0]     len_reg;
    logic [7:0]     payload_buf [0:MAX_PAYLOAD-1];
    logic [2:0]     payload_idx;
    logic [7:0]     chk_accum;
    logic [16:0]    timeout_ctr;

    // -----------------------------------------------------------------------
    // TX responder FSM
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {
        T_IDLE     = 3'd0,
        T_STREAM   = 3'd1,
        T_DATA_RAW = 3'd2,
        T_DATA_RLE = 3'd3,
        T_END_RLE  = 3'd4
    } tx_state_t;

    tx_state_t   t_state;

    logic [7:0]  stage [0:STAGE_DEPTH-1];
    logic [4:0]  stage_len;
    logic [4:0]  stage_ptr;
    logic        tx_hold;
    logic [7:0]  tx_hold_data;
    logic        get_reg_resp_pending;

    localparam logic [1:0] RD_MODE_NONE = 2'd0,
                           RD_MODE_RAW  = 2'd1,
                           RD_MODE_RLE  = 2'd2;
    logic [1:0]  rd_mode_pending;
    logic        rd_kick_pending;

    logic [ADDR_BITS:0]   stream_samples_left;
    logic [1:0]           raw_byte_phase;
    logic [1:0]           raw_fetch_state;
    logic [1:0]           raw_prefill_wait;
    logic [PROBE_WIDTH-1:0] current_sample;
    logic [1:0]           rle_fetch_state;
    logic [1:0]           rle_prefill_wait;
    logic                 rle_have_sample;
    logic [PROBE_WIDTH-1:0] rle_sample_buf;

    // Accept RX bytes only while TX responder is idle
    logic rx_byte_stb;
    assign rx_byte_stb = rx_valid && (t_state == T_IDLE);

    // -----------------------------------------------------------------------
    // Main sequential block
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_state              <= P_IDLE;
            t_state              <= T_IDLE;
            cmd_reg              <= '0;
            len_reg              <= '0;
            payload_idx          <= '0;
            chk_accum            <= '0;
            timeout_ctr          <= '0;
            stage_len            <= '0;
            stage_ptr            <= '0;
            tx_hold              <= 1'b0;
            tx_hold_data         <= '0;
            get_reg_resp_pending <= 1'b0;
            rd_mode_pending      <= RD_MODE_NONE;
            rd_kick_pending      <= 1'b0;
            stream_samples_left  <= '0;
            raw_byte_phase       <= '0;
            raw_fetch_state      <= '0;
            raw_prefill_wait     <= '0;
            current_sample       <= '0;
            rle_fetch_state      <= '0;
            rle_prefill_wait     <= '0;
            rle_have_sample      <= 1'b0;
            rle_sample_buf       <= '0;
            tx_data              <= '0;
            tx_valid             <= 1'b0;
            do_arm               <= 1'b0;
            do_abort             <= 1'b0;
            reg_wr_en            <= 1'b0;
            reg_wr_addr          <= '0;
            reg_wr_data          <= '0;
            reg_rd_addr          <= '0;
            rd_next              <= 1'b0;
            rle_raw_valid        <= 1'b0;
            rle_stream_end       <= 1'b0;
            rle_enc_ready        <= 1'b0;
            for (int i = 0; i < STAGE_DEPTH; i++) stage[i] <= '0;
            for (int i = 0; i < MAX_PAYLOAD;  i++) payload_buf[i] <= '0;
        end else begin

            // ---------------------------------------------------------------
            // Pulse defaults
            // ---------------------------------------------------------------
            do_arm         <= 1'b0;
            do_abort       <= 1'b0;
            reg_wr_en      <= 1'b0;
            rd_next        <= 1'b0;
            rle_raw_valid  <= 1'b0;
            rle_stream_end <= 1'b0;
            rle_enc_ready  <= 1'b0;
            tx_valid       <= 1'b0;

            // Keep tx_valid high until uart_tx accepts (tx_ready falls)
            if (tx_hold) begin
                tx_data  <= tx_hold_data;
                tx_valid <= 1'b1;
                if (!tx_ready) tx_hold <= 1'b0;
            end

            // Deferred GET_REG response (reg_rd_data valid one cycle after addr write)
            if (get_reg_resp_pending && (t_state == T_IDLE)) begin
                stage[0] <= MAGIC;
                stage[1] <= CMD_GET_REG;
                stage[2] <= 8'd4;
                stage[3] <= reg_rd_data[31:24];
                stage[4] <= reg_rd_data[23:16];
                stage[5] <= reg_rd_data[15:8];
                stage[6] <= reg_rd_data[7:0];
                stage[7] <= CMD_GET_REG ^ 8'd4
                            ^ reg_rd_data[31:24] ^ reg_rd_data[23:16]
                            ^ reg_rd_data[15:8]  ^ reg_rd_data[7:0];
                stage_len            <= 5'd8;
                stage_ptr            <= '0;
                t_state              <= T_STREAM;
                get_reg_resp_pending <= 1'b0;
            end

            // ---------------------------------------------------------------
            // TX responder
            // ---------------------------------------------------------------
            case (t_state)
                T_IDLE: ; // wait for dispatch

                T_STREAM: begin
                    if (!tx_hold && tx_ready) begin
                        if (stage_ptr < stage_len) begin
                            tx_hold_data <= stage[stage_ptr];
                            tx_hold      <= 1'b1;
                            stage_ptr    <= stage_ptr + 1'b1;
                        end else begin
                            t_state <= T_IDLE;
                        end
                    end
                end

                T_DATA_RAW: begin
                    if ((stream_samples_left == 0) &&
                        (raw_byte_phase == 2'd0) && (raw_fetch_state == 2'd0)) begin
                        t_state <= T_IDLE;
                    end else begin
                        if (rd_kick_pending) begin
                            rd_next          <= 1'b1;
                            rd_kick_pending  <= 1'b0;
                            raw_prefill_wait <= 2'd2;
                            raw_fetch_state  <= 2'd0;
                        end else case (raw_byte_phase)
                            2'd0: begin
                                if (raw_prefill_wait != 0) begin
                                    raw_prefill_wait <= raw_prefill_wait - 1'b1;
                                end else case (raw_fetch_state)
                                    2'd0: begin
                                        rd_next         <= 1'b1;
                                        raw_fetch_state <= 2'd1;
                                    end
                                    2'd1: begin
                                        if (rd_sample_valid) begin
                                            current_sample  <= rd_sample;
                                            raw_byte_phase  <= 2'd1;
                                            raw_fetch_state <= 2'd0;
                                        end
                                    end
                                    default: raw_fetch_state <= 2'd0;
                                endcase
                            end
                            2'd1: begin
                                if (!tx_hold && tx_ready) begin
                                    tx_hold_data <= (PROBE_WIDTH > 8)
                                                    ? current_sample[PROBE_WIDTH-1 -: 8]
                                                    : 8'h00;
                                    tx_hold        <= 1'b1;
                                    raw_byte_phase <= 2'd2;
                                end
                            end
                            2'd2: begin
                                if (!tx_hold && tx_ready) begin
                                    tx_hold_data <= current_sample[7:0];
                                    tx_hold      <= 1'b1;
                                    if (stream_samples_left > 0)
                                        stream_samples_left <= stream_samples_left - 1'b1;
                                    raw_byte_phase <= 2'd0;
                                end
                            end
                            default: raw_byte_phase <= 2'd0;
                        endcase
                    end
                end

                T_DATA_RLE: begin
                    rle_enc_ready <= tx_ready && !tx_hold;
                    if (rle_enc_valid && tx_ready && !tx_hold) begin
                        tx_hold_data <= rle_enc_byte;
                        tx_hold      <= 1'b1;
                    end
                    if ((stream_samples_left > 0) || rle_have_sample
                        || (rle_fetch_state != 2'd0)) begin
                        if (rd_kick_pending) begin
                            rd_next          <= 1'b1;
                            rd_kick_pending  <= 1'b0;
                            rle_prefill_wait <= 2'd2;
                            rle_fetch_state  <= 2'd0;
                        end else begin
                            if (!rle_have_sample && stream_samples_left > 0) begin
                                if (rle_prefill_wait != 0) begin
                                    rle_prefill_wait <= rle_prefill_wait - 1'b1;
                                end else case (rle_fetch_state)
                                    2'd0: begin
                                        rd_next         <= 1'b1;
                                        rle_fetch_state <= 2'd1;
                                    end
                                    2'd1: begin
                                        if (rd_sample_valid) begin
                                            rle_sample_buf  <= rd_sample;
                                            rle_have_sample <= 1'b1;
                                            rle_fetch_state <= 2'd0;
                                        end
                                    end
                                    default: rle_fetch_state <= 2'd0;
                                endcase
                            end
                            if (rle_have_sample && rle_raw_ready) begin
                                rle_raw_data  <= rle_sample_buf;
                                rle_raw_valid <= 1'b1;
                                rle_have_sample <= 1'b0;
                                if (stream_samples_left > 0)
                                    stream_samples_left <= stream_samples_left - 1'b1;
                                if (stream_samples_left == 1)
                                    rle_stream_end <= 1'b1;
                            end
                        end
                    end else begin
                        rle_have_sample <= 1'b0;
                        if (!rle_enc_valid)
                            t_state <= T_END_RLE;
                    end
                end

                T_END_RLE: begin
                    if (stage_ptr == 0 && tx_ready && !tx_hold) begin
                        tx_hold_data <= MAGIC;
                        tx_hold      <= 1'b1;
                        stage_ptr    <= 5'd1;
                    end else if (stage_ptr == 1 && tx_ready && !tx_hold) begin
                        tx_hold_data <= MAGIC_END_RLE;
                        tx_hold      <= 1'b1;
                        t_state      <= T_IDLE;
                        stage_ptr    <= '0;
                    end
                end

                default: t_state <= T_IDLE;
            endcase

            // ---------------------------------------------------------------
            // RX parser timeout
            // ---------------------------------------------------------------
            if (p_state != P_IDLE) begin
                if (rx_byte_stb) begin
                    timeout_ctr <= '0;
                end else begin
                    timeout_ctr <= timeout_ctr + 1'b1;
                    if (timeout_ctr >= TIMEOUT_CLKS[16:0]) begin
                        p_state     <= P_IDLE;
                        timeout_ctr <= '0;
                    end
                end
            end else begin
                timeout_ctr <= '0;
            end

            // ---------------------------------------------------------------
            // RX parser FSM
            // ---------------------------------------------------------------
            case (p_state)
                P_IDLE: begin
                    if (rx_byte_stb && rx_data == MAGIC)
                        p_state <= P_CMD;
                end

                P_CMD: begin
                    if (rx_byte_stb) begin
                        cmd_reg   <= rx_data;
                        chk_accum <= rx_data;
                        p_state   <= P_LEN;
                    end
                end

                P_LEN: begin
                    if (rx_byte_stb) begin
                        if (rx_data > MAX_PAYLOAD) begin
                            p_state <= P_IDLE;
                        end else begin
                            len_reg     <= rx_data;
                            chk_accum   <= chk_accum ^ rx_data;
                            payload_idx <= '0;
                            p_state     <= (rx_data == 0) ? P_CHECKSUM : P_PAYLOAD;
                        end
                    end
                end

                P_PAYLOAD: begin
                    if (rx_byte_stb) begin
                        payload_buf[payload_idx] <= rx_data;
                        chk_accum   <= chk_accum ^ rx_data;
                        payload_idx <= payload_idx + 1'b1;
                        if (payload_idx == len_reg - 1)
                            p_state <= P_CHECKSUM;
                    end
                end

                P_CHECKSUM: begin
                    if (rx_byte_stb) begin
                        if (rx_data !== chk_accum)
                            p_state <= P_IDLE;
                        else
                            p_state <= P_DISPATCH;
                    end
                end

                // -----------------------------------------------------------
                // P_DISPATCH: build response into stage[] with non-blocking
                // assignments only.  Each command writes stage[0..N-1] and
                // stage_len directly — no tasks, no blocking assignments.
                // -----------------------------------------------------------
                P_DISPATCH: begin
                    p_state <= P_IDLE;

                    case (cmd_reg)

                        // PING → AA 01 01 55 CHK(=55)
                        CMD_PING: if (len_reg == 8'd0) begin
                            stage[0] <= MAGIC;
                            stage[1] <= CMD_PING;
                            stage[2] <= 8'd1;
                            stage[3] <= 8'h55;
                            stage[4] <= CMD_PING ^ 8'd1 ^ 8'h55;
                            stage_len       <= 5'd5;
                            stage_ptr       <= '0;
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                            t_state         <= T_STREAM;
                        end

                        // SET_REG → AA 02 01 STS_OK CHK
                        CMD_SET_REG: if (len_reg == 8'd5) begin
                            reg_wr_addr <= payload_buf[0];
                            reg_wr_data <= {payload_buf[1], payload_buf[2],
                                            payload_buf[3], payload_buf[4]};
                            reg_wr_en   <= 1'b1;
                            stage[0] <= MAGIC;
                            stage[1] <= CMD_SET_REG;
                            stage[2] <= 8'd1;
                            stage[3] <= STS_OK;
                            stage[4] <= CMD_SET_REG ^ 8'd1 ^ STS_OK;
                            stage_len       <= 5'd5;
                            stage_ptr       <= '0;
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                            t_state         <= T_STREAM;
                        end

                        // GET_REG: defer one cycle for reg_rd_data to settle
                        CMD_GET_REG: if (len_reg == 8'd1) begin
                            reg_rd_addr          <= payload_buf[0];
                            get_reg_resp_pending <= 1'b1;
                            rd_mode_pending      <= RD_MODE_NONE;
                            rd_kick_pending      <= 1'b0;
                        end

                        // ARM → AA 04 01 status CHK
                        CMD_ARM: if (len_reg == 8'd0) begin
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                            if (fsm_state == 3'd1 || fsm_state == 3'd3) begin
                                stage[3] <= STS_ALREADY_ARMED;
                                stage[4] <= CMD_ARM ^ 8'd1 ^ STS_ALREADY_ARMED;
                            end else begin
                                do_arm   <= 1'b1;
                                stage[3] <= STS_OK;
                                stage[4] <= CMD_ARM ^ 8'd1 ^ STS_OK;
                            end
                            stage[0] <= MAGIC;
                            stage[1] <= CMD_ARM;
                            stage[2] <= 8'd1;
                            stage_len <= 5'd5;
                            stage_ptr <= '0;
                            t_state   <= T_STREAM;
                        end

                        // ABORT → AA 05 01 STS_OK CHK
                        CMD_ABORT_CMD: if (len_reg == 8'd0) begin
                            do_abort <= 1'b1;
                            stage[0] <= MAGIC;
                            stage[1] <= CMD_ABORT_CMD;
                            stage[2] <= 8'd1;
                            stage[3] <= STS_OK;
                            stage[4] <= CMD_ABORT_CMD ^ 8'd1 ^ STS_OK;
                            stage_len       <= 5'd5;
                            stage_ptr       <= '0;
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                            t_state         <= T_STREAM;
                        end

                        // STATUS → AA 06 04 state tp_hi tp_lo rle_en CHK
                        CMD_STATUS: if (len_reg == 8'd0) begin
                            stage[0] <= MAGIC;
                            stage[1] <= CMD_STATUS;
                            stage[2] <= 8'd4;
                            stage[3] <= {5'b0, fsm_state};
                            stage[4] <= 8'(trig_position >> 8);
                            stage[5] <= 8'(trig_position);
                            stage[6] <= {7'b0, rle_en};
                            stage[7] <= CMD_STATUS ^ 8'd4
                                        ^ {5'b0, fsm_state}
                                        ^ 8'(trig_position >> 8)
                                        ^ 8'(trig_position)
                                        ^ {7'b0, rle_en};
                            stage_len       <= 5'd8;
                            stage_ptr       <= '0;
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                            t_state         <= T_STREAM;
                        end

                        // READ_DATA: stream raw or RLE capture data
                        CMD_READ_DATA: begin
                            if (len_reg != 8'd0) begin
                                rd_mode_pending <= RD_MODE_NONE;
                                rd_kick_pending <= 1'b0;
                            end else if (fsm_state != 3'd4) begin
                                // Not COMPLETE → NAK
                                stage[0] <= MAGIC;
                                stage[1] <= 8'hFF;
                                stage[2] <= 8'd1;
                                stage[3] <= STS_NOT_COMPLETE;
                                stage[4] <= 8'hFF ^ 8'd1 ^ STS_NOT_COMPLETE;
                                stage_len       <= 5'd5;
                                stage_ptr       <= '0;
                                rd_mode_pending <= RD_MODE_NONE;
                                rd_kick_pending <= 1'b0;
                                t_state         <= T_STREAM;
                            end else begin
                                stream_samples_left <= total_samples;
                                raw_byte_phase      <= 2'd0;
                                raw_fetch_state     <= 2'd0;
                                raw_prefill_wait    <= 2'd0;
                                rle_fetch_state     <= 2'd0;
                                rle_prefill_wait    <= 2'd0;
                                rle_have_sample     <= 1'b0;
                                rd_kick_pending     <= 1'b1;
                                if (!rle_en) begin
                                    // Raw: AA 07 LEN_HI LEN_LO  (LEN = total_samples*2)
                                    stage[0] <= MAGIC;
                                    stage[1] <= CMD_READ_DATA;
                                    stage[2] <= 8'((16'(total_samples) << 1) >> 8);
                                    stage[3] <= 8'(16'(total_samples) << 1);
                                    stage_len       <= 5'd4;
                                    stage_ptr       <= '0;
                                    t_state         <= T_STREAM;
                                    rd_mode_pending <= RD_MODE_RAW;
                                end else begin
                                    // RLE: AA 07 FF FF  (sentinel-terminated)
                                    stage[0] <= MAGIC;
                                    stage[1] <= CMD_READ_DATA;
                                    stage[2] <= 8'hFF;
                                    stage[3] <= 8'hFF;
                                    stage_len       <= 5'd4;
                                    stage_ptr       <= '0;
                                    t_state         <= T_STREAM;
                                    rd_mode_pending <= RD_MODE_RLE;
                                end
                            end
                        end

                        default: begin
                            rd_mode_pending <= RD_MODE_NONE;
                            rd_kick_pending <= 1'b0;
                        end
                    endcase
                end

                default: p_state <= P_IDLE;
            endcase

            // ---------------------------------------------------------------
            // T_STREAM → T_DATA_* handoff once header is fully queued
            // ---------------------------------------------------------------
            if (t_state == T_STREAM && stage_ptr == stage_len) begin
                if (rd_mode_pending == RD_MODE_RAW) begin
                    t_state         <= T_DATA_RAW;
                    stage_ptr       <= '0;
                    rd_mode_pending <= RD_MODE_NONE;
                end else if (rd_mode_pending == RD_MODE_RLE) begin
                    t_state         <= T_DATA_RLE;
                    stage_ptr       <= '0;
                    rd_mode_pending <= RD_MODE_NONE;
                end
            end

        end // else (not reset)
    end // always_ff

endmodule

`default_nettype wire
