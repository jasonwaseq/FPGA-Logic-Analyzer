// uart_tx.sv — UART transmitter
// 8N1 format.  tx_ready is high when the transmitter can accept a new byte.
// Assert tx_valid for one cycle with tx_data to begin transmission.
// tx_ready falls on the cycle tx_valid is sampled; rises again after stop bit.
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module uart_tx #(
    parameter int CLK_FREQ  = 12_000_000,
    parameter int BAUD_RATE = 115_200
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic [7:0] tx_data,   // byte to transmit
    input  logic       tx_valid,  // pulse high to start transmission
    output logic       tx_ready,  // high when idle and ready for new byte
    output logic       tx_pin     // UART TX pin
);

    // ---------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------
    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam int CTR_BITS     = $clog2(CLKS_PER_BIT + 1);

    // ---------------------------------------------------------------------------
    // State machine
    // ---------------------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE  = 2'd0,
        START = 2'd1,
        DATA  = 2'd2,
        STOP  = 2'd3
    } state_t;

    state_t              state;
    logic [CTR_BITS-1:0] baud_ctr;
    logic [2:0]          bit_idx;   // 0..7
    logic [7:0]          shift_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            baud_ctr  <= '0;
            bit_idx   <= '0;
            shift_reg <= '0;
            tx_pin    <= 1'b1; // idle high
            tx_ready  <= 1'b1;
        end else begin
            case (state)
                // ---------------------------------------------------------------
                IDLE: begin
                    tx_pin   <= 1'b1;
                    tx_ready <= 1'b1;
                    if (tx_valid) begin
                        shift_reg <= tx_data;
                        baud_ctr  <= CTR_BITS'(CLKS_PER_BIT - 1);
                        bit_idx   <= '0;
                        tx_ready  <= 1'b0;
                        tx_pin    <= 1'b0; // start bit
                        state     <= START;
                    end
                end

                // ---------------------------------------------------------------
                // START: hold start bit for one full bit period
                START: begin
                    if (baud_ctr == '0) begin
                        // Transmit LSB of shift_reg
                        tx_pin   <= shift_reg[0];
                        baud_ctr <= CTR_BITS'(CLKS_PER_BIT - 1);
                        state    <= DATA;
                    end else begin
                        baud_ctr <= baud_ctr - 1'b1;
                    end
                end

                // ---------------------------------------------------------------
                DATA: begin
                    if (baud_ctr == '0) begin
                        if (bit_idx == 3'd7) begin
                            // All 8 bits sent; move to stop bit
                            tx_pin   <= 1'b1;
                            baud_ctr <= CTR_BITS'(CLKS_PER_BIT - 1);
                            state    <= STOP;
                        end else begin
                            shift_reg <= {1'b0, shift_reg[7:1]};
                            tx_pin    <= shift_reg[1]; // next bit after shift
                            baud_ctr  <= CTR_BITS'(CLKS_PER_BIT - 1);
                            bit_idx   <= bit_idx + 1'b1;
                        end
                    end else begin
                        baud_ctr <= baud_ctr - 1'b1;
                    end
                end

                // ---------------------------------------------------------------
                STOP: begin
                    if (baud_ctr == '0) begin
                        tx_ready <= 1'b1;
                        state    <= IDLE;
                    end else begin
                        baud_ctr <= baud_ctr - 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
