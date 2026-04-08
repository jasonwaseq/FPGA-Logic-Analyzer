// uart_rx.sv — UART receiver
// 8N1 format, oversampling at CLK_FREQ/BAUD_RATE ticks per bit.
// rx_valid pulses for exactly one clock when a valid byte is received.
// rx_error pulses for exactly one clock on a framing error (stop bit low).
//
// Synthesis: iCE40-portable, no vendor IP.

`default_nettype none

module uart_rx #(
    parameter int CLK_FREQ  = 12_000_000,
    parameter int BAUD_RATE = 115_200
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx,          // UART RX pin (idle high)
    output logic [7:0] rx_data,     // received byte; valid when rx_valid=1
    output logic       rx_valid,    // one-cycle pulse: byte ready
    output logic       rx_error     // one-cycle pulse: framing error
);

    // ---------------------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------------------
    localparam int CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;   // 104 @ 12 MHz/115200
    localparam int HALF_BIT      = CLKS_PER_BIT / 2;
    localparam int CTR_BITS      = $clog2(CLKS_PER_BIT + 1);
    localparam int BIT_CTR_BITS  = 4; // counts 0..7 data bits

    // ---------------------------------------------------------------------------
    // Input synchroniser (2-FF metastability protection)
    // ---------------------------------------------------------------------------
    logic rx_s0, rx_sync;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s0   <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_s0   <= rx;
            rx_sync <= rx_s0;
        end
    end

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
    logic [CTR_BITS-1:0] baud_ctr;    // bit-period countdown
    logic [BIT_CTR_BITS-1:0] bit_idx; // which data bit we're sampling (0..7)
    logic [7:0]          shift_reg;   // shift register, LSB first

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            baud_ctr  <= '0;
            bit_idx   <= '0;
            shift_reg <= '0;
            rx_data   <= '0;
            rx_valid  <= 1'b0;
            rx_error  <= 1'b0;
        end else begin
            // Default outputs (pulse for one cycle only)
            rx_valid <= 1'b0;
            rx_error <= 1'b0;

            case (state)
                // ---------------------------------------------------------------
                IDLE: begin
                    if (!rx_sync) begin
                        // Start-bit edge detected; wait half a bit period to
                        // sample at the centre of the start bit.
                        baud_ctr <= CTR_BITS'(HALF_BIT - 1);
                        state    <= START;
                    end
                end

                // ---------------------------------------------------------------
                START: begin
                    if (baud_ctr == '0) begin
                        // Sample centre of start bit — must still be low
                        if (!rx_sync) begin
                            // Valid start bit; set up for 8 data bits
                            baud_ctr <= CTR_BITS'(CLKS_PER_BIT - 1);
                            bit_idx  <= '0;
                            state    <= DATA;
                        end else begin
                            // Glitch; back to idle
                            state <= IDLE;
                        end
                    end else begin
                        baud_ctr <= baud_ctr - 1'b1;
                    end
                end

                // ---------------------------------------------------------------
                DATA: begin
                    if (baud_ctr == '0) begin
                        // Sample centre of data bit; shift in LSB-first
                        shift_reg <= {rx_sync, shift_reg[7:1]};
                        baud_ctr  <= CTR_BITS'(CLKS_PER_BIT - 1);
                        if (bit_idx == 3'd7) begin
                            bit_idx <= '0;
                            state   <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        baud_ctr <= baud_ctr - 1'b1;
                    end
                end

                // ---------------------------------------------------------------
                STOP: begin
                    if (baud_ctr == '0) begin
                        if (rx_sync) begin
                            // Valid stop bit
                            rx_data  <= shift_reg;
                            rx_valid <= 1'b1;
                        end else begin
                            // Framing error
                            rx_error <= 1'b1;
                        end
                        state <= IDLE;
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
