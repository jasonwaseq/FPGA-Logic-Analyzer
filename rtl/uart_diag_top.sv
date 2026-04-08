// uart_diag_top.sv — minimal UART hardware diagnostic image for iCEBreaker
//
// Purpose:
//   - Continuously emits 0x55 beacon bytes at 5 Hz
//   - Echoes any received UART byte back to host
//   - Lets us isolate physical UART routing issues from LA protocol logic

`timescale 1ns/1ps
`default_nettype none

module uart_diag_top #(
    parameter int CLK_FREQ  = 12_000_000,
    parameter int BAUD_RATE = 115_200
) (
    input  logic clk,
    input  logic rst_n,
    input  logic uart_rx_pin,
    output logic uart_tx_pin,
    output logic led_armed,
    output logic led_triggered,
    output logic led_complete
);
    logic [2:0] por_count = '0;
    logic       rst_n_sys = 1'b0;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            por_count <= '0;
            rst_n_sys <= 1'b0;
        end else if (!rst_n_sys) begin
            por_count <= por_count + 3'd1;
            if (&por_count) rst_n_sys <= 1'b1;
        end
    end

    logic [7:0] rx_data;
    logic       rx_valid;
    logic       rx_error;

    logic [7:0] tx_data;
    logic       tx_valid;
    logic       tx_ready;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_rx (
        .clk     (clk),
        .rst_n   (rst_n_sys),
        .rx      (uart_rx_pin),
        .rx_data (rx_data),
        .rx_valid(rx_valid),
        .rx_error(rx_error)
    );

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tx (
        .clk    (clk),
        .rst_n  (rst_n_sys),
        .tx_data (tx_data),
        .tx_valid(tx_valid),
        .tx_ready(tx_ready),
        .tx_pin  (uart_tx_pin)
    );

    localparam int BEACON_CLKS = CLK_FREQ / 5; // 5 Hz
    localparam int BCW = $clog2(BEACON_CLKS + 1);

    logic [BCW-1:0] beacon_ctr;
    logic [7:0]     pending_data;
    logic           pending_valid;
    logic           seen_rx;
    logic           seen_rx_err;

    always_ff @(posedge clk) begin
        if (!rst_n_sys) begin
            beacon_ctr    <= '0;
            pending_data  <= 8'h00;
            pending_valid <= 1'b0;
            seen_rx       <= 1'b0;
            seen_rx_err   <= 1'b0;
            tx_data       <= 8'h00;
            tx_valid      <= 1'b0;
        end else begin
            tx_valid <= 1'b0;

            if (rx_valid) begin
                seen_rx <= 1'b1;
                if (!pending_valid) begin
                    pending_data  <= rx_data;
                    pending_valid <= 1'b1;
                end
            end
            if (rx_error) seen_rx_err <= 1'b1;

            if (beacon_ctr == BEACON_CLKS - 1) begin
                beacon_ctr <= '0;
                if (!pending_valid) begin
                    pending_data  <= 8'h55;
                    pending_valid <= 1'b1;
                end
            end else begin
                beacon_ctr <= beacon_ctr + 1'b1;
            end

            if (pending_valid && tx_ready) begin
                tx_data       <= pending_data;
                tx_valid      <= 1'b1;
                pending_valid <= 1'b0;
            end
        end
    end

    assign led_armed     = ~beacon_ctr[BCW-1];
    assign led_triggered = seen_rx;
    assign led_complete  = seen_rx_err;
endmodule

`default_nettype wire
