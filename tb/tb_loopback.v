// tb_loopback.v — thin Verilog wrapper that ties uart_tx.tx_pin → uart_rx.rx
// so cocotb test_uart_loopback.py can instantiate a single DUT top-level.

`default_nettype none

module tb_loopback #(
    parameter CLK_FREQ  = 12_000_000,
    parameter BAUD_RATE = 115_200
) (
    input  wire       clk,
    input  wire       rst_n,
    // TX side (driven by cocotb)
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    // RX side (read by cocotb)
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire       rx_error
);
    wire tx_pin;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_data(tx_data), .tx_valid(tx_valid),
        .tx_ready(tx_ready), .tx_pin(tx_pin)
    );

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_rx (
        .clk(clk), .rst_n(rst_n),
        .rx(tx_pin),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_error(rx_error)
    );
endmodule

`default_nettype wire
