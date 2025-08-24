//==============================================================================
// File   : ltc2666_top.v
// Amaç  : ltc2666_controller ↔ spi_master arasındaki ince bağlayıcı.
// Not   : spi_master portları ekran görüntündeki isimlere göre bağlandı.
//==============================================================================

`timescale 1ns/1ps
module ltc2666_top #(
    parameter integer FRAME_BITS           = 32,
    parameter        APPEND_NOOP_FOR_ECHO = 1
)(
    input               clk_i,
    input               rst_n_i,

    // Mesaj arayüzü
    input               msg_valid_i,
    output              msg_ready_o,
    input       [3:0]   msg_cmd_i,
    input       [7:0]   msg_chan_mask_i,
    input       [15:0]  msg_data_i,
    input               clear_errors_i,

    // CLR/OVRTMP
    input               clr_pulse_i,      // CLR için pulse isteği
    output              ltc_clr_n_o,      // DC2196 üzerindeki CLR pinine
    input               ltc_ovrtmp_i,     // DC2196 OVRTMP pini (pull-up ile)

    // Fiziksel SPI pinleri (DAC’a gider)
    output              spi_sclk_o,
    output              spi_mosi_o,
    input               spi_miso_i,
    output              spi_cs_o,

    // Durum/Hata
    output              busy_o,
    output              done_o,
    output              err_illegal_o,
    output              err_echo_o,
    output              ovrtmp_irq_o,
    output              ovrtmp_sticky_o
);
    // Controller<->master handshake
    wire                 w_spi_start;
    wire [31:0]          w_spi_tx;
    wire [31:0]          w_spi_rx;
    wire                 w_spi_busy;
    wire                 w_spi_done;

    // Controller
    ltc2666_controller #(
        .FRAME_BITS           (FRAME_BITS),
        .APPEND_NOOP_FOR_ECHO (APPEND_NOOP_FOR_ECHO),
        .CLR_PULSE_CC         (4)
    ) u_ctrl (
        .clk_i          (clk_i),
        .rst_n_i        (rst_n_i),

        .msg_valid_i    (msg_valid_i),
        .msg_ready_o    (msg_ready_o),
        .msg_cmd_i      (msg_cmd_i),
        .msg_chan_mask_i(msg_chan_mask_i),
        .msg_data_i     (msg_data_i),
        .clear_errors_i (clear_errors_i),

        .spi_start_o    (w_spi_start),
        .spi_tx_o       (w_spi_tx),
        .spi_rx_i       (w_spi_rx),
        .spi_busy_i     (w_spi_busy),
        .spi_done_i     (w_spi_done),

        .clr_pulse_i    (clr_pulse_i),
        .clr_n_o        (ltc_clr_n_o),
        .ovrtmp_i       (ltc_ovrtmp_i),
        .ovrtmp_irq_o   (ovrtmp_irq_o),
        .ovrtmp_sticky_o(ovrtmp_sticky_o),

        .busy_o         (busy_o),
        .done_o         (done_o),
        .err_illegal_o  (err_illegal_o),
        .err_echo_o     (err_echo_o),
        .last_echo_exp_o(),
        .last_echo_rx_o ()
    );

    // SPI Master – ekran görüntüsündeki port isimleri
    spi_master #(
        .SPI_WIDTH (FRAME_BITS),
        .SPI_MODE  (0)           // Mode-0
    ) u_spi (
        .clk_i        (clk_i),
        .rst_a_l      (rst_n_i),                       // aktif düşük reset
        .spi_enable_i (w_spi_start),
        .tx_data_i    (w_spi_tx[FRAME_BITS-1:0]),
        .rx_data_o    (w_spi_rx[FRAME_BITS-1:0]),
        .mosi_o       (spi_mosi_o),
        .miso_i       (spi_miso_i),
        .sclk_o       (spi_sclk_o),
        .cs_o         (spi_cs_o),
        .data_valid_o (w_spi_done),
        .busy_o       (w_spi_busy)
    );
endmodule
