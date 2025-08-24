`timescale 1ns/1ps

// Basit spi_master modeli (yalnız handshake + echo)
module spi_master_model #(
    parameter integer WIDTH = 32,
    parameter integer LATENCY_CYC = 8,
    parameter        INJECT_ONE_BAD_ECHO = 1
)(
    input               clk_i,
    input               rst_n_i,       // aktif-high reset bekliyorum burada
    input               spi_enable_i,
    input  [WIDTH-1:0]  tx_data_i,
    output [WIDTH-1:0]  rx_data_o,
    output              data_valid_o,
    output              busy_o,
    output              mosi_o,
    input               miso_i,
    output              sclk_o,
    output              cs_o
);
    reg [WIDTH-1:0] r_rx, r_prev, r_tx;
    reg [7:0]       r_cnt;
    reg             r_busy, r_done, r_bad_used;
    assign rx_data_o = r_rx;
    assign data_valid_o = r_done;
    assign busy_o = r_busy;
    assign mosi_o = 1'b0; assign sclk_o = 1'b0; assign cs_o = 1'b1;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            r_rx<=0; r_prev<=0; r_tx<=0; r_cnt<=0; r_busy<=0; r_done<=0; r_bad_used<=0;
        end else begin
            r_done<=0;
            if (spi_enable_i && !r_busy) begin
                r_busy<=1; r_cnt<=LATENCY_CYC[7:0]; r_tx<=tx_data_i;
            end else if (r_busy) begin
                if (r_cnt==0) begin
                    r_busy<=0; r_done<=1;
                    if (INJECT_ONE_BAD_ECHO && !r_bad_used && (r_prev!=0)) begin
                        r_rx <= r_prev ^ {WIDTH{1'b1}}; r_bad_used<=1;
                    end else r_rx <= r_prev;
                    r_prev <= r_tx;
                end else r_cnt<=r_cnt-1;
            end
        end
    end
endmodule

module tb_ltc2666_top;
    localparam integer FRAME_BITS = 32;
    reg clk, rst_n;

    // Mesaj sinyalleri
    reg         msg_valid;
    wire        msg_ready;
    reg  [3:0]  msg_cmd;
    reg  [7:0]  msg_mask;
    reg  [15:0] msg_data;
    reg         clear_errors;

    // CLR / OVRTMP
    reg         clr_pulse;
    wire        clr_n;
    reg         ovrtmp_in;
    wire        ovrtmp_irq, ovrtmp_sticky;

    // SPI pinleri
    wire sclk, mosi, cs; wire miso; assign miso = 1'b0;

    // Durum/Hata
    wire busy, done, err_illegal, err_echo;

    // Controller (doğrudan master model ile bağlayalım)
    wire w_spi_start; wire [31:0] w_spi_tx; wire [31:0] w_spi_rx; wire w_spi_busy, w_spi_done;

    ltc2666_controller #(
        .FRAME_BITS           (FRAME_BITS),
        .APPEND_NOOP_FOR_ECHO (1),
        .CLR_PULSE_CC         (4)
    ) u_ctrl (
        .clk_i(clk), .rst_n_i(rst_n),

        .msg_valid_i    (msg_valid),
        .msg_ready_o    (msg_ready),
        .msg_cmd_i      (msg_cmd),
        .msg_chan_mask_i(msg_mask),
        .msg_data_i     (msg_data),
        .clear_errors_i (clear_errors),

        .spi_start_o (w_spi_start),
        .spi_tx_o    (w_spi_tx),
        .spi_rx_i    (w_spi_rx),
        .spi_busy_i  (w_spi_busy),
        .spi_done_i  (w_spi_done),

        .clr_pulse_i (clr_pulse),
        .clr_n_o     (clr_n),
        .ovrtmp_i    (ovrtmp_in),
        .ovrtmp_irq_o(ovrtmp_irq),
        .ovrtmp_sticky_o(ovrtmp_sticky),

        .busy_o      (busy),
        .done_o      (done),
        .err_illegal_o(err_illegal),
        .err_echo_o  (err_echo),
        .last_echo_exp_o(),
        .last_echo_rx_o()
    );

    // Master modeli
    spi_master_model #(.WIDTH(FRAME_BITS), .LATENCY_CYC(6), .INJECT_ONE_BAD_ECHO(1))
    u_spi_m (
        .clk_i        (clk),
        .rst_n_i      (rst_n),
        .spi_enable_i (w_spi_start),
        .tx_data_i    (w_spi_tx[FRAME_BITS-1:0]),
        .rx_data_o    (w_spi_rx[FRAME_BITS-1:0]),
        .data_valid_o (w_spi_done),
        .busy_o       (w_spi_busy),
        .mosi_o       (mosi),
        .miso_i       (miso),
        .sclk_o       (sclk),
        .cs_o         (cs)
    );

    // Saat/reset
    initial begin clk=0; forever #5 clk=~clk; end // 100 MHz
    initial begin rst_n=0; msg_valid=0; msg_cmd=0; msg_mask=0; msg_data=0;
                   clear_errors=0; clr_pulse=0; ovrtmp_in=1;
        #100 rst_n=1;
    end

    // Yardımcı task
    task send_msg(input [3:0] cmd, input [7:0] mask, input [15:0] data);
    begin
        @(posedge clk); while(!msg_ready) @(posedge clk);
        msg_cmd<=cmd; msg_mask<=mask; msg_data<=data; msg_valid<=1;
        @(posedge clk); msg_valid<=0;
        wait(done); @(posedge clk);
    end endtask

    // Senaryo
    initial begin
        @(posedge rst_n);
        // CLR darbesi
        @(posedge clk); clr_pulse<=1; @(posedge clk); clr_pulse<=0;

        // 1) CONFIG – iç referans açık (RD=0), termal shutdown aktif (TS=0)
        send_msg(4'b0111, 8'h00, 16'b00);
        // 2) WRITE_SPAN_N – tüm kanallar ±2.5 V (S2S1S0=100)
        send_msg(4'b0110, 8'hFF, 16'h0004);
        // 3) WRITE_CODE_N_UPD_N – ch0 ve ch2’ye orta kod
        send_msg(4'b0011, 8'b0000_0101, 16'h8000);
        // 4) UPDATE_ALL
        send_msg(4'b1001, 8'h00, 16'h0000);

        // OVRTMP düşür – irq ve sticky beklenir
        @(posedge clk); ovrtmp_in <= 0;
        @(posedge clk); ovrtmp_in <= 1;

        // echo hatası sticky → temizle
        @(posedge clk); clear_errors<=1; @(posedge clk); clear_errors<=0;

        #2000 $finish;
    end
endmodule
