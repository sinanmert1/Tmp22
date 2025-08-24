//==============================================================================
// File        : ltc2666_controller.v
// Amaç       : Kaynak-agnostik (CPU/UART/DMA) komut mesajlarını LTC2666 DAC
//              için 24/32-bit SPI çerçevelerine dönüştürmek. LDAC kullanılmaz.
//              32-bit modda SDO “echo” ile önceki çerçeveyi doğrular.
//              CLR (active-low) pin sürümü ve OVRTMP alarm girişi içerir.
// Zamanlama   : SPI Mode-0 (CPOL=0, CPHA=0). SCK ≤ 15 MHz (master tarafı).
//==============================================================================

`timescale 1ns/1ps
module ltc2666_controller #(
    parameter integer FRAME_BITS           = 32, // 24→echo kapalı, 32→echo açık
    parameter        APPEND_NOOP_FOR_ECHO = 1,  // son gerçek çerçeve echo’su için NOOP ekle
    parameter integer CLR_PULSE_CC        = 4   // CLR aktif (LOW) tutulacak clock sayısı (min 30ns+)
)(
    // Sistem saat & reset
    input               clk_i,
    input               rst_n_i,

    //---------------------- Kaynak (CPU/UART/DMA) mesaj arayüzü ----------------
    // Sözleşme:
    //  msg_cmd_i[3:0]  : LTC2666 komutu (aşağıdaki sabitler)
    //  msg_chan_mask_i : n-tipi komutlarda hedef kanallar (bit=1 → seçili)
    //  msg_data_i      : komuta göre anlam:
    //    WRITE_CODE_*     : 16-bit straight-binary kod
    //    WRITE_SPAN_*     : data[2:0]={S2,S1,S0}
    //    UPDATE_*         : don’t care
    //    POWER_DOWN_*     : don’t care
    //    ANALOG_MUX       : data[4:0]={M4..M0}
    //    TOGGLE_SELECT    : data[7:0]={T7..T0}
    //    GLOBAL_TOGGLE    : data[0]=TGB
    //    CONFIG           : data[1:0]={TS,RD}
    input               msg_valid_i,
    output              msg_ready_o,
    input       [3:0]   msg_cmd_i,
    input       [7:0]   msg_chan_mask_i,
    input       [15:0]  msg_data_i,
    input               clear_errors_i,   // echo hata latch temizleme

    //---------------------- SPI master el sıkışma -------------------------------
    // Not: Bu arayüz senin spi_master.v modülüne bağlanır (enable/tx/rx/done/busy).
    output              spi_start_o,      // → spi_enable_i (1-cycle pulse)
    output      [31:0]  spi_tx_o,         // → tx_data_i[FRAME_BITS-1:0]
    input       [31:0]  spi_rx_i,         // ← rx_data_o
    input               spi_busy_i,       // ← busy_o
    input               spi_done_i,       // ← data_valid_o

    //---------------------- Ek pinler / durum ----------------------------------
    // CLR: Datasheet’e göre aktif LOW. Burada pulse üretmek için clr_pulse_i var.
    input               clr_pulse_i,      // 1-cycle verildiğinde CLR minimum süre LOW yapılır
    output              clr_n_o,          // DAC CLR pinine gider (default HIGH)
    // OVRTMP: open-drain alarm girişi. LOW’a düştüğünde irq üretir (edge).
    input               ovrtmp_i,
    output              ovrtmp_irq_o,     // LOW’a düşüşte 1-cycle pulse
    output              ovrtmp_sticky_o,  // bir kez LOW gördüyse set kalır; clear_errors_i ile silinir

    // Durum / tanılama
    output              busy_o,           // mesaj işlenirken 1
    output              done_o,           // mesaj bittiğinde 1-pulse
    output              err_illegal_o,    // geçersiz kombinasyon (örn. N+UPD_ALL ama mask multi-bit)
    output              err_echo_o,       // echo uyuşmazlığı (sticky, clear_errors_i ile temizlenir)
    output      [31:0]  last_echo_exp_o,  // son beklenen echo (önceki tx)
    output      [31:0]  last_echo_rx_o    // son alınan echo (rx)
);

    //---------------------- Komut sabitleri (C3..C0) ---------------------------
    localparam [3:0] CMD_WRITE_CODE_N            = 4'b0000;
    localparam [3:0] CMD_UPDATE_N                = 4'b0001;
    localparam [3:0] CMD_WRITE_CODE_N_UPD_ALL    = 4'b0010;
    localparam [3:0] CMD_WRITE_CODE_N_UPD_N      = 4'b0011;
    localparam [3:0] CMD_POWER_DOWN_N            = 4'b0100;
    localparam [3:0] CMD_POWER_DOWN_CHIP         = 4'b0101;
    localparam [3:0] CMD_WRITE_SPAN_N            = 4'b0110;
    localparam [3:0] CMD_CONFIG                  = 4'b0111;
    localparam [3:0] CMD_WRITE_CODE_ALL          = 4'b1000;
    localparam [3:0] CMD_UPDATE_ALL              = 4'b1001;
    localparam [3:0] CMD_WRITE_CODE_ALL_UPD_ALL  = 4'b1010;
    localparam [3:0] CMD_ANALOG_MUX              = 4'b1011;
    localparam [3:0] CMD_TOGGLE_SELECT           = 4'b1100;
    localparam [3:0] CMD_GLOBAL_TOGGLE           = 4'b1101;
    localparam [3:0] CMD_NOOP                    = 4'b1111;

    //---------------------- Yardımcı fonksiyonlar ------------------------------
    function [23:0] frame24;
        input [3:0]  cmd; input [3:0] addr; input [15:0] data;
        begin frame24 = {cmd, addr, data}; end
    endfunction
    function [31:0] frame32_pad;
        input [23:0] f24;
        begin frame32_pad = {8'h00, f24}; end
    endfunction
    function [3:0] popcount8;
        input [7:0] x; integer i; reg [3:0] c;
        begin c=0; for (i=0;i<8;i=i+1) c=c+x[i]; popcount8=c; end
    endfunction

    //---------------------- İç kayıtlar (portlar assign ile sürülür) ----------
    reg              r_spi_start;
    reg      [31:0]  r_spi_tx;

    reg              r_busy, r_done;
    reg              r_err_illegal, r_err_echo;
    reg      [31:0]  r_last_echo_exp, r_last_echo_rx;

    reg              r_clr_n;
    reg       [7:0]  r_clr_cnt;
    reg              r_ovrtmp_d, r_ovrtmp_irq, r_ovrtmp_sticky;

    assign spi_start_o      = r_spi_start;
    assign spi_tx_o         = r_spi_tx;
    assign busy_o           = r_busy;
    assign done_o           = r_done;
    assign err_illegal_o    = r_err_illegal;
    assign err_echo_o       = r_err_echo;
    assign last_echo_exp_o  = r_last_echo_exp;
    assign last_echo_rx_o   = r_last_echo_rx;

    assign clr_n_o          = r_clr_n;           // aktif LOW
    assign ovrtmp_irq_o     = r_ovrtmp_irq;
    assign ovrtmp_sticky_o  = r_ovrtmp_sticky;

    // hazır bilgisi (IDLE durumunda 1)
    wire w_msg_ready;
    assign msg_ready_o = w_msg_ready;

    //---------------------- Mesaj bağlamı / FSM değişkenleri -------------------
    reg  [3:0]  cur_cmd;
    reg  [7:0]  cur_mask;
    reg  [15:0] cur_data;
    reg  [3:0]  chan_idx;
    reg         use_all, per_channel, single_n_upd_all;

    reg         have_prev;       // echo için “önceki tx” var mı?
    reg  [31:0] prev_word;       // beklenen echo (bir önceki tx)
    reg         appended_noop;   // son gerçek çerçeveden sonra NOOP eklendi mi?

    wire echo_en = (FRAME_BITS == 32);

    // Durumlar:
    //  S_IDLE  : Boşta; yeni mesaj bekle
    //  S_LOAD  : Sıradaki SPI kelimesini oluştur
    //  S_ISSUE : SPI master’ı start ile tetikle
    //  S_WAIT  : Master done pulse’ını bekle
    //  S_NEXT  : Sonraki kanal/işleme karar ver
    //  S_DONE  : Mesaj tamam (done pulse)
    localparam [2:0] S_IDLE=0, S_LOAD=1, S_ISSUE=2, S_WAIT=3, S_NEXT=4, S_DONE=5;
    reg [2:0] st, st_n;

    assign w_msg_ready = (st == S_IDLE);
    wire [3:0] addr_from_idx = chan_idx[3:0];

    //---------------------- Senkron kısım -------------------------------------
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            // genel
            st               <= S_IDLE;
            r_busy           <= 1'b0; r_done <= 1'b0;
            r_err_illegal    <= 1'b0; r_err_echo <= 1'b0;
            r_spi_start      <= 1'b0; r_spi_tx   <= 32'd0;
            r_last_echo_exp  <= 32'd0; r_last_echo_rx <= 32'd0;
            have_prev        <= 1'b0; prev_word  <= 32'd0;
            appended_noop    <= 1'b0;
            cur_cmd          <= 4'd0; cur_mask   <= 8'd0; cur_data <= 16'd0;
            chan_idx         <= 4'd0; use_all    <= 1'b0; per_channel <= 1'b0;
            single_n_upd_all <= 1'b0;

            // CLR & OVRTMP
            r_clr_n          <= 1'b1; r_clr_cnt <= 0;
            r_ovrtmp_d       <= 1'b1; r_ovrtmp_irq <= 1'b0; r_ovrtmp_sticky <= 1'b0;
        end else begin
            st            <= st_n;
            r_spi_start   <= 1'b0;    // start pulse default 0
            r_done        <= 1'b0;
            r_err_illegal <= 1'b0;
            r_ovrtmp_irq  <= 1'b0;

            // echo hata temizleme ve OVRTMP sticky temizleme
            if (clear_errors_i) begin
                r_err_echo       <= 1'b0;
                r_ovrtmp_sticky  <= 1'b0;
            end

            //--- ECHO yakalama (her transfer bitiminde) ---
            if (spi_done_i && echo_en) begin
                r_last_echo_rx  <= spi_rx_i;
                r_last_echo_exp <= prev_word;
                if (have_prev && (spi_rx_i != prev_word)) r_err_echo <= 1'b1;
            end

            //--- CLR pulse jeneratörü (aktif LOW) ---
            if (clr_pulse_i)        r_clr_cnt <= (CLR_PULSE_CC>0) ? CLR_PULSE_CC[7:0] : 8'd1;
            if (r_clr_cnt != 0) begin
                r_clr_n  <= 1'b0;
                r_clr_cnt<= r_clr_cnt - 1'b1;
            end else begin
                r_clr_n  <= 1'b1;   // default de-asserted
            end

            //--- OVRTMP kenar yakalama (LOW’a düşüşte irq + sticky) ---
            r_ovrtmp_d <= ovrtmp_i;
            if (r_ovrtmp_d && !ovrtmp_i) begin
                r_ovrtmp_irq    <= 1'b1;
                r_ovrtmp_sticky <= 1'b1;
            end

            //--- FSM ana akış ---
            case (st)
            // IDLE: yeni mesaj bekle; kabul edilince bağlamı yükle
            S_IDLE: begin
                r_busy        <= 1'b0;
                have_prev     <= 1'b0;       // yeni mesaj → echo zincirini sıfırla
                appended_noop <= 1'b0;
                if (msg_valid_i) begin
                    cur_cmd  <= msg_cmd_i;
                    cur_mask <= msg_chan_mask_i;
                    cur_data <= msg_data_i;

                    use_all   <= (msg_cmd_i==CMD_WRITE_CODE_ALL) ||
                                 (msg_cmd_i==CMD_UPDATE_ALL) ||
                                 (msg_cmd_i==CMD_WRITE_CODE_ALL_UPD_ALL);
                    per_channel <= (msg_cmd_i==CMD_WRITE_CODE_N)       ||
                                   (msg_cmd_i==CMD_UPDATE_N)           ||
                                   (msg_cmd_i==CMD_WRITE_SPAN_N)       ||
                                   (msg_cmd_i==CMD_WRITE_CODE_N_UPD_N) ||
                                   (msg_cmd_i==CMD_POWER_DOWN_N);
                    single_n_upd_all <= (msg_cmd_i==CMD_WRITE_CODE_N_UPD_ALL);

                    // Kural: WRITE_CODE_N_UPD_ALL için mask tek bit olmalı
                    if ((msg_cmd_i==CMD_WRITE_CODE_N_UPD_ALL) && (popcount8(msg_chan_mask_i)!=1)) begin
                        r_err_illegal <= 1'b1;
                        r_done        <= 1'b1;
                    end else begin
                        r_busy   <= 1'b1;
                        chan_idx <= 4'd0;
                    end
                end
            end

            // ISSUE: master meşgul değilse tek transfer başlat
            S_ISSUE: begin
                if (!spi_busy_i) begin
                    r_spi_start <= 1'b1;                 // 1 clock enable
                    if (echo_en) begin
                        prev_word <= r_spi_tx;           // bir SONRAKİ transferde echo olarak gelmeli
                        have_prev <= 1'b1;
                    end
                end
            end

            // DONE: mesaj tamam pulse
            S_DONE: begin
                r_busy <= 1'b0;
                r_done <= 1'b1;
            end

            default: ; // S_LOAD/S_WAIT/S_NEXT kombinasyonelde ele alınıyor
            endcase
        end
    end

    //---------------------- Kombinasyonel: next-state + TX builder -------------
    reg [31:0] tx_next;
    integer    k; reg [3:0] oneidx;

    always @* begin
        st_n    = st;
        tx_next = r_spi_tx;

        // IDLE→LOAD koşulu
        if (st==S_IDLE && msg_valid_i &&
           !((msg_cmd_i==CMD_WRITE_CODE_N_UPD_ALL)&&(popcount8(msg_chan_mask_i)!=1)))
            st_n = S_LOAD;

        case (st)
        // S_LOAD: sıradaki gönderilecek kelimeyi üret
        S_LOAD: begin
            if (per_channel) begin
                // maskede 1 olan bir kanal bulunana dek taranır
                if (cur_mask[chan_idx]) begin
                    tx_next = (FRAME_BITS==32) ?
                              frame32_pad(frame24(cur_cmd, addr_from_idx, cur_data)) :
                              {8'h00, frame24(cur_cmd, addr_from_idx, cur_data)};
                    st_n = S_ISSUE;
                end else if (chan_idx==4'd7) begin
                    // maske boşsa: echo için gerekiyorsa NOOP ekle
                    if (echo_en && APPEND_NOOP_FOR_ECHO && have_prev && !appended_noop) begin
                        tx_next = frame32_pad(frame24(CMD_NOOP,4'h0,16'h0000));
                        st_n    = S_ISSUE;
                    end else begin
                        st_n = S_DONE;
                    end
                end else begin
                    st_n = S_LOAD; // taramaya devam; chan_idx ilerleyişi senkron blokta
                end
            end else if (use_all) begin
                tx_next = (FRAME_BITS==32) ?
                          frame32_pad(frame24(cur_cmd, 4'h0, cur_data)) :
                          {8'h00, frame24(cur_cmd, 4'h0, cur_data)};
                st_n = S_ISSUE;
            end else if (single_n_upd_all) begin
                oneidx = 4'd0; for (k=0;k<8;k=k+1) if (cur_mask[k]) oneidx = k[3:0];
                tx_next = (FRAME_BITS==32) ?
                          frame32_pad(frame24(cur_cmd, oneidx, cur_data)) :
                          {8'h00, frame24(cur_cmd, oneidx, cur_data)};
                st_n = S_ISSUE;
            end else begin
                // tek atımlık komutlar: CONFIG, MUX, TOGGLE*, PWRDN_CHIP, NOOP
                tx_next = (FRAME_BITS==32) ?
                          frame32_pad(frame24(cur_cmd, 4'h0, cur_data)) :
                          {8'h00, frame24(cur_cmd, 4'h0, cur_data)};
                st_n = S_ISSUE;
            end
        end

        // S_ISSUE: master tetiklendiyse WAIT’e geç
        S_ISSUE:  if (!spi_busy_i) st_n = S_WAIT;

        // S_WAIT: master done pulse üretince NEXT’e geç
        S_WAIT :  if (spi_done_i)  st_n = S_NEXT;

        // S_NEXT: sonraki hedef
        S_NEXT: begin
            if (per_channel) begin
                if (chan_idx < 4'd7) begin
                    st_n = S_LOAD; // senkron blok chan_idx’i artıracak
                end else begin
                    // son kanal sonrası: echo için NOOP eklenebilir
                    if (echo_en && APPEND_NOOP_FOR_ECHO && !appended_noop) begin
                        tx_next = frame32_pad(frame24(CMD_NOOP,4'h0,16'h0000));
                        st_n    = S_ISSUE;
                    end else begin
                        st_n = S_DONE;
                    end
                end
            end else begin
                if (echo_en && APPEND_NOOP_FOR_ECHO && !appended_noop) begin
                    tx_next = frame32_pad(frame24(CMD_NOOP,4'h0,16'h0000));
                    st_n    = S_ISSUE;
                end else begin
                    st_n = S_DONE;
                end
            end
        end

        default: ;
        endcase
    end

    // TX yaz ve kanal taramasını ilerlet; NOOP eklendi flag’ini güncelle
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            r_spi_tx      <= 32'd0;
            chan_idx      <= 4'd0;
            appended_noop <= 1'b0;
        end else begin
            if (st==S_LOAD) r_spi_tx <= tx_next;

            if (st==S_LOAD && per_channel && !cur_mask[chan_idx] && (chan_idx<4'd7))
                chan_idx <= chan_idx + 1'b1;
            else if (st==S_NEXT && per_channel && (chan_idx<4'd7))
                chan_idx <= chan_idx + 1'b1;

            if (st==S_ISSUE && r_spi_start && (r_spi_tx[31:24]==8'h00) && (r_spi_tx[23:20]==CMD_NOOP))
                appended_noop <= 1'b1;
        end
    end
endmodule
