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
/*-/*-/*/-*/*-/*-
//==============================================================================
//  File   : ltc2666_controller_single.v
//  Amaç   : LTC2666 için tek modül controller (içinde SPI Master instantiation)
//           - Açılış/RESET/CLR sonrası oto-init: CONFIG + SPAN(±2.5V, CH0..CH7)
//           - Echo doğrulaması (init sonunda TEK iç NOOP; RUN'da NOOP YOK)
//           - CPU'dan gelen tekil kanal/16-bit kod komutlarını sürme
//           - IN_RANGE koruması: ±ALLOWED_MV dışındaki kodları sürme (drop + flag)
//  Notlar : * Tüm state/karar/çıkış TEK clocked blokta (combinational yok)
//           * Active-low uçlar `_al` sonekli (rst_al_i, cs_al, clr_al, ovrtmp_al)
//           * İç sinyaller *_r sonekli coding standardına uygun
//==============================================================================

`timescale 1ns/1ps
module ltc2666_controller_single #(
    // Sistem/saat
    parameter integer CLK_HZ                = 100_000_000,

    // SPI
    parameter integer SPI_SCK_HZ            = 1_000_000,   // 1 MHz
    parameter integer FRAME_BITS            = 32,          // 24 veya 32 (echo için 32 önerilir)

    // Init / Retry
    // INIT_RETRY_MAX=0 => SINIRSIZ tekrar dene (başarana kadar)
    parameter integer INIT_RETRY_MAX        = 0,

    // CLR darbe süresi (clock sayısı)
    parameter integer CLR_PULSE_CC          = 8,

    // Soft-span sabiti (±2.5V için data=0x0004; datasheet)
    parameter [15:0]  SOFTSPAN_CODE         = 16'h0004,

    // IN_RANGE koruması (mV), span = ±2500 mV varsayımı
    parameter integer SPAN_MV               = 2500,
    parameter integer ALLOWED_MV            = 1500,

    // Kod eşikleri (Offset-Binary)
    parameter [15:0]  CODE_ZERO             = 16'h8000, // 0V
    parameter [15:0]  CODE_POS_FS           = 16'hFFFF, // +FS
    parameter [15:0]  CODE_NEG_FS           = 16'h0000  // -FS
)(
    // Saat/Reset (aktif-düşük)
    input  clk_i,
    input  rst_al_i,

    // CPU komutu: 1-clk yaz palsi (handshake yok)
    input              cmd_we_i,        // 1: bu clock'ta yaz
    input      [2:0]   cmd_chan_i,      // 0..7
    input      [15:0]  cmd_code_i,      // LTC2666 16-bit ham kod (Offset-Binary)

    // Kontrol
    input              clr_req_i,       // 1-clk: CLR pinini low yap + oto-init
    input              start_init_i,    // 1-clk: manuel init tetik
    input              clear_errors_i,  // sticky bayrakları temizle

    // Donanım alarm (harici I/O expander'dan, aktif-düşük)
    input              ovrtmp_al_i,     // 0: aşırı sıcak alarm

    // Durum / hatalar
    output             busy_o,                 // init veya SPI aktifken 1
    output             inited_o,               // init tamam ve doğrulama OK
    output             conf_readback_error_o,  // init boyunca 1; başarıyla bitince 0
    output             init_failed_o,          // (yalnız sınırlı retry modunda) denemeler biterse 1
    output             echo_error_o,           // echo uyuşmazlığı (sticky)
    output             in_range_error_o,       // izinli aralık dışı komut geldi (sticky)
    output             ovrtmp_sticky_o,        // overtemp sticky

    // Debug (opsiyonel)
    output     [31:0]  last_tx_o,
    output     [31:0]  last_rx_o,

    // LTC2666 SPI hatları (aktif-düşük CS)
    output             spi_sclk_o,
    output             spi_mosi_o,
    input              spi_miso_i,
    output             spi_cs_al_o,

    // LTC2666 CLR (aktif-düşük)
    output             ltc_clr_al_o
);

    //--------------------------------------------------------------------------
    // Komut nibble’ları
    //--------------------------------------------------------------------------
    localparam [3:0] CMD_WRITE_CODE_N           = 4'b0000;
    localparam [3:0] CMD_UPDATE_N               = 4'b0001;
    localparam [3:0] CMD_WRITE_CODE_N_UPD_N     = 4'b0011;
    localparam [3:0] CMD_WRITE_SPAN_N           = 4'b0110;
    localparam [3:0] CMD_CONFIG                 = 4'b0111;
    localparam [3:0] CMD_NOOP                   = 4'b1111;

    //--------------------------------------------------------------------------
    // IN_RANGE eşikleri (Offset-Binary)
    //--------------------------------------------------------------------------
    localparam integer CODE_HALF_RANGE = (CODE_POS_FS > CODE_ZERO) ? (CODE_POS_FS - CODE_ZERO)
                                                                   : (CODE_ZERO - CODE_POS_FS);
    localparam integer ALLOWED_DELTA_CODE = (ALLOWED_MV * CODE_HALF_RANGE) / SPAN_MV;
    localparam [15:0]  CODE_MIN_ALLOWED = CODE_ZERO - ALLOWED_DELTA_CODE[15:0];
    localparam [15:0]  CODE_MAX_ALLOWED = CODE_ZERO + ALLOWED_DELTA_CODE[15:0];

    // 24/32-bit çerçeveyi 32-bit'e pad'ler (karşılaştırma kolaylığı)
    function [31:0] frame_f;
        input [3:0]  cmd;
        input [3:0]  addr;
        input [15:0] data;
        begin
            frame_f = {8'h00, cmd, addr, data}; // 24'te de üst 8 bit 0
        end
    endfunction

    //--------------------------------------------------------------------------
    // İç sinyaller (*_r)
    //--------------------------------------------------------------------------
    // Tek girişlik komut tamponu
    reg        pend_valid_r;
    reg [2:0]  pend_chan_r;
    reg [15:0] pend_code_r;

    // CLR kontrolü (aktif-düşük çıkış)
    reg        clr_busy_r;
    reg [15:0] clr_cnt_r;
    reg        ltc_clr_al_r;  assign ltc_clr_al_o = ltc_clr_al_r;

    // SPI master arayüzü
    reg         spi_start_r;
    reg  [31:0] spi_tx_r;
    wire [31:0] spi_rx_w;
    wire        spi_done_w;
    wire        spi_busy_w;

    // Echo takibi
    reg         have_prev_r;
    reg  [31:0] prev_tx_r;
    reg  [31:0] last_rx_r, last_tx_r;
    assign last_rx_o = last_rx_r;
    assign last_tx_o = last_tx_r;

    // Hata/state bayrakları
    reg conf_err_r;         // açılışta 1; init başarıyla bitince 0
    reg init_ok_r;
    reg init_failed_r;
    reg echo_error_r;       // sticky
    reg in_range_error_r;   // sticky

    // OVRTMP sticky (aktif-düşük giriş)
    reg ovrtmp_q_r, ovrtmp_sticky_r;

    // Init FSM (tamamen senkron)
    localparam [3:0]
        S_RESET     = 4'd0,  // açılış/CLR sonrası
        S_INIT_CFG  = 4'd1,  // CONFIG gönder
        S_INIT_SPAN = 4'd2,  // CHn SPAN gönder
        S_INIT_WAIT = 4'd3,  // DONE bekle (echo işleme)
        S_INIT_NEXT = 4'd4,  // sonraki adım / kanal ilerlet
        S_INIT_NOOP = 4'd5,  // init sonu: echo almak için tek NOOP
        S_RUN_ISSUE = 4'd6,  // çalışma: yazış başlat
        S_RUN_WAIT  = 4'd7,  // çalışma: DONE + echo kontrol
        S_RUN_IDLE  = 4'd9;  // normal bekleme

    reg [3:0] init_state_r;
    reg [2:0] span_chan_r;
    reg [7:0] retry_left_r; // 0=limitsiz için kullanılmıyor

    // Dış durum
    assign busy_o                = (init_state_r != S_RUN_IDLE) || spi_busy_w;
    assign inited_o              = init_ok_r && !conf_err_r;
    assign conf_readback_error_o = conf_err_r;
    assign init_failed_o         = init_failed_r;
    assign echo_error_o          = echo_error_r;
    assign in_range_error_o      = in_range_error_r;
    assign ovrtmp_sticky_o       = ovrtmp_sticky_r;

    //==========================================================================
    // SPI MASTER – kendi çekirdeğinle eşle (port adları uyumluysa direkt çalışır)
    //==========================================================================
    spi_master_core #(
        .CLK_HZ     (CLK_HZ),
        .SPI_SCK_HZ (SPI_SCK_HZ),
        .FRAME_BITS (FRAME_BITS)
    ) u_spi (
        .clk_i        (clk_i),
        .rst_al_i     (rst_al_i),
        .spi_enable_i (spi_start_r),             // 1-clk start
        .tx_data_i    ( (FRAME_BITS==32) ? spi_tx_r
                                         : {8'h00, spi_tx_r[23:0]} ), // garanti 32->24 pad
        .rx_data_o    (spi_rx_w),
        .data_valid_o (spi_done_w),              // DONE
        .busy_o       (spi_busy_w),
        .mosi_o       (spi_mosi_o),
        .miso_i       (spi_miso_i),
        .sclk_o       (spi_sclk_o),
        .cs_al_o      (spi_cs_al_o)              // aktif-düşük CS
    );

    //==========================================================================
    // TEK CLOCKED BLOK
    //==========================================================================
    always @(posedge clk_i or negedge rst_al_i) begin
        if (!rst_al_i) begin
            // Reset
            pend_valid_r      <= 1'b0;

            ltc_clr_al_r      <= 1'b1;
            clr_busy_r        <= 1'b0;
            clr_cnt_r         <= 16'd0;

            spi_start_r       <= 1'b0;
            spi_tx_r          <= 32'd0;

            have_prev_r       <= 1'b0;
            prev_tx_r         <= 32'd0;
            last_rx_r         <= 32'd0;
            last_tx_r         <= 32'd0;

            conf_err_r        <= 1'b1;    // init bitene kadar 1
            init_ok_r         <= 1'b0;
            init_failed_r     <= 1'b0;
            echo_error_r      <= 1'b0;
            in_range_error_r  <= 1'b0;

            ovrtmp_q_r        <= 1'b1;    // aktif-düşük; 1=normal
            ovrtmp_sticky_r   <= 1'b0;

            init_state_r      <= S_RESET;
            span_chan_r       <= 3'd0;
            retry_left_r      <= (INIT_RETRY_MAX==0) ? 8'd0 : INIT_RETRY_MAX[7:0];

        end else begin
            // ----- clock başı defaultlar
            spi_start_r <= 1'b0;

            // DONE geldi mi? Echo işleme (her transfer sonrası)
            if (spi_done_w) begin
                last_rx_r <= (FRAME_BITS==32) ? spi_rx_w : {8'h00, spi_rx_w[23:0]};
                last_tx_r <= prev_tx_r; // bir önce yollanan (beklenen echo)
                if (have_prev_r && ( ((FRAME_BITS==32)?spi_rx_w:{8'h00,spi_rx_w[23:0]}) != prev_tx_r ))
                    echo_error_r <= 1'b1; // sticky
                prev_tx_r   <= spi_tx_r;  // bu transferde yollanan sonraki frame için referans
                have_prev_r <= 1'b1;
            end

            // Sticky clear
            if (clear_errors_i) begin
                echo_error_r      <= 1'b0;
                in_range_error_r  <= 1'b0;
                ovrtmp_sticky_r   <= 1'b0;
                // conf_err_r init akışında düşecek; burada dokunmuyoruz
            end

            // OVRTMP sticky (aktif-düşük giriş)
            ovrtmp_q_r <= ovrtmp_al_i;
            if (ovrtmp_q_r==1'b1 && ovrtmp_al_i==1'b0)
                ovrtmp_sticky_r <= 1'b1;

            // Tek girişlik komut tamponu (busy'de iken yazılırsa görmezden gelinir)
            if (cmd_we_i && !pend_valid_r) begin
                pend_valid_r <= 1'b1;
                pend_chan_r  <= cmd_chan_i;
                pend_code_r  <= cmd_code_i;
            end

            // CLR darbesi (aktif-düşük çıkış) + init’e dön
            if (clr_req_i && !clr_busy_r) begin
                clr_busy_r    <= 1'b1;
                clr_cnt_r     <= CLR_PULSE_CC[15:0];
                ltc_clr_al_r  <= 1'b0;  // low
                // init parametreleri
                init_state_r  <= S_RESET;
                conf_err_r    <= 1'b1;
                init_ok_r     <= 1'b0;
                init_failed_r <= 1'b0;
                echo_error_r  <= 1'b0;
                have_prev_r   <= 1'b0;
                span_chan_r   <= 3'd0;
                retry_left_r  <= (INIT_RETRY_MAX==0) ? 8'd0 : INIT_RETRY_MAX[7:0];
            end else if (clr_busy_r) begin
                if (clr_cnt_r==0) begin
                    clr_busy_r    <= 1'b0;
                    ltc_clr_al_r  <= 1'b1;  // high
                end else begin
                    clr_cnt_r <= clr_cnt_r - 16'd1;
                end
            end

            // Manuel init tetik
            if (start_init_i) begin
                init_state_r  <= S_RESET;
                conf_err_r    <= 1'b1;
                init_ok_r     <= 1'b0;
                init_failed_r <= 1'b0;
                echo_error_r  <= 1'b0;
                have_prev_r   <= 1'b0;
                span_chan_r   <= 3'd0;
                retry_left_r  <= (INIT_RETRY_MAX==0) ? 8'd0 : INIT_RETRY_MAX[7:0];
            end

            //========================
            //  FSM — TAMAMEN SENKRON
            //========================
            case (init_state_r)

                // ------------------------- Açılış / INIT başlangıç
                S_RESET: begin
                    // İlk adım: CONFIG (iç referans + thermal enable; datasheet: RD=0, TS=0)
                    if (!spi_busy_w && !clr_busy_r) begin
                        spi_tx_r     <= frame_f(CMD_CONFIG, 4'h0, 16'h0000);
                        spi_start_r  <= 1'b1;
                        init_state_r <= S_INIT_WAIT;
                    end
                end

                S_INIT_CFG: begin
                    // Bu state kullanılmıyor; S_RESET doğrudan CONFIG gönderiyor
                    init_state_r <= S_INIT_WAIT;
                end

                S_INIT_SPAN: begin
                    // CHn SPAN: ±2.5V (0x0004)
                    if (!spi_busy_w) begin
                        spi_tx_r     <= frame_f(CMD_WRITE_SPAN_N, {1'b0,span_chan_r}, SOFTSPAN_CODE);
                        spi_start_r  <= 1'b1;
                        init_state_r <= S_INIT_WAIT;
                    end
                end

                S_INIT_WAIT: begin
                    // DONE gelince echo işlendi ve buradan NEXT'e çıkarız
                    if (spi_done_w)
                        init_state_r <= S_INIT_NEXT;
                end

                S_INIT_NEXT: begin
                    // Echo hatası → retry / başarısız politikası
                    if (echo_error_r) begin
                        if (INIT_RETRY_MAX==0) begin
                            // sınırsız retry
                            conf_err_r   <= 1'b1;
                            init_ok_r    <= 1'b0;
                            echo_error_r <= 1'b0;
                            have_prev_r  <= 1'b0;
                            span_chan_r  <= 3'd0;
                            init_state_r <= S_RESET;
                        end else if (retry_left_r != 0) begin
                            retry_left_r <= retry_left_r - 8'd1;
                            conf_err_r   <= 1'b1;
                            init_ok_r    <= 1'b0;
                            echo_error_r <= 1'b0;
                            have_prev_r  <= 1'b0;
                            span_chan_r  <= 3'd0;
                            init_state_r <= S_RESET;
                        end else begin
                            init_failed_r<= 1'b1;
                            init_state_r <= S_RUN_IDLE; // yine de çalışmaya geç (conf_err 1 kalır)
                        end
                    end else begin
                        // Hangi adımı bitirdik? last_tx_r (beklenen echo) komut nibblesından okunur
                        if (last_tx_r[23:20]==CMD_CONFIG) begin
                            // CONFIG → SPAN CH0
                            span_chan_r  <= 3'd0;
                            init_state_r <= S_INIT_SPAN;
                        end else if (last_tx_r[23:20]==CMD_WRITE_SPAN_N) begin
                            if (span_chan_r < 3'd7) begin
                                span_chan_r  <= span_chan_r + 3'd1;
                                init_state_r <= S_INIT_SPAN;
                            end else begin
                                // Son SPAN yazıldı → son echo'yu görmek için TEK NOOP
                                init_state_r <= S_INIT_NOOP;
                            end
                        end else if (last_tx_r[23:20]==CMD_NOOP) begin
                            // Init tamam ve doğrulandı
                            conf_err_r    <= 1'b0;
                            init_ok_r     <= 1'b1;
                            init_state_r  <= S_RUN_IDLE;
                        end else begin
                            // CONFIG sonrası ilk S_INIT_WAIT→NEXT geldiyse:
                            init_state_r  <= S_INIT_SPAN;
                        end
                    end
                end

                S_INIT_NOOP: begin
                    if (!spi_busy_w) begin
                        spi_tx_r     <= frame_f(CMD_NOOP, 4'h0, 16'h0000);
                        spi_start_r  <= 1'b1;
                        init_state_r <= S_INIT_WAIT;
                    end
                end

                // ------------------------- RUN (normal çalışma)
                S_RUN_IDLE: begin
                    if (pend_valid_r) begin
                        // IN_RANGE kontrolü (Offset-Binary)
                        if ( (pend_code_r < CODE_MIN_ALLOWED) || (pend_code_r > CODE_MAX_ALLOWED) ) begin
                            in_range_error_r <= 1'b1;   // sticky
                            pend_valid_r     <= 1'b0;   // komutu düş
                        end else if (!spi_busy_w) begin
                            // Tek frame: WRITE_CODE_N_UPD_N
                            spi_tx_r     <= frame_f(CMD_WRITE_CODE_N_UPD_N, {1'b0,pend_chan_r}, pend_code_r);
                            spi_start_r  <= 1'b1;
                            pend_valid_r <= 1'b0;       // komut tüketildi
                            init_state_r <= S_RUN_WAIT;
                        end
                    end
                end

                S_RUN_WAIT: begin
                    if (spi_done_w)
                        init_state_r <= S_RUN_IDLE;    // RUN'da NOOP yok; echo'yu bir sonraki yazışta görürüz
                end

                default: begin
                    init_state_r <= S_RESET;
                end
            endcase
        end
    end

endmodule

