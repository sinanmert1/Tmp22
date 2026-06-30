//
//
//
//
//---------------------------------------------------------------------------------
// MODULE NAME: ltc2666_dc2196a_test_top
//---------------------------------------------------------------------------------
// MODULE DESCRIPTION:
// *******************
// 1- This module is a simple deterministic hardware test top for the LTC2666
//    controller and the DC2196A-A evaluation board.
// 2- No UART or CPU interface is used in this first test version.
// 3- frame_vld_i of ltc2666_controller is tied to 1'b1. The controller updates the
//    DAC only when ch_update_mask_i or any enabled chX_code_i changes.
// 4- The test sequence applies simple DC codes with a fixed dwell time.
// 5- Default LTC2666 controller settings use internal reference and +/-2.5V SoftSpan.
// 6- In bipolar +/-2.5V range, 16'h8000 is approximately 0V.
// 7- Approximate code examples for +/-2.5V range:
//      16'h8000 :  0.0V
//      16'h999A : +0.5V
//      16'h6666 : -0.5V
//      16'hB333 : +1.0V
//      16'h4CCD : -1.0V
//      16'hE666 : +2.0V
//      16'h199A : -2.0V
// 8- Suggested measurement:
//      - Measure each VOUTx with DMM or ADC.
//      - Check that inactive channels do not move when another channel is updated.
//      - Check adjacent channel coupling by toggling CH0/CH1 while observing the other.
//      - Check code-to-voltage accuracy at 0V, +/-0.5V, +/-1V and +/-2V.
// *******************
// TEST SEQUENCE:
// *******************
// STEP0  : All channels 0V
// STEP1  : CH0 +1V
// STEP2  : CH0 0V
// STEP3  : CH1 +1V
// STEP4  : CH1 0V
// STEP5  : CH0 +1V, CH1 -1V
// STEP6  : CH0 0V,  CH1 0V
// STEP7  : All channels +0.5V
// STEP8  : All channels -0.5V
// STEP9  : All channels 0V
// STEP10 : CH0 +2V
// STEP11 : CH0 -2V
// STEP12 : CH0 0V
// STEP13 : CH7 +1V
// STEP14 : CH7 0V
// STEP15 : Loop back to STEP0
//---------------------------------------------------------------------------------

module ltc2666_dc2196a_test_top #(
    parameter           CLOCK_FREQ        = 100_000_000, // // system clock frequency
    parameter           TEST_STEP_TIME_MS = 2000         // // dwell time per test step
)(
    input               clk_i,             // // system clk
    input               rst_al_i,          // // active-low asynchronous reset

    // LTC2666 SPI interface
    input               ltc_sdo_i,         // // LTC2666 SDO
    output              ltc_cs_ld_o,       // // LTC2666 CS/LD
    output              ltc_sck_o,         // // LTC2666 SCK
    output              ltc_sdi_o,         // // LTC2666 SDI

    // LTC2666 control/status
    output              ltc_clr_al_o,      // // LTC2666 CLR active-low
    input               ltc_ovrtmp_al_i,   // // LTC2666 OVRTMP active-low

    // Debug/status outputs
    output [3:0]        test_step_o,       // // active test step
    output              init_done_o,       // // controller init done
    output              cfg_rb_err_o,      // // controller configuration echo error
    output              run_rb_err_o,      // // controller runtime echo error
    output              ovrtmp_err_o,      // // controller overtemperature error
    output              frame_busy_o,      // // controller frame busy
    output              frame_done_o,      // // controller frame done pulse
    output              fault_o,           // // controller global fault
    output [2:0]        active_ch_o        // // active DAC channel
);
    // ####################################################################################
    //                                     CONSTANTS
    // ####################################################################################
    localparam CLOCK_PERIOD_NS = (10 ** 9) / CLOCK_FREQ;
    localparam TEST_STEP_TIMER = ((TEST_STEP_TIME_MS * 1_000_000) / CLOCK_PERIOD_NS);

    localparam [15:0] DAC_CODE_0V      = 16'h8000;
    localparam [15:0] DAC_CODE_POS_0P5 = 16'h999A;
    localparam [15:0] DAC_CODE_NEG_0P5 = 16'h6666;
    localparam [15:0] DAC_CODE_POS_1P0 = 16'hB333;
    localparam [15:0] DAC_CODE_NEG_1P0 = 16'h4CCD;
    localparam [15:0] DAC_CODE_POS_2P0 = 16'hE666;
    localparam [15:0] DAC_CODE_NEG_2P0 = 16'h199A;

    // ####################################################################################
    //                                      SIGNALS
    // ####################################################################################
    reg [31:0]          step_timer_cntr_r;
    reg [3:0]           test_step_r;
    reg [1:0]           test_state_r;

    reg [7:0]           ch_update_mask_r;
    reg [15:0]          ch0_code_r;
    reg [15:0]          ch1_code_r;
    reg [15:0]          ch2_code_r;
    reg [15:0]          ch3_code_r;
    reg [15:0]          ch4_code_r;
    reg [15:0]          ch5_code_r;
    reg [15:0]          ch6_code_r;
    reg [15:0]          ch7_code_r;

    wire                ctrl_init_done_w;
    wire                ctrl_cfg_rb_err_w;
    wire                ctrl_run_rb_err_w;
    wire                ctrl_ovrtmp_err_w;
    wire                ctrl_frame_busy_w;
    wire                ctrl_frame_done_w;
    wire                ctrl_frame_overrun_err_w;
    wire                ctrl_fault_w;
    wire [2:0]          ctrl_active_ch_w;
    wire [31:0]         ctrl_last_echo_data_w;

    // ####################################################################################
    //                                  TEST FSM STATES
    // ####################################################################################
    localparam [1:0] T_WAIT_INIT = 2'd0,
                     T_APPLY     = 2'd1,
                     T_WAIT_DONE = 2'd2,
                     T_HOLD      = 2'd3;

    // ####################################################################################
    //                              LTC2666 CONTROLLER INSTANCE
    // ####################################################################################
    ltc2666_controller #(
        .CLOCK_FREQ              (CLOCK_FREQ),
        .CFG_RETRY_FOREVER_EN    (1'b1),
        .CFG_RETRY_MAX           (10),
        .FRAME_SYNC_UPDATE_EN    (1'b0),     // // '0': independent Write Code n, Update n
        .SPAN_INIT_ALL_EN        (1'b0),     // // configure each channel separately

        .CH0_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH1_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH2_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH3_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH4_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH5_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH6_SOFT_SPAN           (3'b100),   // // +/-2.5V
        .CH7_SOFT_SPAN           (3'b100),   // // +/-2.5V

        .CH0_INIT_CODE           (DAC_CODE_0V),
        .CH1_INIT_CODE           (DAC_CODE_0V),
        .CH2_INIT_CODE           (DAC_CODE_0V),
        .CH3_INIT_CODE           (DAC_CODE_0V),
        .CH4_INIT_CODE           (DAC_CODE_0V),
        .CH5_INIT_CODE           (DAC_CODE_0V),
        .CH6_INIT_CODE           (DAC_CODE_0V),
        .CH7_INIT_CODE           (DAC_CODE_0V),

        .MUXOUT_INIT_EN          (1'b0),
        .MUXOUT_INIT_SEL         (5'b00000),

        .CH_POWER_DOWN_INIT_EN   (1'b0),
        .CH_POWER_DOWN_INIT_MASK (8'h00),
        .CHIP_POWER_DOWN_INIT_EN (1'b0)
    ) u_ltc2666_controller (
        .clk_i                   (clk_i),
        .rst_al_i                (rst_al_i),

        .sdo_i                   (ltc_sdo_i),
        .cs_ld_o                 (ltc_cs_ld_o),
        .sck_o                   (ltc_sck_o),
        .sdi_o                   (ltc_sdi_o),

        .clr_al_o                (ltc_clr_al_o),
        .ovrtmp_al_i             (ltc_ovrtmp_al_i),

        .frame_vld_i             (1'b1),              // // update enable
        .ch_update_mask_i        (ch_update_mask_r),
        .ch0_code_i              (ch0_code_r),
        .ch1_code_i              (ch1_code_r),
        .ch2_code_i              (ch2_code_r),
        .ch3_code_i              (ch3_code_r),
        .ch4_code_i              (ch4_code_r),
        .ch5_code_i              (ch5_code_r),
        .ch6_code_i              (ch6_code_r),
        .ch7_code_i              (ch7_code_r),

        .init_done_o             (ctrl_init_done_w),
        .cfg_rb_err_o            (ctrl_cfg_rb_err_w),
        .run_rb_err_o            (ctrl_run_rb_err_w),
        .ovrtmp_err_o            (ctrl_ovrtmp_err_w),
        .frame_busy_o            (ctrl_frame_busy_w),
        .frame_done_o            (ctrl_frame_done_w),
        .frame_overrun_err_o     (ctrl_frame_overrun_err_w),
        .fault_o                 (ctrl_fault_w),
        .active_ch_o             (ctrl_active_ch_w),
        .last_echo_data_o        (ctrl_last_echo_data_w)
    );

    // ####################################################################################
    //                                  MAIN TEST PROCESS
    // ####################################################################################
    always @(posedge clk_i or negedge rst_al_i) begin
        if (!rst_al_i) begin
            test_state_r       <= T_WAIT_INIT;
            test_step_r        <= 0;
            step_timer_cntr_r  <= 0;
            ch_update_mask_r   <= 8'h00;
            ch0_code_r         <= DAC_CODE_0V;
            ch1_code_r         <= DAC_CODE_0V;
            ch2_code_r         <= DAC_CODE_0V;
            ch3_code_r         <= DAC_CODE_0V;
            ch4_code_r         <= DAC_CODE_0V;
            ch5_code_r         <= DAC_CODE_0V;
            ch6_code_r         <= DAC_CODE_0V;
            ch7_code_r         <= DAC_CODE_0V;
        end else begin
            case (test_state_r)
                T_WAIT_INIT: begin
                    step_timer_cntr_r <= 0;
                    ch_update_mask_r  <= 8'h00;
                    if (ctrl_init_done_w && !ctrl_fault_w) begin
                        test_step_r  <= 0;
                        test_state_r <= T_APPLY;
                    end
                end

                T_APPLY: begin
                    // Default every step starts from the existing register values.
                    // Only ch_update_mask_r-selected channels are written by the controller.
                    case (test_step_r)
                        4'd0: begin
                            ch_update_mask_r <= 8'hFF;
                            ch0_code_r       <= DAC_CODE_0V;
                            ch1_code_r       <= DAC_CODE_0V;
                            ch2_code_r       <= DAC_CODE_0V;
                            ch3_code_r       <= DAC_CODE_0V;
                            ch4_code_r       <= DAC_CODE_0V;
                            ch5_code_r       <= DAC_CODE_0V;
                            ch6_code_r       <= DAC_CODE_0V;
                            ch7_code_r       <= DAC_CODE_0V;
                        end
                        4'd1: begin
                            ch_update_mask_r <= 8'h01;
                            ch0_code_r       <= DAC_CODE_POS_1P0;
                        end
                        4'd2: begin
                            ch_update_mask_r <= 8'h01;
                            ch0_code_r       <= DAC_CODE_0V;
                        end
                        4'd3: begin
                            ch_update_mask_r <= 8'h02;
                            ch1_code_r       <= DAC_CODE_POS_1P0;
                        end
                        4'd4: begin
                            ch_update_mask_r <= 8'h02;
                            ch1_code_r       <= DAC_CODE_0V;
                        end
                        4'd5: begin
                            ch_update_mask_r <= 8'h03;
                            ch0_code_r       <= DAC_CODE_POS_1P0;
                            ch1_code_r       <= DAC_CODE_NEG_1P0;
                        end
                        4'd6: begin
                            ch_update_mask_r <= 8'h03;
                            ch0_code_r       <= DAC_CODE_0V;
                            ch1_code_r       <= DAC_CODE_0V;
                        end
                        4'd7: begin
                            ch_update_mask_r <= 8'hFF;
                            ch0_code_r       <= DAC_CODE_POS_0P5;
                            ch1_code_r       <= DAC_CODE_POS_0P5;
                            ch2_code_r       <= DAC_CODE_POS_0P5;
                            ch3_code_r       <= DAC_CODE_POS_0P5;
                            ch4_code_r       <= DAC_CODE_POS_0P5;
                            ch5_code_r       <= DAC_CODE_POS_0P5;
                            ch6_code_r       <= DAC_CODE_POS_0P5;
                            ch7_code_r       <= DAC_CODE_POS_0P5;
                        end
                        4'd8: begin
                            ch_update_mask_r <= 8'hFF;
                            ch0_code_r       <= DAC_CODE_NEG_0P5;
                            ch1_code_r       <= DAC_CODE_NEG_0P5;
                            ch2_code_r       <= DAC_CODE_NEG_0P5;
                            ch3_code_r       <= DAC_CODE_NEG_0P5;
                            ch4_code_r       <= DAC_CODE_NEG_0P5;
                            ch5_code_r       <= DAC_CODE_NEG_0P5;
                            ch6_code_r       <= DAC_CODE_NEG_0P5;
                            ch7_code_r       <= DAC_CODE_NEG_0P5;
                        end
                        4'd9: begin
                            ch_update_mask_r <= 8'hFF;
                            ch0_code_r       <= DAC_CODE_0V;
                            ch1_code_r       <= DAC_CODE_0V;
                            ch2_code_r       <= DAC_CODE_0V;
                            ch3_code_r       <= DAC_CODE_0V;
                            ch4_code_r       <= DAC_CODE_0V;
                            ch5_code_r       <= DAC_CODE_0V;
                            ch6_code_r       <= DAC_CODE_0V;
                            ch7_code_r       <= DAC_CODE_0V;
                        end
                        4'd10: begin
                            ch_update_mask_r <= 8'h01;
                            ch0_code_r       <= DAC_CODE_POS_2P0;
                        end
                        4'd11: begin
                            ch_update_mask_r <= 8'h01;
                            ch0_code_r       <= DAC_CODE_NEG_2P0;
                        end
                        4'd12: begin
                            ch_update_mask_r <= 8'h01;
                            ch0_code_r       <= DAC_CODE_0V;
                        end
                        4'd13: begin
                            ch_update_mask_r <= 8'h80;
                            ch7_code_r       <= DAC_CODE_POS_1P0;
                        end
                        4'd14: begin
                            ch_update_mask_r <= 8'h80;
                            ch7_code_r       <= DAC_CODE_0V;
                        end
                        default: begin
                            ch_update_mask_r <= 8'hFF;
                            ch0_code_r       <= DAC_CODE_0V;
                            ch1_code_r       <= DAC_CODE_0V;
                            ch2_code_r       <= DAC_CODE_0V;
                            ch3_code_r       <= DAC_CODE_0V;
                            ch4_code_r       <= DAC_CODE_0V;
                            ch5_code_r       <= DAC_CODE_0V;
                            ch6_code_r       <= DAC_CODE_0V;
                            ch7_code_r       <= DAC_CODE_0V;
                        end
                    endcase
                    test_state_r <= T_WAIT_DONE;
                end

                T_WAIT_DONE: begin
                    if (!ctrl_init_done_w || ctrl_fault_w) begin
                        test_state_r <= T_WAIT_INIT;
                    end else if (ctrl_frame_done_w) begin
                        step_timer_cntr_r <= 0;
                        test_state_r      <= T_HOLD;
                    end
                end

                T_HOLD: begin
                    if (!ctrl_init_done_w || ctrl_fault_w) begin
                        test_state_r <= T_WAIT_INIT;
                    end else if (step_timer_cntr_r == TEST_STEP_TIMER - 1) begin
                        step_timer_cntr_r <= 0;
                        if (test_step_r == 4'd14) begin
                            test_step_r <= 0;
                        end else begin
                            test_step_r <= test_step_r + 1;
                        end
                        test_state_r <= T_APPLY;
                    end else begin
                        step_timer_cntr_r <= step_timer_cntr_r + 1;
                    end
                end

                default: begin
                    test_state_r <= T_WAIT_INIT;
                end
            endcase
        end
    end

    //#########################################################################
    //                               WIRE ASSIGNMENT
    //#########################################################################
    assign test_step_o   = test_step_r;
    assign init_done_o   = ctrl_init_done_w;
    assign cfg_rb_err_o  = ctrl_cfg_rb_err_w;
    assign run_rb_err_o  = ctrl_run_rb_err_w;
    assign ovrtmp_err_o  = ctrl_ovrtmp_err_w;
    assign frame_busy_o  = ctrl_frame_busy_w;
    assign frame_done_o  = ctrl_frame_done_w;
    assign fault_o       = ctrl_fault_w;
    assign active_ch_o   = ctrl_active_ch_w;

endmodule
