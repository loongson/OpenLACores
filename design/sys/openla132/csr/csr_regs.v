// ------------------------------------------------------------------------------------//
//  Copyright (c) 2026 Loongson Technology Corporation Limited.
//
//  OpenLACores is licensed under Mulan PSL v2.
// 
//  You can use this software according to the terms and conditions of the Mulan PSL v2.
//
//  You may obtain a copy of Mulan PSL v2 at:
//           http://license.coscl.org.cn/MulanPSL2
// 
//  THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY
//  KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
//  NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// 
//  See the Mulan PSL v2 for more details.
// ------------------------------------------------------------------------------------//

`include "openla132_global.h"

module csr_regs #(
    parameter HAS_BTB       = -1,
    parameter MDU_TYPE      = -1,
    parameter HAS_ICC       = -1,
    parameter ICC_SET_ASSOC = -1,
    parameter ICC_WAYSIZE   = -1,
    parameter HAS_DCC       = -1,
    parameter DCC_SET_ASSOC = -1,
    parameter DCC_WAYSIZE   = -1,
    parameter PERF_CNT_NUM  = -1,
    parameter MPU_NUM       = -1
) (
    input  wire                                                clk_g,
    input  wire                                                clk_ng,
    input  wire                                                clk_count,
    input  wire                                                clk_cr,
    input  wire                                                sync_sresetn,
    input  wire                                                reset,
    input  wire                                                data_reset,
    input  wire [                                       3: 0]  prid_revision,
    input  wire [                                       9: 0]  cpunum,
    input  wire [                                      31: 0]  boot_pc,
    input  wire [                                       7: 0]  ext_int,
    input  wire                                                ex_cancel,
    input  wire                                                rtu_lsu_commit,
    input  wire                                                rtu_lsu_allowin,
    input  wire                                                idu_lsu_fpu_clear_skip,
    input  wire [                                      31: 0]  idu_lsu_follow_pc,
    output wire [     `OPEN132_CSR_CACHE_CTRL_BUS_WIDTH-1: 0]  csr_cache_ctrl_bus,
`ifdef CONFIG_MMU
    output wire [       `OPEN132_CSR_MMU_CTRL_BUS_WIDTH-1: 0]  csr_mmu_ctrl_bus,
`endif
    output wire [       `OPEN132_CSR_IFU_CTRL_BUS_WIDTH-1: 0]  csr_ifu_ctrl_bus,
    output wire [       `OPEN132_CSR_IDU_CTRL_BUS_WIDTH-1: 0]  csr_idu_ctrl_bus,
    output wire [                                       1: 0]  csr_bpu_ctrl_bus,
    input  wire [`OPEN132_CACHE_LSU_ECC_FATAL_BUS_WIDTH-1: 0]  cache_lsu_ecc_fatal_bus,
`ifdef OPEN132_HAS_LJTAG
    input  wire                                                clk_slp_en,
    output wire [                                       7: 0]  asid,
    input  wire                                                jtag_core_break,
    output wire                                                lsu_clk_breakpoint_enable,
    output wire                                                lsu_jtag_debug_mode,
    output wire                                                lsu_ifu_debug_map_valid,
    output wire [                                      13: 0]  lsu_idu_debug_bus,
    output wire [                                      80: 0]  jtag_idu_ihb_req_bus,
    input  wire [                                      31: 0]  idu_jtag_ihb_resp_bus,
`endif
    input  wire [                           (1+32+32)*4-1: 0]  ibus_win,
    input  wire [                           (1+32+32)*4-1: 0]  dbus_win,
`ifdef GODSON_DEBUG
    output wire [                                      31: 0]  gs_core_csr0,
    output wire [                                      31: 0]  gs_core_csr1,
    output wire [                                      31: 0]  gs_core_csr4,
    output wire [                                      31: 0]  gs_core_csr5,
    output wire [                                      31: 0]  gs_core_csr6,
    output wire [                                      31: 0]  gs_core_csr7,
`endif
    output reg                                                 sleeping,
    input  wire                                                valid,
    input  wire                                                ihb_debug,
    input  wire                                                ihb_signal,
    input  wire                                                ex_dbgss,
    input  wire [                                       3: 0]  inst_lisa,
    input  wire                                                see_int_flush,
    input  wire                                                idle_ertn,
    input  wire                                                ex,
    input  wire [                                      13: 0]  lisa_si14,
    input  wire [                                      31: 0]  result,
    input  wire [                                      31: 0]  pc,
    input  wire                                                llbit,
    input  wire [                                      31: 0]  ld_bdata_stldlr,
    output reg                                                 da_r,
`ifdef CONFIG_MMU
    output reg                                                 pg_r,
`endif
    output reg  [                                      31: 0]  era_r,
    output reg  [                                      31: 0]  merrera_r,
    output reg                                                 userdef_supexc,
    output wire                                                debug_mode,
    input  wire                                                mem_nmi_ccerr,
    output wire                                                ctrl_nowrap,
    output wire                                                ctrl_sleep_cg,
    output wire                                                ctrl_null_cg,
    output wire                                                lsu_clk_csr_enable,
    input  wire                                                ex_dbcl,
    input  wire                                                fetch_ade,
    input  wire                                                ex_lserr,
    input  wire                                                mem_is_ld,
    input  wire                                                mem_is_st,
    input  wire                                                mem_is_ll_valid,
    input  wire                                                mdu_is_run,
    input  wire                                                mem_valid_go,
    input  wire                                                mem_cps_data_ready,
    input  wire [                                      31: 0]  mem_cps_data_rdata,
    input  wire                                                lddata_err,
    input  wire                                                lddata_exok,
    input  wire [                                       5: 0]  mem_cc_op,
    input  wire                                                de_perf_inst_ibus,
    input  wire                                                de_perf_inst_dbus,
    input  wire                                                de_perf_inst_sbus,
    input  wire                                                de_perf_load_stall,
    input  wire                                                de_perf_nop,
    input  wire                                                dcc_uncached_sbus,
    input  wire                                                icc_cached_sbus,
    input  wire                                                dcc_all_ecc_err_add1,
    input  wire                                                dcc_refill_add1,
    input  wire                                                icc_all_ecc_err_add1,
    input  wire                                                icc_refill_add1,
    output wire                                                dcc_fatal_in,
    output wire                                                find_ccfatal,
    input  wire                                                inst_oth_ex_pesudoint,
    input  wire                                                mem_ex_pesudoint,
    output wire                                                inst_idle,
    input  wire                                                inst_oth_noex,
    input  wire                                                valid_go_wb,
    input  wire                                                mem_load_wait_back,
    input  wire                                                noex_lsok_go,
    output wire                                                csr_csr0_da,
    output wire [                                      23: 0]  csr_csr190_segpa,
    input  wire [                                      31: 0]  memls_vaddr,
    output wire                                                icc_bypass,
    output wire                                                dcc_bypass,
    output wire [                                      21: 0]  ctrl_taglo_pa,
    output wire                                                ctrl_taglo_v,
    output wire                                                ctrl_taglo_d,
    output wire                                                ctrl_taglo_l,
    output wire                                                ctrl_userdef_supexc,
    output wire                                                ctrl_llbit_keep,
    output wire                                                ctrl_llbit_wclr,
    output wire                                                ertn_write,
    input  wire                                                ifc_storeq_null,
    output wire                                                debug_stall_nmi,
    input  wire                                                nmi_end,
    output wire                                                inst_split_2nd,
    output wire                                                debug_ctrl_nocce,
    input  wire                                                mark_ccerr,
    input  wire                                                ccerr_wr_csr,
    output wire [                                      12: 0]  ctrl_im,
    output wire [                                      12: 0]  ctrl_ip,
    output wire                                                ctrl_ie,
    input  wire                                                mark_nmi_ccerr_int,
`ifdef CONFIG_MMU
    input  wire                                                ex_mmu,
    input  wire                                                ex_page,
`endif
    input  wire                                                mem_ex,
    input  wire                                                ex_general,
    output wire                                                ex_watch_inst,
`ifdef CONFIG_MMU
    input  wire [                                       5: 0]  mem_statcode,
`else
    input  wire [                                       4: 0]  mem_statcode,
`endif
    input  wire [                                       1: 0]  mem_sub_excode,
    output wire                                                debug_ex,
    input  wire                                                debug_mode_ex,
    input  wire                                                mem_pesudoint,
    output wire [                                       2: 0]  ctrl_lisa_vs,
    output wire [                                      19: 0]  ctrl_eentry,
    output wire [                                      19: 0]  ctrl_merrentry,
    output wire                                                inst_ertn,
    output wire                                                lisa_csr_rdt_cfg,
    output wire                                                inst_stall_fetch,
    output wire [                                      31: 0]  brpc_or_pc,
    output wire [                                      31: 0]  csr_csr500,
    output wire                                                csr_csr90_errmode,
    output wire [                                      31: 0]  rdtime_result,
    output wire [                                      31: 0]  cpucfg_result,
    output wire [                                      31: 0]  csrrd_result,
    output wire [                                      31: 0]  dertn_result
);

localparam HAS_ICC_CPUCFG       = HAS_ICC;
localparam ICC_SET_ASSOC_CPUCFG = ICC_SET_ASSOC;
localparam ICC_WAYSIZE_CPUCFG   = ICC_WAYSIZE;
localparam HAS_DCC_CPUCFG       = HAS_DCC;
localparam DCC_SET_ASSOC_CPUCFG = DCC_SET_ASSOC;
localparam DCC_WAYSIZE_CPUCFG   = DCC_WAYSIZE;

localparam TOTAL_NUM_MPU_REGION0 = MPU_NUM;
localparam THIS_NUM_MPU_REGION0  = 1;
localparam TOTAL_NUM_MPU_REGION1 = MPU_NUM;
localparam THIS_NUM_MPU_REGION1  = 2;
localparam TOTAL_NUM_MPU_REGION2 = MPU_NUM;
localparam THIS_NUM_MPU_REGION2  = 3;
localparam TOTAL_NUM_MPU_REGION3 = MPU_NUM;
localparam THIS_NUM_MPU_REGION3  = 4;

genvar i;

//============================================================//
//                     Local Declarations
//============================================================//
    reg mem_validout_ex_r;
    reg mem_is_ld_r;
    reg mem_is_st_r;
    reg [  1: 0] mem_hit_bus_r;
    reg watchen_r;
    reg [  1: 0] dacm_r;
    reg [  1: 0] dacf_r;
    reg ie_r;
    reg [  1: 0] plv_r;
    reg pwatchen_r;
    reg pie_r;
    reg [  1: 0] pplv_r;
    reg fpen_r;
    reg rdtime_disp3_r;
    reg [  2: 0] vs_r;
    reg [ 12: 0] im_r;
    reg [  1: 0] sub_excode_r;
    reg [  5: 0] excode_r;
    reg [ 12: 0] ip_r;
    reg [ 31: 0] timer_id_r;
    reg [ 29: 0] timer_val_r;
    reg timer_period_r;
    reg timer_en_r;
    reg timer_enable;
    reg [ 31: 0] timer_r;
    reg [ 31: 0] cnt_off_r;
`ifdef OPEN132_COUNT64
    reg [ 63: 0] count_r;
`else
    reg [ 31: 0] count_r;
`endif
    reg keepllbit_r;
    reg user_ccop_r;
    reg lifeen_r;
    reg [  1: 0] errcode_r;
    reg [  1: 0] errpdacm_r;
    reg [  1: 0] errpdacf_r;
    reg errpda_r;
`ifdef CONFIG_MMU
    reg errppg_r;
`endif
    reg errpwatchen_r;
    reg errpie_r;
    reg [  1: 0] errpplv_r;
    reg errmode_r;
    reg [ 19: 0] merrentry_vpn;
    reg [ 31: 0] csr190_r;
    reg [ 31: 0] csr191_r;
    reg napen_r;
    reg [ 31: 0] badv_r;
    reg [ 19: 0] eentry_vpn;
`ifdef CONFIG_MMU
    reg [  9: 0] asid_r;
    reg [ 31: 0] pgdl_r;
    reg [ 31: 0] pgdh_r;
    reg [ 31: 0] pwctl0_r;
    reg [ 31: 0] pwctl1_r;
    reg [ 31: 0] dmw0_r;
    reg [ 31: 0] dmw1_r;
`else
    reg [  7: 0] asid_r;
`endif
    reg [ 31: 0] save0_r;
    reg [ 31: 0] save1_r;
    reg userdef_dccbp;
    reg userdef_iccbp;
    reg userdef_nowrap;
    reg userdef_dc;
    reg userdef_enable_pred;
`ifdef OPEN132_HAS_LJTAG
    reg debug_dm;
    reg debug_slp;
    reg debug_clkn;
    reg debug_nocce;
    reg debug_dc;
    reg [  4: 0] debug_destat;
    reg debug_sse;
    reg debug_dint;
    reg debug_ihb;
    reg debug_dbc;
    reg debug_ssflag;
    reg ihb_skip;
    reg [ 31: 0] dera_r;
    reg [ 31: 0] dsave_r;
`endif
    reg wait_fatal_mark;
    reg ccerr0_impr_ccop;
    reg ccerr0_impr_rf;
    reg ccerr0_impr_wr;
    reg ccerr0_rd;
    reg ccerr0_intagv;
    reg [  3: 0] ccerr0_way;
    reg [  8: 0] ccerr0_indx;
    reg ccerr0_inerm;
    reg ccerr0_dcc;
    reg ccerr0_icc;
    reg ccerr0_pending;
    reg [ 21: 0] ccerr1_dirty_tag;
    reg ccerr1_dirtywb;
    reg [  7: 0] ccerr1_indata;
    reg [ 21: 0] taglo_pa;
    reg taglo_v;
    reg taglo_d;
    reg taglo_l;
    wire inst_csr_write_stall_chk;
    wire [           13: 0] csr_num_stall_chk;
    wire ctrl_stallf_stall_chk;
    wire [           31: 0] vaddr_vaddr_sel;
    wire [(1+32+32)*4-1: 0] ibus_win_vaddr_sel;
    wire [(1+32+32)*4-1: 0] dbus_win_vaddr_sel;
    wire ibus_hit_vaddr_sel;
    wire dbus_hit_vaddr_sel;
    wire [            3: 0] prid_revision_cpucfg;
    wire inst_cpucfg_cpucfg;
    wire [           31: 0] cpucfg_addr_cpucfg;
    wire [           31: 0] cpucfg_result_cpucfg;
    wire clk_mpu_region0;
    wire reset_mpu_region0;
    wire data_reset_mpu_region0;
    wire lisa_kernel_mode_mpu_region0;
    wire mpuhi_wen_mpu_region0;
    wire mpulo_wen_mpu_region0;
    wire [           31: 0] ctrl_wrdata_mpu_region0;
    wire [           31: 0] ctrl_wrmask_mpu_region0;
    wire [           31: 0] csr_mpuhi_mpu_region0;
    wire [           31: 0] csr_mpulo_mpu_region0;
    wire ctrl_mpuhi_valid_mpu_region0;
    wire [           19: 0] ctrl_mpuhi_base_mpu_region0;
    wire [           19: 0] ctrl_mpulo_mask_mpu_region0;
    wire [            3: 0] ctrl_mpulo_calg_mpu_region0;
    wire clk_mpu_region1;
    wire reset_mpu_region1;
    wire data_reset_mpu_region1;
    wire lisa_kernel_mode_mpu_region1;
    wire mpuhi_wen_mpu_region1;
    wire mpulo_wen_mpu_region1;
    wire [           31: 0] ctrl_wrdata_mpu_region1;
    wire [           31: 0] ctrl_wrmask_mpu_region1;
    wire [           31: 0] csr_mpuhi_mpu_region1;
    wire [           31: 0] csr_mpulo_mpu_region1;
    wire ctrl_mpuhi_valid_mpu_region1;
    wire [           19: 0] ctrl_mpuhi_base_mpu_region1;
    wire [           19: 0] ctrl_mpulo_mask_mpu_region1;
    wire [            3: 0] ctrl_mpulo_calg_mpu_region1;
    wire clk_mpu_region2;
    wire reset_mpu_region2;
    wire data_reset_mpu_region2;
    wire lisa_kernel_mode_mpu_region2;
    wire mpuhi_wen_mpu_region2;
    wire mpulo_wen_mpu_region2;
    wire [           31: 0] ctrl_wrdata_mpu_region2;
    wire [           31: 0] ctrl_wrmask_mpu_region2;
    wire [           31: 0] csr_mpuhi_mpu_region2;
    wire [           31: 0] csr_mpulo_mpu_region2;
    wire ctrl_mpuhi_valid_mpu_region2;
    wire [           19: 0] ctrl_mpuhi_base_mpu_region2;
    wire [           19: 0] ctrl_mpulo_mask_mpu_region2;
    wire [            3: 0] ctrl_mpulo_calg_mpu_region2;
    wire clk_mpu_region3;
    wire reset_mpu_region3;
    wire data_reset_mpu_region3;
    wire lisa_kernel_mode_mpu_region3;
    wire mpuhi_wen_mpu_region3;
    wire mpulo_wen_mpu_region3;
    wire [           31: 0] ctrl_wrdata_mpu_region3;
    wire [           31: 0] ctrl_wrmask_mpu_region3;
    wire [           31: 0] csr_mpuhi_mpu_region3;
    wire [           31: 0] csr_mpulo_mpu_region3;
    wire ctrl_mpuhi_valid_mpu_region3;
    wire [           19: 0] ctrl_mpuhi_base_mpu_region3;
    wire [           19: 0] ctrl_mpulo_mask_mpu_region3;
    wire [            3: 0] ctrl_mpulo_calg_mpu_region3;
    wire [  7: 0] csr_ifu_lisa_segplv;
    wire csr_ifu_lisa_da;
    wire csr_ifu_user_mode;
    wire csr_idu_lisa_da;
    wire csr_idu_lisa_ihb_skip;
    wire [  1: 0] csr_idu_lisa_dacm;
    wire [  7: 0] csr_idu_lisa_segplv;
    wire [  7: 0] csr_idu_lisa_segca_dev;
    wire csr_idu_user_ccop;
    wire csr_idu_rdtime_dis;
    wire csr_idu_fpen;
    wire csr_idu_user_mode;
    wire [4*`OPEN132_CFGSEG_NUM-1: 0] csr_cache_cfgseg_calg;
    wire csr_cache_lisa_da;
    wire [                      1: 0] csr_cache_lisa_dacf;
    wire [                      1: 0] csr_cache_lisa_dacm;
    wire [                     23: 0] csr_cache_lisa_segpa;
    wire [                  1*4-1: 0] csr_cache_mpu_valid;
    wire [                 20*4-1: 0] csr_cache_mpu_base;
    wire [                 20*4-1: 0] csr_cache_mpu_mask;
    wire [                  4*4-1: 0] csr_cache_mpu_calg;
    wire csr_cache_dcc_bypass;
    wire csr_cache_icc_bypass;
    wire inst_split_1st;
    wire user_mode;
    wire kernel_mode;
    wire lisa_csrrd;
    wire lisa_csr_write;
    wire lisa_rdtime;
    wire lisa_rdtime_h;
    wire lisa_rdtime_2nd;
    wire lisa_cpucfg;
    wire lisa_ibar;
`ifdef CONFIG_MMU
    wire lisa_invtlb;
    wire csr_inv_vld;
    wire [  4: 0] csr_inv_op;
    wire [  9: 0] csr_inv_asid;
    wire [ 31: 0] csr_inv_vaddr;
`endif
    wire inst_ctrl_stallf;
    wire badv_wen;
    wire taglo_cc_wen;
    wire [ 31: 0] badv_wrdata;
    wire n_general_ex_wen;
    wire [ 31: 0] ex_main_pc;
    wire [ 31: 0] era_in;
    wire perf_ex;
    wire perf_main_cmt;
    wire perf_mdu_cmt;
    wire perf_fpu_cmt;
    wire mem_hit_ibus;
    wire mem_hit_dbus;
    wire mem_perf_go;
    wire perf_load;
    wire perf_st;
    wire perf_ls;
    wire perf_ls_ibus;
    wire perf_ls_dbus;
    wire perf_ls_sbus;
    wire perf_uncachels_sbus;
    wire perf_cacheft_sbus;
    wire perf_dcc_refill_add1;
    wire perf_icc_refill_add1;
    wire perf_dcc_all_ecc_err_add1;
    wire perf_icc_all_ecc_err_add1;
    wire [25-1: 0] perf_events;
    wire [ 31: 0] csr_wrdata;
    wire [ 31: 0] csr_wrmask;
    wire [ 13: 0] csr_num;
    wire csr_wen;
    wire [ 31: 0] csr_csr0;
    wire [ 31: 0] csr_csr1;
    wire [ 31: 0] csr_csr2;
    wire [ 31: 0] csr_csr3;
    wire [ 31: 0] csr_csr4;
    wire [ 31: 0] csr_csr5;
    wire [ 31: 0] csr_csr6;
    wire [ 31: 0] csr_csr7;
    wire [ 31: 0] csr_csrc;
    wire [ 31: 0] csr_csr18;
`ifdef CONFIG_MMU
    wire [ 31: 0] csr_csr19;
    wire [ 31: 0] csr_csr1a;
    wire [ 31: 0] csr_csr1b;
    wire [ 31: 0] csr_csr1c;
    wire [ 31: 0] csr_csr1d;
`endif
    wire [ 31: 0] csr_csr20;
    wire [ 31: 0] csr_csr21;
`ifdef CONFIG_MMU
    wire [ 31: 0] csr_csr22;
    wire [ 31: 0] csr_csr23;
`endif
    wire [ 31: 0] csr_csr30;
    wire [ 31: 0] csr_csr31;
    wire [ 31: 0] csr_csr40;
    wire [ 31: 0] csr_csr41;
    wire [ 31: 0] csr_csr42;
    wire [ 31: 0] csr_csr43;
    wire [ 31: 0] csr_count;
    wire [ 31: 0] csr_csr60;
    wire [ 31: 0] csr_csr80;
    wire [ 31: 0] csr_csr90;
    wire [ 31: 0] csr_csr91;
    wire [ 31: 0] csr_csr92;
    wire [ 31: 0] csr_csr93;
    wire [ 31: 0] csr_csr94;
    wire [ 31: 0] csr_csr96;
    wire [ 31: 0] csr_csr98;
    wire [ 31: 0] csr_csraf;
    wire [ 31: 0] csr_csrff;
    wire [ 31: 0] csr_csr180;
    wire [ 31: 0] csr_csr181;
    wire [ 31: 0] csr_csr184;
    wire [ 31: 0] csr_csr185;
    wire [ 31: 0] csr_csr188;
    wire [ 31: 0] csr_csr189;
    wire [ 31: 0] csr_csr18c;
    wire [ 31: 0] csr_csr18d;
    wire [ 31: 0] csr_csr190;
    wire [ 31: 0] csr_csr191;
    wire [ 31: 0] csr_csr381;
    wire [ 31: 0] csr_csrihb;
    wire [ 31: 0] csr_csr501;
    wire [ 31: 0] csr_csr502;
    wire [ 31: 0] csr_csr508;
    wire [  1: 0] csr_csr0_dacf;
    wire [  1: 0] csr_csr0_dacm;
    wire csr_csr0_ie;
    wire csr_csr0_we;
    wire [  1: 0] csr_csr0_plv;
    wire csr_csr1_pie;
    wire csr_csr1_pwe;
    wire [  1: 0] csr_csr1_pplv;
    wire csr_csr2_fpen;
    wire csr_csr3_rdtime_disp3;
    wire [ 12: 0] csr_csr4_im;
    wire [  2: 0] csr_csr4_vs;
    wire [ 12: 0] csr_csr5_ip;
    wire [ 31: 0] csr_csr41_timerinit;
    wire csr_csr41_timerperiod;
    wire csr_csr60_wcllbit;
    wire csr_csr60_keepllbit;
    wire csr_csr80_user_ccop;
    wire [  1: 0] csr_csr90_errpdacm;
    wire [  1: 0] csr_csr90_errpdacf;
    wire csr_csr90_errpda;
`ifdef CONFIG_MMU
    wire csr_csr90_errppg;
`endif
    wire csr_csr90_errpwe;
    wire csr_csr90_errpie;
    wire [  1: 0] csr_csr90_errpplv;
    wire csr_csraf_dc;
    wire csr_csrff_napen;
    wire [  7: 0] csr_csr190_segplv;
    wire [ 31: 0] csr_csr191_segcalg;
    wire [  7: 0] csr_csr191_segca_dev;
    wire csr_csr381_ihb_skip;
    wire lisa_user_mode;
    wire lisa_kernel_mode;
    wire csr0_ren;
    wire csr1_ren;
    wire csr2_ren;
    wire csr3_ren;
    wire csr4_ren;
    wire csr5_ren;
    wire csr6_ren;
    wire csr7_ren;
    wire csrc_ren;
    wire csr18_ren;
`ifdef CONFIG_MMU
    wire csr19_ren;
    wire csr1a_ren;
    wire csr1b_ren;
    wire csr1c_ren;
    wire csr1d_ren;
`endif
    wire csr20_ren;
    wire csr21_ren;
`ifdef CONFIG_MMU
    wire csr22_ren;
    wire csr23_ren;
`endif
    wire csr30_ren;
    wire csr31_ren;
    wire csr40_ren;
    wire csr41_ren;
    wire csr42_ren;
    wire csr43_ren;
    wire csr60_ren;
    wire csr80_ren;
    wire csr90_ren;
    wire csr91_ren;
    wire csr92_ren;
    wire csr93_ren;
    wire csr94_ren;
    wire csr96_ren;
    wire csr98_ren;
    wire csraf_ren;
    wire csrff_ren;
    wire csr180_ren;
    wire csr181_ren;
    wire csr184_ren;
    wire csr185_ren;
    wire csr188_ren;
    wire csr189_ren;
    wire csr18c_ren;
    wire csr18d_ren;
    wire csr190_ren;
    wire csr191_ren;
    wire csr381_ren;
    wire csr500_ren;
    wire csr501_ren;
    wire csr502_ren;
    wire csr508_ren;
    wire csrihb_ren;
    wire csr0_wen;
    wire csr1_wen;
    wire csr2_wen;
    wire csr3_wen;
    wire csr4_wen;
    wire csr5_wen;
    wire csr6_wen;
    wire csr7_wen;
    wire csrc_wen;
    wire csr18_wen;
`ifdef CONFIG_MMU
    wire csr19_wen;
    wire csr1a_wen;
    wire csr1c_wen;
    wire csr1d_wen;
`endif
    wire csr30_wen;
    wire csr31_wen;
    wire csr40_wen;
    wire csr41_wen;
    wire csr43_wen;
    wire csr44_wen;
    wire csr60_wen;
    wire csr80_wen;
    wire csr90_wen;
    wire csr91_wen;
    wire csr93_wen;
    wire csr94_wen;
    wire csr96_wen;
    wire csr98_wen;
    wire csraf_wen;
    wire csrff_wen;
    wire csr180_wen;
    wire csr181_wen;
    wire csr184_wen;
    wire csr185_wen;
    wire csr188_wen;
    wire csr189_wen;
    wire csr18c_wen;
    wire csr18d_wen;
    wire csr190_wen;
    wire csr191_wen;
    wire csr300_wen;
    wire csr301_wen;
    wire csr380_wen;
    wire csr381_wen;
    wire csr501_wen;
    wire csr502_wen;
    wire csr508_wen;
    wire ex_norm_wen;
    wire ex_mc_wen;
    wire perf_enp3_r;
    wire csr_timer_int;
    wire ctrl_csraf_dc;
    wire debug_ctrl_dc;
    wire count_en;
`ifdef OPEN132_COUNT64
    wire [ 63: 0] count_read;
    wire csr_count_mask;
    wire [ 31: 0] count_offset_mask;
`else
    wire csr_count_mask;
`endif
    wire [ 31: 0] reset_addr;
`ifdef CONFIG_MMU
    wire [  9: 0] ctrl_asid;
`else
    wire [  7: 0] ctrl_asid;
`endif
    wire [  3: 0] ctrl_mpuhi_valid;
    wire [ 19: 0] ctrl_mpuhi_base [  3: 0];
    wire [ 19: 0] ctrl_mpulo_mask [  3: 0];
    wire [  3: 0] ctrl_mpulo_calg [  3: 0];
    wire [4*`OPEN132_CFGSEG_NUM-1: 0] ctrl_segcalg;
    wire [ 31: 0] ctrl_wrdata;
    wire [ 31: 0] ctrl_wrmask;
    wire [ 31: 0] merrera_in;
    wire real_userdef_dccbp;
    wire real_userdef_iccbp;
    wire userdef_clr_pred;
`ifdef OPEN132_HAS_LJTAG
    wire ex_dbgss_ok;
    wire ex_ihb_ok_no_skip;
    wire ex_ihb_ok;
    wire ex_dbcl_ok;
    wire ex_dint_ok;
    wire debug_ex_wen;
    wire debug_mode_ex_wen;
    wire dbgss_set;
    wire dint_set;
    wire ihb_set;
    wire dbc_set;
    wire ihb_mask;
    wire dbc_mask;
    wire dertn_write;
    wire debug_cacheep;
    wire pesudoint_go;
    wire mem_ex_go;
    wire dertn_go;
    wire mem_noex;
    wire [ 11: 0] ihb_csr_off;
    wire ihb_csr_req_valid;
    wire ihb_csr_req_wr;
    wire [ 31: 0] ihb_csr_req_wdata;
    wire [ 31: 0] ihb_csr_wrmask;
`endif
    wire dcc_fatal;
    wire icc_fatal;
    wire fatal_rd;
    wire impr_fatal_wr;
    wire impr_fatal_ccop;
    wire impr_fatal_rf;
    wire impr_fatal_nc;
    wire fatal_intagv;
    wire [  7: 0] fatal_indata;
    wire [  3: 0] fatal_way;
    wire [  8: 0] fatal_indx;
    wire fatal_dirtywb;
    wire [ 21: 0] fatal_dirty_tag;
    wire ccfatal_in;
    wire true_ccerr0_wen;
    wire true_taglo_wen;

//============================================================//
//                    Module Implementation
//============================================================//

// x_csr_stall_chk
assign inst_csr_write_stall_chk  =  lisa_csr_write;
assign csr_num_stall_chk         =  lisa_si14;
assign inst_ctrl_stallf          =  ctrl_stallf_stall_chk;
// x_mmu_vaddr_sel_bus
assign vaddr_vaddr_sel     =  memls_vaddr;
assign ibus_win_vaddr_sel  =  ibus_win;
assign dbus_win_vaddr_sel  =  dbus_win;
assign mem_hit_ibus        =  ibus_hit_vaddr_sel;
assign mem_hit_dbus        =  dbus_hit_vaddr_sel;
// x_csr_cpucfg
assign prid_revision_cpucfg  =  prid_revision;
assign inst_cpucfg_cpucfg    =  lisa_cpucfg;
assign cpucfg_addr_cpucfg    =  result;
assign cpucfg_result         =  cpucfg_result_cpucfg;
// x_mmu_mpu_0
assign clk_mpu_region0               =  clk_cr;
assign reset_mpu_region0             =  reset;
assign data_reset_mpu_region0        =  data_reset;
assign lisa_kernel_mode_mpu_region0  =  lisa_kernel_mode;
assign mpuhi_wen_mpu_region0         =  csr180_wen;
assign mpulo_wen_mpu_region0         =  csr181_wen;
assign ctrl_wrdata_mpu_region0       =  ctrl_wrdata;
assign ctrl_wrmask_mpu_region0       =  ctrl_wrmask;
assign csr_csr180                    =  csr_mpuhi_mpu_region0;
assign csr_csr181                    =  csr_mpulo_mpu_region0;
assign ctrl_mpuhi_valid[0]           =  ctrl_mpuhi_valid_mpu_region0;
assign ctrl_mpuhi_base[0]            =  ctrl_mpuhi_base_mpu_region0;
assign ctrl_mpulo_mask[0]            =  ctrl_mpulo_mask_mpu_region0;
assign ctrl_mpulo_calg[0]            =  ctrl_mpulo_calg_mpu_region0;
// x_mmu_mpu_1
assign clk_mpu_region1               =  clk_cr;
assign reset_mpu_region1             =  reset;
assign data_reset_mpu_region1        =  data_reset;
assign lisa_kernel_mode_mpu_region1  =  lisa_kernel_mode;
assign mpuhi_wen_mpu_region1         =  csr184_wen;
assign mpulo_wen_mpu_region1         =  csr185_wen;
assign ctrl_wrdata_mpu_region1       =  ctrl_wrdata;
assign ctrl_wrmask_mpu_region1       =  ctrl_wrmask;
assign csr_csr184                    =  csr_mpuhi_mpu_region1;
assign csr_csr185                    =  csr_mpulo_mpu_region1;
assign ctrl_mpuhi_valid[1]           =  ctrl_mpuhi_valid_mpu_region1;
assign ctrl_mpuhi_base[1]            =  ctrl_mpuhi_base_mpu_region1;
assign ctrl_mpulo_mask[1]            =  ctrl_mpulo_mask_mpu_region1;
assign ctrl_mpulo_calg[1]            =  ctrl_mpulo_calg_mpu_region1;
// x_mmu_mpu_2
assign clk_mpu_region2               =  clk_cr;
assign reset_mpu_region2             =  reset;
assign data_reset_mpu_region2        =  data_reset;
assign lisa_kernel_mode_mpu_region2  =  lisa_kernel_mode;
assign mpuhi_wen_mpu_region2         =  csr188_wen;
assign mpulo_wen_mpu_region2         =  csr189_wen;
assign ctrl_wrdata_mpu_region2       =  ctrl_wrdata;
assign ctrl_wrmask_mpu_region2       =  ctrl_wrmask;
assign csr_csr188                    =  csr_mpuhi_mpu_region2;
assign csr_csr189                    =  csr_mpulo_mpu_region2;
assign ctrl_mpuhi_valid[2]           =  ctrl_mpuhi_valid_mpu_region2;
assign ctrl_mpuhi_base[2]            =  ctrl_mpuhi_base_mpu_region2;
assign ctrl_mpulo_mask[2]            =  ctrl_mpulo_mask_mpu_region2;
assign ctrl_mpulo_calg[2]            =  ctrl_mpulo_calg_mpu_region2;
// x_mmu_mpu_3
assign clk_mpu_region3               =  clk_cr;
assign reset_mpu_region3             =  reset;
assign data_reset_mpu_region3        =  data_reset;
assign lisa_kernel_mode_mpu_region3  =  lisa_kernel_mode;
assign mpuhi_wen_mpu_region3         =  csr18c_wen;
assign mpulo_wen_mpu_region3         =  csr18d_wen;
assign ctrl_wrdata_mpu_region3       =  ctrl_wrdata;
assign ctrl_wrmask_mpu_region3       =  ctrl_wrmask;
assign csr_csr18c                    =  csr_mpuhi_mpu_region3;
assign csr_csr18d                    =  csr_mpulo_mpu_region3;
assign ctrl_mpuhi_valid[3]           =  ctrl_mpuhi_valid_mpu_region3;
assign ctrl_mpuhi_base[3]            =  ctrl_mpuhi_base_mpu_region3;
assign ctrl_mpulo_mask[3]            =  ctrl_mpulo_mask_mpu_region3;
assign ctrl_mpulo_calg[3]            =  ctrl_mpulo_calg_mpu_region3;


//                      csr_ifu_ctrl_bus
//------------------------------------------------------------//
assign csr_ifu_ctrl_bus  =  {csr_ifu_lisa_segplv,  // 9:2
                              csr_ifu_lisa_da    ,  // 1
                              csr_ifu_user_mode  // 0
                            };
// End csr_ifu_ctrl_bus

//------------------------------------------------------------//
//                      csr_idu_ctrl_bus
//------------------------------------------------------------//
assign csr_idu_ctrl_bus  =  {csr_idu_lisa_ihb_skip ,  // 23
                              csr_idu_lisa_da       ,  // 22
                              csr_idu_lisa_dacm     ,  // 21:20
                              csr_idu_lisa_segplv   ,  // 19:12
                              csr_idu_lisa_segca_dev,  // 11:4
                              csr_idu_user_ccop     ,  // 3:3
                              csr_idu_rdtime_dis    ,  // 2:2
                              csr_idu_fpen          ,  // 1:1
                              csr_idu_user_mode  // 0:0
                            };
// End csr_idu_ctrl_bus

//------------------------------------------------------------//
//                     csr_cache_ctrl_bus
//------------------------------------------------------------//

assign csr_cache_ctrl_bus  =  {csr_cache_cfgseg_calg     ,
                                csr_cache_lisa_da         ,
                                csr_cache_lisa_dacf       ,
                                csr_cache_lisa_dacm       ,
                                csr_cache_lisa_segpa      ,
                                csr_cache_mpu_valid       ,
                                csr_cache_mpu_base        ,
                                csr_cache_mpu_mask        ,
                                csr_cache_mpu_calg        ,
                                csr_cache_dcc_bypass      ,
                                csr_cache_icc_bypass
                              };
// End csr_cache_ctrl_bus

//------------------------------------------------------------//
`ifdef CONFIG_MMU
//                      csr_mmu_ctrl_bus
//------------------------------------------------------------//

assign csr_mmu_ctrl_bus  =  {plv_r,             // 194:193, current PLV
                              da_r,              // 192, direct-address mode
                              pg_r,              // 191, paging mode
                              dacf_r,            // 190:189, direct fetch MAT
                              dacm_r,            // 188:187, direct data MAT
                              asid_r,            // 186:177, current ASID
                              pgdl_r,            // 176:145, low-half PGD
                              pgdh_r,            // 144:113, high-half PGD
                              dmw0_r,            // 112:81, direct-map window 0
                              dmw1_r,            // 80:49, direct-map window 1
                              1'b1,              // 48, hardware PTW present
                              csr_inv_vld,        // 47, INVTLB valid
                              csr_inv_op,         // 46:42, INVTLB operation
                              csr_inv_asid,       // 41:32, INVTLB ASID
                              csr_inv_vaddr};     // 31:0, INVTLB virtual address
// End csr_mmu_ctrl_bus

//------------------------------------------------------------//
`endif

//                   User Super Kernel Mode
//------------------------------------------------------------//
// End User Super Kernel Mode

//------------------------------------------------------------//
//                      Csr Instruction
//------------------------------------------------------------//
assign {inst_idle,inst_ertn}  =  {2{idle_ertn}} & result[31:30];

assign ertn_write        =  inst_ertn && inst_oth_noex;
assign lisa_csr_rdt_cfg  =  inst_lisa[3:2]!=2'd0;
assign lisa_csrrd        =  inst_lisa[3:2]==2'd1;
assign lisa_csr_write    =  inst_lisa[3:2]==2'd1 && !inst_lisa[1];
assign lisa_rdtime       =  inst_lisa[3:2]==2'd2;
assign lisa_rdtime_h     =  inst_lisa[0  ]==1'b1;
assign lisa_rdtime_2nd   =  inst_lisa[1  ]==1'b1;
assign lisa_cpucfg       =  inst_lisa[3:2]==2'd3;
assign lisa_ibar         =  inst_lisa==4'd1;
`ifdef CONFIG_MMU
assign lisa_invtlb       =  inst_lisa==4'd2;

assign csr_inv_vld    =  lisa_invtlb && inst_oth_noex;
assign csr_inv_op     =  lisa_si14[4:0];
assign csr_inv_asid   =  result[9:0];
assign csr_inv_vaddr  =  ld_bdata_stldlr;
`endif
assign inst_split_1st    =  lisa_rdtime&&!lisa_rdtime_2nd&&!ex;
assign inst_split_2nd    =  lisa_rdtime&&lisa_rdtime_2nd;
// for cr-cg
assign lsu_clk_csr_enable  =  lisa_csr_write&&valid;

`ifdef CONFIG_MMU
assign inst_stall_fetch  =  inst_ctrl_stallf || lisa_ibar || lisa_invtlb;
`else
assign inst_stall_fetch  =  inst_ctrl_stallf || lisa_ibar;
`endif
csr_stall_chk x_csr_stall_chk (
    .inst_csr_write (inst_csr_write_stall_chk),
    .csr_num        (csr_num_stall_chk       ),
    .ctrl_stallf    (ctrl_stallf_stall_chk   )
);
// End Csr Instruction

//------------------------------------------------------------//
//                          Csr Regs
//------------------------------------------------------------//
//------------------------------------------------------------//
//                 CSR Register Write Enables
//------------------------------------------------------------//
// cache csr

assign badv_wen      =  !mark_nmi_ccerr_int && valid_go_wb && !debug_mode
`ifdef CONFIG_MMU
                        && (fetch_ade || ex_lserr || ex_page || ex_mmu);
`else
                        &&(fetch_ade || ex_lserr);  // adel/ades
`endif
assign taglo_cc_wen  =  inst_oth_noex && mem_cps_data_ready && mem_cc_op[4];  // load tag
// End CSR Register Write Enables

`ifdef CONFIG_MMU
assign badv_wrdata  =  (fetch_ade || ex_page) && !ex_mmu
                        ? {pc[31:1],pc[0]} : memls_vaddr;
`else
assign badv_wrdata  =  fetch_ade ? {pc[31:1],pc[0]} : memls_vaddr;
`endif

assign n_general_ex_wen  =  ex_general & rtu_lsu_allowin & ~ex_cancel & ~debug_mode & ~debug_ex;

assign ex_main_pc  =  pc;
assign brpc_or_pc  =  ex_main_pc;
assign era_in      =  sleeping ? idu_lsu_follow_pc : brpc_or_pc;  // idu_lsu_follow_pc is pc_fllowing_wait

always @ ( posedge clk_g )
begin
    mem_validout_ex_r <= valid_go_wb&&mem_ex;
end
assign perf_ex  =  mem_validout_ex_r;

assign perf_main_cmt  =  rtu_lsu_commit;
assign perf_mdu_cmt   =  1'b0;
assign perf_fpu_cmt   =  1'b0;

// wire mem_hit_sbus;
mmu_vaddr_sel_bus x_mmu_vaddr_sel_bus (
    .vaddr      (vaddr_vaddr_sel   ),
    .ibus_win   (ibus_win_vaddr_sel),  // I, (1+32+32)*4
    .dbus_win   (dbus_win_vaddr_sel),  // I, (1+32+32)*4
    .ibus_hit   (ibus_hit_vaddr_sel),
    .dbus_hit   (dbus_hit_vaddr_sel),
    .sysbus_hit (                  )
);
assign mem_perf_go  =  noex_lsok_go;
always @ ( posedge clk_g )
begin
    mem_is_ld_r   <= mem_perf_go & mem_is_ld;
    mem_is_st_r   <= mem_perf_go & mem_is_st;
    mem_hit_bus_r <= {mem_hit_ibus,mem_hit_dbus};
end

assign perf_load     =  mem_is_ld_r;
assign perf_st       =  mem_is_st_r;
assign perf_ls       =  perf_load||perf_st;
assign perf_ls_ibus  =  perf_ls&&mem_hit_bus_r[1];
assign perf_ls_dbus  =  perf_ls&&mem_hit_bus_r[0];
assign perf_ls_sbus  =  perf_ls&&mem_hit_bus_r==2'd0;

assign perf_uncachels_sbus  =  dcc_uncached_sbus;
assign perf_cacheft_sbus    =  icc_cached_sbus;

assign perf_dcc_refill_add1  =  dcc_refill_add1;
assign perf_icc_refill_add1  =  icc_refill_add1;

assign perf_dcc_all_ecc_err_add1  =  dcc_all_ecc_err_add1;
assign perf_icc_all_ecc_err_add1  =  icc_all_ecc_err_add1;

assign perf_events  =  {de_perf_inst_ibus        ,  // 1 , total:25, inst from ibus
                         de_perf_inst_dbus        ,  // 1 ,         , inst from dbus
                         de_perf_inst_sbus        ,  // 1 ,         , inst from sysbus
                         de_perf_load_stall       ,  // 1 ,         , load-to-use
                         de_perf_nop              ,  // 1 ,         , NOP
                         perf_ex                  ,  // 1 , total:20, exception
                         perf_main_cmt            ,  // 1 ,         , main commit
                         perf_mdu_cmt             ,  // 1 ,         , mdu commit
                         perf_fpu_cmt             ,  // 1 ,         , fpu commit
                         perf_ls                  ,  // 1 , total:16, load/store
                         perf_load                ,  // 1 ,         , load
                         perf_st                  ,  // 1 ,         , store
                         perf_ls_ibus             ,  // 1 ,         , load/store to ibus
                         perf_ls_dbus             ,  // 1 ,         , load/store to dbus
                         perf_ls_sbus             ,  // 1 ,         , load/store to sysbus
                         1'b0                     ,  // 1 , total:10, reserved
                         1'b0                     ,  // 1 ,         , reserved
                         perf_uncachels_sbus      ,  // 1 ,         , uncached ls to sysbus
                         perf_cacheft_sbus        ,  // 1 ,         , cached ft to sysbus
                         perf_dcc_refill_add1     ,  // 1 , total: 6, dcc refill
                         perf_icc_refill_add1     ,  // 1 ,         , icc refill
                         1'b0                     ,  // 1 ,         ,
                         perf_dcc_all_ecc_err_add1,  // 1 ,         , dcc all ecc 1bit err
                         1'b0                     ,  // 1 ,         ,
                         perf_icc_all_ecc_err_add1  // 1 ,         , icc all ecc 1bit err
                       };

// End Csr Regs

//------------------------------------------------------------//
//                          Lisa Csr
//------------------------------------------------------------//
assign csr_wrdata  =  ld_bdata_stldlr;
assign csr_wrmask  =  result;
assign csr_num     =  lisa_si14;
assign csr_wen     =  lisa_csr_write && inst_oth_noex;

//------------------------------------------------------------//
//                          Csr Regs
//------------------------------------------------------------//
// End Csr Regs

//------------------------------------------------------------//
//                     Csr Regs Subfield
//------------------------------------------------------------//

`ifdef GODSON_DEBUG
assign gs_core_csr0  =  csr_csr0;
assign gs_core_csr1  =  csr_csr1;
assign gs_core_csr4  =  csr_csr4;
assign gs_core_csr5  =  csr_csr5;
assign gs_core_csr6  =  csr_csr6;
assign gs_core_csr7  =  csr_csr7;
`endif

assign lisa_user_mode    =  csr_csr0_plv!=2'd0;
assign lisa_kernel_mode  =  csr_csr0_plv==2'd0;
// End Csr Regs Subfield
assign csr0_ren    =  lisa_csrrd && csr_num==14'h0  ;
assign csr1_ren    =  lisa_csrrd && csr_num==14'h1  ;
assign csr2_ren    =  lisa_csrrd && csr_num==14'h2  ;
assign csr3_ren    =  lisa_csrrd && csr_num==14'h3  ;
assign csr4_ren    =  lisa_csrrd && csr_num==14'h4  ;
assign csr5_ren    =  lisa_csrrd && csr_num==14'h5  ;
assign csr6_ren    =  lisa_csrrd && csr_num==14'h6  ;
assign csr7_ren    =  lisa_csrrd && csr_num==14'h7  ;
assign csrc_ren    =  lisa_csrrd && csr_num==14'hc  ;
assign csr18_ren   =  lisa_csrrd && csr_num==14'h18 ;
`ifdef CONFIG_MMU
assign csr19_ren   =  lisa_csrrd && csr_num==14'h19 ;
assign csr1a_ren   =  lisa_csrrd && csr_num==14'h1a ;
assign csr1b_ren   =  lisa_csrrd && csr_num==14'h1b ;
assign csr1c_ren   =  lisa_csrrd && csr_num==14'h1c ;
assign csr1d_ren   =  lisa_csrrd && csr_num==14'h1d ;
`endif
assign csr20_ren   =  lisa_csrrd && csr_num==14'h20 ;
assign csr21_ren   =  lisa_csrrd && csr_num==14'h21 ;
`ifdef CONFIG_MMU
assign csr22_ren   =  lisa_csrrd && csr_num==14'h22 ;
assign csr23_ren   =  lisa_csrrd && csr_num==14'h23 ;
`endif
assign csr30_ren   =  lisa_csrrd && csr_num==14'h30 ;
assign csr31_ren   =  lisa_csrrd && csr_num==14'h31 ;
assign csr40_ren   =  lisa_csrrd && csr_num==14'h40 ;
assign csr41_ren   =  lisa_csrrd && csr_num==14'h41 ;
assign csr42_ren   =  lisa_csrrd && csr_num==14'h42 ;
assign csr43_ren   =  lisa_csrrd && csr_num==14'h43 ;
assign csr60_ren   =  lisa_csrrd && csr_num==14'h60 ;
assign csr80_ren   =  lisa_csrrd && csr_num==14'h80 ;
assign csr90_ren   =  lisa_csrrd && csr_num==14'h90 ;
assign csr91_ren   =  lisa_csrrd && csr_num==14'h91 ;
assign csr92_ren   =  lisa_csrrd && csr_num==14'h92 ;
assign csr93_ren   =  lisa_csrrd && csr_num==14'h93 ;
assign csr94_ren   =  lisa_csrrd && csr_num==14'h94 ;
assign csr96_ren   =  lisa_csrrd && csr_num==14'h96 ;
assign csr98_ren   =  lisa_csrrd && csr_num==14'h98 ;
assign csraf_ren   =  lisa_csrrd && csr_num==14'haf ;
assign csrff_ren   =  lisa_csrrd && csr_num==14'hff ;
assign csr180_ren  =  lisa_csrrd && csr_num==14'h180;
assign csr181_ren  =  lisa_csrrd && csr_num==14'h181;
assign csr184_ren  =  lisa_csrrd && csr_num==14'h184;
assign csr185_ren  =  lisa_csrrd && csr_num==14'h185;
assign csr188_ren  =  lisa_csrrd && csr_num==14'h188;
assign csr189_ren  =  lisa_csrrd && csr_num==14'h189;
assign csr18c_ren  =  lisa_csrrd && csr_num==14'h18c;
assign csr18d_ren  =  lisa_csrrd && csr_num==14'h18d;
assign csr190_ren  =  lisa_csrrd && csr_num==14'h190;
assign csr191_ren  =  lisa_csrrd && csr_num==14'h191;
assign csr381_ren  =  lisa_csrrd && csr_num==14'h381;
assign csr500_ren  =  lisa_csrrd && csr_num==14'h500;
assign csr501_ren  =  lisa_csrrd && csr_num==14'h501;
assign csr502_ren  =  lisa_csrrd && csr_num==14'h502;
assign csr508_ren  =  lisa_csrrd && csr_num==14'h508;

assign rdtime_result  =  csr_count | {32{lisa_rdtime&&lisa_rdtime_2nd}}&csr_csr40;
csr_cpucfg #(
    .HAS_ICC       (HAS_ICC_CPUCFG      ),
    .ICC_SET_ASSOC (ICC_SET_ASSOC_CPUCFG),
    .ICC_WAYSIZE   (ICC_WAYSIZE_CPUCFG  ),
    .HAS_DCC       (HAS_DCC_CPUCFG      ),
    .DCC_SET_ASSOC (DCC_SET_ASSOC_CPUCFG),
    .DCC_WAYSIZE   (DCC_WAYSIZE_CPUCFG  )
) x_csr_cpucfg (
    .prid_revision (prid_revision_cpucfg),
    .inst_cpucfg   (inst_cpucfg_cpucfg  ),
    .cpucfg_addr   (cpucfg_addr_cpucfg  ),  ///[31:0]
    .cpucfg_result (cpucfg_result_cpucfg)
);
assign csrrd_result  =  ( {32{csr0_ren  }} & csr_csr0  )
                        | ( {32{csr1_ren  }} & csr_csr1  )
                        | ( {32{csr2_ren  }} & csr_csr2  )
                        | ( {32{csr3_ren  }} & csr_csr3  )
                        | ( {32{csr4_ren  }} & csr_csr4  )
                        | ( {32{csr5_ren  }} & csr_csr5  )
                        | ( {32{csr6_ren  }} & csr_csr6  )
                        | ( {32{csr7_ren  }} & csr_csr7  )
                        | ( {32{csrc_ren  }} & csr_csrc  )
                        | ( {32{csr18_ren }} & csr_csr18 )
`ifdef CONFIG_MMU
                        | ( {32{csr19_ren }} & csr_csr19 )
                        | ( {32{csr1a_ren }} & csr_csr1a )
                        | ( {32{csr1b_ren }} & csr_csr1b )
                        | ( {32{csr1c_ren }} & csr_csr1c )
                        | ( {32{csr1d_ren }} & csr_csr1d )
`endif
                        | ( {32{csr20_ren }} & csr_csr20 )
                        | ( {32{csr21_ren }} & csr_csr21 )
`ifdef CONFIG_MMU
                        | ( {32{csr22_ren }} & csr_csr22 )
                        | ( {32{csr23_ren }} & csr_csr23 )
`endif
                        | ( {32{csr30_ren }} & csr_csr30 )
                        | ( {32{csr31_ren }} & csr_csr31 )
                        | ( {32{csr40_ren }} & csr_csr40 )
                        | ( {32{csr41_ren }} & csr_csr41 )
                        | ( {32{csr42_ren }} & csr_csr42 )
                        | ( {32{csr43_ren }} & csr_csr43 )
                        | ( {32{csr60_ren }} & csr_csr60 )
                        | ( {32{csr80_ren }} & csr_csr80 )
                        | ( {32{csr90_ren }} & csr_csr90 )
                        | ( {32{csr91_ren }} & csr_csr91 )
                        | ( {32{csr92_ren }} & csr_csr92 )
                        | ( {32{csr93_ren }} & csr_csr93 )
                        | ( {32{csr94_ren }} & csr_csr94 )
                        | ( {32{csr96_ren }} & csr_csr96 )
                        | ( {32{csr98_ren }} & csr_csr98 )
                        | ( {32{csraf_ren }} & csr_csraf )
                        | ( {32{csrff_ren }} & csr_csrff )
`ifdef CONFIG_MMU
                        | ( {32{csr180_ren}} & dmw0_r    )
                        | ( {32{csr181_ren}} & dmw1_r    )
`else
                        | ( {32{csr180_ren}} & csr_csr180)
                        | ( {32{csr181_ren}} & csr_csr181)
`endif
                        | ( {32{csr184_ren}} & csr_csr184)
                        | ( {32{csr185_ren}} & csr_csr185)
                        | ( {32{csr188_ren}} & csr_csr188)
                        | ( {32{csr189_ren}} & csr_csr189)
                        | ( {32{csr18c_ren}} & csr_csr18c)
                        | ( {32{csr18d_ren}} & csr_csr18d)
                        | ( {32{csr190_ren}} & csr_csr190)
                        | ( {32{csr191_ren}} & csr_csr191)
                        | ( {32{csr381_ren}} & csr_csr381)
                        | ( {32{csrihb_ren}} & csr_csrihb)
                        | ( {32{csr500_ren}} & csr_csr500)
                        | ( {32{csr501_ren}} & csr_csr501)
                        | ( {32{csr502_ren}} & csr_csr502)
                        | ( {32{csr508_ren}} & csr_csr508);
//------------------------------------------------------------//
//                    Csr Regs Read Enable
//------------------------------------------------------------//
// End Csr Regs Read Enable

//------------------------------------------------------------//
//                 CSR Register Write Enables
//------------------------------------------------------------//
assign csr0_wen    =  csr_wen&&csr_num==14'h0  ;
assign csr1_wen    =  csr_wen&&csr_num==14'h1  ;
assign csr2_wen    =  csr_wen&&csr_num==14'h2  ;
assign csr3_wen    =  csr_wen&&csr_num==14'h3  ;
assign csr4_wen    =  csr_wen&&csr_num==14'h4  ;
assign csr5_wen    =  csr_wen&&csr_num==14'h5  ;
assign csr6_wen    =  csr_wen&&csr_num==14'h6  ;
assign csr7_wen    =  csr_wen&&csr_num==14'h7  ;
assign csrc_wen    =  csr_wen&&csr_num==14'hc  ;
assign csr18_wen   =  csr_wen&&csr_num==14'h18 ;
`ifdef CONFIG_MMU
assign csr19_wen   =  csr_wen&&csr_num==14'h19 ;
assign csr1a_wen   =  csr_wen&&csr_num==14'h1a ;
assign csr1c_wen   =  csr_wen&&csr_num==14'h1c ;
assign csr1d_wen   =  csr_wen&&csr_num==14'h1d ;
`endif
assign csr30_wen   =  csr_wen&&csr_num==14'h30 ;
assign csr31_wen   =  csr_wen&&csr_num==14'h31 ;
assign csr40_wen   =  csr_wen&&csr_num==14'h40 ;
assign csr41_wen   =  csr_wen&&csr_num==14'h41 ;
assign csr43_wen   =  csr_wen&&csr_num==14'h43 ;
assign csr44_wen   =  csr_wen&&csr_num==14'h44 ;
assign csr60_wen   =  csr_wen&&csr_num==14'h60 ;
assign csr80_wen   =  csr_wen&&csr_num==14'h80 ;
assign csr90_wen   =  csr_wen&&csr_num==14'h90 ;
assign csr91_wen   =  csr_wen&&csr_num==14'h91 ;
assign csr93_wen   =  csr_wen&&csr_num==14'h93 ;
assign csr94_wen   =  csr_wen&&csr_num==14'h94 ;
assign csr96_wen   =  csr_wen&&csr_num==14'h96 ;
assign csr98_wen   =  csr_wen&&csr_num==14'h98 ;
assign csraf_wen   =  csr_wen&&csr_num==14'haf ;
assign csrff_wen   =  csr_wen&&csr_num==14'hff ;
assign csr180_wen  =  csr_wen&&csr_num==14'h180;
assign csr181_wen  =  csr_wen&&csr_num==14'h181;
assign csr184_wen  =  csr_wen&&csr_num==14'h184;
assign csr185_wen  =  csr_wen&&csr_num==14'h185;
assign csr188_wen  =  csr_wen&&csr_num==14'h188;
assign csr189_wen  =  csr_wen&&csr_num==14'h189;
assign csr18c_wen  =  csr_wen&&csr_num==14'h18c;
assign csr18d_wen  =  csr_wen&&csr_num==14'h18d;
assign csr190_wen  =  csr_wen&&csr_num==14'h190;
assign csr191_wen  =  csr_wen&&csr_num==14'h191;
assign csr300_wen  =  csr_wen&&csr_num==14'h300;
assign csr301_wen  =  csr_wen&&csr_num==14'h301;
assign csr380_wen  =  csr_wen&&csr_num==14'h380;
assign csr381_wen  =  csr_wen&&csr_num==14'h381;
assign csr501_wen  =  csr_wen&&csr_num==14'h501;
assign csr502_wen  =  csr_wen&&csr_num==14'h502;
assign csr508_wen  =  csr_wen&&csr_num==14'h508;
// End CSR Register Write Enables

//------------------------------------------------------------//
//                 Ex And Ertn To Modify Csr
//------------------------------------------------------------//
assign ex_norm_wen  =  n_general_ex_wen;
assign ex_mc_wen    =  nmi_end||ccerr_wr_csr;
// End Ex And Ertn To Modify Csr

//------------------------------------------------------------//
//                         csr0: PLV
//------------------------------------------------------------//
assign csr_csr0       =  { 22'd0             ,  // 31:10
                           watchen_r         ,  // 9
                           dacm_r            ,  // 8:7 ,
                           dacf_r            ,  // 6:5 ,
`ifdef CONFIG_MMU
                           pg_r              ,  // 4   , PG
`else
                           1'b0              ,  // 4   , PG=0
`endif
                           da_r              ,  // 3   , DA=1
                           ie_r              ,  // 2
                           plv_r  // 1 :0, plv
                         };
assign csr_csr0_da    =  da_r;
assign csr_csr0_we    =  watchen_r;
assign csr_csr0_dacf  =  dacf_r;
assign csr_csr0_dacm  =  dacm_r;
assign csr_csr0_ie    =  ie_r;
assign csr_csr0_plv   =  plv_r;
always @ ( posedge clk_g )
begin
    // dacm_r
    if(reset || ex_mc_wen) begin
        dacm_r <= 2'd0;
    end
    else if(csr0_wen) begin
        dacm_r <= csr_wrmask[8:7]&csr_wrdata[8:7]|~csr_wrmask[8:7]&dacm_r;
    end
    else if(ertn_write&&!debug_mode&&csr_csr90_errmode) begin
        dacm_r <= csr_csr90_errpdacm;
    end
    // dacf_r
    if(reset || ex_mc_wen) begin
        dacf_r <= 2'd0;
    end
    else if(csr0_wen) begin
        dacf_r <= csr_wrmask[6:5]&csr_wrdata[6:5]|~csr_wrmask[6:5]&dacf_r;
    end
    else if(ertn_write&&!debug_mode&&csr_csr90_errmode) begin
        dacf_r <= csr_csr90_errpdacf;
    end
    // da_r
    if(reset || ex_mc_wen) begin
        da_r <= 1'b1;
    end
    else if(csr0_wen) begin
        da_r <= csr_wrmask[3]&csr_wrdata[3]|~csr_wrmask[3]&da_r;
    end
    else if(ertn_write&&!debug_mode&&csr_csr90_errmode) begin
        da_r <= csr_csr90_errpda;
    end
`ifdef CONFIG_MMU
    // pg_r
    if(reset || ex_mc_wen) begin
        pg_r <= 1'b0;
    end
    else if(csr0_wen) begin
        pg_r <= csr_wrmask[4]&csr_wrdata[4]|~csr_wrmask[4]&pg_r;
    end
    else if(ertn_write&&!debug_mode&&csr_csr90_errmode) begin
        pg_r <= csr_csr90_errppg;
    end
`endif
    // ie_r
    if(reset || ex_mc_wen || ex_norm_wen) begin
        ie_r <= 1'b0;
    end
    else if(csr0_wen) begin
        ie_r <= csr_wrmask[2]&csr_wrdata[2]|~csr_wrmask[2]&ie_r;
    end
    else if(ertn_write&&!debug_mode) begin
        ie_r <= csr_csr90_errmode ? csr_csr90_errpie : csr_csr1_pie;
    end
    // plv_r
    if(reset || ex_mc_wen || ex_norm_wen) begin
        plv_r <= 2'd0;  // only 0 or 3
    end
    else if(csr0_wen && csr_wrmask[1:0]==2'b11) begin
        plv_r <= csr_wrdata[1:0]==2'b11 ? 2'b11 :  // write to 3
        csr_wrdata[1:0] ==2'b00 ? 2'b00 : plv_r;  // write to 0
    end
    else if(ertn_write&&!debug_mode) begin
        plv_r <= csr_csr90_errmode ? csr_csr90_errpplv : csr_csr1_pplv;
    end
    // watchen_r
    if(reset || ex_mc_wen || ex_norm_wen) begin
        watchen_r <= 1'b0;
    end
    else if(csr0_wen) begin
        watchen_r <= csr_wrmask[9]&csr_wrdata[9]|~csr_wrmask[9]&watchen_r;
    end
    else if(ertn_write&&!debug_mode) begin
        watchen_r <= csr_csr90_errmode ? csr_csr90_errpwe : csr_csr1_pwe;
    end
end
// End csr0: PLV

//------------------------------------------------------------//
//                         csr1: PPLV
//------------------------------------------------------------//
assign csr_csr1       =  { 28'd0    ,  // 31:4
                           pwatchen_r,  // 3
                           pie_r    ,  // 2
                           pplv_r  // 1 :0, plv
                         };
assign csr_csr1_pie   =  pie_r;
assign csr_csr1_pwe   =  pwatchen_r;
assign csr_csr1_pplv  =  pplv_r;
always @ ( posedge clk_g )
begin
    // we_r
    if(reset) begin
        pwatchen_r <= 1'b0;
    end
    else if(ex_norm_wen) begin
        pwatchen_r <= csr_csr0_we;
    end
    else if(csr1_wen) begin
        pwatchen_r <= csr_wrmask[3]&csr_wrdata[3]|~csr_wrmask[3]&pwatchen_r;
    end
    // ie_r
    if(reset) begin
        pie_r <= 1'b0;
    end
    else if(ex_norm_wen) begin
        pie_r <= csr_csr0_ie;
    end
    else if(csr1_wen) begin
        pie_r <= csr_wrmask[2]&csr_wrdata[2]|~csr_wrmask[2]&pie_r;
    end
    // plv_r
    if(reset) begin
        pplv_r <= 2'd0;  // only 0 or 3
    end
    else if(ex_norm_wen) begin
        pplv_r <= csr_csr0_plv;
    end
    else if(csr1_wen && csr_wrmask[1:0]==2'b11) begin
        pplv_r <= csr_wrdata[1:0]==2'b11 ? 2'b11 :  // write to 3
        csr_wrdata[1:0] ==2'b00 ? 2'b00 : pplv_r;  // write to 0
    end
end
// End csr1: PPLV

//------------------------------------------------------------//
//                       csr2: Unit En
//------------------------------------------------------------//
assign csr_csr2       =  { 30'd0   ,  // 31:1
                           fpen_r  // 0
                         };
assign csr_csr2_fpen  =  fpen_r;

always @ ( posedge clk_g )
begin
    if(reset) begin
        fpen_r <= 1'b0;
    end
end
// End csr2: Unit En

//------------------------------------------------------------//
//                       csr3: Func Cfg
//------------------------------------------------------------//
assign perf_enp3_r            =  1'b0;
assign csr_csr3               =  { 20'd0         ,  // 31:12
                                   perf_enp3_r   ,  // 11
                                   3'd0          ,  // 10:8
                                   rdtime_disp3_r,  // 7
                                   7'd0  // 6:0
                                 };
assign csr_csr3_rdtime_disp3  =  rdtime_disp3_r;
always @ ( posedge clk_cr )
begin
    // rdtime_disp3_r
    if(reset) begin
        rdtime_disp3_r <= 1'b0;
    end
    else if(csr3_wen) begin
        rdtime_disp3_r <= csr_wrmask[7]&csr_wrdata[7]|~csr_wrmask[7]&rdtime_disp3_r;
    end
end
// End csr3: Func Cfg

//------------------------------------------------------------//
//                        csr4: Ex Cfg
//------------------------------------------------------------//
assign csr_csr4     =  { 13'd0         ,  // 31:12
                         vs_r          ,  // 18:16
                         3'd0          ,  // 15:13
                         im_r  // 12:0
                       };
assign csr_csr4_vs  =  vs_r;
assign csr_csr4_im  =  im_r;
always @ ( posedge clk_cr )
begin
    // vs_r
    if(reset) begin
        vs_r <= 3'd0;
    end
    else if(csr4_wen) begin
        vs_r <= csr_wrmask[18:16]&csr_wrdata[18:16]|~csr_wrmask[18:16]&vs_r;
    end
    // im_r
    if(reset) begin
        im_r <= 13'd0;
    end
    else if(csr4_wen) begin
        im_r <= csr_wrmask[12:0]&csr_wrdata[12:0]|~csr_wrmask[12:0]&im_r;
    end
end
// End csr4: Ex Cfg

//------------------------------------------------------------//
//                        csr5: Estat
//------------------------------------------------------------//
assign csr_csr5     =  { 8'd0          ,  // 31:24
                         sub_excode_r  ,  // 23:22
                         excode_r      ,  // 21:16
                         2'd0          ,  // 15:14
                         1'b0          ,  // 13, nmi=0, because nmi is pulse in OPEN132E
                         ip_r  // 12:0
                       };
assign csr_csr5_ip  =  ip_r;
always @ ( posedge clk_g )
begin
    // sub_excode_r
    if(reset) begin
        sub_excode_r <= 2'd0;
    end
    else if(ex_norm_wen) begin
        sub_excode_r <= mem_sub_excode;
    end
    // excode_r
    if(reset) begin
        excode_r <= 6'd0;
    end
    else if(ex_norm_wen) begin
`ifdef CONFIG_MMU
        excode_r <= mem_statcode;
`else
        excode_r <= {1'b0,mem_statcode[4:0]};  // now, only 5bits excode.
`endif
    end

    // ip_r[1:0]
    if(reset) begin
        ip_r[1:0] <= 2'b00;
    end
    else if(csr5_wen) begin
        ip_r[1:0] <= csr_wrmask[1:0]&csr_wrdata[1:0]|~csr_wrmask[1:0]&ip_r[1:0];
    end
end

always @ ( posedge clk_ng )
begin
    // ip_r[12:2]
    ip_r[12] <= 1'b0;  // no ipi for open132e
    if(reset) begin  // TI
        ip_r[11] <= 1'b0;
    end
    else if(csr44_wen&&csr_wrmask[0]&&csr_wrdata[0]) begin  // write CSR44[0]
        ip_r[11] <= 1'b0;
    end
    else if(csr_timer_int) begin
        ip_r[11] <= 1'b1;
    end
    ip_r[10 ] <= 1'b0;  // PCI
    ip_r[9:2] <= ext_int;
end
// End csr5: Estat

// csr6: era.
// csr7: badv.
// csr8: badinst. No
// csrc: eentry.
// csr18: asid.

//------------------------------------------------------------//
//                        csr20: Cpuno
//------------------------------------------------------------//
assign csr_csr20  =  {22'd0, cpunum};
// End csr20: Cpuno

//------------------------------------------------------------//
//                    csr21: num_indicate
//------------------------------------------------------------//
assign csr_csr21  =  {10'd0 ,  // 31:22
                       6'h1f ,  // 21:16, timer bits is 31+1
                       1'b0  ,  // 15   ,
                       3'h7  ,  // 14:12, vs_max is 7
                       2'd0  ,  // 11:10,
                       6'h1f ,  // 9 :4 , counter bits is 31+1
                       4'd2  // 3 :0 , save number is 2
                     };
// End csr21: num_indicate
`ifdef CONFIG_MMU
//------------------------------------------------------------//
//                    csr22-csr23: MMU Config
//------------------------------------------------------------//

assign csr_csr22  =  32'h0000_1000;  // Only the 4 KiB page size is supported.
assign csr_csr23  =  32'h0000_0071;  // One fully associative eight-entry TLB.
// End csr22-csr23: MMU Config
`endif

// csr30: save0
// csr31: save1

//------------------------------------------------------------//
//                       csr40: TimerID
//------------------------------------------------------------//
assign csr_csr40  =  timer_id_r;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        timer_id_r <= 32'd0 | cpunum;
    end
    else if(csr40_wen) begin
        timer_id_r <= csr_wrmask&csr_wrdata|~csr_wrmask&timer_id_r;
    end
end
// End csr40: TimerID

//------------------------------------------------------------//
//                      csr41: Timercfg
//------------------------------------------------------------//
assign csr_csr41              =  {timer_val_r   ,  // 31:2
                                   timer_period_r,  // 1
                                   timer_en_r  // 0
                                 };
assign csr_csr41_timerinit    =  {timer_val_r,2'd0};
assign csr_csr41_timerperiod  =  timer_period_r;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        timer_val_r    <= 30'd0;
        timer_period_r <= 1'b0;
        timer_en_r     <= 1'b0;
    end
    else if(csr41_wen) begin
        timer_val_r    <= csr_wrmask[31:2]&csr_wrdata[31:2]|~csr_wrmask[31:2]&timer_val_r;
        timer_period_r <= csr_wrmask[1]&csr_wrdata[1]|~csr_wrmask[1]&timer_period_r;
        timer_en_r     <= csr_wrmask[0]&csr_wrdata[0]|~csr_wrmask[0]&timer_en_r;
    end
end
// End csr41: Timercfg

//------------------------------------------------------------//
//                       Timer For Lisa
//------------------------------------------------------------//
assign csr_timer_int  =  (timer_r==32'd0) && timer_enable;
always @ ( posedge clk_count )
begin
    if(reset) begin
        timer_enable <= 1'd0;
    end
    else if(csr41_wen) begin
        timer_enable <= csr_wrmask[0]&csr_wrdata[0]|~csr_wrmask[0]&timer_en_r;
    end
    else if( csr_timer_int ) begin
        timer_enable <= csr_csr41_timerperiod;
    end

    if(reset) begin
        timer_r <= 32'hffffffff;
    end
    else if(csr41_wen) begin
        timer_r[31:2] <= csr_wrmask[31:2]&csr_wrdata[31:2]|~csr_wrmask[31:2]&timer_val_r;
        timer_r[1 :0] <= 2'd0;
    end
    else if( csr_timer_int ) begin
        timer_r <= !csr_csr41_timerperiod ? 32'hffff_ffff : csr_csr41_timerinit;
    end
    else if(timer_enable) begin
        timer_r <= timer_r - 1'b1;
    end
end
// End Timer For Lisa

//------------------------------------------------------------//
//                     csr42: Timerticks
//------------------------------------------------------------//
assign csr_csr42  =  timer_r;
// End csr42: Timerticks

//------------------------------------------------------------//
//                      csr43: CntOffset
//------------------------------------------------------------//
assign csr_csr43  =  cnt_off_r;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        cnt_off_r <= 32'b0;
    end
    else if(csr43_wen) begin
        cnt_off_r <= csr_wrmask&csr_wrdata|~csr_wrmask&cnt_off_r;
    end
end
// End csr43: CntOffset

//------------------------------------------------------------//
//                       Count For Lisa
//------------------------------------------------------------//

assign count_en  =  ~ctrl_csraf_dc & (~debug_mode | debug_ctrl_dc);

`ifdef OPEN132_COUNT64
assign csr_count_mask     =  lisa_rdtime&&!lisa_rdtime_2nd;
assign count_offset_mask  =  csr_csr43&{32{csr_count_mask}};
assign count_read         =  {count_r&{64{csr_count_mask}}} + {{32{count_offset_mask[31]}},count_offset_mask};
assign csr_count          =  lisa_rdtime_h ? count_read[63:32] : count_read[31:0];
always @ ( posedge clk_count )
begin
    if(reset) begin
        count_r <= 64'd1;
    end
    else if(count_en) begin
        count_r <= count_r + 1'd1;
    end
end
`else
assign csr_count_mask  =  lisa_rdtime&&!lisa_rdtime_h&&!lisa_rdtime_2nd;
assign csr_count       =  (count_r&{32{csr_count_mask}}) + (csr_csr43&{32{csr_count_mask}});
always @ ( posedge clk_count )
begin
    if(reset) begin
        count_r <= 32'd1;
    end
    else if(count_en) begin
        count_r <= count_r + 1'b1;
    end
end
// End Count For Lisa
`endif

//------------------------------------------------------------//
//                         csr60: LLb
//------------------------------------------------------------//
assign csr_csr60            =  {29'd0      ,  // 31:3
                                 keepllbit_r,  // 2, KEEPLLB, clear by ERET(non debug_mode)
                                 1'b0       ,  // 1, WCLLB, write 1 to clear, but read-only 0
                                 llbit  // 0
                               };
assign csr_csr60_wcllbit    =  csr60_wen&&csr_wrmask[1]&&csr_wrdata[1];
assign csr_csr60_keepllbit  =  keepllbit_r;
always @ ( posedge clk_g )
begin
    if(reset) begin
        keepllbit_r <= 1'b0;
    end
    else if(csr60_wen) begin
        keepllbit_r <= csr_wrmask[2]&csr_wrdata[2]|~csr_wrmask[2]&keepllbit_r;
    end
    else if(ertn_write&&!debug_mode) begin
        keepllbit_r <= 1'b0;
    end
end
// End csr60: LLb

//------------------------------------------------------------//
//                      csr80: Gsconfig
//------------------------------------------------------------//
assign csr_csr80            =  {20'd0      ,  // 31:12
                                 user_ccop_r,  // 11,
                                 3'd0       ,  // 10:8
                                 lifeen_r   ,  // 7,
                                 7'd0  // 6:0
                               };
assign csr_csr80_user_ccop  =  user_ccop_r;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        user_ccop_r <= 1'b0;
    end
    else if(csr80_wen) begin
        user_ccop_r <= csr_wrmask[11]&csr_wrdata[11]|~csr_wrmask[11]&user_ccop_r;
    end

    if(reset) begin
        lifeen_r <= 1'b1;
    end
    else if(csr80_wen) begin
        lifeen_r <= csr_wrmask[7]&csr_wrdata[7]|~csr_wrmask[7]&lifeen_r;
    end
end
// End csr80: Gsconfig

//------------------------------------------------------------//
//                      csr90: Errorctrl
//------------------------------------------------------------//
assign csr_csr90  =  {14'd0             ,  // 31:18
                       errcode_r         ,  // 17:16
                       3'd0              ,  // 15:13
                       errpdacm_r        ,  // 12:11
                       errpdacf_r        ,  // 10:9
`ifdef CONFIG_MMU
                       errppg_r          ,  // 8   , previous PG
`else
                       1'b0              ,  // 8   , ppg=0
`endif
                       errpda_r          ,  // 7   ,
                       errpwatchen_r     ,  // 6   ,
                       1'b0              ,  // 5   , pvm=0
                       errpie_r          ,  // 4   ,
                       errpplv_r         ,  // 3:2 ,
                       1'b0              ,  // 1   , returnable=0
                       errmode_r  // 0
                     };

assign csr_csr90_errpdacm  =  errpdacm_r;
assign csr_csr90_errpdacf  =  errpdacf_r;
assign csr_csr90_errpda    =  errpda_r;
`ifdef CONFIG_MMU
assign csr_csr90_errppg    =  errppg_r;
`endif
assign csr_csr90_errpie    =  errpie_r;
assign csr_csr90_errpwe    =  errpwatchen_r;
assign csr_csr90_errpplv   =  errpplv_r;
assign csr_csr90_errmode   =  errmode_r;
always @ ( posedge clk_g )
begin
    // errcode_r
    if(reset) begin
        errcode_r <= 2'd0;
    end
    else if(ex_mc_wen) begin
        errcode_r <= {1'b0, nmi_end};  // 1: NMI; 0: CacheErr
    end
    // errpdacm_r
    if(reset) begin
        errpdacm_r <= 2'd0;
    end
    else if(ex_mc_wen) begin
        errpdacm_r <= csr_csr0_dacm;
    end
    else if(csr90_wen) begin
        errpdacm_r <= csr_wrmask[12:11]&csr_wrdata[12:11]|~csr_wrmask[12:11]&errpdacm_r;
    end
    // errpdacf_r
    if(reset) begin
        errpdacf_r <= 2'd0;
    end
    else if(ex_mc_wen) begin
        errpdacf_r <= csr_csr0_dacf;
    end
    else if(csr90_wen) begin
        errpdacf_r <= csr_wrmask[10:9]&csr_wrdata[10:9]|~csr_wrmask[10:9]&errpdacf_r;
    end
    // errpda_r
    if(reset) begin
        errpda_r <= 1'b0;
    end
    else if(ex_mc_wen) begin
        errpda_r <= csr_csr0_da;
    end
    else if(csr90_wen) begin
        errpda_r <= csr_wrmask[7]&csr_wrdata[7]|~csr_wrmask[7]&errpda_r;
    end
`ifdef CONFIG_MMU
    // errppg_r
    if(reset) begin
        errppg_r <= 1'b0;
    end
    else if(ex_mc_wen) begin
        errppg_r <= pg_r;
    end
    else if(csr90_wen) begin
        errppg_r <= csr_wrmask[8]&csr_wrdata[8]|~csr_wrmask[8]&errppg_r;
    end
`endif
    // errpwatchen_r
    if(reset) begin
        errpwatchen_r <= 1'b0;
    end
    else if(ex_mc_wen) begin
        errpwatchen_r <= csr_csr0_we;
    end
    else if(csr90_wen) begin
        errpwatchen_r <= csr_wrmask[6]&csr_wrdata[6]|~csr_wrmask[6]&errpwatchen_r;
    end
    // errpie_r
    if(reset) begin
        errpie_r <= 1'b0;
    end
    else if(ex_mc_wen) begin
        errpie_r <= csr_csr0_ie;
    end
    else if(csr90_wen) begin
        errpie_r <= csr_wrmask[4]&csr_wrdata[4]|~csr_wrmask[4]&errpie_r;
    end
    // errpplv_r
    if(reset) begin
        errpplv_r <= 2'd0;  // only 0 or 3
    end
    else if(ex_mc_wen) begin
        errpplv_r <= csr_csr0_plv;
    end
    else if(csr90_wen && csr_wrmask[3:2]==2'b11) begin
        errpplv_r <= csr_wrdata[3:2]==2'b11 ? 2'b11 :  // write to 3
        csr_wrdata[3:2] ==2'b00 ? 2'b00 : errpplv_r;  // write to 0
    end
    // errmode_r
    if(reset) begin
        errmode_r <= 1'b0;  // only 0 or 3
    end
    else if(ex_mc_wen) begin
        errmode_r <= 1'b1;
    end
`ifdef CONFIG_MMU
    else if(csr90_wen) begin
        errmode_r <= csr_wrmask[0]&csr_wrdata[0]|~csr_wrmask[0]&errmode_r;
    end
`endif
    else if(ertn_write&&!debug_mode) begin
        errmode_r <= 1'b0;
    end
end
// End csr90: Errorctrl

// csr91: ccerr0, see{Cache}
// csr92: ccerr1, see{Cache}

//------------------------------------------------------------//
//                           csr93
//------------------------------------------------------------//
assign csr_csr93       =  {merrentry_vpn, 12'd0};
assign ctrl_merrentry  =  merrentry_vpn;
assign reset_addr      =  boot_pc;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        merrentry_vpn <= reset_addr[31:12];
    end
    else if(csr93_wen) begin
        merrentry_vpn <= csr_wrmask[31:12]&csr_wrdata[31:12]
                       |~csr_wrmask[31:12]&merrentry_vpn    ;
    end
end
// End csr93

// csr94: merrera,
// csr98: taglo, see{Cache}

// csraf: userdef,
// csr180~18f: mpu,

//------------------------------------------------------------//
//                       csr190: Segpa
//------------------------------------------------------------//
assign csr_csr190         =  csr190_r;
assign csr_csr190_segplv  =  {csr190_r[31   ],csr190_r[27   ],csr190_r[23   ],csr190_r[19   ],
                               csr190_r[15   ],csr190_r[11   ],csr190_r[ 7   ],csr190_r[ 3   ]};
assign csr_csr190_segpa   =  {csr190_r[30:28],csr190_r[26:24],csr190_r[22:20],csr190_r[18:16],
                               csr190_r[14:12],csr190_r[10: 8],csr190_r[ 6: 4],csr190_r[ 2: 0]};
always @ ( posedge clk_cr )
begin
    if(reset) begin
`ifdef CONFIG_MMU
        csr190_r <= 32'h7654ba98;
`else
        csr190_r <= 32'h76543210;
`endif
    end
    else if(csr190_wen) begin
        csr190_r <= csr_wrmask&csr_wrdata | ~csr_wrmask&csr190_r;
    end
end
// End csr190: Segpa

//------------------------------------------------------------//
//                       csr191: Segca
//------------------------------------------------------------//
assign csr_csr191            =  csr191_r;
assign csr_csr191_segcalg    =  {csr191_r[31:28],csr191_r[27:24],csr191_r[23:20],csr191_r[19:16],
                                  csr191_r[15:12],csr191_r[11: 8],csr191_r[ 7: 4],csr191_r[ 3: 0]};
assign csr_csr191_segca_dev  =  {csr191_r[31:28]==4'hf,csr191_r[27:24]==4'hf,
                                  csr191_r[23:20]==4'hf,csr191_r[19:16]==4'hf,
                                  csr191_r[15:12]==4'hf,csr191_r[11: 8]==4'hf,
                                  csr191_r[ 7: 4] ==4'hf,csr191_r[ 3: 0]==4'hf};
always @ ( posedge clk_cr )
begin
    if(reset) begin
        csr191_r <= 32'hffffffff;
    end
    else if(csr191_wen) begin
        csr191_r <= csr_wrmask&csr_wrdata | ~csr_wrmask&csr191_r;
    end
end
// End csr191: Segca

//------------------------------------------------------------//
//                       Csrff: Elusion
//------------------------------------------------------------//
assign csr_csrff        =  {31'd0  ,  // 31:1, 0
                             napen_r  // 0, napen
                           };
assign csr_csrff_napen  =  napen_r;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        napen_r <= 1'b1;
    end
    else if(csrff_wen) begin
        napen_r <= csr_wrmask[0] ? csr_wrdata[0] : napen_r;
    end
end
// End Csrff: Elusion
// End Lisa Csr

//------------------------------------------------------------//
//                      Select From Lisa
//------------------------------------------------------------//

assign user_mode    =  !debug_mode&&lisa_user_mode  ;
assign kernel_mode  =  !debug_mode&&lisa_kernel_mode;

assign ctrl_segcalg  =  csr_csr191_segcalg;

assign ctrl_sleep_cg  =  csr_csrff_napen;
assign ctrl_null_cg   =  csr_csrff_napen;
assign ctrl_ie        =  csr_csr0_ie;
assign ctrl_im        =  csr_csr4_im;
assign ctrl_ip        =  csr_csr5_ip;

assign ctrl_csraf_dc  =  csr_csraf_dc;
assign ctrl_lisa_vs   =  csr_csr4_vs;

assign ctrl_llbit_keep  =  csr_csr60_keepllbit;
assign ctrl_llbit_wclr  =  csr_csr60_wcllbit;

assign ctrl_wrdata  =  csr_wrdata;
assign ctrl_wrmask  =  csr_wrmask;

//------------------------------------------------------------//
//                         csr6: Era
//------------------------------------------------------------//
assign csr_csr6  =  era_r;
always @ ( posedge clk_g )
begin
    if(data_reset) begin
        era_r <= 32'd0;
    end
    else if(ex_norm_wen) begin
        era_r <= era_in;  // brpc or pc or wakeup_pc(following wait)
    end
    else if(csr6_wen) begin
        era_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&era_r;
    end
end
// End csr6: Era

//------------------------------------------------------------//
//                         csr7: Badv
//------------------------------------------------------------//
assign csr_csr7  =  badv_r;
always @ ( posedge clk_g )
begin
    if(reset) begin
        badv_r <= 32'd0;
    end
    else if(badv_wen) begin
        badv_r <= badv_wrdata;
    end
    else if(csr7_wen) begin
        badv_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&badv_r;
    end
end
// End csr7: Badv

//------------------------------------------------------------//
//                        Csrc: Eentry
//------------------------------------------------------------//
assign csr_csrc     =  {eentry_vpn, 12'd0};
assign ctrl_eentry  =  eentry_vpn;
always @ ( posedge clk_cr )
begin
    if(reset) begin
        eentry_vpn <= reset_addr[31:12];
    end
    else if(csrc_wen) begin
        eentry_vpn <= ctrl_wrmask[31:12]&ctrl_wrdata[31:12] |
        ~ctrl_wrmask[31:12]&eentry_vpn         ;
    end
end
// End csr7: Eentry

//------------------------------------------------------------//
//                        csr18: Asid
//------------------------------------------------------------//
assign csr_csr18  =  {8'd0  ,  // 31:24
`ifdef CONFIG_MMU
                       8'd10 ,  // 23:16, implemented ASID width
                       6'd0  ,  // 15:10
                       asid_r  // 9 :0
`else
                       8'd8  ,  // 23:16, ASID bits, 8 for OPEN132E
                       8'd0  ,  // 15:8
                       asid_r  // 7 :0
`endif
                     };
assign ctrl_asid  =  asid_r;
always @ ( posedge clk_cr )
begin
    if(data_reset) begin
`ifdef CONFIG_MMU
        asid_r <= 10'd0;
`else
        asid_r <= 8'd0;
`endif
    end
    else if(csr18_wen) begin
`ifdef CONFIG_MMU
        asid_r  <= ctrl_wrmask[9:0]&ctrl_wrdata[9:0]
                 |~ctrl_wrmask[9:0]&asid_r;
`else
        asid_r  <= ctrl_wrmask[7:0]&ctrl_wrdata[7:0]
                 |~ctrl_wrmask[7:0]&asid_r;
`endif
    end
end
// End csr18: Asid
`ifdef CONFIG_MMU

//------------------------------------------------------------//
//                   csr19-csr1d: Page Walker
//------------------------------------------------------------//

assign csr_csr19  =  pgdl_r;
assign csr_csr1a  =  pgdh_r;
assign csr_csr1b  =  badv_r[31] ? pgdh_r : pgdl_r;
assign csr_csr1c  =  pwctl0_r;
assign csr_csr1d  =  pwctl1_r;

always @ ( posedge clk_cr )
begin
    if(data_reset) begin
        pgdl_r   <= 32'd0;
        pgdh_r   <= 32'd0;
        pwctl0_r <= 32'd0;
        pwctl1_r <= 32'd0;
    end
    else begin
        if(csr19_wen) begin
            pgdl_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&pgdl_r;
        end
        if(csr1a_wen) begin
            pgdh_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&pgdh_r;
        end
        if(csr1c_wen) begin
            pwctl0_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&pwctl0_r;
        end
        if(csr1d_wen) begin
            pwctl1_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&pwctl1_r;
        end
    end
end
// End csr19-csr1d: Page Walker

//------------------------------------------------------------//
//                   csr180-csr181: DMW0/DMW1
//------------------------------------------------------------//

// DMW state is kept independently from the legacy MPU shadow so that the
// architectural VSEG, PSEG, MAT, and PLV fields retain their written values.
// The existing MPU instances still observe the same writes for direct-mode
// compatibility through csr_cache_ctrl_bus.
always @ ( posedge clk_cr )
begin
    if(data_reset) begin
        dmw0_r <= 32'd0;
        dmw1_r <= 32'd0;
    end
    else begin
        if(csr180_wen) begin
            dmw0_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&dmw0_r;
        end
        if(csr181_wen) begin
            dmw1_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&dmw1_r;
        end
    end
end
// End csr180-csr181: DMW0/DMW1
`endif

//------------------------------------------------------------//
//                        csr30: save0
//------------------------------------------------------------//
assign csr_csr30  =  save0_r;
always @ ( posedge clk_cr )
begin
    if(data_reset) begin
        save0_r <= 32'd0;
    end
    else if(csr30_wen) begin
        save0_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&save0_r;
    end
end
// End csr30: save0

//------------------------------------------------------------//
//                        csr31: save1
//------------------------------------------------------------//
assign csr_csr31  =  save1_r;
always @ ( posedge clk_cr )
begin
    if(data_reset) begin
        save1_r <= 32'd0;
    end
    else if(csr31_wen) begin
        save1_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&save1_r;
    end
end
// End csr31: save1

//------------------------------------------------------------//
//                       csr94: Merrera
//------------------------------------------------------------//
assign merrera_in  =  era_in;

assign csr_csr94  =  merrera_r;
always @ ( posedge clk_g )
begin
    if(data_reset) begin
        merrera_r <= 32'd0;
    end
    else if(!sync_sresetn || nmi_end || ccerr_wr_csr) begin
        merrera_r <= merrera_in;
    end
    else if(csr94_wen) begin
        merrera_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&ctrl_wrdata;
    end
end
// End csr94: Merrera

//------------------------------------------------------------//
//                       Csraf: Userdef
//------------------------------------------------------------//
assign csr_csraf     =  {10'd0             ,  // 31:22
                          1'd0              ,  // 21
                          userdef_enable_pred,  // 20
                          3'd0,  // 19:17
                          userdef_supexc    ,  // 16
                          5'd0              ,  // 15 :11
                          userdef_nowrap    ,  // 10
                          real_userdef_dccbp,  // 9
                          real_userdef_iccbp,  // 8
                          3'd0              ,  // 7 :5
                          userdef_dc        ,  // 4
                          2'd0              ,  // 3 :2
                          1'd0              ,  // 1,
                          1'b0  // 0,
                        };
assign csr_csraf_dc  =  userdef_dc;
always @ ( posedge clk_g )
begin
    if(reset) begin
        userdef_supexc <= 1'b0;
    end
    else if(mem_is_ll_valid&&mem_load_wait_back&&!lddata_err) begin
        userdef_supexc <= lddata_exok;
    end
end

always @ ( posedge clk_cr )
begin
    if(reset) begin
        userdef_nowrap <= 1'b1;
    end
    else if(csraf_wen) begin
        userdef_nowrap <= ctrl_wrmask[10]? ctrl_wrdata[10] : userdef_nowrap;
    end

    if(reset) begin
        userdef_dccbp <= 1'b0;
        userdef_iccbp <= 1'b0;
    end
    else if(csraf_wen) begin
        userdef_dccbp <= ctrl_wrmask[9]? ctrl_wrdata[9] : userdef_dccbp;
        userdef_iccbp <= ctrl_wrmask[8]? ctrl_wrdata[8] : userdef_iccbp;
    end
    // only lisa: dc
    if(reset) begin
`ifdef CONFIG_MMU
        userdef_dc <= 1'b0;
`else
        userdef_dc <= 1'b1;
`endif
    end
    else if(csraf_wen) begin
        userdef_dc <= ctrl_wrmask[4]? ctrl_wrdata[4]: userdef_dc;
    end
    // disable branch prediction
    if(reset) begin
        userdef_enable_pred <= 1'b0;
    end
    else if(csraf_wen&&HAS_BTB) begin
        userdef_enable_pred <= ctrl_wrmask[20]? ctrl_wrdata[20]: userdef_enable_pred;
    end

end
assign ctrl_userdef_supexc  =  userdef_supexc;
assign icc_bypass           =  real_userdef_iccbp;
assign dcc_bypass           =  real_userdef_dccbp;
assign userdef_clr_pred     =  csraf_wen&&csr_wrmask[21]&&csr_wrdata[21];
assign csr_bpu_ctrl_bus     =  {userdef_clr_pred,userdef_enable_pred};

// axi no wrap.
assign ctrl_nowrap  =  userdef_nowrap;
// End Csraf: Userdef

//------------------------------------------------------------//
//                    csr180~18f: csrr_mpu
//------------------------------------------------------------//
mmu_mpu #(
    .TOTAL_NUM (TOTAL_NUM_MPU_REGION0),  // MPU_NUM
    .THIS_NUM  (THIS_NUM_MPU_REGION0 )   // 1st mpu
) x_mmu_mpu_0 (
    .clk              (clk_mpu_region0             ),
    .reset            (reset_mpu_region0           ),
    .data_reset       (data_reset_mpu_region0      ),
    .lisa_kernel_mode (lisa_kernel_mode_mpu_region0),
    .mpuhi_wen        (mpuhi_wen_mpu_region0       ),
    .mpulo_wen        (mpulo_wen_mpu_region0       ),
    .ctrl_wrdata      (ctrl_wrdata_mpu_region0     ),
    .ctrl_wrmask      (ctrl_wrmask_mpu_region0     ),
    .csr_mpuhi        (csr_mpuhi_mpu_region0       ),
    .csr_mpulo        (csr_mpulo_mpu_region0       ),
    .ctrl_mpuhi_valid (ctrl_mpuhi_valid_mpu_region0),
    .ctrl_mpuhi_base  (ctrl_mpuhi_base_mpu_region0 ),
    .ctrl_mpulo_mask  (ctrl_mpulo_mask_mpu_region0 ),
    .ctrl_mpulo_calg  (ctrl_mpulo_calg_mpu_region0 )
);
mmu_mpu #(
    .TOTAL_NUM (TOTAL_NUM_MPU_REGION1),  // MPU_NUM
    .THIS_NUM  (THIS_NUM_MPU_REGION1 )   // 2nd mpu
) x_mmu_mpu_1 (
    .clk              (clk_mpu_region1             ),
    .reset            (reset_mpu_region1           ),
    .data_reset       (data_reset_mpu_region1      ),
    .lisa_kernel_mode (lisa_kernel_mode_mpu_region1),
    .mpuhi_wen        (mpuhi_wen_mpu_region1       ),
    .mpulo_wen        (mpulo_wen_mpu_region1       ),
    .ctrl_wrdata      (ctrl_wrdata_mpu_region1     ),
    .ctrl_wrmask      (ctrl_wrmask_mpu_region1     ),
    .csr_mpuhi        (csr_mpuhi_mpu_region1       ),
    .csr_mpulo        (csr_mpulo_mpu_region1       ),
    .ctrl_mpuhi_valid (ctrl_mpuhi_valid_mpu_region1),
    .ctrl_mpuhi_base  (ctrl_mpuhi_base_mpu_region1 ),
    .ctrl_mpulo_mask  (ctrl_mpulo_mask_mpu_region1 ),
    .ctrl_mpulo_calg  (ctrl_mpulo_calg_mpu_region1 )
);
mmu_mpu #(
    .TOTAL_NUM (TOTAL_NUM_MPU_REGION2),  // MPU_NUM
    .THIS_NUM  (THIS_NUM_MPU_REGION2 )   // 3th mpu
) x_mmu_mpu_2 (
    .clk              (clk_mpu_region2             ),
    .reset            (reset_mpu_region2           ),
    .data_reset       (data_reset_mpu_region2      ),
    .lisa_kernel_mode (lisa_kernel_mode_mpu_region2),
    .mpuhi_wen        (mpuhi_wen_mpu_region2       ),
    .mpulo_wen        (mpulo_wen_mpu_region2       ),
    .ctrl_wrdata      (ctrl_wrdata_mpu_region2     ),
    .ctrl_wrmask      (ctrl_wrmask_mpu_region2     ),
    .csr_mpuhi        (csr_mpuhi_mpu_region2       ),
    .csr_mpulo        (csr_mpulo_mpu_region2       ),
    .ctrl_mpuhi_valid (ctrl_mpuhi_valid_mpu_region2),
    .ctrl_mpuhi_base  (ctrl_mpuhi_base_mpu_region2 ),
    .ctrl_mpulo_mask  (ctrl_mpulo_mask_mpu_region2 ),
    .ctrl_mpulo_calg  (ctrl_mpulo_calg_mpu_region2 )
);
mmu_mpu #(
    .TOTAL_NUM (TOTAL_NUM_MPU_REGION3),  // MPU_NUM
    .THIS_NUM  (THIS_NUM_MPU_REGION3 )   // 4th mpu
) x_mmu_mpu_3 (
    .clk              (clk_mpu_region3             ),
    .reset            (reset_mpu_region3           ),
    .data_reset       (data_reset_mpu_region3      ),
    .lisa_kernel_mode (lisa_kernel_mode_mpu_region3),
    .mpuhi_wen        (mpuhi_wen_mpu_region3       ),
    .mpulo_wen        (mpulo_wen_mpu_region3       ),
    .ctrl_wrdata      (ctrl_wrdata_mpu_region3     ),
    .ctrl_wrmask      (ctrl_wrmask_mpu_region3     ),
    .csr_mpuhi        (csr_mpuhi_mpu_region3       ),
    .csr_mpulo        (csr_mpulo_mpu_region3       ),
    .ctrl_mpuhi_valid (ctrl_mpuhi_valid_mpu_region3),
    .ctrl_mpuhi_base  (ctrl_mpuhi_base_mpu_region3 ),
    .ctrl_mpulo_mask  (ctrl_mpulo_mask_mpu_region3 ),
    .ctrl_mpulo_calg  (ctrl_mpulo_calg_mpu_region3 )
);
// End csr180~18f: csrr_mpu
// End Select From Lisa

//                 csr_cache_ctrl_bus Signals
//------------------------------------------------------------//
assign csr_cache_cfgseg_calg  =  ctrl_segcalg;
assign csr_cache_lisa_da      =  csr_csr0_da;
assign csr_cache_lisa_dacf    =  csr_csr0_dacf;
assign csr_cache_lisa_dacm    =  csr_csr0_dacm;
assign csr_cache_lisa_segpa   =  csr_csr190_segpa;
// mpu0~3
assign csr_cache_mpu_valid   =  ctrl_mpuhi_valid;
`GEN_2D_TO_1D(4, 20, ctrl_mpuhi_base, csr_cache_mpu_base)
`GEN_2D_TO_1D(4, 20, ctrl_mpulo_mask, csr_cache_mpu_mask)
`GEN_2D_TO_1D(4,  4, ctrl_mpulo_calg, csr_cache_mpu_calg)
assign csr_cache_dcc_bypass  =  dcc_bypass;
assign csr_cache_icc_bypass  =  icc_bypass;
// End csr_cache_ctrl_bus Signals

//------------------------------------------------------------//
//                  csr_ifu_ctrl_bus Signals
//------------------------------------------------------------//
assign csr_ifu_lisa_segplv  =  csr_csr190_segplv;
assign csr_ifu_lisa_da      =  csr_csr0_da;
assign csr_ifu_user_mode    =  user_mode;
// End csr_ifu_ctrl_bus Signals

//------------------------------------------------------------//
//                  csr_idu_ctrl_bus Signals
//------------------------------------------------------------//
assign csr_idu_lisa_da         =  csr_csr0_da          ;  // lisa
assign csr_idu_lisa_ihb_skip   =  csr_csr381_ihb_skip  ;  // lisa
assign csr_idu_lisa_dacm       =  csr_csr0_dacm        ;  // lisa
assign csr_idu_lisa_segplv     =  csr_csr190_segplv    ;  // lisa
assign csr_idu_lisa_segca_dev  =  csr_csr191_segca_dev ;  // lisa
assign csr_idu_user_ccop       =  csr_csr80_user_ccop  ;  // lisa
assign csr_idu_rdtime_dis      =  csr_csr3_rdtime_disp3;  // lisa
assign csr_idu_fpen            =  csr_csr2_fpen        ;  // lisa
assign csr_idu_user_mode       =  user_mode;

// End csr_idu_ctrl_bus Signals

//                          Sleeping
//------------------------------------------------------------//
always @ ( posedge clk_ng )
begin
    if(reset || mark_nmi_ccerr_int || debug_ex) begin
        sleeping <= 1'b0;
    end
    else if(inst_idle && valid_go_wb && !inst_oth_ex_pesudoint && ifc_storeq_null && !mdu_is_run) begin
        sleeping <= 1'b1;
    end
end
// End Sleeping

//------------------------------------------------------------//
//                           LJTAG
//------------------------------------------------------------//
`ifdef OPEN132_HAS_LJTAG
assign ex_dbgss_ok  =  valid && ex_dbgss;

assign ex_ihb_ok_no_skip  =  valid && !mark_nmi_ccerr_int && ihb_signal;
assign ex_ihb_ok          =  ex_ihb_ok_no_skip && !csr_csr381_ihb_skip;
assign ex_watch_inst      =  ex_ihb_ok && !ihb_debug;

assign ex_dbcl_ok  =  valid && !debug_mode && !mark_nmi_ccerr_int && ex_dbcl;
assign ex_dint_ok  =  valid && !debug_mode && jtag_core_break;

assign debug_stall_nmi  =  debug_mode || ex_dbgss_ok || ex_dint_ok;
// if in debug mode, can't set debug_ex.
assign debug_ex      =  ex_dbgss_ok || ex_dint_ok ||
                          ex_ihb_ok&&ihb_debug || ex_dbcl_ok ;
assign debug_ex_wen  =  debug_ex && rtu_lsu_allowin && !ex_cancel;

assign debug_mode_ex_wen  =  debug_mode_ex && rtu_lsu_allowin && !ex_cancel;

assign ihb_mask   =  ex_dbgss_ok||ex_dint_ok;
assign dbc_mask   =  ihb_mask ||ex_ihb_ok;
assign dbgss_set  =  ex_dbgss_ok;
assign dint_set   =  !ex_dbgss_ok&& ex_dint_ok ;
assign ihb_set    =  !ihb_mask && ex_ihb_ok  ;
assign dbc_set    =  !dbc_mask && ex_dbcl_ok;

assign dertn_write  =  ertn_write&&debug_mode && inst_oth_noex;

//------------------------------------------------------------//
//                         Csrr Debug
//------------------------------------------------------------//
assign debug_cacheep  =  find_ccfatal;

//------------------------------------------------------------//
//                        csr381: FWPS
//------------------------------------------------------------//
// only mem_stage only maintain ihb_skip
assign csr_csr381  =  {15'd0,  // 31:17
                        ihb_skip,  // 16
                        12'd0,  // 15:4
                        4'b0  // 3:0
                      };

always @ ( posedge clk_g )
begin
    if(reset||!lifeen_r) begin
        ihb_skip <= 1'b0;
    end
    else if(csr381_wen) begin
        ihb_skip <= csr_wrmask[16]&csr_wrdata[16]|~csr_wrmask[16]&ihb_skip;
    end
    else if(ex_ihb_ok_no_skip&&valid_go_wb&&(!inst_split_1st&&!see_int_flush&&mem_valid_go||mem_ex)) begin
        ihb_skip <= 1'b0;
    end
    else if(idu_lsu_fpu_clear_skip) begin
        ihb_skip <= 1'b0;
    end
end

assign csr_csr381_ihb_skip  =  ihb_skip;
assign csr_csrihb           =  idu_jtag_ihb_resp_bus;

assign csr_csr500        =  {10'd0                ,  // 31:22
                              debug_destat         ,  // 21:16
                              3'd0                 ,  // 15:13
                              debug_ssflag         ,  // 12
                              1'b0                 ,  // 11
                              debug_ihb            ,  // 10
                              debug_dbc            ,  // 9
                              debug_dint           ,  // 8
                              7'd0                 ,  // 7:1
                              debug_dm  // 0
                            };
assign csr_csr508        =  {6'd0         ,  // 31:26
                              debug_slp    ,  // 25
                              debug_clkn   ,  // 24
                              7'd0         ,  // 23:17
                              debug_cacheep,  // 16
                              9'd0         ,  // 15:7
                              debug_nocce  ,  // 6    , RW
                              debug_dc     ,  // 5    , RW
                              4'd0         ,  // 4:1
                              debug_sse  // 0    , RW
                            };
assign debug_ctrl_dc     =  debug_dc;
assign debug_ctrl_nocce  =  debug_nocce;
always @ ( posedge clk_g )
begin
    debug_dm       <= reset             ? 1'b0           :
    debug_ex_wen      ? 1'b1           :
    dertn_write       ? 1'b0           : debug_dm;

    debug_slp      <= reset             ? 1'b0           :
    debug_ex_wen      ? sleeping       : debug_slp;

    debug_clkn     <= reset             ? 1'b0           :
    debug_ex_wen      ? !clk_slp_en    : debug_clkn;

    debug_destat   <= reset             ? 5'd0           :
`ifdef CONFIG_MMU
    debug_mode_ex_wen ? mem_statcode[4:0] : debug_destat;
`else
    debug_mode_ex_wen ? mem_statcode   : debug_destat;
`endif

    debug_dint     <= reset             ? 1'b0           :
    debug_mode_ex_wen ? 1'b0           :
    debug_ex_wen      ? dint_set       : debug_dint;

    debug_ihb      <= reset             ? 1'b0           :
    debug_mode_ex_wen ? 1'b0           :
    debug_ex_wen      ? ihb_set        : debug_ihb;

    debug_dbc      <= reset             ? 1'b0           :
    debug_mode_ex_wen ? 1'b0           :
    debug_ex_wen      ? dbc_set        : debug_dbc;
    debug_ssflag   <= reset             ? 1'b0           :
    debug_mode_ex_wen ? 1'b0           :
    debug_ex_wen      ? dbgss_set        : debug_ssflag;
end

always @ ( posedge clk_g )
begin
    debug_dc       <= reset                      ? 1'b0          :
    csr508_wen&&csr_wrmask[5]  ? csr_wrdata[5 ]:
    debug_dc      ;

    debug_nocce    <= reset                               ? 1'b0          :
    debug_ex_wen||debug_mode_ex_wen     ? 1'b1          :
    dertn_write                         ? 1'b0          :
    /*lisa debug2*/   csr508_wen&&csr_wrmask[6]           ? csr_wrdata[6 ]:
    debug_nocce   ;

    debug_sse      <= reset                               ? 1'b0          :
    /*lisa debug2*/   csr508_wen&&csr_wrmask[0]           ? csr_wrdata[0] :
    debug_sse     ;
end

// End Csrr Debug

//------------------------------------------------------------//
//                         Csrr Dera
//------------------------------------------------------------//
assign csr_csr501  =  dera_r;
always @ ( posedge clk_g )
begin
    if(data_reset) begin
        dera_r <= 32'd0;
    end
    else if(debug_ex_wen||debug_mode_ex_wen) begin
        dera_r <= era_in;
    end
    else if(csr501_wen) begin
        dera_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&dera_r;
    end
end
// End Csrr Dera

//------------------------------------------------------------//
//                         Csrr Dsave
//------------------------------------------------------------//
assign csr_csr502  =  dsave_r;
always @ ( posedge clk_cr )
begin
    if(data_reset) begin
        dsave_r <= 32'd0;
    end
    else if(csr502_wen) begin
        dsave_r <= ctrl_wrmask&ctrl_wrdata | ~ctrl_wrmask&dsave_r;
    end
end
// End Csrr Dsave

//------------------------------------------------------------//
//                Input And Output Connections
//------------------------------------------------------------//
assign lsu_clk_breakpoint_enable  =  debug_dm || lisa_csr_write&&valid;
assign lsu_jtag_debug_mode        =  debug_dm ;
assign lsu_ifu_debug_map_valid    =  debug_dm ;
`ifdef CONFIG_MMU
assign asid                       =  ctrl_asid[7:0];
`else
assign asid                       =  ctrl_asid;
`endif
assign pesudoint_go               =  valid_go_wb &&!mem_ex  && mem_pesudoint;
assign mem_ex_go                  =  rtu_lsu_allowin  &&!ex_cancel  && mem_ex  ;
assign dertn_go                   =  dertn_write;
assign mem_noex                   =  !(mem_ex_pesudoint || mem_nmi_ccerr);
assign lsu_idu_debug_bus          =  {pesudoint_go,
                                       mem_ex_go   ,
                                       dertn_go    ,
                                       debug_sse   ,
                                       debug_dm    ,
                                       mem_noex    ,
`ifdef CONFIG_MMU
                                       asid
`else
                                       ctrl_asid
`endif
                                     };

assign csrihb_ren            =  ihb_csr_req_valid;
assign jtag_idu_ihb_req_bus  =  {plv_r,
                                  csr_csr0_we,
                                  ihb_csr_off,
                                  ihb_csr_req_valid,
                                  ihb_csr_req_wr,
                                  ihb_csr_req_wdata,
                                  ihb_csr_wrmask
                                };
assign ihb_csr_off           =  csr_num[11:0];
assign ihb_csr_req_wr        =  csr_wen;
assign ihb_csr_req_valid     =  lisa_csrrd && (csr_num[13:7]==7'h7);
assign ihb_csr_req_wdata     =  csr_wrdata;
assign ihb_csr_wrmask        =  csr_wrmask;

// End Input And Output Connections

//------------------------------------------------------------//
//                           Others
//------------------------------------------------------------//
assign debug_mode    =  debug_dm;
assign dertn_result  =  dera_r;
// End Others
`else
assign debug_mode           =  1'b0;
assign debug_stall_nmi      =  1'b0;
assign debug_ctrl_nocce     =  1'b0;
assign debug_ctrl_dc        =  1'b0;
assign debug_ex             =  1'b0;
assign csr_csr500           =  32'd0;
assign csr_csr501           =  32'd0;
assign csr_csr502           =  32'd0;
assign csr_csr508           =  32'd0;
assign dertn_result         =  32'd0;
assign csr_csrihb           =  32'b0;
assign csr_csr381_ihb_skip  =  1'b0;
assign ex_watch_inst        =  1'b0;
`endif
// End LJTAG

//------------------------------------------------------------//
//                           CACHE
//------------------------------------------------------------//
//                            HAS
//------------------------------------------------------------//
//                     Cache 2 Bits Fatal
//------------------------------------------------------------//
assign  {dcc_fatal      ,  // 1, total 52
         icc_fatal      ,  // 1
         fatal_rd       ,  // 1, total 50
         impr_fatal_wr  ,  // 1
         impr_fatal_ccop,  // 1
         impr_fatal_rf  ,  // 1
         impr_fatal_nc  ,  // 1
         fatal_intagv   ,  // 1, total 45
         fatal_indata   ,  // 8
         fatal_way      ,  // 4, total 36
         fatal_indx     ,  // 9, total 32
         fatal_dirtywb  ,  // 1
         fatal_dirty_tag  // 22
       }  =  cache_lsu_ecc_fatal_bus;
assign ccfatal_in  =  dcc_fatal || icc_fatal;
always @ ( posedge clk_g )
begin
    if(reset) begin
        wait_fatal_mark <= 1'b0;
    end
    else if(ccfatal_in) begin  // debug.cacheep
        wait_fatal_mark <= 1'b1;
    end
    else if(nmi_end || mark_ccerr) begin  // fatal_in is prior to ex_cancel
        wait_fatal_mark <= 1'b0;
    end
end
// cache err
assign dcc_fatal_in  =  dcc_fatal;
assign find_ccfatal  =  wait_fatal_mark;
// End Cache 2 Bits Fatal

//------------------------------------------------------------//
//                   Cache Err Csr Csr Reg
//------------------------------------------------------------//
//                ccerr0, ccerr1, Ccsf, Taglo
//------------------------------------------------------------//
//                       csr91: ccerr0
//------------------------------------------------------------//
assign true_ccerr0_wen  =  csr91_wen;

assign csr_csr91  =  {ccerr0_impr_ccop,  // 31, cache err is in cache op, imprecise
                       ccerr0_impr_rf  ,  // 30, cache err is in refill  , imprecise
                       1'b0            ,  // 29, 0
                       ccerr0_impr_wr  ,  // 28, cache err is in store, imprecise
                       ccerr0_rd       ,  // 27, cache err is load or fetch
                       ccerr0_intagv   ,  // 26, tagv ram cache err
                       ccerr0_way      ,  // 25:22, cache err way
                       8'd0            ,  // 21:14, 0
                       ccerr0_indx     ,  // 13: 5, cache err index
                       1'b0            ,  // 4 , 0
                       ccerr0_inerm    ,  // 3 , cache err is found in errormode
                       ccerr0_dcc      ,  // 2 , 2 bit ecc error in Dcache
                       ccerr0_icc      ,  // 1 , 2 bit ecc error in Icache
                       ccerr0_pending  // 0 , 2 bit ecc error pending because NMI
                     };
always @ ( posedge clk_g )
begin
    // ccerr_pending
    if(reset) begin
        ccerr0_pending <= 1'b0;
    end
    else if(nmi_end && find_ccfatal) begin
        ccerr0_pending <= 1'b1;
    end
    else if(true_ccerr0_wen) begin
        ccerr0_pending <= ctrl_wrmask[0]&&!ctrl_wrdata[0] ? 1'b0 : ccerr0_pending;
    end
    // ccerr_inerm
    if(reset) begin
        ccerr0_inerm <= 1'b0;
    end
    else if(mark_ccerr) begin
        ccerr0_inerm <= csr_csr90_errmode;
    end

    if(reset) begin
        ccerr0_impr_ccop <= 1'b0;
        ccerr0_impr_rf   <= 1'b0;
        ccerr0_intagv    <= 1'b0;
        ccerr0_way       <= 4'd0;
        ccerr0_indx      <= 9'd0;
        ccerr0_impr_wr   <= 1'b0;
        ccerr0_rd        <= 1'b0;
        ccerr0_dcc       <= 1'b0;
        ccerr0_icc       <= 1'b0;
    end
    else if( ccfatal_in&&!(wait_fatal_mark&&ccerr0_dcc) ) begin
        ccerr0_impr_ccop <= impr_fatal_ccop;
        ccerr0_impr_rf   <= impr_fatal_rf  ;
        ccerr0_intagv    <= fatal_intagv   ;
        ccerr0_way       <= fatal_way      ;
        ccerr0_indx      <= fatal_indx     ;
        ccerr0_impr_wr   <= impr_fatal_wr  ;
        ccerr0_rd        <= fatal_rd       ;
        ccerr0_dcc       <= dcc_fatal      ;
        ccerr0_icc       <= icc_fatal      ;
    end
    else if(true_ccerr0_wen) begin
        ccerr0_dcc <= ctrl_wrmask[2]&&!ctrl_wrdata[2] ? 1'b0 : ccerr0_dcc;
        ccerr0_icc <= ctrl_wrmask[1]&&!ctrl_wrdata[1] ? 1'b0 : ccerr0_icc;
    end
end
// End csr91: ccerr0

//------------------------------------------------------------//
//                       csr92: ccerr1
//------------------------------------------------------------//
assign csr_csr92  =  { ccerr1_dirty_tag,  // 31:10, writebacked tag
                       1'b0            ,  // 9    , 0
                       ccerr1_dirtywb  ,  // 8    , dcache error has been writebacked
                       ccerr1_indata  // 7 : 0, data(8 bank) ram cache err
                     };
always @ ( posedge clk_g )
begin
    if(reset) begin
        ccerr1_dirty_tag <= 22'd0;
        ccerr1_dirtywb   <= 1'b0;
        ccerr1_indata    <= 8'd0;
    end
    else if(ccfatal_in&&!(wait_fatal_mark&&ccerr0_dcc) ) begin
        ccerr1_dirty_tag <= fatal_dirty_tag;
        ccerr1_dirtywb   <= fatal_dirtywb;
        ccerr1_indata    <= fatal_indata   ;
    end
end
// End csr92: ccerr1

//------------------------------------------------------------//
//                      csr96: Cache Sf
//------------------------------------------------------------//
assign csr_csr96  =  32'd0;
// End csr96: Cache Sf

//------------------------------------------------------------//
//                        csr98: Taglo
//------------------------------------------------------------//
assign true_taglo_wen  =  csr98_wen;
assign csr_csr98       =  {taglo_pa,  // 31:10
                            2'd0,  // 9:8
                            taglo_v,  // 7
                            taglo_d,  // 6
                            taglo_l,  // 5
                            5'd0  // 4:0
                          };
assign ctrl_taglo_pa   =  taglo_pa;
assign ctrl_taglo_v    =  taglo_v;
assign ctrl_taglo_d    =  taglo_d;
assign ctrl_taglo_l    =  taglo_l;
always @ ( posedge clk_g )
begin
    if(data_reset) begin
        taglo_pa <= 22'd0;
        taglo_v  <= 1'b0;
        taglo_d  <= 1'b0;
        taglo_l  <= 1'b0;
    end
    else if(true_taglo_wen) begin
        taglo_pa <= ctrl_wrmask[31:10]&ctrl_wrdata[31:10]
                  |~ctrl_wrmask[31:10]&taglo_pa         ;
        taglo_v <= ctrl_wrmask[7]? ctrl_wrdata[7] : taglo_v;
        taglo_d <= ctrl_wrmask[6]? ctrl_wrdata[6] : taglo_d;
        taglo_l <= ctrl_wrmask[5]? ctrl_wrdata[5] : taglo_l;
    end
    else if(taglo_cc_wen) begin  // inst_cache result
        taglo_pa <= mem_cps_data_rdata[31:10];
        taglo_v  <= mem_cps_data_rdata[7];
        taglo_d  <= mem_cps_data_rdata[6];
        taglo_l  <= mem_cps_data_rdata[5];
    end
end
// End csr98: Taglo
// End Cache Err Csr Csr Reg

generate
    if (HAS_ICC) begin: gen_has_icc
        assign real_userdef_iccbp = userdef_iccbp;
    end else begin: gen_no_icc
        assign real_userdef_iccbp = 1'b1;
    end
endgenerate

generate
    if (HAS_DCC) begin: gen_has_dcc
        assign real_userdef_dccbp = userdef_dccbp;
    end else begin: gen_no_dcc
        assign real_userdef_dccbp = 1'b1;
    end
endgenerate
// End CACHE

endmodule  // csr_regs
