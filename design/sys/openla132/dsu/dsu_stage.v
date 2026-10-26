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

module dsu_stage #(
    parameter HAS_BTB = -1,  // 0 or 1
    parameter IHB_NUM = -1   // 0(no ihb), 1~15
) (
    input  wire                                          clk_g,
    input  wire                                          clk_gpr,
    input  wire                                          sync_hresetn,
    input  wire                                          reset,
    input  wire                                          data_reset,
    input  wire                                          debug_int,

    input  wire                                          ifu_idu_inst_valid,
    input  wire [   `OPEN132_IFU_IDU_INST_BUS_WIDTH-1: 0]  ifu_idu_inst_bus,
    input  wire [                                31: 0]  ifu_idu_seq_pc,  // de_pc+8 for link branch

    input  wire                                          exu_idu_allowin,
    input  wire                                          lsu_idu_allowin,
    input  wire                                          lsu_idu_forward_valid,
    input  wire [`OPEN132_LSU_IDU_FORWARD_BUS_WIDTH-1: 0]  lsu_idu_forward_bus,
    input  wire [   `OPEN132_CSR_IDU_CTRL_BUS_WIDTH-1: 0]  csr_idu_ctrl_bus,
    input  wire                                          rtu_idu_forward_valid,
    input  wire [`OPEN132_RTU_IDU_FORWARD_BUS_WIDTH-1: 0]  rtu_idu_forward_bus,
    input  wire                                          ex_cancel,

    output wire                                          idu_ifu_allowin,
    output wire [                                35: 0]  idu_ifu_branch_bus,
    output wire                                          idu_lsu_inst_valid,
    output wire [   `OPEN132_IDU_LSU_INST_BUS_WIDTH-1: 0]  idu_lsu_inst_bus,
    output wire [                                31: 0]  idu_lsu_follow_pc,
    output wire [                                33: 0]  idu_lsu_load_attr_bus,  // memory attribute load
    output wire [                                34: 0]  idu_lsu_store_data_bus,  // store/ldl/ldr r1 data
    output wire [   `OPEN132_IDU_CSR_PERF_BUS_WIDTH-1: 0]  idu_csr_perf_bus,
    output wire [    `OPEN132_IDU_EXU_MDU_BUS_WIDTH-1: 0]  idu_exu_mdu_bus,  // for mdu,there is only decode stage

    output wire [    `OPEN132_IDU_LSU_APP_BUS_WIDTH-1: 0]  idu_lsu_app_bus

    ,
    input  wire [   `OPEN132_IFU_IDU_PRED_BUS_WIDTH-1: 0]  ifu_idu_pred_bus,
    output wire [ `OPEN132_IDU_BPU_UPDATE_BUS_WIDTH-1: 0]  idu_bpu_update_bus,
    output wire [                                32: 0]  idu_ifu_return_bus

`ifdef OPEN132_HAS_LJTAG
    ,
    input  wire                                          clk_dm,  // for ihb
    input  wire [                                13: 0]  lsu_idu_debug_bus,
    input  wire [                                80: 0]  jtag_idu_ihb_req_bus,
    output wire [                                31: 0]  idu_jtag_ihb_resp_bus
`endif

    ,
    input  wire                                          clk_fpr,  // fpu regfiles clk
    input  wire                                          lsu_idu_mdu_busy,  // from mem
    input  wire                                          fpu_idu_exception_enable,
    output wire                                          dec_is_fpu,  // from fpu-cg
    output wire                                          idu_fpu_inst_valid,
    output wire                                          idu_lsu_fpu_clear_skip,
    output wire [   `OPEN132_IDU_FPU_INST_BUS_WIDTH-1: 0]  idu_fpu_inst_bus,
    input  wire [ `OPEN132_FPU_IDU_RESULT_BUS_WIDTH-1: 0]  fpu_idu_result_bus,
    // decode read FCRs
    output wire [                                 3: 0]  idu_fpu_fcsr_read_enable,  // fcsr2gr read
    input  wire [                                31: 0]  fpu_idu_fcsr_read_data

`ifdef GODSON_DEBUG
    ,
    output wire                                          gs_core_exe_valid,
    output wire [                                31: 0]  gs_core_exe_pc,
    output wire [                                31: 0]  gs_core_exe_inst,
    output wire                                          gs_core_gr_we,
    output wire [                                 4: 0]  gs_core_gr_waddr,
    output wire [                                31: 0]  gs_core_gr_wdata,
    output wire [                                 4: 0]  gs_core_gr_raddr0,
    output wire [                                31: 0]  gs_core_gr_rdata0,
    output wire [                                 4: 0]  gs_core_gr_raddr1,
    output wire [                                31: 0]  gs_core_gr_rdata1
`endif
);

genvar i;

//============================================================//
//                     Regs Declarations
//============================================================//

// de-stage
reg          valid;
reg [  1: 0] inst_whichbus;
reg [ 31: 0] inst;
reg [  4: 0] inst_rk;
reg [ 31: 0] pc;
reg          fetch_ade;
reg          fetch_ex;       // ex: exception
`ifdef CONFIG_MMU
reg [  5: 0] fetch_excode;
`endif

reg          split_2nd;  // only be 1 when valid.

reg          order_flag;       // keep the order of fixed and mdu inst
reg          pre_inst_inmain;  // 1: previous inst run only in the main-pipeline;

// jbr_cancel
// likely   ex_cancel the next inst if not taken;
// no delay-slot also ex_cancel the next inst if taken
reg          jbr_cancel_flag;

reg [  1: 0] inst_whichbus_r;
reg          inst_nop_r;
`ifdef OPEN132_HAS_LJTAG

reg          ss_flag;  // 1: need set ss-ex; 0: no set ss-ex.

`endif

//============================================================//
//                     Wire Declarations
//============================================================//

wire [  4: 0] in_lmask_dec0;
wire [ 31: 0] out_lmask_dec0;
wire [  4: 0] in_hmask_dec;
wire [ 31: 0] out_hmask_dec;
wire [  2: 0] decode_index_dec3_lane0;
wire [  7: 0] decode_onehot_dec3_lane0;
wire [  2: 0] decode_index_dec3_lane1;
wire [  7: 0] decode_onehot_dec3_lane1;
wire          inst_csr_write_stall_chk;
wire [ 13: 0] csr_num_stall_chk;
wire          ctrl_stallf_stall_chk;
wire          clock_gpr;
wire          data_reset_gpr;
wire [  4: 0] raddr0_gpr;
wire [ 31: 0] rvalue0_gpr;
wire [  4: 0] raddr1_0_gpr;
wire [ 31: 0] rvalue1_gpr;
wire [  4: 0] raddr1_1_gpr;
wire          raddr1_s_gpr;
wire          wen0_gpr;
wire [  4: 0] waddr0_gpr;
wire [ 31: 0] wvalue0_gpr;
wire [ 31: 0] ra_value_gpr;
wire [  4: 0] in_lmask_dec1;
wire [ 31: 0] out_lmask_dec1;
wire [  3: 0] in_count1_lane0;
wire [  2: 0] out_count1_lane0;
wire [  3: 0] in_count1_lane1;
wire [  2: 0] out_count1_lane1;
wire [  3: 0] in_count1_lane2;
wire [  2: 0] out_count1_lane2;
wire [  3: 0] in_count1_lane3;
wire [  2: 0] out_count1_lane3;
wire [  3: 0] in_count1_lane4;
wire [  2: 0] out_count1_lane4;
wire [  3: 0] in_count1_lane5;
wire [  2: 0] out_count1_lane5;
wire [  3: 0] in_count1_lane6;
wire [  2: 0] out_count1_lane6;
wire [  3: 0] in_count1_lane7;
wire [  2: 0] out_count1_lane7;
wire [  3: 0] in_count1_lane8;
wire [  2: 0] out_count1_lane8;
wire [  3: 0] in_count1_lane9;
wire [  2: 0] out_count1_lane9;
wire [ 31: 0] nc_seq_pc;

wire          de_allowin;
wire          nxt_validout;

wire          jbr_cancel_nxt;  // the branch_likely inst is not-taken or jump no delay-slot

wire [  1: 0] ft_inst_whichbus;
wire          ft_npc_jbr;        // fetch next-pc is sending jbr_taken.
wire [ 31: 0] ft_inst;
wire [ 31: 0] ft_pc;
wire [ 31: 0] ft_follow_pc;      // follow de-stage pc
`ifdef CONFIG_MMU
wire [  5: 0] ft_excode;
`endif
wire          ft_ade;            // fetch ex: ADE
wire          ft_ex;             // fetch ex: include IBE(bus error) and ADE.

wire          nxt_fetch_ade;
wire          nxt_fetch_ex;
`ifdef CONFIG_MMU
wire [  5: 0] nxt_fetch_excode;
`endif

// true now pc
wire [ 31: 0] true_now_pc;

// next inst
wire [ 31: 0] nxt_inst_pc;

wire          mem_nmi_ccerr;  // nmi/ccerr/int/debug_int need instruction to mem
wire          mem_see_int;
wire          mem_ls_cmplt;   // load/store complete, for sync
wire          mem_roll;
wire          mem_isalu;
wire [  4: 0] mem_dest;
wire [ 31: 0] mem_result;
wire [ 31: 0] lsu_idu_pc;     // for branch pc if delay-slot is in de-satge.
wire          mem_valid;

wire          lisa_ihb_skip;
wire          lisa_da;
wire [  1: 0] lisa_dacm;
wire [  7: 0] lisa_segplv;
wire [  7: 0] lisa_segca_dev;
wire          lisa_user_ccop;
wire          lisa_rdtime_dis;
wire          lisa_fpen;
wire          user_mode;

wire          debug_mode;       // see{lsu_idu_debug_bus}
wire          lisa_dbgcall_en;

wire [  4: 0] wb_dest;
wire [ 31: 0] wb_result;
wire [ 31: 0] wb_fd_result;  // wb2de forward data
wire          wb_mdu_stall;  // mdu instruction can't be forward
wire          wb_stall;      // wb is stalled
wire          wb_valid;

wire [ 31: 0] pred_cancel_target;
wire          pred_cancel;
wire          de_stall_ft;
wire          de_stall_npc;
wire          de_jbr_last;         // inst jbr last clock

// ra_bus
wire [ 31: 0] idu_ifu_ra_value;
wire          idu_ifu_ra_change;

wire          de_ihb_debug;      // ihb: ihb-debug-break
wire          de_ihb_signal;     // ihb: ihb-break
wire          de_ex_dbgss;       // single-step
wire [  3: 0] de_inst_lisa;      // csr/rdtime/cpucfg
wire          de_mdu_inst;       // the inst in the main pipeline is a mdu inst
wire          de_fpu_inst;       // the inst in the main pipeline is a fpu inst
wire          de_see_int_flush;  // inst(mdu/fpu) go main because of seeing interrupt in de-stage
wire          de_idle_ertn;      // the inst is a wait or ertn
wire          de_ex;             // there is a general exceptions

// de_ls_cps is reused for Cache_op and load/store
wire          de_ls_cps_valid;  // inst is load/store/cps
wire [  8: 0] de_ls_cps;
wire          de_ls_sel;        // de_ls_cps sel load/store
wire [  1: 0] de_ls;            // 10:load uncache; 01:store 00:not load or store
wire          de_ls_dbgm;
wire [  1: 0] de_lswidth;       // 00:1byte;01:2byte;11:3byte;10:4byte
wire          de_ll_sc;         // ll/sc
wire          de_lbh;           // lb/lbu/lh/lhu
wire          de_lbhsign_ldl;   // lb/lbh or ldl
wire          de_lb_ldr;        // lb/lbu or ldr
wire [  1: 0] de_inst_cps;      // cache(01)/pref(10)/isync(11) instruction
wire          de_cc_op_i;       // cache op to icache
wire [  2: 0] de_cc_op;         // cache op

// de_ex_info is for {de_ex_dbcl,fetch_ade,de_ex_lserr,de_excode}
`ifdef CONFIG_MMU
wire [  8: 0] de_ex_info;
`else
wire [  7: 0] de_ex_info;
`endif
wire          de_ex_dbcl;   // dbcl
wire          de_ex_lserr;  // load/store address priv or align
`ifdef CONFIG_MMU
wire [  5: 0] de_excode;
`else
wire [  4: 0] de_excode;
`endif

// de_lisa_si14: csr number; sub_excode;
wire [ 13: 0] de_lisa_si14;
wire [ 13: 0] de_csr_num;
wire [  1: 0] de_sub_excode;

wire [  4: 0] de_dest;
wire [ 31: 0] de_result;
wire [  2: 0] de_nc;
wire          de_mdu_flag;  // keep the order of fixed and mdu inst
wire [ 31: 0] de_pc;

// for cache load
wire [ 31: 0] de_mald_vaddr;
wire          de_mald_go;
wire          de_load_nll;    // not ll

// for store/csrwr
wire [ 31: 0] de_r1_value;
wire          de_store_valid;
wire          de_ldl_ldr_valid;
wire          de_csr_write;

wire          ft_pred_valid;
wire          ft_pred_brlo_hit;
wire          ft_pred_jrlo_hit;
wire          ft_pred_jrra;
wire [  3: 0] ft_pred_pos;
wire [  2: 0] ft_pred_cnt;

wire [ 31: 0] de_pred_pc;
wire [ 31: 0] de_pred_npc;
wire          de_pred_jrra;
wire          de_pred_jrlo_hit;
wire          de_pred_brlo_hit;
wire [  3: 0] de_pred_pos;
wire [  2: 0] de_pred_cnt;
wire          de_pred_correct;
wire          de_pred_valid;

wire          perf_inst_ibus;
wire          perf_inst_dbus;
wire          perf_inst_sbus;   // sysbus
wire          perf_load_stall;  // load to use
wire          perf_nop;

wire [ 31: 0] source_data1;
wire [ 31: 0] source_data2;
wire          multiply;
wire          divide;
wire          signmdu;        // unsign or sign
wire          mdu_togpr;
wire          selhl;
wire          mdu_flag;
wire          idu_exu_valid;

wire [  4: 0] rj;
wire [  4: 0] rk;
wire [  4: 0] lrd;

wire [  4: 0] rd_msb;
wire [  4: 0] sa_lsb;
wire [  4: 0] cond;

wire [ 31: 0] msb_dec;    // like 00000111,use lowmask
wire [ 31: 0] lsb_dec;    // like 11111000,use highmask
wire [  7: 0] cj_l3_dec;  // cj[2:0] one hot
wire [  7: 0] ca_dec;     // ca[2:0] one hot

// the detail is displayed in the end
// branch
wire          l_BCEQZ;
wire          l_BCNEZ;

// Arithemetic
wire          l_FADD_S;
wire          l_FSUB_S;
wire          l_FMUL_S;
wire          l_FDIV_S;
wire          l_FMAX_S;
wire          l_FMIN_S;
wire          l_FMAXA_S;
wire          l_FMINA_S;
wire          l_FABS_S;
wire          l_FNEG_S;
wire          l_FCOPYSIGN_S;
wire          l_FSQRT_S;
wire          l_FRECIP_S;
wire          l_FRSQRT_S;
wire          l_FMADD_S;
wire          l_FMSUB_S;
wire          l_FNMADD_S;
wire          l_FNMSUB_S;

// Compare
wire          l_FCMP_COND_S;
wire          l_FCLASS_S;

// Convert
wire          l_FTINTRM_W_S;
wire          l_FTINTRP_W_S;
wire          l_FTINTRZ_W_S;
wire          l_FTINTRNE_W_S;
wire          l_FTINT_W_S;
wire          l_FFINT_S_W;

// Load/Store
wire          l_FLD_S;
wire          l_FST_S;
wire          l_FLDX_S;
wire          l_FSTX_S;

// Move
wire          l_FMOV_S;
wire          l_FSEL;
wire          l_MOVGR2FR_W;
wire          l_MOVFR2GR_S;
wire          l_MOVGR2FCSR;
wire          l_MOVFCSR2GR;
wire          l_MOVFR2CF;
wire          l_MOVCF2FR;
wire          l_MOVGR2CF;
wire          l_MOVCF2GR;

wire [    `OPEN132_IDU_DECODE_BUS_WIDTH-1: 0] lisa_inst_decode;
wire [`OPEN132_IDU_FPU_DECODE_BUS_WIDTH-1: 0] lisa_fpu_inst_decode;

// arithmetic
wire          l_ADD_W;
wire          l_ADDI_W;
wire          l_ADDU12I_W;
wire          l_ALSL_W;
wire          l_SUB_W;
wire          l_PCADDI;
wire          l_PCALAU12I;
wire          l_PCADDU12I;
wire          l_SLT;
wire          l_SLTU;
wire          l_SLTI;
wire          l_SLTUI;
wire          l_CLO_W;
wire          l_CLZ_W;
wire          l_CTO_W;
wire          l_CTZ_W;
wire          l_EXT_W_H;
wire          l_EXT_W_B;

// LUI
wire          l_LU12I_W;

// logic
wire          l_AND;
wire          l_OR;
wire          l_XOR;
wire          l_ANDI;
wire          l_ORI;
wire          l_XORI;
wire          l_ANDN;
wire          l_ORN;
wire          l_NOR;

// shift
wire          l_SLL_W;
wire          l_SRL_W;
wire          l_SRA_W;
wire          l_ROTR_W;
wire          l_SLLI_W;
wire          l_SRLI_W;
wire          l_SRAI_W;
wire          l_ROTRI_W;

// STRINS STRPICK
wire          l_BSTRINS_W;
wire          l_BSTRPICK_W;

// swap
wire          l_REVB_2H;
wire          l_BITREV_4B;
wire          l_BITREV_W;

// MASKEQZ MASKNEQ
wire          l_MASKEQZ;
wire          l_MASKNEZ;

// ALIGN
wire          l_BYTEPICK_W;

// MDU
wire          l_MUL_W;
wire          l_MULH_W;
wire          l_MULH_WU;
wire          l_DIV_W;
wire          l_MOD_W;
wire          l_DIV_WU;
wire          l_MOD_WU;

// BREAK SYSCALL DBGCALL
wire          l_BREAK;
wire          l_DBGCALL;
wire          l_SYSCALL;

// CPUCFG RDTIME
wire          l_RDTIMEL_W;
wire          l_RDTIMEH_W;
wire          l_CPUCFG;

// CSR
wire          l_CSRRD;
wire          l_CSRWR;
wire          l_CSRXCHG;
wire          l_ERTN;
wire          l_IDLE;
wire          l_CACOP;

// mem
wire          l_LL_W;
wire          l_SC_W;
wire          l_LD_B;
wire          l_LD_H;
wire          l_LD_W;
wire          l_LD_BU;
wire          l_LD_HU;
wire          l_ST_B;
wire          l_ST_H;
wire          l_ST_W;
wire          l_PRELD;
wire          l_LDL_W;
wire          l_LDR_W;
wire          l_STL_W;
wire          l_STR_W;

// BAR
wire          l_DBAR;
wire          l_IBAR;

// branch
wire          l_BEQZ;
wire          l_BNEZ;
wire          l_BEQ;
wire          l_BNE;
wire          l_BLT;
wire          l_BGE;
wire          l_BLTU;
wire          l_BGEU;
wire          l_B;
wire          l_BL;
wire          l_JIRL;

wire          ade_priv;
wire          addr_mem;  // address is memory attribute

wire          inst_pref;       // pref
wire          inst_cacop_sup;
wire          inst_cacop_ker;  // kernel cache_op
wire          inst_cps;        // cache or pref or isync

wire          cc_op_user;  // user cache_op
wire          cc_op_sup;
wire          cc_op_i;
wire [  2: 0] cc_op;
wire          cc_isync;
wire          cc_pref;

wire [  4: 0] l_cache_op;
wire          inst_cacop;

wire          inst_clz;    // clz;
wire          inst_clo;    // clo;
wire          inst_ctz;
wire          inst_cto;
wire          inst_cltzo;  // clz/clo/cto/czo

wire          inst_extw;
wire          inst_extw_h;

wire          inst_lui;

wire          inst_swap;
wire          inst_revbh;
wire          inst_bitrev_4b;
wire          inst_bitrev_w;

wire          inst_add;
wire          inst_sub;
wire          inst_add_shf;
wire          inst_add_pc;
wire          inst_add_m12;  // add_result & ~0xfff

wire          inst_bytepick;
wire          inst_maskz;
wire          inst_maskz_eq;

wire          inst_lbhsign;    // lb/lh
wire          inst_lbh;        // lb/lh/lbu/lhu
wire          inst_lb_lbu;     // lb/lbu
wire          inst_ldl;
wire          inst_ldr;
wire          inst_sb;         // sb
wire          inst_sh;         // sh
wire          inst_stl;
wire          inst_load;       // load
wire          inst_store_nf;   // store, not fpu-store
wire          inst_store;      // store
wire          inst_ld_st;      // load or store
wire          inst_ll_sc;      // ll/sc
wire          inst_ld_st_b;    // byte
wire          inst_ld_st_h;    // half-word
wire          inst_ld_st_w;    // word
wire          inst_ld_st_l;    // ldl/stl
wire          inst_ld_st_r;    // ldr/str
wire          inst_ldl_ldr;    // ldl/ldr
wire          inst_ld_st_cps;  // load or store or cache or pref or isync

// mdu inst
wire          inst_mul;
wire          inst_div;
wire          inst_signmdu;
wire          inst_mdu;
wire          inst_mdu_hi;

// logic
wire          inst_logic;
wire          inst_and;
wire          inst_or;
wire          inst_nor;
wire          inst_xor;
wire          inst_r1_not;  // for ANDN and ORN

// shift
wire          inst_shift;
wire          inst_sll;
wire          inst_sr;
wire          inst_rotr;
wire          inst_shiftv;
wire          inst_shift_sa;
wire          inst_sr_a;

// ext/ins
wire          inst_ext;
wire          inst_ins;
wire          inst_ext_ins;

// exception
wire          inst_break;
wire          inst_syscall;
wire          inst_dbcl;

// jump and branch
wire          inst_jbr_andlink;
wire          inst_jbr;

// slt
wire          inst_slt_s;         // slt/slti
wire          inst_slt_u;         // sltu/sltiu
wire          inst_slt;           // slt/slti/sltu/sltiu
wire          inst_sub_trap_slt;  // sub

// bar
wire          inst_dbar;
wire          inst_ibar;

// csr instruction
wire          inst_idle;
wire          inst_ertn;
wire          inst_idle_ertn;

wire          inst_csr;        // include csrrd/csrwr/csrxchg
wire          inst_csrrd;
wire          inst_csrwr;
wire          inst_csrxchg;
wire          inst_rdtime;     // rdtimel/rdtimeh
wire          inst_rdtimel_w;
wire          inst_rdtimeh_w;
wire          inst_cpucfg;
`ifdef CONFIG_MMU
wire          inst_invtlb;
`endif

wire          inst_lsa;

wire          lisa_jbr;
wire          lisa_jr;
wire          lisa_br;
wire          lisa_b;
wire          lisa_br_eq;
wire          lisa_br_ne;
wire          lisa_br_lt;
wire          lisa_br_ge;
wire          lisa_br_cmpz;
wire          lisa_br_uns;

wire          inst_lbh_u;

wire          inst_lh_lhu;

wire          inst_str;
wire          inst_ll;
wire          inst_sc;
wire          inst_lw;
wire          inst_sw;

wire          inst_sr_l;

wire          inst_ctrl_stallf;  // csrwr need flush pipeline
wire          inst_stall_fetch;

wire          inst_mov_r0;    // inst_cpucfg || inst_csrxchg
wire          inst_mov_all1;  // inst_csrwr

wire          inst_onlymain;  // instruction will only run in main-pipelin
wire          inst_mdu_main;  // instruction will run both in mdu and main-pipelin
wire          inst_reserved;

wire          inst_privilege;

wire          inst_imm_zeroext;
wire          inst_imm_signext;

wire          inst_split_2nd;  // see{instruction split}
wire [  4: 0] inst_dest;
wire [  4: 0] lisa_dest;

// lisa dest}
wire          lisa_dest_isrj;
wire          lisa_dest_isrd;
wire          lisa_dest_is1;

// lisa need rj/rk/rd
wire          lisa_no_src;
wire          lisa_needrj;
wire          lisa_needrkrd;

// now, only rdtimel/h is split instructions.
wire          idu_lsu_uop_cmplt;  // see{pipeline control}
wire          de2nxt_inst_cmplt;
wire          inst_split;

// to select de_result
// wire   mov_result_sel;
// wire   csr_result_sel;
wire          andlink_result_sel;
wire          extw_result_sel;
wire          revbh_result_sel;
wire          bytepick_result_sel;
wire          bitrev_4b_result_sel;
wire          bitrev_w_result_sel;

// wire   lui_result_sel;
wire          add_result_sel;
wire          ld_st_cps_result_sel;

// wire   cmp_result_sel;
// wire   logic_result_sel;
// wire   clzo_result_sel;
// wire   shift_result_sel;
// wire   ext_result_sel;
// wire   ins_result_sel;
// wire   shf_ext_ins_result_sel;
wire          maskz_result_sel;

wire [ 31: 0] r0_readdata;
wire [ 31: 0] r1_readdata;

wire          lisa_r1_rd;
wire [  4: 0] la_r0;
wire [  4: 0] la_r1;

wire          la_r0_hit_mem;
wire          la_r1_hit_mem;
wire          la_r0_hit_wb;
wire          la_r1_hit_wb;

// fd: forward
wire          needr0;
wire          needr1;
wire          r0_hit_mem;
wire          r1_hit_mem;
wire          r0_hit_wb;
wire          r1_hit_wb;
wire          mem_stall_r0;
wire          mem_stall_r1;
wire          mem_fd_r0;
wire          mem_fd_r1;
wire          wb_stall_r0;
wire          wb_stall_r1;
wire          wb_fd_r0;
wire          wb_fd_r1;
wire [ 31: 0] r0_value;
wire [ 31: 0] r1_value;

// r0/r1 stall
wire          r0_stall;
wire          r0r1_stall;

// forward value
wire [ 31: 0] fd_r0_value;
wire [ 31: 0] fd_r1_value;

wire          r0_sel_fd;
wire          r1_sel_fd;

// for lisa branch
wire          r0_lt_r1;
wire          l_br_lt;
wire          l_br_ge;
wire          l_br_eq;
wire          l_br_ne;

// regfiles
wire [  4: 0] raddr0;
wire [  4: 0] raddr1_0;
wire [  4: 0] raddr1_1;
wire          raddr1_s;

// ra_value
wire [ 31: 0] regfile_ra_value;

wire [  1: 0] lisa_sa2;
wire [  4: 0] lisa_ui5;
wire [  4: 0] lisa_si5;
wire [ 11: 0] lisa_si12;
wire [ 11: 0] lisa_ui12;
wire [ 13: 0] lisa_si14;
wire [ 19: 0] lisa_si20;
wire [ 15: 0] lisa_ls_imm;
wire [ 31: 0] lisa_arth_imm;
wire [ 25: 0] lisa_jbr_imm;

wire          lisa_imm_12_s0;   // s_e(imm[11:0]<<0 )
wire          lisa_imm_5_s12;   // s_e(imm[4 :0]<<12)
wire          lisa_imm_20_s2;   // s_e(imm[19:0]<<2 )
wire          lisa_imm_20_s12;  //    imm[19:0]<<12

wire          lisa_jbr_m;  // [30]==1'b1
wire          lisa_b_m;    // [28:27]==2'b10
wire          lisa_bz_m;   // x1000x or x100x0

// final imm
wire [ 27: 0] br_imm;
wire [ 15: 0] lsa_imm;
wire [ 19: 0] lui_imm;
wire [ 11: 0] logic_imm;
wire [ 31: 0] arth_imm;
wire [  4: 0] shf_sa;

wire [ 31: 0] vaddr;               // used to generate the value of lswidth
wire [ 31: 0] ld_st_cps_addr_res;  // select vaddr for load/store/cache/pref/isync

wire [ 31: 0] de_brpc;  // see{jump and branch}

// lsa adder ; calculate load/store address.
wire          lsa_lowp_m;
wire [ 31: 0] lsa_add_src1;
wire [ 31: 0] lsa_add_src2;
wire [ 31: 0] lsa_add_res;

wire          ls_addr_err;  // load/store address error, see{exception}
wire [ 31: 0] ls_vaddr;     // if ls_addr_err is set, ls_vaddr[1:0] need be unchanged.

wire          dbgm;  // debug memory

// see{exception}segement
wire          ex_ipe;         // instruction privilege
wire          de_ex_canc_ls;  // fetch_ex or adel/ades

wire [ 31: 0] result_final;  // alu final result

// SB/LB:1byte;SH/LH:2byte;
//        vaddr[1:0]
// LDL/STL, 00:1byte; 01;2byte; 10:3byte; 11:4byte;
// LDR/STR, 00:4byte; 01:3byte; 10:2byte; 11:1byte;
wire          ls_left_00;
wire          ls_left_01;
wire          ls_left_10;
wire          ls_left_11;
wire          ls_right_00;
wire          ls_right_01;
wire          ls_right_10;
wire          ls_right_11;
wire          ls_4bytes;

// for ldl/ldr, stl/str, r1_value need rotr
wire          ld_st_rotr_1;
wire          ld_st_rotr_2;
wire          ld_st_rotr_3;
wire [ 31: 0] r1_rotr_value;

wire [ 31: 0] mdu_r0_mask;
wire [ 31: 0] mdu_r1_mask;

// Prediction
wire          jbr_valid;
wire          jbr_stall;
wire          jbr_taken;
wire          jbr_taken_valid;
wire [ 31: 0] jbr_target;

wire          pred_taken;

wire          pred_jbr_wrong;

wire          ihb_signal;
wire          ihb_debug;
wire          ihb_te_be;   // currently ihb enable

wire          mm_stall_main;  // main/mdu stall main

// wire store_stall = de_store_valid  && !storeq_allowin;
wire          store_stall;
wire          sync_stall;   // BAR/LL/SC/stall-fetch(csrwr/ibar)
wire          other_stall;

wire          skip_trigged;  // 1: need to set ihb signal; 0: don't set ihb signal.

// The second half of a split instruction cannot mark an interrupt.
wire          willex_go_mem;

wire          mem_rdygo;
wire          mdu_rdygo;

wire          idu_exu_uop_cmplt;  // idu_exu_mdu_bus' u-op will be received by mdu
wire          dec2fpu_uop_cmplt;  // idu_fpu_inst_bus' u-op will be received by mdu

// wire idu_lsu_uop_cmplt; //idu_lsu_inst_bus' u-op  will be received by mem
wire          idu_exu_inst_cmplt;  // idu_exu_mdu_bus' inst will be received by mdu
wire          dec2fpu_inst_cmplt;  // idu_fpu_inst_bus' inst will be received by mdu
wire          idu_lsu_inst_cmplt;  // idu_lsu_inst_bus' inst  will be received by mem

wire          mdu_turn_flag;

wire          ex_break;

wire          ex_syscall;

wire          ex_dbcl;

wire          ex_ri;  // reserved instruction

// for store and load
wire          ls_ade_nalign;
wire          ls_ade_priv;
wire          ex_ls_nalign;
wire          ex_ls_ade;

// ade_priv:see memory management.
`ifdef CONFIG_MMU
wire [  5: 0] ade_excode;
wire [  5: 0] exe_excode;  // the new ex generated by this stage
`else
wire [  4: 0] ade_excode;
wire [  4: 0] exe_excode;  // the new ex generated by this stage
`endif

wire          debug_ex;

wire          lisa_jbr_taken;

// 32+28 adder ; calculate jbr_target
wire [ 31: 0] add_32_28_src1;
wire [ 27: 0] br_imm_lowp_m;
wire [ 31: 0] add_32_28_src2;
wire [ 31: 0] add_32_28_res;
wire [ 31: 0] mode_jbr_src1;

// target address
wire [ 31: 0] br_target;

// idu_lsu_next_pc
wire [ 31: 0] idu_lsu_next_pc;  // pc of next instruction fllowing mem

// result
wire [ 31: 0] mov_result;        // r0_value-->
wire [ 31: 0] rt_result;         // r1_value-->
wire [ 31: 0] csr_result;        // inst-->
wire [ 31: 0] andlink_result;    // ifu_idu_seq_pc(pc+4)-->
wire [ 31: 0] extw_result;       // r1_value-->mux2_1-->
wire [ 31: 0] lui_result;        // imm-->
wire [ 31: 0] revbh_result;      // r1_value-->
wire [ 31: 0] bitrev_4b_result;
wire [ 31: 0] bitrev_w_result;
wire [ 31: 0] align_result;
wire [ 31: 0] add_result;        // r1_value-->not-->mux4_1-->adder_32-->

// vaddr:add_result-->3bit_mux-->
wire [ 31: 0] cmp_result;          // add_result-->in32_or-->1bit_mux2_1
wire [ 31: 0] logic_result;        // r1_value-->mux2_1-->or_32-->not-->mux4_1
wire [ 31: 0] cltzo_result;        // r0_value-->not-->in4_and-->in4_and-->in5_and
wire [ 31: 0] shf_ext_ins_result;  // r1_value-->shift_result_temp-->and_32
wire [ 31: 0] maskz_result;        // maskeqz masknez result

wire [ 31: 0] result_temp1;

wire [ 31: 0] result_temp2;

wire [ 31: 0] result_temp3;

wire [ 31: 0] result_temp4;

wire [ 31: 0] result_temp5;

wire [ 31: 0] result_temp6;

// andlink
wire [  2: 0] link_off;
wire [  2: 0] link_off_lm;  // lowpower mask
wire [ 31: 0] link_pc;

// SEB/SEH
wire [ 15: 0] extw_data;

// BITREV 4B
wire [ 31: 0] bitswap_result_temp;

// BITREV W
wire [ 31: 0] bitrev_w_result_temp;

wire          align_rotr_0;
wire          align_rotr_1;
wire          align_rotr_2;
wire          align_rotr_3;
wire          align_rotr_4;
wire [  1: 0] align_rotr_sa;

// 32-32 adder
wire          add_lowp_m;
wire [ 31: 0] add_r0_data;   // two sign bit
wire [ 32: 0] add_srcdata1;  // two sign bit
wire [ 31: 0] add_r1_imm;
wire [ 32: 0] add_srcdata2;
wire [  1: 0] add_cout;

wire [ 31: 0] add_r0_shf;       // for ALSL
wire [ 31: 0] add_result_temp;
wire          add_sel_imm;

// compare:use adder
// add_result==32'd0,src1==src2;
// {ov,add_result[31]},10:src1<src2; 11:src1>src2;
// {ov,add_result[31]},01:src1<src2; 00:src1>src2;
// in a word,for sign_trap,add_cout[0],1:src1<src2;0:src1>=src2;
// for unsign_trap, add_cout[1],1:src1>=src2;0:src1<src2;
wire          cmp_lt;
wire          cmp_geu;
wire          cmp_ne;

// logic
wire [ 31: 0] logic_srcdata2;
wire [ 31: 0] and_result;
wire [ 31: 0] or_result;
wire [ 31: 0] nor_result;
wire [ 31: 0] xor_result;

// ext and ins reuse shift, and data is r0_value
wire [ 31: 0] mode_shf_data;
wire [  4: 0] mode_shf_sa_v;
wire [ 31: 0] mode_shf_data_rev;

wire          sll_or_ins;

// ext and ins reuse shift, and data is r0_value
wire [ 31: 0] shift_data;

// shift data extension
// rotr:  rj/r1_value
// sra: {32{rj/r1_value[31]}}
// sll/srl/ext/ins: 0
wire [ 31: 0] shift_data_ext;

// shift sa
wire [  4: 0] shift_sa;

// shift result: do rotate word right
wire [ 63: 0] shift_result_temp_t;
wire [ 31: 0] shift_result_temp;

// EXT
// use ext_ins_lm,it need be like 000111
wire [ 31: 0] ext_ins_lm;

// lisa ext_lm
wire          lisa_inst_ext;
wire [  4: 0] lisa_ext_msbd;
wire [ 31: 0] lisa_ext_lm;

// INS.
// use ext_ins_lm, it need be like 000111
// use lsb_dec, it need be like 111000
wire [ 31: 0] ins_mask;

// shift/EXT/INS result
wire [ 31: 0] shf_ext_ins_r1_mask;
wire [ 31: 0] shf_ext_ins_shf_mask;

// the following is the fundamental of clo/clz
// 31____28 27____24 23____20 19____16 15____12 11____8  7____4  3____0
//  |____|   |____|   |____|   |____|   |____|   |____|  |____|  |____|
//   || |     || |     || |     || |     || |     || |    || |    || |
//  a0[2:0]  a1[2:0]  a2[2:0]  a3[2:0]  a4[2:0]  a5[2:0] a6[2:0] a7[2:0]
//      |________|________|________|        |________|_______|_______|
//      |__________________________|        |________________________|
//         ||                     |            ||                   |
//      b0[2:1]                 b0[0]       b1[2:1]               b1[0]
// explanation:
//       a0~a7 is means step1_0~step1_1
//       b0 is step2_0, b1 is step2_1
//          ____
//       a |____| is a count_one_4_3 module, input: in[3:0], output: out[2:0]
// for count_one_4_3, out[2:1] means the number of the leading one, out[0] means all is one
// for example:  in[3:0]      out[2:1]  out[0]
//              4'b0xxx        2'b00    1'b0
//              4'b10xx        2'b01    1'b0
//              4'b110x        2'b10    1'b0
//              4'b1110        2'b11    1'b0
//              4'b1111        2'bxx    1'b1
wire [ 31: 0] r0_value_reverse;

wire [ 31: 0] clo_data;

wire [  2: 0] step1_0;
wire [  2: 0] step1_1;
wire [  2: 0] step1_2;
wire [  2: 0] step1_3;
wire [  2: 0] step1_4;
wire [  2: 0] step1_5;
wire [  2: 0] step1_6;
wire [  2: 0] step1_7;

wire [  2: 0] step2_0;
wire [  2: 0] step2_1;

wire          de_perf_go;
`ifdef OPEN132_HAS_LJTAG

wire          pesudoint_go;
wire          mem_ex_go;
wire          dertn_go;
wire          debug_sse;

// wire       debug_mode  ;
wire          mem_noex;
wire [  7: 0] asid;      // for ihb

`endif

// input
wire                                        clk_fpr_nc;
wire                                        main_mdu_is_run_nc;
wire                                        fpu_ex_enable_nc;
wire [`OPEN132_FPU_IDU_RESULT_BUS_WIDTH-1: 0] fpu2dec_bus_nc;
wire [                               31: 0] fcrs2dec_rdata_nc;

`ifdef CONFIG_MMU

`endif
//============================================================//
//                    Module Implementation
//============================================================//

// Instance Parameter Aliases
// x_jtag_ihb

//------------------------------------------------------------//
//                      Bus Connections
//------------------------------------------------------------//
assign nc_seq_pc  =  ifu_idu_seq_pc;

//------------------------------------------------------------//
//                           de_bus
//------------------------------------------------------------//
// Instance Boundary Alias Mapping
// x_exu_dec_lmask_5_32_0

assign in_lmask_dec0  =  rd_msb;
assign msb_dec        =  out_lmask_dec0;
// x_exu_dec_hmask_5_32
assign in_hmask_dec  =  sa_lsb;
assign lsb_dec       =  out_hmask_dec;
// x_dsu_dec_n_m_8_0
assign decode_index_dec3_lane0  =  rj[2:0];
assign cj_l3_dec                =  decode_onehot_dec3_lane0;
// x_dsu_dec_n_m_8_1
assign decode_index_dec3_lane1  =  inst[17:15];
assign ca_dec                   =  decode_onehot_dec3_lane1;
// x_csr_stall_chk
assign inst_csr_write_stall_chk  =  inst_csrwr||inst_csrxchg;
assign csr_num_stall_chk         =  de_csr_num;
assign inst_ctrl_stallf          =  ctrl_stallf_stall_chk;
// x_dsu_gpr
assign clock_gpr         =  clk_gpr;
assign data_reset_gpr    =  data_reset;
assign raddr0_gpr        =  raddr0;
assign r0_readdata       =  rvalue0_gpr;
assign raddr1_0_gpr      =  raddr1_0;
assign r1_readdata       =  rvalue1_gpr;
assign raddr1_1_gpr      =  raddr1_1;
assign raddr1_s_gpr      =  raddr1_s;
assign wen0_gpr          =  wb_valid;
assign waddr0_gpr        =  wb_dest;
assign wvalue0_gpr       =  wb_result;
assign regfile_ra_value  =  ra_value_gpr;
// x_exu_dec_lmask_5_32_1
assign in_lmask_dec1  =  lisa_ext_msbd;
assign lisa_ext_lm    =  out_lmask_dec1;
// x_exu_count1_4_3_0
assign in_count1_lane0  =  clo_data[31:28];
assign step1_0          =  out_count1_lane0;
// x_exu_count1_4_3_1
assign in_count1_lane1  =  clo_data[27:24];
assign step1_1          =  out_count1_lane1;
// x_exu_count1_4_3_2
assign in_count1_lane2  =  clo_data[23:20];
assign step1_2          =  out_count1_lane2;
// x_exu_count1_4_3_3
assign in_count1_lane3  =  clo_data[19:16];
assign step1_3          =  out_count1_lane3;
// x_exu_count1_4_3_4
assign in_count1_lane4  =  clo_data[15:12];
assign step1_4          =  out_count1_lane4;
// x_exu_count1_4_3_5
assign in_count1_lane5  =  clo_data[11:8];
assign step1_5          =  out_count1_lane5;
// x_exu_count1_4_3_6
assign in_count1_lane6  =  clo_data[7:4];
assign step1_6          =  out_count1_lane6;
// x_exu_count1_4_3_7
assign in_count1_lane7  =  clo_data[3:0];
assign step1_7          =  out_count1_lane7;
// x_exu_count1_4_3_8
assign in_count1_lane8  =  {step1_0[0],step1_1[0],step1_2[0],step1_3[0]};
assign step2_0          =  out_count1_lane8;
// x_exu_count1_4_3_9
assign in_count1_lane9  =  {step1_4[0],step1_5[0],step1_6[0],step1_7[0]};
assign step2_1          =  out_count1_lane9;
assign nxt_validout  =  ifu_idu_inst_valid;

assign {
        ft_inst_whichbus,  // [100:99]
        ft_npc_jbr      ,  // [98:98]
        ft_inst         ,  // [97:66]
        ft_pc           ,  // [65:34]
        ft_follow_pc    ,  // [33:2]
`ifdef CONFIG_MMU
        ft_excode       ,
`endif
        ft_ade          ,  // [1 :1]
        ft_ex  // [0 :0]
       }  =  ifu_idu_inst_bus;

`ifdef GODSON_DEBUG
assign gs_core_exe_valid  =  valid;
assign gs_core_exe_pc     =  pc;
assign gs_core_exe_inst   =  inst;
`endif
assign nxt_fetch_ade  =  ft_ade;
assign nxt_fetch_ex   =  ft_ex ;
`ifdef CONFIG_MMU
assign nxt_fetch_excode  =  ft_excode;
`endif
always @ ( posedge clk_g )
begin
    if( reset ) begin
        valid <= 1'b0;
    end
    else if( ex_cancel ) begin
        valid <= 1'b0;
    end
    else if(de_allowin) begin
        valid <= nxt_validout && !jbr_cancel_nxt;
    end
end

always @ ( posedge clk_g )
begin
    if(data_reset) begin
        inst_whichbus <= 2'd0;
        pc            <= 32'd0;
        fetch_ade     <= 1'd0;  // ADE
        fetch_ex      <= 1'd0;  // IBE and ADE
`ifdef CONFIG_MMU
        fetch_excode  <= 6'd0;
`endif
    end
    else if(de_allowin) begin
        if(nxt_validout) begin
            pc <= ft_pc;
        end

        if(ifu_idu_inst_valid) begin
            inst_whichbus <= ft_inst_whichbus;
            fetch_ade     <= nxt_fetch_ade   ;  // ADE
            fetch_ex      <= nxt_fetch_ex    ;  // IBE and ADE
`ifdef CONFIG_MMU
            fetch_excode  <= nxt_fetch_excode;
`endif
        end
    end
end

// inst[31:16]
always @ ( posedge clk_g )
begin
    if(data_reset) begin
        inst[31:16] <= 16'd0;
        inst_rk     <= 5'd0;
    end
    else if(de_allowin) begin
        if(ifu_idu_inst_valid) begin
            inst[31:0]   <= ft_inst[31:0];
            inst_rk[4:0] <= ft_inst[14:10];
        end
    end
end
// End de_bus

//------------------------------------------------------------//
//                         Pc Next Pc
//------------------------------------------------------------//
assign true_now_pc  =  pc;

assign nxt_inst_pc  =  ft_follow_pc;
// End Pc Next Pc

//------------------------------------------------------------//
//                    lsu_idu_forward_bus
//------------------------------------------------------------//
assign mem_valid  =  lsu_idu_forward_valid;
assign {mem_nmi_ccerr,  // 73:73
        mem_see_int  ,  // 72:72
        mem_ls_cmplt ,  // 71:71
        mem_roll     ,  // 70:70
        mem_isalu    ,  // 69:69
        mem_dest     ,  // 68:64
        mem_result   ,  // 63:32
        lsu_idu_pc  // 31:0
       }  =  lsu_idu_forward_bus;
// End lsu_idu_forward_bus

//------------------------------------------------------------//
//                      csr_idu_ctrl_bus
//------------------------------------------------------------//
assign {lisa_ihb_skip  ,  // 23
        lisa_da        ,  // 22
        lisa_dacm      ,  // 21:20
        lisa_segplv    ,  // 19:12
        lisa_segca_dev ,  // 11:4
        lisa_user_ccop ,  // 3:3
        lisa_rdtime_dis,  // 2:2
        lisa_fpen      ,  // 1:1
        user_mode  // 0:0
       }  =  csr_idu_ctrl_bus;

assign lisa_dbgcall_en  =  !user_mode || debug_mode;
// End csr_idu_ctrl_bus
//
//------------------------------------------------------------//
//                    rtu_idu_forward_bus
//------------------------------------------------------------//
assign wb_valid  =  rtu_idu_forward_valid;
assign {wb_dest,
        wb_result,
        wb_fd_result,
        wb_mdu_stall,
        wb_stall     }  =  rtu_idu_forward_bus;

// End rtu_idu_forward_bus

//------------------------------------------------------------//
//                     idu_ifu_branch_bus
//------------------------------------------------------------//
assign idu_ifu_branch_bus  =  {pred_cancel_target,
                                pred_cancel    ,
                                de_stall_ft    ,
                                de_stall_npc   ,
                                de_jbr_last    };
assign idu_ifu_return_bus  =  {idu_ifu_ra_change,
                                idu_ifu_ra_value};
// End idu_ifu_branch_bus

//------------------------------------------------------------//
//                      idu_lsu_inst_bus
//------------------------------------------------------------//

assign de_ls_cps  =  de_ls_sel ? {de_ls,de_ls_dbgm,de_lswidth,de_ll_sc,de_lbh,de_lbhsign_ldl,de_lb_ldr} :
                       {3'd0, de_cc_op, de_cc_op_i, de_inst_cps } ;  // [8:7] must be 2'd0;

assign de_ex_info    =  {de_ex_dbcl,fetch_ade,de_ex_lserr,de_excode};
`ifdef CONFIG_MMU
assign de_lisa_si14  =  inst_invtlb ? {9'd0, inst[4:0]}
                                     : de_csr_num | {12'd0,de_sub_excode};
`else
assign de_lisa_si14  =  de_csr_num | {12'd0,de_sub_excode};
`endif

assign idu_lsu_inst_bus        =  {de_ihb_debug    ,  // 1 , total:120
                                    de_ihb_signal   ,  // 1 , total:119
                                    de_ex_dbgss     ,  // 1 , total:118
                                    de_inst_lisa    ,  // 4 , total:115
                                    de_mdu_inst     ,  // 1
                                    de_fpu_inst     ,  // 1 , total: 110
                                    de_see_int_flush,  // 1 , inst(mdu/fpu) go main because of seeing interrupt
                                    de_idle_ertn    ,  // 1
                                    de_ex           ,  // 1
                                    de_ls_cps_valid ,  // 1 , total:105
                                    de_ls_cps       ,  // 9
`ifdef CONFIG_MMU
                                    de_ex_info      ,  // 9
`else
                                    de_ex_info      ,  // 8 , total:95
`endif
                                    de_lisa_si14    ,  // 14, total:87
                                    de_dest         ,  // 5 , total:73
                                    de_result       ,  // 32, total:68
                                    de_nc           ,  // 3 , total:36
                                    de_mdu_flag     ,  // 1
                                    de_pc  // 32, total:32
                                  };
assign idu_lsu_load_attr_bus   =  {de_mald_vaddr,
                                    de_mald_go,
                                    de_load_nll  };
assign idu_lsu_store_data_bus  =  {de_r1_value,
                                    de_store_valid,
                                    de_ldl_ldr_valid,
                                    de_csr_write};
// End idu_lsu_inst_bus

//------------------------------------------------------------//
//                          pred_bus
//------------------------------------------------------------//

assign {ft_pred_valid,
        ft_pred_brlo_hit,
        ft_pred_jrlo_hit,
        ft_pred_jrra,
        ft_pred_pos,
        ft_pred_cnt}  =  ifu_idu_pred_bus;
// End pred_bus
//------------------------------------------------------------//
//                     idu_bpu_update_bus
//------------------------------------------------------------//

assign idu_bpu_update_bus  =  {de_pred_pc     ,
                                de_pred_npc    ,
                                de_pred_jrra   ,
                                de_pred_jrlo_hit,
                                de_pred_brlo_hit,
                                de_pred_pos    ,
                                de_pred_cnt    ,
                                de_pred_correct,
                                de_pred_valid
                              };

//------------------------------------------------------------//
//                    Inst Follow De Stage
//------------------------------------------------------------//
assign idu_lsu_follow_pc  =  nxt_inst_pc;

//------------------------------------------------------------//
//                      idu_csr_perf_bus
//------------------------------------------------------------//
assign idu_csr_perf_bus  =  {perf_inst_ibus,
                              perf_inst_dbus,
                              perf_inst_sbus,
                              perf_load_stall,
                              perf_nop       };
// End idu_csr_perf_bus

//------------------------------------------------------------//
//                      idu_exu_mdu_bus
//------------------------------------------------------------//
assign idu_exu_mdu_bus  =  {source_data1,
                             source_data2,
                             multiply,
                             divide,
                             signmdu,
                             mdu_togpr,
                             selhl,
                             mdu_flag,
                             idu_exu_valid};
// End idu_exu_mdu_bus
// End Bus Connections

//------------------------------------------------------------//
//                     Instruction Decode
//------------------------------------------------------------//
assign rj   =  inst[9 : 5];
assign rk   =  inst[14:10];
assign lrd  =  inst[ 4: 0];

assign rd_msb  =  inst[20:16];
assign sa_lsb  =  inst[14:10];
assign cond    =  inst[19:15];

exu_dec_lmask_5_32 x_exu_dec_lmask_5_32_0 (
    .in  (in_lmask_dec0 ),
    .out (out_lmask_dec0)
);
exu_dec_hmask_5_32 x_exu_dec_hmask_5_32 (
    .in  (in_hmask_dec ),
    .out (out_hmask_dec)
);
dsu_dec_n_m #(.M(8)) x_dsu_dec_n_m_8_0 (
    .decode_index  (decode_index_dec3_lane0 ),
    .decode_onehot (decode_onehot_dec3_lane0)
);
dsu_dec_n_m #(.M(8)) x_dsu_dec_n_m_8_1 (
    .decode_index  (decode_index_dec3_lane1 ),
    .decode_onehot (decode_onehot_dec3_lane1)
);
//------------------------------------------------------------//
//                          CACHE Op
//------------------------------------------------------------//

assign inst_pref  =  l_PRELD;
assign inst_cps   =  inst_cacop_sup|inst_pref;

assign l_cache_op      =  lrd;
assign inst_cacop      =  l_CACOP;
assign inst_cacop_sup  =  inst_cacop && cc_op_sup;
assign inst_cacop_ker  =  inst_cacop && (!cc_op_user||!lisa_user_ccop);
// H_W_INV and FILL can used for user_mode in lisa_mode;
assign cc_op_user  =  l_cache_op[4]&&(!l_cache_op[3]||l_cache_op[1]);
assign cc_op_sup   =  l_cache_op[2:1]==2'b00||l_cache_op[4:2]==3'b111;
assign cc_op_i     =  inst_cacop &&(!l_cache_op[0]||l_cache_op[1]);
assign cc_isync    =  inst_cacop&&l_cache_op==5'b11111;  // no care addr_cached and ade_priv(set ex-ade)
assign cc_pref     =  inst_pref  && !ade_priv;
assign cc_op[2]    =  inst_cacop &&
                        (l_cache_op[4]&&(!l_cache_op[3]||l_cache_op[1]));
assign cc_op[1]    =  inst_cacop &&
                        (l_cache_op[4:3]==2'b01 || l_cache_op[4:3]==2'b10 ||
                        l_cache_op[2:1] ==2'b10);  // no care 11111(isync)
assign cc_op[0]    =  inst_cacop &&
                        (l_cache_op[4:3]!=2'b11);
// lisa mode
// lrd[4:0]   ICACHE       DCACHE    cc_op
//  00000     I_S_TAG        -        001
//  00001        -        I_S_TAG     001
//  01000     I_W_INV        -        011
//  01001        -        I_W_INV     011
//  10000     H_W_INV        -        111
//  10001        -        H_W_INV     111
//  11000     I_L_TAG        -        000
//  11001        -        I_L_TAG     000
//  11100       LOCK         -        010
//  11101        -          LOCK      010
//  11110       FILL         -        100
//  11111     H_W_INV     H_W_INV     111 (isync)
//
// cc_op     ICACHE       DCACHE
// 011       I_W_INV      I_W_INV
// 000       I_L_TAG     I_L_TAG
// 001       I_S_TAG     I_S_TAG
// 110        H_INV       H_INV
// 111       H_W_INV     H_W_INV
// 101         H_W         H_W
// 100        FILL          -
// 010        LOCK         LOCK
// End CACHE Op

//------------------------------------------------------------//
//                 Instruction Classification
//------------------------------------------------------------//
//------------------------------------------------------------//
//                     Class By Function
//------------------------------------------------------------//

// trap

assign inst_clz    =  l_CLZ_W;
assign inst_clo    =  l_CLO_W;
assign inst_ctz    =  l_CTZ_W;
assign inst_cto    =  l_CTO_W;
assign inst_cltzo  =  inst_clz || inst_clo || inst_ctz || inst_cto;

assign inst_extw    =  l_EXT_W_B||l_EXT_W_H;
assign inst_extw_h  =  l_EXT_W_H;

assign inst_lui  =  l_LU12I_W;

assign inst_swap       =  inst_revbh || inst_bitrev_4b || inst_bitrev_w;
assign inst_revbh      =  l_REVB_2H;
assign inst_bitrev_4b  =  l_BITREV_4B;
assign inst_bitrev_w   =  l_BITREV_W;

assign inst_add      =  l_ADD_W  || l_ADDU12I_W || l_ADDI_W ||
                          l_ALSL_W || inst_add_pc;
assign inst_sub      =  l_SUB_W ;
assign inst_add_shf  =  l_ALSL_W ;
assign inst_add_pc   =  l_PCADDI || l_PCALAU12I || l_PCADDU12I;
assign inst_add_m12  =  l_PCALAU12I;

assign inst_bytepick  =  l_BYTEPICK_W;
assign inst_maskz     =  l_MASKEQZ||l_MASKNEZ;
assign inst_maskz_eq  =  l_MASKEQZ   ;

assign inst_lbhsign    =  l_LD_B || l_LD_H;
assign inst_lbh_u      =  l_LD_BU|| l_LD_HU;
assign inst_lbh        =  inst_lbhsign       || inst_lbh_u;
assign inst_lb_lbu     =  l_LD_B || l_LD_BU;
assign inst_lh_lhu     =  l_LD_H || l_LD_HU;
assign inst_ldl        =  l_LDL_W;
assign inst_ldr        =  l_LDR_W;
assign inst_sb         =  l_ST_B;
assign inst_sh         =  l_ST_H;
assign inst_stl        =  l_STL_W;
assign inst_str        =  l_STR_W;
assign inst_ll         =  l_LL_W;
assign inst_sc         =  l_SC_W;
assign inst_lw         =  l_LD_W;
assign inst_sw         =  l_ST_W;
assign inst_load       =  inst_lbh  || inst_ll  || inst_lw    ||
                            inst_ldl  || inst_ldr ;
assign inst_store      =  inst_sb   || inst_sh  || inst_sc    ||
                            inst_sw   || inst_stl || inst_str   ;
assign inst_ld_st      =  inst_load || inst_store;
assign inst_ll_sc      =  inst_ll   || inst_sc   ;
assign inst_ld_st_b    =  inst_lb_lbu || inst_sb ;
assign inst_ld_st_h    =  inst_lh_lhu || inst_sh ;
assign inst_ld_st_w    =  inst_lw     || inst_ll || inst_sw || inst_sc;
assign inst_ld_st_l    =  inst_ldl    || inst_stl;
assign inst_ld_st_r    =  inst_ldr    || inst_str;
assign inst_ldl_ldr    =  inst_ldl    || inst_ldr;
assign inst_ld_st_cps  =  inst_ld_st  || inst_cps;

// mdu inst
assign inst_mul      =  l_MUL_W || l_MULH_W || l_MULH_WU;
assign inst_div      =  l_DIV_W || l_MOD_W || l_DIV_WU || l_MOD_WU;
assign inst_signmdu  =  l_MUL_W  || l_MULH_W || l_DIV_W || l_MOD_W;
assign inst_mdu      =  inst_mul || inst_div;
assign inst_mdu_hi   =  l_MULH_W || l_MULH_WU || l_MOD_W || l_MOD_WU;

// logic
assign inst_logic   =  inst_and || inst_or || inst_nor  || inst_xor;
assign inst_and     =  l_AND    || l_ANDI  || l_ANDN;
assign inst_or      =  l_OR     || l_ORI   || l_ORN;
assign inst_nor     =  l_NOR;
assign inst_xor     =  l_XOR    || l_XORI;
assign inst_r1_not  =  l_ANDN   ||l_ORN;

// shift
assign inst_shift     =  inst_sll || inst_sr || inst_rotr;
assign inst_sll       =  l_SLL_W || l_SLLI_W;
assign inst_rotr      =  l_ROTR_W   || l_ROTRI_W;
assign inst_shiftv    =  l_SLL_W    || l_SRA_W    || l_SRL_W  || l_ROTR_W;
assign inst_shift_sa  =  l_SLLI_W   || l_SRAI_W   || l_SRLI_W || l_ROTRI_W;
assign inst_sr_a      =  l_SRA_W    || l_SRAI_W;
assign inst_sr_l      =  l_SRL_W    || l_SRLI_W;
assign inst_sr        =  inst_sr_l || inst_sr_a;

// move

// ext/ins
assign inst_ext      =  l_BSTRPICK_W;
assign inst_ins      =  l_BSTRINS_W;
assign inst_ext_ins  =  inst_ext || inst_ins;

// trap

// exception
assign inst_break    =  l_BREAK;
assign inst_syscall  =  l_SYSCALL;
assign inst_dbcl     =  l_DBGCALL;

// jump and branch
// lisa jbr
assign lisa_br_eq    =  l_BEQ  || l_BEQZ;
assign lisa_br_ne    =  l_BNE  || l_BNEZ;
assign lisa_br_lt    =  l_BLT  || l_BLTU;
assign lisa_br_ge    =  l_BGE  || l_BGEU;
assign lisa_br_cmpz  =  l_BEQZ || l_BNEZ;  // cmp zero
assign lisa_br_uns   =  l_BLTU || l_BGEU;  // unsigned cmp
assign lisa_br       =  lisa_br_eq||lisa_br_ne||lisa_br_lt||lisa_br_ge;
assign lisa_b        =  l_B   || l_BL;
assign lisa_jr       =  l_JIRL;
assign lisa_jbr      =  lisa_jr|| lisa_br || lisa_b;

assign inst_jbr_andlink  =  l_BL || l_JIRL;
assign inst_jbr          =  lisa_jbr;

// slt
assign inst_slt_s         =  l_SLT    || l_SLTI;
assign inst_slt_u         =  l_SLTU   || l_SLTUI;
assign inst_slt           =  inst_slt_s || inst_slt_u;  // slt/slti/sltu/sltiu
assign inst_sub_trap_slt  =  inst_sub || inst_slt;
// sync
assign inst_dbar  =  l_DBAR;
assign inst_ibar  =  l_IBAR;
`ifdef CONFIG_MMU
assign inst_invtlb  =  inst[31:15] == 17'b00000110010010011
                       && inst[4:0] <= 5'd6;
`endif

assign inst_idle         =  l_IDLE;
assign inst_ertn         =  l_ERTN;
assign inst_idle_ertn    =  inst_idle | inst_ertn;
`ifdef CONFIG_MMU
assign inst_stall_fetch  =  inst_ctrl_stallf || inst_ibar || inst_invtlb;
`else
assign inst_stall_fetch  =  inst_ctrl_stallf || inst_ibar;
`endif
csr_stall_chk x_csr_stall_chk (
    .inst_csr_write (inst_csr_write_stall_chk),
    .csr_num        (csr_num_stall_chk       ),
    .ctrl_stallf    (ctrl_stallf_stall_chk   )
);

// csrrd. csrwr. csrxchg
assign inst_csr      =  (l_CSRRD||l_CSRWR||l_CSRXCHG);
assign inst_csrrd    =  l_CSRRD;
assign inst_csrwr    =  l_CSRWR;
assign inst_csrxchg  =  l_CSRXCHG;

assign inst_rdtime     =  inst_rdtimel_w || inst_rdtimeh_w;
assign inst_rdtimel_w  =  l_RDTIMEL_W;
assign inst_rdtimeh_w  =  l_RDTIMEH_W;
assign inst_cpucfg     =  l_CPUCFG;
// End Class By Function

`ifdef CONFIG_MMU
assign inst_mov_r0    =  inst_cpucfg||inst_csrxchg||inst_invtlb;
`else
assign inst_mov_r0    =  inst_cpucfg||inst_csrxchg;
`endif
assign inst_mov_all1  =  inst_csrwr;

//------------------------------------------------------------//
//          Class By Running In Mdu Or Main Pipeline
//------------------------------------------------------------//
assign inst_onlymain  =  inst_add   | inst_lui   | inst_swap  | inst_logic
                         |inst_break | inst_syscall
                         |inst_dbar  | inst_dbcl | inst_ld_st_cps
                         |inst_cltzo | inst_sub_trap_slt       | inst_extw
                         |inst_shift | inst_ext_ins
                         |inst_jbr   | inst_idle_ertn
                         |inst_bytepick | inst_maskz |
`ifdef CONFIG_MMU
                         |inst_csr   | inst_rdtime|inst_cpucfg|inst_ibar
                         |inst_invtlb;
`else
                         |inst_csr   | inst_rdtime|inst_cpucfg|inst_ibar;
`endif
assign inst_mdu_main  =  inst_mdu;
assign inst_reserved  =  ~inst_onlymain & ~inst_mdu_main;
// End Class By Running In Mdu Or Main Pipeline

`ifdef CONFIG_MMU
assign inst_privilege  =  inst_csr||inst_ertn||inst_idle||inst_cacop_ker
                           ||inst_invtlb;
`else
assign inst_privilege  =  inst_csr||inst_ertn||inst_idle||inst_cacop_ker;
`endif

//------------------------------------------------------------//
//           Class By Imm Need Zero Or Sign Extend
//------------------------------------------------------------//
assign inst_imm_zeroext  =  l_ANDI     || l_ORI    || l_XORI;
assign inst_imm_signext  =  l_SLTI     || l_SLTUI  || l_ADDI_W ||
                              l_ADDU12I_W|| inst_add_pc;
// End Class By Imm Need Zero Or Sign Extend

//------------------------------------------------------------//
//                       Class By Dest
//------------------------------------------------------------//
// for SAVE/RESTORE, only RESTORE or update sp will set macro_dest
//                  e_dest must be zero forever.
assign inst_dest  =  lisa_dest;

assign lisa_dest       =  ({5{lisa_dest_isrd}}&lrd  ) |
                            ({5{lisa_dest_isrj}}&rj   ) |
                            ({5{lisa_dest_is1 }}&5'd1 ) ;
assign lisa_dest_isrd  =  inst_add    | inst_sub   | inst_slt    | inst_cltzo
                          |inst_extw   | inst_lui   | inst_logic  | inst_shift
                          |inst_swap   | inst_maskz | inst_ext_ins| l_JIRL
                          |inst_mdu    | inst_load
                          |inst_sc     | inst_bytepick
                          |inst_csr    | inst_cpucfg| inst_rdtime&&!inst_split_2nd ;
assign lisa_dest_isrj  =  inst_rdtime&&inst_split_2nd;
assign lisa_dest_is1   =  l_BL;

// End Class By Dest

//------------------------------------------------------------//
//            Class By Using Rs Or Rt In De Stage
//------------------------------------------------------------//
assign lisa_no_src    =  inst_ertn ||inst_idle||
                           inst_break||inst_syscall||inst_dbcl;
assign lisa_needrj    =  !(inst_add_pc ||inst_lui   ||inst_dbar  ||inst_ibar||
                           inst_csrwr  ||inst_cpucfg||inst_rdtime||
                           lisa_b      ||lisa_no_src );
assign lisa_needrkrd  =  l_ADD_W     || l_ALSL_W  || l_SUB_W || l_SLT   || l_SLTU   ||
                           l_AND       || l_OR      || l_XOR   || l_ANDN  || l_ORN    ||
                           l_NOR       || l_SLL_W   || l_SRL_W || l_SRA_W || l_ROTR_W ||
                           l_FLDX_S    || l_FSTX_S  ||
                           l_BYTEPICK_W|| inst_maskz|| inst_mdu ||  // need rd
                           inst_ins    || inst_csrwr|| inst_csrxchg ||
`ifdef CONFIG_MMU
                           inst_store  || lisa_br&&!lisa_br_cmpz    || inst_ldl_ldr
                           ||inst_invtlb;
`else
                           inst_store  || lisa_br&&!lisa_br_cmpz    || inst_ldl_ldr;
`endif
// End Class By Using Rs Or Rt In De Stage
// End Instruction Classification

//------------------------------------------------------------//
//                     Instruction Split
//------------------------------------------------------------//

assign inst_split      =  inst_rdtime;
assign inst_split_2nd  =  split_2nd;  // only be 1 when valid
always @ ( posedge clk_g )
begin
    if( reset ) begin
        split_2nd <= 1'b0;
    end
    else if( ex_cancel ) begin
        split_2nd <= 1'b0;
    end
    else if( inst_split&&idu_lsu_uop_cmplt ) begin
        split_2nd <= ~split_2nd;
    end
end
// End Instruction Split

//------------------------------------------------------------//
//                   Control Signal Decode
//------------------------------------------------------------//

// result sel
assign andlink_result_sel    =  inst_jbr_andlink;
assign extw_result_sel       =  inst_extw;
assign revbh_result_sel      =  inst_revbh;
assign bytepick_result_sel   =  inst_bytepick;
assign bitrev_4b_result_sel  =  inst_bitrev_4b;
assign bitrev_w_result_sel   =  inst_bitrev_w;
// assign lui_result_sel     = inst_lui;
assign add_result_sel        =  inst_add | inst_sub;
assign ld_st_cps_result_sel  =  inst_ld_st_cps;
// assign cmp_result_sel     = inst_slt;
// assign logic_result_sel   = inst_logic;
// assign clzo_result_sel    = inst_clzo;
// assign shift_result_sel   = inst_shift;
// assign ext_result_sel     = inst_ext;
// assign ins_result_sel     = inst_ins;
// assign shf_ext_ins_result_sel = shift_result_sel || ext_result_sel || ins_result_sel;
assign maskz_result_sel  =  inst_maskz;
// End Control Signal Decode

//------------------------------------------------------------//
//                           r0 r1
//------------------------------------------------------------//
`ifdef CONFIG_MMU
assign lisa_r1_rd     =  !inst_invtlb
                         && (inst[30]|inst[26]|inst[24]|inst[22]) & (~inst[31]);
`else
assign lisa_r1_rd     =  (inst[30]|inst[26]|inst[24]|inst[22]) & (~inst[31]);
`endif
assign la_r0          =  rj;
assign la_r1          =  lisa_r1_rd ? lrd : inst_rk;
assign la_r0_hit_mem  =  (mem_dest == la_r0) && (|la_r0);
assign la_r1_hit_mem  =  (mem_dest == la_r1) && (|la_r1);
assign la_r0_hit_wb   =  (wb_dest  == la_r0) && (|la_r0);
assign la_r1_hit_wb   =  (wb_dest  == la_r1) && (|la_r1);
// End r0 r1

assign needr0  =  lisa_needrj;
assign needr1  =  lisa_needrkrd;

assign r0_hit_mem  =  la_r0_hit_mem;
assign r1_hit_mem  =  la_r1_hit_mem;
assign r0_hit_wb   =  la_r0_hit_wb ;
assign r1_hit_wb   =  la_r1_hit_wb ;

assign mem_stall_r0  =  needr0 & mem_valid & ~mem_isalu & r0_hit_mem;
assign mem_stall_r1  =  needr1 & mem_valid & ~mem_isalu & r1_hit_mem;
assign mem_fd_r0     =  mem_valid &  mem_isalu & r0_hit_mem;
assign mem_fd_r1     =  mem_valid &  mem_isalu & r1_hit_mem;

assign wb_stall_r0  =  ~mem_fd_r0 & needr0 & wb_mdu_stall & r0_hit_wb;
assign wb_stall_r1  =  ~mem_fd_r1 & needr1 & wb_mdu_stall & r1_hit_wb;
assign wb_fd_r0     =  wb_valid & r0_hit_wb;
assign wb_fd_r1     =  wb_valid & r1_hit_wb;

assign r0_stall    =  mem_stall_r0|wb_stall_r0;
assign r0r1_stall  =  mem_stall_r0|mem_stall_r1|wb_stall_r0|wb_stall_r1;

assign fd_r0_value  =  mem_fd_r0 ? mem_result : wb_fd_result;
assign fd_r1_value  =  mem_fd_r1 ? mem_result : wb_fd_result;

assign r0_sel_fd  =  (mem_fd_r0||wb_fd_r0);
assign r1_sel_fd  =  (mem_fd_r1||wb_fd_r1);
assign r0_value   =  r0_sel_fd           ? fd_r0_value   : r0_readdata;
assign r1_value   =  r1_sel_fd           ? fd_r1_value   : r1_readdata;

assign r0_lt_r1  =  r0_value < r1_value;
assign l_br_lt   =  (r0_value[31]&&!r1_value[31]&&!lisa_br_uns)
                    ||(r0_value[31]== r1_value[31]|| lisa_br_uns)&&r0_lt_r1;
assign l_br_ge   =  !l_br_lt;
assign l_br_eq   =  r0_value==(r1_value & {32{!lisa_br_cmpz}});
assign l_br_ne   =  !l_br_eq;

assign raddr0    =  la_r0;
assign raddr1_0  =  inst_rk;
assign raddr1_1  =  lrd;
assign raddr1_s  =  lisa_r1_rd;
dsu_gpr x_dsu_gpr (
    .clock      (clock_gpr     ),
    .data_reset (data_reset_gpr),
    .raddr0     (raddr0_gpr    ),
    .rvalue0    (rvalue0_gpr   ),
    .raddr1_0   (raddr1_0_gpr  ),
    .rvalue1    (rvalue1_gpr   ),
    .raddr1_1   (raddr1_1_gpr  ),
    .raddr1_s   (raddr1_s_gpr  ),
    .wen0       (wen0_gpr      ),
    .waddr0     (waddr0_gpr    ),
    .wvalue0    (wvalue0_gpr   ),
    .ra_value   (ra_value_gpr  )
);

`ifdef GODSON_DEBUG
assign gs_core_gr_we      =  wb_valid;
assign gs_core_gr_waddr   =  wb_dest;
assign gs_core_gr_wdata   =  wb_result;
assign gs_core_gr_raddr0  =  raddr0;
assign gs_core_gr_rdata0  =  r0_readdata;
assign gs_core_gr_raddr1  =  raddr1_s ? raddr1_1 : raddr1_0;
assign gs_core_gr_rdata1  =  r1_readdata;
`endif
// End RegFile

//------------------------------------------------------------//
//                            Imm
//------------------------------------------------------------//
//------------------------------------------------------------//
//                          Lisa Imm
//------------------------------------------------------------//
assign lisa_sa2   =  inst[16:15];
assign lisa_ui5   =  inst[14:10];
assign lisa_si5   =  inst[14:10];
assign lisa_si12  =  inst[21:10];
assign lisa_ui12  =  inst[21:10];
assign lisa_si14  =  inst[23:10];
assign lisa_si20  =  inst[24:5];

assign lisa_imm_12_s0       =  l_SLTI      || l_SLTUI  || l_ADDI_W;
assign lisa_imm_5_s12       =  l_ADDU12I_W;
assign lisa_imm_20_s2       =  l_PCADDI;
assign lisa_imm_20_s12      =  l_PCALAU12I || l_PCADDU12I;
assign lisa_ls_imm          =  inst_ll_sc ? {lisa_si14,2'b0} : {{4{lisa_si12[11]}},lisa_si12};
assign lisa_arth_imm        =  {{32{lisa_imm_12_s0 }} & {{20{lisa_si12[11]}},lisa_si12}}
                               |{{32{lisa_imm_5_s12 }} & {{15{lisa_si5 [ 4]}},lisa_si5,12'b0}}
                               |{{32{lisa_imm_20_s2 }} & {{10{lisa_si20[19]}},lisa_si20,2'b0}}
                               |{{32{lisa_imm_20_s12}} & {lisa_si20,12'b0}};
assign lisa_jbr_m           =  inst[30];
assign lisa_b_m             =  lisa_jbr_m && inst[28:27]==2'b10;
assign lisa_bz_m            =  lisa_jbr_m &&!inst[29]&&!inst[28]&&(!inst[27]||!inst[26]);
assign lisa_jbr_imm[15: 0]  =  {16{lisa_jbr_m}} & inst[25:10];
assign lisa_jbr_imm[20:16]  =  lisa_b_m||lisa_bz_m ? inst[4:0] : {5{lisa_jbr_imm[15]}};
assign lisa_jbr_imm[25:21]  =  lisa_b_m            ? inst[9:5] : {5{lisa_jbr_imm[20]}};

// End Lisa Imm

assign br_imm     =  {lisa_jbr_imm, 2'b0};  // lisa
assign lsa_imm    =  lisa_ls_imm;
assign lui_imm    =  lisa_si20;
assign logic_imm  =  lisa_ui12;
assign arth_imm   =  lisa_arth_imm;
assign shf_sa     =  lisa_ui5;
// End Imm

//------------------------------------------------------------//
//                     Memory Management
//------------------------------------------------------------//

assign lsa_lowp_m    =  inst_ld_st_cps||`OPEN132_NOT_LOWP;
assign lsa_add_src1  =  r0_value&{32{lsa_lowp_m}};
assign lsa_add_src2  =  {{16{lsa_imm[15]}},lsa_imm};
assign lsa_add_res   =  lsa_add_src1 + lsa_add_src2;
assign vaddr         =  lsa_add_res;

assign ls_vaddr[31:2]      =  vaddr[31:2];
assign ls_vaddr[1 :0]      =  vaddr[1 :0] & {2{!inst_ld_st_l || ls_addr_err}};
assign ld_st_cps_addr_res  =  ls_vaddr;

mmu_vaddr_check x_mmu_vaddr_check (
    .lisa_da        (lisa_da       ),
    .lisa_dacm      (lisa_dacm     ),
    .lisa_segplv    (lisa_segplv   ),  // 1: can access by PLV3; 0: only access by PLV0
    .lisa_segca_dev (lisa_segca_dev),  // 1: device-attribute

    .vaddr          (vaddr         ),
    .dbgm           (dbgm          ),
    .user_mode      (user_mode     ),
    .addr_mem       (addr_mem      ),
    .ade_priv       (ade_priv      )
);
// End Memory Management

//------------------------------------------------------------//
//                  idu_lsu_inst_bus Signals
//------------------------------------------------------------//

// inst_lisa
assign de_inst_lisa[3:2]  =  inst_csr    ? 2'd1 :
                               inst_rdtime ? 2'd2 :
                               inst_cpucfg ? 2'd3 : 2'd0;
assign de_inst_lisa[1  ]  =  inst_split_2nd  // set if rdtime-2nd
`ifdef CONFIG_MMU
                             ||inst_csrrd  // set if csrrd
                             ||inst_invtlb;
`else
                             ||inst_csrrd;  // set if csrrd
`endif
assign de_inst_lisa[0  ]  =  inst_rdtimeh_w  // set if rdtimeh
                             ||inst_csrxchg  // set if csrxchg
                             ||inst_ibar;  // set if ibar
// de_lisa_si14
// if has fetch_ex, set de_csr_num to 0, because de_csr_num will or sub-excode
assign de_csr_num  =  lisa_si14 & {14{inst_csr&&!fetch_ex}};
// de_sub_excode see{exceptions}

assign de_mdu_inst       =  inst_mdu_main;
assign de_fpu_inst       =  1'b0;
assign de_see_int_flush  =  inst_mdu_main
                            &&(mem_see_int||debug_int) && !inst_split_2nd;  // split_2nd can't mark int
assign de_idle_ertn      =  inst_idle_ertn;

assign de_ls_sel       =  inst_ld_st;
assign de_ls[1]        =  inst_load ;
assign de_ls[0]        =  (addr_mem||inst_store);  // ma-load or store
assign de_ls_dbgm      =  dbgm;
assign de_ll_sc        =  inst_ll_sc;
assign de_lbh          =  inst_lbh;
assign de_lbhsign_ldl  =  inst_lbhsign | inst_ldl;
assign de_lb_ldr       =  inst_lb_lbu  | inst_ldr;

assign ls_left_00   =  (inst_ld_st_l) & (vaddr[1:0]==2'b00);
assign ls_left_01   =  (inst_ld_st_l) & (vaddr[1:0]==2'b01);
assign ls_left_10   =  (inst_ld_st_l) & (vaddr[1:0]==2'b10);
assign ls_left_11   =  (inst_ld_st_l) & (vaddr[1:0]==2'b11);
assign ls_right_00  =  (inst_ld_st_r) & (vaddr[1:0]==2'b00);
assign ls_right_01  =  (inst_ld_st_r) & (vaddr[1:0]==2'b01);
assign ls_right_10  =  (inst_ld_st_r) & (vaddr[1:0]==2'b10);
assign ls_right_11  =  (inst_ld_st_r) & (vaddr[1:0]==2'b11);
assign ls_4bytes    =  inst_ld_st_w | ls_left_11|ls_right_00;
assign de_lswidth   =  ( {2{(inst_ld_st_b|ls_left_00|ls_right_11)}} & 2'b00)  // 00:1byte
                       |( {2{(inst_ld_st_h|ls_left_01|ls_right_10)}} & 2'b01)  // 01:2byte
                       |( {2{( ls_left_10|ls_right_01)            }} & 2'b11)  // 11:3byte
                       |( {2{  ls_4bytes                          }} & 2'b10);  // 10:4byte
// sel de_dest;
assign de_dest    =  inst_dest;
assign de_result  =  result_final;
assign de_nc      =  3'd0;

// for cache load
assign de_mald_vaddr  =  {ls_vaddr[31:2],2'b00};
// A stalled or excepting MA load cannot issue a request.
assign de_mald_go   =  addr_mem
                       & (inst_load&~inst_ll)
                       & valid
                       & ~r0r1_stall
                       & ~ex_cancel
                       & ~de_ex_canc_ls;
assign de_load_nll  =  inst_load&&!inst_ll && valid && !ex_cancel;

//              lswidth  vaddr[1:0]  need rotr
// stl(1byte)      00       00         3bytes
// ldl(1byte)      00       00         3bytes
// stl(2bytes)     01       01         2bytes
// ldl(2bytes)     01       01         2bytes
// stl(3bytes)     11       10         1byte
// ldl(3bytes)     11       10         1byte
// stl(4bytes)     10       11            -
// ldl(4bytes)     10       11            -
// str(4byte)      10       00            -
// ldr(4byte)      10       00            -
// str(3bytes)     11       01         3bytes
// ldr(3bytes)     11       01         3bytes
// str(2bytes)     01       10         2bytes
// ldr(2bytes)     01       10         2bytes
// str(1bytes)     00       11         1byte
// ldr(1bytes)     00       11         1byte
//     sb          00       00            -
//     sb          00       01         3bytes
//     sb          00       10         2bytes
//     sb          00       11         1byte
//     sh          10       00            -
//     sh          10       10         2bytes
assign ld_st_rotr_1   =  (  inst_ld_st_l                      & vaddr[1:0]==2'b10)
                         |( (inst_ld_st_r | inst_sb          ) & vaddr[1:0]==2'b11);
assign ld_st_rotr_2   =  (  inst_ld_st_l                      & vaddr[1:0]==2'b01)
                         |( (inst_ld_st_r | inst_sb | inst_sh) & vaddr[1:0]==2'b10);
assign ld_st_rotr_3   =  (  inst_ld_st_l                      & vaddr[1:0]==2'b00)
                         |( (inst_ld_st_r | inst_sb          ) & vaddr[1:0]==2'b01);
assign r1_rotr_value  =  {32{ld_st_rotr_1}} &{r1_value[7 : 0],r1_value[31:24],r1_value[23:16],r1_value[15: 8]}
                         |{32{ld_st_rotr_2}} &{r1_value[15: 8],r1_value[7 : 0],r1_value[31:24],r1_value[23:16]}
                         |{32{ld_st_rotr_3}} &{r1_value[23:16],r1_value[15: 8],r1_value[7 : 0],r1_value[31:24]}
                         |{32{~ld_st_rotr_1 & ~ld_st_rotr_2 & ~ld_st_rotr_3}} & r1_value;

// for store queue, csr write
// csr_write value is is in r1_rotr_value.
assign de_r1_value       =  r1_rotr_value;
assign de_store_valid    =  inst_store   & valid;
assign de_ldl_ldr_valid  =  inst_ldl_ldr & valid;
`ifdef CONFIG_MMU
assign de_csr_write      =  inst_csrwr||inst_csrxchg||inst_invtlb;
`else
assign de_csr_write      =  inst_csrwr||inst_csrxchg;
`endif

// load/store/cache/pref/isync
assign de_ls_cps_valid  =  inst_ld_st || inst_cacop || inst_pref;
// for cache operation
assign de_inst_cps  =  ({1'b0,inst_cacop_sup}|
                         {(cc_pref||cc_isync),cc_isync});
assign de_cc_op_i   =  cc_op_i|cc_isync;
assign de_cc_op     =  cc_op|{3{cc_isync}};
// End idu_lsu_inst_bus Signals

//------------------------------------------------------------//
//                  idu_exu_mdu_bus Signals
//------------------------------------------------------------//
assign mdu_r0_mask   =  32'hffffffff;
assign mdu_r1_mask   =  32'hffffffff;
assign source_data1  =  r0_value & mdu_r0_mask & {32{inst_mdu_main}};
assign source_data2  =  r1_value & mdu_r1_mask & {32{inst_mdu_main}};
assign multiply      =  inst_mul;
assign divide        =  inst_div;
assign signmdu       =  inst_signmdu;  // unsign or sign
assign mdu_togpr     =  inst_mdu;
assign selhl         =  inst_mdu_hi;
// End idu_exu_mdu_bus Signals

//------------------------------------------------------------//
//                 idu_bpu_update_bus Signals
//------------------------------------------------------------//

assign pred_jbr_wrong = valid &&
                        (((inst_jbr && !jbr_stall && !jbr_taken || !inst_jbr) && pred_taken) ||
                         (inst_jbr && !jbr_stall &&  jbr_taken && (ft_pc != jbr_target)));
assign pred_cancel_target = jbr_taken_valid || !HAS_BTB ? jbr_target : (true_now_pc + 4);
assign pred_cancel        = pred_jbr_wrong;

generate
    if (HAS_BTB) begin: gen_has_btb
        reg        pred_valid;
        reg        pred_brlo_hit;
        reg        pred_jrlo_hit;
        reg        pred_jrra;
        reg [ 3:0] pred_pos;
        reg [ 2:0] pred_cnt;

        wire [2:0] pred_taken_nxt = pred_cnt == 3'b111 || pred_cnt == 3'b110
                                    ? 3'b111 : pred_cnt + 3'd2;
        wire [2:0] pred_non_nxt   = pred_cnt == 3'b000 || pred_cnt == 3'b001
                                    ? 3'b000 : pred_cnt - 3'd2;
        wire [2:0] pred_cnt_nxt   = jbr_taken ? pred_taken_nxt : pred_non_nxt;

        assign pred_taken = pred_valid & (|pred_cnt[2:1]);

        always @ (posedge clk_g) begin
            if (data_reset) begin
                pred_valid    <= 1'b0;
                pred_brlo_hit <= 1'b0;
                pred_jrlo_hit <= 1'b0;
                pred_jrra     <= 1'b0;
                pred_pos      <= 4'b0;
                pred_cnt      <= 3'b0;
            end
            else if (de_allowin && ifu_idu_inst_valid) begin
                pred_valid    <= ft_pred_valid;
                pred_brlo_hit <= ft_pred_brlo_hit;
                pred_jrlo_hit <= ft_pred_jrlo_hit;
                pred_jrra     <= ft_pred_jrra;
                pred_pos      <= ft_pred_pos;
                pred_cnt      <= ft_pred_cnt;
            end
        end

        assign de_pred_valid = !de_ex && valid && de2nxt_inst_cmplt &&
                               (pred_jbr_wrong && !pred_jrlo_hit || pred_valid);
        assign de_pred_correct  = !pred_jbr_wrong;
        assign de_pred_cnt      = pred_valid ? pred_cnt_nxt :
                                  3'b010 + {1'b0, pc[2]^pc[3]^pc[5], pc[3]^pc[4]};
        assign de_pred_pos      = pred_pos;
        assign de_pred_brlo_hit = pred_brlo_hit;
        assign de_pred_jrlo_hit = pred_jrlo_hit;
        assign de_pred_jrra     = lisa_jr && rj == 5'd1 || pred_valid && pred_jrra;
        assign de_pred_npc      = jbr_target;
        assign de_pred_pc       = true_now_pc;

        assign idu_ifu_ra_value = valid && (wb_dest == 5'd1) && !wb_mdu_stall
                                  ? wb_fd_result : regfile_ra_value;
        assign idu_ifu_ra_change = valid && de_dest == 5'd1 ||
                                   mem_valid && mem_dest == 5'd1 ||
                                   wb_valid && wb_dest == 5'd1 && wb_mdu_stall;
    end
endgenerate

generate
    if (HAS_BTB == 1'b0) begin: gen_no_btb
        assign pred_taken = 1'b0;

        assign de_pred_valid     = 1'b0;
        assign de_pred_correct   = 1'b0;
        assign de_pred_cnt       = 2'd0;
        assign de_pred_pos       = 4'd0;
        assign de_pred_brlo_hit  = 1'b0;
        assign de_pred_jrlo_hit  = 1'b0;
        assign de_pred_jrra      = 1'b0;
        assign de_pred_npc       = 32'd0;
        assign de_pred_pc        = 32'd0;

        assign idu_ifu_ra_change = 1'b0;
        assign idu_ifu_ra_value  = 32'b0;
    end
endgenerate
//------------------------------------------------------------//
//                      Pipeline Control
//------------------------------------------------------------//

assign store_stall  =  1'b0;
assign sync_stall   =  (inst_dbar||inst_ll_sc||inst_stall_fetch) && !mem_ls_cmplt;
assign other_stall  =  store_stall    || sync_stall || mm_stall_main;

assign willex_go_mem  =  (fetch_ex  || mem_see_int || mem_nmi_ccerr ||
                           de_ex_dbgss || debug_int ||
                           de_ihb_signal&&(!lisa_ihb_skip||skip_trigged)) &&
                           !inst_split_2nd;
// if split_2nd has ex, split_1st must has ex.

assign mem_rdygo  =  !other_stall && !r0r1_stall;
assign mdu_rdygo  =  !r0r1_stall && !wb_stall && inst_mdu_main;

assign idu_lsu_inst_valid  =  valid && ( mem_rdygo&&(!inst_mdu_main||exu_idu_allowin&&!wb_stall) );
assign idu_exu_valid       =  valid && (mdu_rdygo&&(!inst_mdu_main||lsu_idu_allowin)) && !willex_go_mem;
assign idu_fpu_inst_valid  =  1'b0;

assign idu_exu_uop_cmplt   =  (idu_exu_valid & exu_idu_allowin);
assign dec2fpu_uop_cmplt   =  idu_fpu_inst_valid               ;
assign idu_lsu_uop_cmplt   =  (idu_lsu_inst_valid  & lsu_idu_allowin);
assign idu_exu_inst_cmplt  =  idu_exu_uop_cmplt;
assign dec2fpu_inst_cmplt  =  dec2fpu_uop_cmplt;
assign idu_lsu_inst_cmplt  =  idu_lsu_uop_cmplt && (!inst_split|| inst_split_2nd);

assign de2nxt_inst_cmplt  =  idu_exu_inst_cmplt || dec2fpu_inst_cmplt || idu_lsu_inst_cmplt;
// Stall-fetch and wait instructions do not allow fetch-in.
assign de_allowin              =  !valid ||
                                    de2nxt_inst_cmplt&&!inst_stall_fetch&&!inst_idle;
assign idu_lsu_fpu_clear_skip  =  1'b0;
// End Pipeline Control

//------------------------------------------------------------//
//            Main Pipeline And Mdu Pipeline Flag
//------------------------------------------------------------//
always @ ( posedge clk_g )
begin
    if( reset ) begin
        pre_inst_inmain <= 1'b0;
    end
    else if( idu_exu_inst_cmplt||dec2fpu_inst_cmplt ) begin
        pre_inst_inmain <= 1'b0;
    end
    else if( idu_lsu_inst_cmplt ) begin
        pre_inst_inmain <= 1'b1;
    end
end

assign mdu_turn_flag  =  idu_exu_inst_cmplt&&inst_mdu_main;
always @ ( posedge clk_g )
begin
    if( reset ) begin
        order_flag <= 1'b0;
    end
    else if( mdu_turn_flag ) begin
        order_flag <= ~order_flag;
    end
end

assign de_mdu_flag  =  order_flag;
// notice:while inst is run both in mdu and main pipeline,
// we give it a different flag in mdu and main pipeline,(but order_flag register no change)
// so that while the inst is marked interrupt in main pipeline,we can ex_cancel it in mdu
assign mdu_flag  =  (pre_inst_inmain | inst_mdu_main) ? ~order_flag : order_flag;
// End Main Pipeline And Mdu Pipeline Flag

//------------------------------------------------------------//
//                         Exceptions
//------------------------------------------------------------//
assign ex_break  =  inst_break;

assign ex_syscall  =  inst_syscall;

assign ex_dbcl  =  inst_dbcl &&lisa_dbgcall_en && !fetch_ex;  // no fetch ex

assign ex_ipe  =  (inst_privilege && user_mode )
                  ||inst_rdtime&& lisa_rdtime_dis  // only for lisa
                  ||inst_dbcl &&!lisa_dbgcall_en;  // only for lisa

assign ex_ri  =  inst_reserved && !ex_ipe;

assign ls_ade_nalign  =  ( inst_ld_st_h &   vaddr[0]   )
                         |( inst_ld_st_w & (|vaddr[1:0]));
assign ls_ade_priv    =  inst_ld_st_cps&&!inst_pref && ade_priv;
assign ls_addr_err    =  ls_ade_nalign||ade_priv;
assign ex_ls_ade      =  ls_ade_priv  &&!ex_ipe;
assign ex_ls_nalign   =  ls_ade_nalign&&!ex_ipe;
`ifdef CONFIG_MMU
assign ade_excode     =  ex_ls_ade ? 6'd8 : 6'd9;
assign exe_excode  =  ( {6{ex_break|ex_dbcl }} &    6'h0c   )
                      |( {6{ex_syscall       }} &    6'h0b   )
                      |( {6{de_ex_lserr      }} & ade_excode)
                      |( {6{ex_ipe           }} &    6'h0e   )
                      |( {6{ex_ri            }} &    6'h0d   );
`else
assign ade_excode     =  ex_ls_ade ? 5'd8 : 5'd9;
assign exe_excode  =  ( {5{ex_break|ex_dbcl }} &    5'hc   )
                      |( {5{ex_syscall       }} &    5'hb   )
                      |( {5{de_ex_lserr      }} & ade_excode)
                      |( {5{ex_ipe           }} &    5'he   )
                      |( {5{ex_ri            }} &    5'hd   );
`endif

assign de_ex_canc_ls  =  fetch_ex || de_ex_lserr || de_ex_dbgss;
assign de_ex          =  ex_break | ex_syscall | ex_ipe | ex_ri | fetch_ex | ex_dbcl
                         |( de_ex_lserr  // while r0r1_stall is valid,
                         & ~r0r1_stall  // and de_ex_lserr can't be set;
                           );
// ex_ipe > ex_ade
assign de_ex_lserr    =  ex_ls_ade || ex_ls_nalign;
`ifdef CONFIG_MMU
assign de_excode      =  fetch_ex ? fetch_excode : exe_excode;
`else
assign de_excode      =  fetch_ex ? fetch_ade ? 5'h8 : 5'h1a
                         : exe_excode;
`endif
assign de_sub_excode  =  fetch_ex    ? 2'd0  :
                           ex_ls_ade   ? 2'd1  : 2'b0;
// debug ex
assign de_ex_dbcl  =  ex_dbcl ;
assign debug_ex    =  de_ex_dbcl  || de_ex_dbgss;
// End Exceptions

//------------------------------------------------------------//
//                      Jump And Branch
//------------------------------------------------------------//
// LoongISA branch resolution.
assign lisa_jbr_taken   =  lisa_jr || lisa_b
                           || lisa_br && (lisa_br_lt  && l_br_lt
                           ||lisa_br_ge  && l_br_ge
                           ||lisa_br_eq  && l_br_eq
                           ||lisa_br_ne  && l_br_ne);
assign jbr_taken        =  lisa_jbr_taken;
assign jbr_valid        =  valid&&inst_jbr;
assign jbr_stall        =  r0r1_stall;
assign jbr_taken_valid  =  jbr_taken & ~fetch_ex & jbr_valid;
assign de_stall_ft      =  valid && inst_idle
                           ||mem_nmi_ccerr;  // nmi/cacheerr stall fetch request
assign de_stall_npc     =  valid && jbr_stall && inst_jbr;
assign de_jbr_last      =  idu_lsu_inst_valid || idu_fpu_inst_valid || idu_exu_valid;

assign jbr_cancel_nxt  =  pred_cancel || jbr_cancel_flag;
always @ ( posedge clk_g )
begin
    if( reset ) begin
        jbr_cancel_flag <= 1'b0;
    end
    else if( ex_cancel ) begin
        jbr_cancel_flag <= 1'b0;
    end
    else if(idu_ifu_allowin&&ifu_idu_inst_valid) begin
        jbr_cancel_flag <= 1'b0;
    end
    else if(de2nxt_inst_cmplt&&pred_cancel) begin
        jbr_cancel_flag <= 1'b1;
    end
end

assign mode_jbr_src1   =  lisa_jr   ? r0_value : true_now_pc;
assign add_32_28_src1  =  mode_jbr_src1 & {32{inst_jbr}};  // include lisa_jr
assign br_imm_lowp_m   =  br_imm        & {28{inst_jbr}};
assign add_32_28_src2  =  {{4{br_imm_lowp_m[27]}},br_imm_lowp_m};
assign add_32_28_res   =  add_32_28_src1 + add_32_28_src2;

assign br_target  =  add_32_28_res;

assign jbr_target  =  {32{inst_jbr}} & br_target;  // lisa_jr is in inst_jbr

// while de-stage is valid, the next_pc following mem is in de_pc or de_pc-4(for branch delay)
// while de-stage is not valid, the next_pc following mem is in fetch_pc or de_pc (for branch delay)
assign idu_lsu_next_pc  =  valid ? true_now_pc : nxt_inst_pc;

// use idu_lsu_next_bd/idu_lsu_next_pc for NMI/CacheErr
assign de_pc  =  idu_lsu_next_pc;

assign de_brpc  =  true_now_pc;
// End Jump And Branch

//------------------------------------------------------------//
//                            ALU
//------------------------------------------------------------//

// select result;
// ----------------------------------------------------------------------------------------//
// csr_result-|                                                                            //
// lui_result-|-->temp1-|                                                                  //
//           mov_result |                                                                  //
//            rt_result |                                                                  //
//         revbh_result-|-->temp2-|                                                        //
// |                                                        //
//                    extw_result-|-->temp3-|                                              //
// |                                              //
//                           andlink_result |                                              //
//                             logic_result |                                              //
//                               add_result-|-->temp4-|                                    //
// |                                    //
//                                        clzo_result-|                                    //
//                                         cmp_result-|-->temp5-|                          //
// |                          //
//                                           ld_st_cps_addr_res-|-->temp6-|                //
// |                //
//                                                     shf_ext_ins_result-|-->result_final //
// //
// wire   result_temp1_sel;
// assign result_temp1_sel = csr_result_sel | lui_result_sel;
assign result_temp1  =  (/*{32{csr_result_sel}} &*/ csr_result   )  // csr_result=0,if sel==0
                        |(/*{32{lui_result_sel}} &*/ lui_result   );  // lui_result=0,if sel==0
// wire   result_temp2_sel;
// assign result_temp2_sel = result_temp1_sel | mov_result_sel | rt_result_sel | revbh_result_sel;
assign result_temp2  =  (/*{32{result_temp1_sel}} &*/ result_temp1 )  // result_temp1=0, if sel==0
                        |(/*{32{mov_result_sel  }} &*/ mov_result   )  // mov_reult=0   , if sel==0
                        |(  {32{maskz_result_sel     }} & maskz_result    )
                        |(  {32{bytepick_result_sel  }} & align_result    )
                        |(  {32{revbh_result_sel     }} & revbh_result    )
                        |(  {32{bitrev_4b_result_sel }} & bitrev_4b_result)
                        |(  {32{bitrev_w_result_sel  }} & bitrev_w_result );
// wire   result_temp3_sel;
// assign result_temp3_sel = extw_result_sel | result_temp2_sel;
assign result_temp3  =  (  {32{extw_result_sel }} &   extw_result  )
                        |(/*{32{result_temp2_sel}} &*/ result_temp2 );  // temp2=0, if sel=0
// wire   result_temp4_sel;
// assign result_temp4_sel = result_temp3_sel|andlink_result_sel|logic_result_sel|add_result_sel;
assign result_temp4  =  (/*{32{result_temp3_sel  }} &*/ result_temp3   )  // temp3=0, if sel=0
                        |(  {32{andlink_result_sel}} &   andlink_result )
                        |(/*{32{logic_result_sel  }} &*/ logic_result   )  // logic=0, if sel=0
                        |(  {32{add_result_sel    }} &   add_result     );
// wire   result_temp5_sel;
// assign result_temp5_sel = result_temp4_sel | clzo_result_sel | cmp_result_sel;
assign result_temp5  =  (/*{32{result_temp4_sel  }} &*/ result_temp4   )  // temp4=0, if sel=0
                        |(/*{32{clzo_result_sel   }} &*/ cltzo_result    )  // clzo=0, if sel=0
                        |(/*{32{cmp_result_sel    }} &*/ cmp_result     );  // cmp=0, if sel=0
// wire   result_temp6_sel;
// assign result_temp6_sel = result_temp5_sel | ld_st_cps_result_sel;
assign result_temp6  =  (/*{32{result_temp5_sel    }} &*/ result_temp5   )  // temp5=0, if sel=0
                        |( {32{ld_st_cps_result_sel}} & ld_st_cps_addr_res);

assign result_final  =  (/* {32{result_temp6_sel      }} &*/ result_temp6      )  // temp6=0, if sel=0
                        |(/* {32{shf_ext_ins_result_sel}} &*/ shf_ext_ins_result);  //=0, if sel=0

assign mov_result  =  {32{inst_mov_r0  }} & r0_value     |
                        {32{inst_mov_all1}} & 32'hffff_ffff;

assign rt_result  =  r1_value;

// cpo instruction
assign csr_result[31:25]  =  {inst_idle,inst_ertn,5'b0};
assign csr_result[24: 5]  =  20'd0;
assign csr_result[ 4: 0]  =  5'b0;

assign link_off        =  3'd4;
assign link_off_lm     =  link_off & {3{inst_jbr_andlink}};
assign link_pc         =  true_now_pc;
assign andlink_result  =  link_pc + link_off_lm;

assign extw_data           =  r0_value[15:0];
assign extw_result[7:0]    =  extw_data[7:0];
assign extw_result[15:8]   =  inst_extw_h ? extw_data[15:8]
                              : {8 {extw_data[7 ]}};
assign extw_result[31:16]  =  inst_extw_h ? {16{extw_data[15]}}
                              : {16{extw_data[7 ]}};

// REVB 2H
assign revbh_result  =  {r0_value[23:16],r0_value[31:24],r0_value[ 7: 0],r0_value[15: 8]};
// BITREV 4B
generate
    for (i = 0; i < 8; i = i + 1) begin: gen_bitswap_res
        assign bitswap_result_temp[i]    = r0_value[7-i];
        assign bitswap_result_temp[i+8]  = r0_value[15-i];
        assign bitswap_result_temp[i+16] = r0_value[23-i];
        assign bitswap_result_temp[i+24] = r0_value[31-i];
    end
endgenerate
assign bitrev_4b_result = bitswap_result_temp;

// BITREV W
generate
    for (i = 0; i < 32; i = i + 1) begin: gen_bitrev_res
        assign bitrev_w_result_temp[i] = r0_value[31-i];
    end
endgenerate
assign bitrev_w_result = bitrev_w_result_temp;

//------------------------------------------------------------//
//                           ALIGN
//------------------------------------------------------------//
assign align_rotr_sa  =  lisa_sa2;
assign align_rotr_0   =  inst_bytepick && align_rotr_sa == 2'b00;
assign align_rotr_1   =  inst_bytepick && align_rotr_sa == 2'b01;
assign align_rotr_2   =  inst_bytepick && align_rotr_sa == 2'b10;
assign align_rotr_3   =  inst_bytepick && align_rotr_sa == 2'b11;
assign align_result   =  {32{align_rotr_0}} & {r1_value}
                         |{32{align_rotr_1}} & {r1_value[23:0],r0_value[31:24]}
                         |{32{align_rotr_2}} & {r1_value[15:0],r0_value[31:16]}
                         |{32{align_rotr_3}} & {r1_value[7 :0],r0_value[31:8]};
// End ALIGN

// maskz
assign maskz_result  =  inst_maskz_eq^(|r1_value) ? 32'd0 : r0_value;

// LUI
assign lui_result  =  {{20{inst_lui}} & lui_imm,12'b0};

assign add_r0_shf                  =  (r0_value&{32{add_lowp_m}}) << ((lisa_sa2+1'b1)&{3{inst_add_shf}});
assign add_lowp_m                  =  inst_sub_trap_slt || inst_add;
assign add_r0_data                 =  inst_add_pc ? {de_brpc[31:2],2'd0} : add_r0_shf;
assign add_srcdata1                =  {add_r0_data[31],add_r0_data};
assign add_sel_imm                 =  inst_imm_signext ;
assign add_r1_imm                  =  add_sel_imm ?  arth_imm : r1_value&{32{add_lowp_m}};
assign add_srcdata2[32]            =  add_srcdata2[31];
assign add_srcdata2[31:0]          =  ({32{inst_sub_trap_slt}} ^ add_r1_imm);
assign {add_cout,add_result_temp}  =  add_srcdata1 + add_srcdata2+inst_sub_trap_slt;
assign add_result                  =  {add_result_temp[31:12],add_result_temp[11:0] & {12{!inst_add_m12}}};
assign cmp_lt                      =  add_cout[0];
assign cmp_geu                     =  add_cout[1];
assign cmp_ne                      =  add_srcdata1[31:0]!=add_r1_imm;  // |add_result;
assign cmp_result[31:1]            =  30'd0;
assign cmp_result[0]               =  ( inst_slt_s &  cmp_lt )
                                      |( inst_slt_u & ~cmp_geu);

assign logic_srcdata2  =  inst_imm_zeroext ? {20'h0000, logic_imm} :
                            inst_r1_not      ? ~r1_value : r1_value;
assign and_result      =  r0_value & logic_srcdata2;
assign or_result       =  r0_value | logic_srcdata2;
assign nor_result      =  ~or_result;
assign xor_result      =  r0_value ^ logic_srcdata2;
assign logic_result    =  ( {32{inst_and}} & and_result ) | ( {32{inst_or}} & or_result )
                          |( {32{inst_nor}} & nor_result ) | ( {32{inst_xor}} & xor_result);

//------------------------------------------------------------//
//                           Shift
//------------------------------------------------------------//
assign mode_shf_data  =  r0_value;
assign mode_shf_sa_v  =  r1_value[4:0];
generate
    for (i = 0; i < 32; i = i + 1) begin: gen_sll_data
        assign mode_shf_data_rev[i] = mode_shf_data[31-i];
    end
endgenerate
assign sll_or_ins  =  inst_sll | inst_ins;
assign shift_data  =  {32{inst_sr ||inst_rotr||inst_ext}} & mode_shf_data    |
                        {32{sll_or_ins                   }} & mode_shf_data_rev;
// reverse bits for inst_sll/inst_ins

assign shift_data_ext  =  {32{inst_sr_a & mode_shf_data[31]}}
                          | {{32{inst_rotr}}&mode_shf_data     };

assign shift_sa  =  {5{inst_shiftv                }} & mode_shf_sa_v|
                      {5{inst_shift_sa||inst_ext_ins}} & shf_sa;

assign shift_result_temp_t  =  {shift_data_ext,shift_data}>>shift_sa;
// sll/ins result need to reverse
generate
    for (i = 0; i < 32; i = i + 1) begin: gen_shift_result
        assign shift_result_temp[i] = sll_or_ins ? shift_result_temp_t[31-i]
                                                : shift_result_temp_t[i];
    end
endgenerate

// End Shift

assign ext_ins_lm     =  msb_dec;
assign lisa_inst_ext  =  l_BSTRPICK_W;
assign lisa_ext_msbd  =  (inst[20:16]&{5{lisa_inst_ext}})-(inst[14:10]&{5{lisa_inst_ext}});
exu_dec_lmask_5_32 x_exu_dec_lmask_5_32_1 (
    .in  (in_lmask_dec1 ),
    .out (out_lmask_dec1)
);

assign ins_mask  =  ext_ins_lm & lsb_dec;

assign shf_ext_ins_r1_mask   =  {32{inst_ins  }} & ~ins_mask    ;
assign shf_ext_ins_shf_mask  =  {32{inst_ins            }} & ins_mask     |
                                  lisa_ext_lm   |
                                  {32{inst_shift          }} & 32'hffffffff ;
assign shf_ext_ins_result    =  shf_ext_ins_r1_mask  & r1_value         |
                                  shf_ext_ins_shf_mask & shift_result_temp;

//------------------------------------------------------------//
//                          CLO CLZ
//------------------------------------------------------------//
assign r0_value_reverse  =  bitrev_w_result_temp;
assign clo_data          =  {32{inst_clz}} & ~r0_value |
                            {32{inst_clo}} &  r0_value |
                            {32{inst_ctz}} & ~r0_value_reverse |
                            {32{inst_cto}} &  r0_value_reverse ;

exu_count1_4_3 x_exu_count1_4_3_0 (
    .in  (in_count1_lane0 ),
    .out (out_count1_lane0)
);
exu_count1_4_3 x_exu_count1_4_3_1 (
    .in  (in_count1_lane1 ),
    .out (out_count1_lane1)
);
exu_count1_4_3 x_exu_count1_4_3_2 (
    .in  (in_count1_lane2 ),
    .out (out_count1_lane2)
);
exu_count1_4_3 x_exu_count1_4_3_3 (
    .in  (in_count1_lane3 ),
    .out (out_count1_lane3)
);
exu_count1_4_3 x_exu_count1_4_3_4 (
    .in  (in_count1_lane4 ),
    .out (out_count1_lane4)
);
exu_count1_4_3 x_exu_count1_4_3_5 (
    .in  (in_count1_lane5 ),
    .out (out_count1_lane5)
);
exu_count1_4_3 x_exu_count1_4_3_6 (
    .in  (in_count1_lane6 ),
    .out (out_count1_lane6)
);
exu_count1_4_3 x_exu_count1_4_3_7 (
    .in  (in_count1_lane7 ),
    .out (out_count1_lane7)
);

exu_count1_4_3 x_exu_count1_4_3_8 (
    .in  (in_count1_lane8 ),
    .out (out_count1_lane8)
);
exu_count1_4_3 x_exu_count1_4_3_9 (
    .in  (in_count1_lane9 ),
    .out (out_count1_lane9)
);

assign cltzo_result[31:6]  =  26'd0;
assign cltzo_result[5]     =  step2_0[0] & step2_1[0];
assign cltzo_result[4]     =  step2_0[0] & ~step2_1[0];
assign cltzo_result[3]     =  (~step2_0[0] & step2_0[2])
                              |( step2_0[0] & ~step2_1[0] & step2_1[2]);
assign cltzo_result[2]     =  (~step2_0[0] & step2_0[1])
                              |( step2_0[0] & ~step2_1[0] & step2_1[1]);
assign cltzo_result[1]     =  (~step2_0[0] & ~step2_0[2] & ~step2_0[1] & step1_0[2])
                              |(~step2_0[0] & ~step2_0[2] &  step2_0[1] & step1_1[2])
                              |(~step2_0[0] &  step2_0[2] & ~step2_0[1] & step1_2[2])
                              |(~step2_0[0] &  step2_0[2] &  step2_0[1] & step1_3[2])
                              |( step2_0[0] & ~step2_1[0] & ~step2_1[2] & ~step2_1[1] & step1_4[2])
                              |( step2_0[0] & ~step2_1[0] & ~step2_1[2] &  step2_1[1] & step1_5[2])
                              |( step2_0[0] & ~step2_1[0] &  step2_1[2] & ~step2_1[1] & step1_6[2])
                              |( step2_0[0] & ~step2_1[0] &  step2_1[2] &  step2_1[1] & step1_7[2]);
assign cltzo_result[0]     =  (~step2_0[0] & ~step2_0[2] & ~step2_0[1] & step1_0[1])
                              |(~step2_0[0] & ~step2_0[2] &  step2_0[1] & step1_1[1])
                              |(~step2_0[0] &  step2_0[2] & ~step2_0[1] & step1_2[1])
                              |(~step2_0[0] &  step2_0[2] &  step2_0[1] & step1_3[1])
                              |( step2_0[0] & ~step2_1[0] & ~step2_1[2] & ~step2_1[1] & step1_4[1])
                              |( step2_0[0] & ~step2_1[0] & ~step2_1[2] &  step2_1[1] & step1_5[1])
                              |( step2_0[0] & ~step2_1[0] &  step2_1[2] & ~step2_1[1] & step1_6[1])
                              |( step2_0[0] & ~step2_1[0] &  step2_1[2] &  step2_1[1] & step1_7[1]);
// End CLO CLZ CTO CTZ

// End ALU

//------------------------------------------------------------//
//                  Decode-Stage Performance
//------------------------------------------------------------//
assign de_perf_go  =  valid&&de2nxt_inst_cmplt&&!mem_roll&&!ex_cancel
                      &&!de_ex&&!debug_ex&&(!mem_see_int||inst_split_2nd)&&!mem_nmi_ccerr;
always @ ( posedge clk_g )
begin
    inst_whichbus_r <= {2{de_perf_go}} & inst_whichbus;
    inst_nop_r      <= 1'b0;
end

assign perf_inst_ibus   =  inst_whichbus_r[1];
assign perf_inst_dbus   =  inst_whichbus_r[0];
assign perf_inst_sbus   =  inst_whichbus_r==2'b00;
assign perf_load_stall  =  (mem_stall_r0 || mem_stall_r1)&&valid;
assign perf_nop         =  inst_nop_r;
// End Decode-Stage Performance

//------------------------------------------------------------//
//                           LJTAG
//------------------------------------------------------------//
`ifdef OPEN132_HAS_LJTAG

assign {pesudoint_go,
        mem_ex_go   ,
        dertn_go    ,
        debug_sse   ,
        debug_mode  ,
        mem_noex    ,
        asid
       }  =  lsu_idu_debug_bus;
assign dbgm  =  (vaddr[31:20]==12'hdb0)&& debug_mode;
//------------------------------------------------------------//
//                        Single Step
//------------------------------------------------------------//
always @ ( posedge clk_g )
begin
    if(dertn_go || pesudoint_go || !debug_sse) begin
        ss_flag <= 1'b0;
    end
    else if( !debug_mode && !ex_cancel && (de2nxt_inst_cmplt||mem_ex_go) ) begin
        ss_flag <= 1'b1;
    end
end
assign de_ex_dbgss  =  ss_flag && debug_sse && !debug_mode && !jbr_cancel_flag;
// End Single Step

//------------------------------------------------------------//
//              Instruction Hardware Breakpoint
//------------------------------------------------------------//
assign de_ihb_signal  =  ihb_signal;
assign de_ihb_debug   =  ihb_debug;

generate
    if (IHB_NUM > 0) begin: gen_has_ihb
        wire ibs_can_set = !debug_mode && de2nxt_inst_cmplt &&
                           mem_noex && !ex_cancel &&
                           (!lisa_ihb_skip || skip_trigged);
        wire [31:0] ihb_pc;

        assign ihb_pc[31:1] = true_now_pc[31:1] & {31{ihb_te_be}};
        assign ihb_pc[0]    = true_now_pc[0] & ihb_te_be;

        ljtag_ihb #(
            .IHB_NUM (IHB_NUM)
        ) x_ljtag_ihb (
            .clk_g                  (clk_g                 ),
            .clk_dm                 (clk_dm                ),
            .sync_hresetn           (sync_hresetn          ),
            .data_reset             (data_reset            ),
            .jtag_idu_ihb_req_bus   (jtag_idu_ihb_req_bus  ),
            .idu_jtag_ihb_resp_bus  (idu_jtag_ihb_resp_bus ),
            .debug_mode             (debug_mode            ),
            .pc_m                   (ihb_pc                ),
            .asid                   (asid                  ),
            .ibs_can_set            (ibs_can_set           ),
            .ihb_te_be              (ihb_te_be             ),
            .ihb_signal             (ihb_signal            ),
            .ihb_debug              (ihb_debug             )
        );

        reg skip_trigged_flag;
        always @ (posedge clk_g) begin
            if (reset || ex_cancel) begin
                skip_trigged_flag <= 1'b0;
            end
            else if (!debug_mode && de2nxt_inst_cmplt && ihb_signal) begin
                skip_trigged_flag <= 1'b1;
            end
        end
        assign skip_trigged = skip_trigged_flag;
    end else begin: gen_no_ihb
        assign ihb_signal             = 1'b0;
        assign ihb_debug              = 1'b0;
        assign idu_jtag_ihb_resp_bus  = 32'd0;
        assign skip_trigged           = 1'b0;
        assign ihb_te_be              = 1'b0;
    end
endgenerate
// End Instruction Hardware Breakpoint
`else
assign dbgm  =  1'b0;
// single-step
assign de_ex_dbgss  =  1'b0;
// ihb
assign de_set_bs      =  1'b0;
assign de_ihb_signal  =  1'b0;
assign de_ihb_debug   =  1'b0;
assign skip_trigged   =  1'b0;
`endif
// End LJTAG

//------------------------------------------------------------//
//                            FPU
//------------------------------------------------------------//
// no_fpu
// branch
assign l_BCEQZ  =  1'b0;
assign l_BCNEZ  =  1'b0;
// Arithemetic
assign l_FADD_S       =  1'b0;
assign l_FSUB_S       =  1'b0;
assign l_FMUL_S       =  1'b0;
assign l_FDIV_S       =  1'b0;
assign l_FMAX_S       =  1'b0;
assign l_FMIN_S       =  1'b0;
assign l_FMAXA_S      =  1'b0;
assign l_FMINA_S      =  1'b0;
assign l_FABS_S       =  1'b0;
assign l_FNEG_S       =  1'b0;
assign l_FCOPYSIGN_S  =  1'b0;
assign l_FSQRT_S      =  1'b0;
assign l_FRECIP_S     =  1'b0;
assign l_FRSQRT_S     =  1'b0;
assign l_FMADD_S      =  1'b0;
assign l_FMSUB_S      =  1'b0;
assign l_FNMADD_S     =  1'b0;
assign l_FNMSUB_S     =  1'b0;
// Compare
assign l_FCMP_COND_S  =  1'b0;
assign l_FCLASS_S     =  1'b0;
// Convert
assign l_FTINTRM_W_S   =  1'b0;
assign l_FTINTRP_W_S   =  1'b0;
assign l_FTINTRZ_W_S   =  1'b0;
assign l_FTINTRNE_W_S  =  1'b0;
assign l_FTINT_W_S     =  1'b0;
assign l_FFINT_S_W     =  1'b0;
// Load/Store
assign l_FLD_S   =  1'b0;
assign l_FST_S   =  1'b0;
assign l_FLDX_S  =  1'b0;
assign l_FSTX_S  =  1'b0;
// Move
assign l_FMOV_S      =  1'b0;
assign l_FSEL        =  1'b0;
assign l_MOVGR2FR_W  =  1'b0;
assign l_MOVFR2GR_S  =  1'b0;
assign l_MOVGR2FCSR  =  1'b0;
assign l_MOVFCSR2GR  =  1'b0;
assign l_MOVFR2CF    =  1'b0;
assign l_MOVCF2FR    =  1'b0;
assign l_MOVGR2CF    =  1'b0;
assign l_MOVCF2GR    =  1'b0;

// pipeline control
assign mm_stall_main  =  1'b0;  // main/mdu stall main

assign clk_fpr_nc          =  clk_fpr;
assign main_mdu_is_run_nc  =  lsu_idu_mdu_busy;
assign fpu_ex_enable_nc    =  fpu_idu_exception_enable;
assign fpu2dec_bus_nc      =  fpu_idu_result_bus;
assign fcrs2dec_rdata_nc   =  fpu_idu_fcsr_read_data;
// output
assign dec_is_fpu                =  1'b0;
assign idu_fpu_inst_bus          =  {`OPEN132_IDU_FPU_INST_BUS_WIDTH{1'b0}};
assign idu_fpu_fcsr_read_enable  =  4'd0;
// End FPU

assign idu_ifu_allowin  =  de_allowin;
//------------------------------------------------------------//
//                        lisa_decode
//------------------------------------------------------------//

dsu_inst_dec x_dsu_inst_dec (
    .inst                 (inst                ),
    .lisa_inst_decode     (lisa_inst_decode    ),
    .lisa_fpu_inst_decode (lisa_fpu_inst_decode)
);

assign {
   l_ADD_W          ,
   l_ADDI_W         ,
   l_ADDU12I_W      ,
   l_ALSL_W         ,
   l_SUB_W          ,
   l_PCADDI         ,
   l_PCALAU12I      ,
   l_PCADDU12I      ,
   l_SLT            ,
   l_SLTU           ,
   l_SLTI           ,
   l_SLTUI          ,
   l_CLO_W          ,
   l_CLZ_W          ,
   l_CTO_W          ,
   l_CTZ_W          ,
   l_EXT_W_H        ,
   l_EXT_W_B        ,
   l_LU12I_W        ,
   l_AND            ,
   l_OR             ,
   l_XOR            ,
   l_ANDI           ,
   l_ORI            ,
   l_XORI           ,
   l_ANDN           ,
   l_ORN            ,
   l_NOR            ,
   l_SLL_W          ,
   l_SRL_W          ,
   l_SRA_W          ,
   l_ROTR_W         ,
   l_SLLI_W         ,
   l_SRLI_W         ,
   l_SRAI_W         ,
   l_ROTRI_W        ,
   l_BSTRINS_W      ,
   l_BSTRPICK_W     ,
   l_REVB_2H        ,
   l_BITREV_4B      ,
   l_BITREV_W       ,
   l_MASKEQZ        ,
   l_MASKNEZ        ,
   l_BYTEPICK_W     ,
   l_MUL_W          ,
   l_MULH_W         ,
   l_MULH_WU        ,
   l_DIV_W          ,
   l_MOD_W          ,
   l_DIV_WU         ,
   l_MOD_WU         ,
   l_BREAK          ,
   l_DBGCALL        ,
   l_SYSCALL        ,
   l_RDTIMEL_W      ,
   l_RDTIMEH_W      ,
   l_CPUCFG         ,
   l_CSRRD          ,
   l_CSRWR          ,
   l_CSRXCHG        ,
   l_ERTN           ,
   l_IDLE           ,
   l_CACOP          ,
   l_LL_W           ,
   l_SC_W           ,
   l_LD_B           ,
   l_LD_H           ,
   l_LD_W           ,
   l_LD_BU          ,
   l_LD_HU          ,
   l_ST_B           ,
   l_ST_H           ,
   l_ST_W           ,
   l_PRELD          ,
   l_LDL_W          ,
   l_LDR_W          ,
   l_STL_W          ,
   l_STR_W          ,
   l_DBAR           ,
   l_IBAR           ,
   l_BEQZ           ,
   l_BNEZ           ,
   l_BEQ            ,
   l_BNE            ,
   l_BLT            ,
   l_BGE            ,
   l_BLTU           ,
   l_BGEU           ,
   l_B              ,
   l_BL             ,
   l_JIRL
  }  =  lisa_inst_decode;

// End lisa_decode

`ifndef OPEN132_APP_SPEC
assign idu_lsu_app_bus  =  {`OPEN132_IDU_LSU_APP_BUS_WIDTH{1'b0}};
`endif
// End Application-Specific Logic
endmodule
