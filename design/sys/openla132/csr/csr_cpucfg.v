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

module csr_cpucfg #(
    // ICache
    parameter HAS_ICC       = -1,  // 0 or 1
    parameter ICC_SET_ASSOC = -1,  // 1/2/4 way set associative
    parameter ICC_WAYSIZE   = -1,  // 1/2/4/8/16 KB Size/way
    // DCache
    parameter HAS_DCC       = -1,  // 0 or 1
    parameter DCC_SET_ASSOC = -1,  // 1/2/4 way set associative
    parameter DCC_WAYSIZE   = -1,  // 1/2/4/8/16 KB Size/way
    parameter PERF_CNT_NUM  = -1   // 0, 1 or 2
) (
    input  wire [  3: 0]  prid_revision,
    input  wire           inst_cpucfg,
    input  wire [ 31: 0]  cpucfg_addr,

    output wire [ 31: 0]  cpucfg_result
);

//============================================================//
//                     Wire Declarations
//============================================================//

wire [ 31: 0] cpucfg0;   // prid
wire [ 31: 0] cpucfg1;   // feature
wire [ 31: 0] cpucfg2;   // unit-description
wire [ 31: 0] cpucfg3;   // function
`ifdef CONFIG_MMU
wire [ 31: 0] cpucfg4;   // stable-counter base frequency
wire [ 31: 0] cpucfg5;   // stable-counter multiplier/divider
`endif
wire [ 31: 0] cpucfg6;   // user_perfcnt
wire [ 31: 0] cpucfg10;  // Cache-summary
wire [ 31: 0] cpucfg11;  // 1st Cache, L1 I$
wire [ 31: 0] cpucfg12;  // 2st Cache, L1 D$

wire          cpucfg_addr_valid;
wire          cpucfg0_ren;
wire          cpucfg1_ren;
wire          cpucfg2_ren;
wire          cpucfg3_ren;
`ifdef CONFIG_MMU
wire          cpucfg4_ren;
wire          cpucfg5_ren;
`endif
wire          cpucfg6_ren;
wire          cpucfg10_ren;
wire          cpucfg11_ren;
wire          cpucfg12_ren;

wire          cfg6_lpmp;    // has performance count
wire [  3: 0] cfg6_pmnum;   // perfcnt_num - 1
wire [  5: 0] cfg6_pmbits;  // perfcnt_bits- 1, 31 or 0

wire          cfg10_L1_I_P;
wire          cfg10_L1_D_P;

wire [ 31: 0] cfg11_I_WAY32;
wire [ 31: 0] cfg11_I_IDX32;
wire [ 31: 0] cfg11_I_LS32;   // cacheline size= 2^5 bytes

wire [ 31: 0] cfg12_D_WAY32;
wire [ 31: 0] cfg12_D_IDX32;
wire [ 31: 0] cfg12_D_LS32;   // cacheline size= 2^5 bytes

//============================================================//
//                    Module Implementation
//============================================================//

//------------------------------------------------------------//
//                          Cpucfgs
//------------------------------------------------------------//
// End Cpucfgs

//------------------------------------------------------------//
//                        Cpucfgs Read
//------------------------------------------------------------//
assign cpucfg_addr_valid  =  cpucfg_addr[31:5] == 27'd0;
assign cpucfg0_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h0;
assign cpucfg1_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h1;
assign cpucfg2_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h2;
assign cpucfg3_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h3;
`ifdef CONFIG_MMU
assign cpucfg4_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h4;
assign cpucfg5_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h5;
`endif
assign cpucfg6_ren        =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h6;
assign cpucfg10_ren       =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h10;
assign cpucfg11_ren       =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h11;
assign cpucfg12_ren       =  inst_cpucfg && cpucfg_addr_valid && cpucfg_addr[4:0] == 5'h12;

assign cpucfg_result  =  {32{cpucfg0_ren}} & cpucfg0 | {32{cpucfg1_ren}} & cpucfg1 | {32{cpucfg2_ren}} & cpucfg2 |
                           {32{cpucfg3_ren}} & cpucfg3 |
`ifdef CONFIG_MMU
                           {32{cpucfg4_ren}} & cpucfg4 | {32{cpucfg5_ren}} & cpucfg5 |
`endif
                           {32{cpucfg6_ren}} & cpucfg6 | {32{cpucfg10_ren}} & cpucfg10 |
                           {32{cpucfg11_ren}} & cpucfg11 | {32{cpucfg12_ren}} & cpucfg12;
// End Cpucfgs Read

//------------------------------------------------------------//
//                       cpucfg0: PRID
//------------------------------------------------------------//
`ifdef CONFIG_MMU
// Company 0x14, LA132 series 0x8, product 0x010, low-nibble revision.
assign cpucfg0                =  32'h0014_8010 | {28'd0, prid_revision};
`else
assign cpucfg0  =  32'h004F3100 | {28'd0, prid_revision};
`endif
// End cpucfg0: PRID

//------------------------------------------------------------//
//                      cpucfg1: Feature
//------------------------------------------------------------//
assign cpucfg1  =  {
                     5'd0,  // 31:27, reserved
                     1'b0,  // 26   , Int_impl is line
                     1'b0,  // 25   , no IOCSR_BRD
                     1'b0,  // 24   , no TLB HugePage
                     1'b0,  // 23   , no TLB RPLV
                     1'b0,  // 22   , no TLB XI
                     1'b0,  // 21   , no TLB RI
                     1'b0,  // 20   , no Load/Store unagligned
                     8'h1f,  // 19:12, VABITS is 31+1
                     8'h1f,  // 11:4 , PABITS is 31+1
                     1'b0,  // 3    , no IOCSR
`ifdef CONFIG_MMU
                     1'b1,  // 2    , has paging MMU
`else
                     1'b0,  // 2    , no PG-MMU
`endif
                     1'b0,  // 1    , is_GR64=0
                     1'b1  // 0    , is_GR32=1
                   };
// End cpucfg1: Feature

//------------------------------------------------------------//
//                 cpucfg2: Unit Description
//------------------------------------------------------------//
assign cpucfg2  =  {
                     7'd0,  // 31:25, reserved
`ifdef CONFIG_MMU
                     1'b1,  // 24   , has hardware page-table walker
`else
                     1'b0,  // 24   , no TLB, so LHPW=0
`endif
                     1'b0,  // 23   , no FP_HP
                     1'b0,  // 22   , no LAMO
                     1'b0,  // 21   , no software page-walk instructions
                     1'b0,  // 20   , no MIPSBT
                     1'b0,  // 19   , no ARMBT
                     1'b0,  // 18   , no X86BT
`ifdef CONFIG_MMU
                     3'd1,  // 17:15, LLFTP version 1
                     1'b1,  // 14   , has stable counter and timer
`else
                     3'd0,  // 17:15, no LLFTP_Ver
                     1'b0,  // 14   , no LLFTP
`endif

                     3'd0,  // 13:11, no VZ_Ver
                     1'b0,  // 10   , no VZ
                     1'b0,  // 9    , no CRYPT
                     1'b0,  // 8    , no COMPLEX
                     1'b0,  // 7    , no LASX
                     1'b0,  // 6    , no LSX
                     3'd1,  // 5 :3 , FP_Rev=0x1
                     1'b0,  // 2    , no FP_DP
                     1'b0,  // 1    , no FP_SP
                     1'b0  // 0    , no FP
                   };
// End cpucfg2: Unit Description

//------------------------------------------------------------//
//                     cpucfg3: Function
//------------------------------------------------------------//
`ifdef CONFIG_MMU
assign cpucfg3                =  {
                     15'd0,  // 31:17, reserved
                     4'd0,  // 16:13, no ReduceVA, so MAX=0
`else
assign cpucfg3  =  {
                     15'd0,  // 31:25, reserved
                     1'b0,  // 16:13, no ReduceVA, so MAX=0
`endif
                     1'b0,  // 12   , no ReduceVA
                     1'b0,  // 11   , no LSPW huge-page helper
                     3'd0,  // 10:8 , no LSPW page-table level
                     1'b0,  // 7    , no ICacheT
                     1'b0,  // 6    , no ITLBT
                     1'b1,  // 5    , has LLSYNC
                     1'b0,  // 4    , no SCDLY
                     1'b0,  // 3    , no LLEXC
                     1'b0,  // 2    , no UcAcc
                     1'b0,  // 1    , no SFB
                     1'b0  // 0    , no CacheDMA
                   };
// End cpucfg3: Function

//------------------------------------------------------------//
`ifdef CONFIG_MMU
//               cpucfg4-cpucfg5: Counter Clock
//------------------------------------------------------------//
// The architectural counter advances once per 100 MHz core clock.
assign cpucfg4                =  32'd100_000_000;
assign cpucfg5                =  {16'd1, 16'd1};  // frequency = base * multiplier / divider
// End cpucfg4-cpucfg5: Counter Clock

//------------------------------------------------------------//
`endif
//                   cpucfg6: user_perfcnt
//------------------------------------------------------------//
assign cfg6_lpmp    =  PERF_CNT_NUM > 0;
assign cfg6_pmnum   =  PERF_CNT_NUM == 2 ? 4'h1 : 4'h0;
assign cfg6_pmbits  =  PERF_CNT_NUM > 0 ? 6'h1f : 6'h0;
assign cpucfg6      =  {
                         17'd0,  // 31:15, reserved
                         1'b0,  // 14   , no user_perfcnt
                         cfg6_pmbits,  // 13:8 , perfcnt bits   - 1
                         cfg6_pmnum,  // 7 :4 , perfcnt number - 1
                         3'd0,  // 3 :1 , perfcnt Ver
                         cfg6_lpmp  // 0    , has perfcnt
                       };
// End cpucfg6: user_perfcnt

//------------------------------------------------------------//
//                  cpucfg10: Cache Summary
//------------------------------------------------------------//
assign cfg10_L1_I_P  =  HAS_ICC ? 1'b1 : 1'b0;
assign cfg10_L1_D_P  =  HAS_DCC ? 1'b1 : 1'b0;
assign cpucfg10      =  {
                          28'd0,  // 31:4 , reserved, no L2/L3
                          1'b0,  // 3    , L1 D$ is not uniform
                          cfg10_L1_D_P,  // 2    , has L1 D$
                          1'b0,  // 1    , L1 I$ is not uniform
                          cfg10_L1_I_P  // 0    , has L1 I$
                        };
// End cpucfg10: Cache Summary

//------------------------------------------------------------//
//                      cpucfg11: L1 I$
//------------------------------------------------------------//
assign cfg11_I_WAY32  =  HAS_ICC == 0 ? 32'd0 : ICC_SET_ASSOC - 1;
assign cfg11_I_IDX32  =  HAS_ICC == 0 ? 32'd0 : `OPEN132_ICC_INDEX_BITS;
assign cfg11_I_LS32   =  HAS_ICC == 0 ? 32'd0 : 32'd5;
assign cpucfg11       =  {
                           1'b0,  // 31   , reserved
                           cfg11_I_LS32[6 : 0],  // 30:24, I$ $log2(cacheline-size)
                           cfg11_I_IDX32[7 : 0],  // 23:16, I$ $log2(Index)
                           cfg11_I_WAY32[15:0]  // 15:0 , I$ way - 1
                         };
// End cpucfg11: L1 I$

//------------------------------------------------------------//
//                      cpucfg12: L1 D$
//------------------------------------------------------------//
assign cfg12_D_WAY32  =  HAS_DCC == 0 ? 32'd0 : DCC_SET_ASSOC - 1;
assign cfg12_D_IDX32  =  HAS_DCC == 0 ? 32'd0 : `OPEN132_DCC_INDEX_BITS;
assign cfg12_D_LS32   =  HAS_DCC == 0 ? 32'd0 : 32'd5;
assign cpucfg12       =  {
                           1'b0,  // 31   , reserved
                           cfg12_D_LS32[6 : 0],  // 30:24, D$ $log2(cacheline-size)
                           cfg12_D_IDX32[7 : 0],  // 23:16, D$ $log2(Index)
                           cfg12_D_WAY32[15:0]  // 15:0 , D$ way - 1
                         };
// End cpucfg12: L1 D$
endmodule
