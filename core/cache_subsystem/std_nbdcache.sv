// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 13.10.2017
// Description: Nonblocking private L1 dcache
`include "common_cells/registers.svh"

module std_nbdcache import std_cache_pkg::*; import ariane_pkg::*; #(
    parameter ariane_cfg_t ArianeCfg        = ArianeDefaultConfig, // contains cacheable regions
    parameter int unsigned AXI_ADDR_WIDTH   = 0,
    parameter int unsigned AXI_DATA_WIDTH   = 0,
    parameter int unsigned AXI_ID_WIDTH     = 0,
    parameter type axi_req_t = ariane_axi::req_t,
    parameter type axi_rsp_t = ariane_axi::resp_t
)(
    input  logic                           clk_i,       // Clock
    input  logic                           rst_ni,      // Asynchronous reset active low
    // Cache management
    input  logic                           enable_i,    // from CSR
    input  logic [ariane_pkg::DCACHE_SET_ASSOC-1:0] dcache_spm_ways_i,
    input  logic                           flush_i,     // high until acknowledged
    output logic                           flush_ack_o, // send a single cycle acknowledge signal when the cache is flushed
    output logic                           miss_o,      // we missed on a LD/ST
    output logic                           busy_o,
    input  logic                           stall_i,   // stall new memory requests
    input  logic                           init_ni,
    // AMOs
    input  amo_req_t                       amo_req_i,
    output amo_resp_t                      amo_resp_o,
    // Request ports
    input  dcache_req_i_t [2:0]            req_ports_i,  // request ports
    output dcache_req_o_t [2:0]            req_ports_o,  // request ports
    // Cache AXI refill port
    output axi_req_t                       axi_data_o,
    input  axi_rsp_t                       axi_data_i,
    output axi_req_t                       axi_bypass_o,
    input  axi_rsp_t                       axi_bypass_i
);

import std_cache_pkg::*;

    // -------------------------------
    // Controller <-> Arbiter
    // -------------------------------
    // 1. Miss handler
    // 2. PTW
    // 3. Load Unit
    // 4. Store unit
    logic        [3:0][DCACHE_SET_ASSOC-1:0]  req;
    logic        [3:0][DCACHE_INDEX_WIDTH-1:0]addr;
    logic        [3:0]                        gnt;
    cache_line_t [DCACHE_SET_ASSOC-1:0]       rdata;
    logic        [3:0][DCACHE_TAG_WIDTH-1:0]  tag;

    cache_line_t [3:0]                        wdata;
    logic        [3:0]                        we;
    cl_be_t      [3:0]                        be;
    logic        [DCACHE_SET_ASSOC-1:0]       hit_way;
    // -------------------------------
    // Controller <-> Miss unit
    // -------------------------------
    logic [2:0]                        busy;
    logic [2:0][55:0]                  mshr_addr;
    logic [2:0]                        mshr_addr_matches;
    logic [2:0]                        mshr_index_matches;
    logic [63:0]                       critical_word;
    logic                              critical_word_valid;

    logic [2:0][$bits(miss_req_t)-1:0] miss_req;
    logic [2:0]                        miss_gnt;
    logic [2:0]                        active_serving;

    logic [2:0]                        bypass_gnt;
    logic [2:0]                        bypass_valid;
    logic [2:0][63:0]                  bypass_data;
    // -------------------------------
    // Arbiter <-> Datram,
    // -------------------------------
    logic [DCACHE_SET_ASSOC-1:0]         req_cache;
    logic [DCACHE_INDEX_WIDTH-1:0]       addr_cache;
    logic                                we_cache;
    cache_line_t                         wdata_cache;
    cache_line_t [DCACHE_SET_ASSOC-1:0]  rdata_cache;
    cl_be_t                              be_cache;
    vldrty_t [DCACHE_SET_ASSOC-1:0]      be_valid_dirty_ram;

    logic [DCACHE_SET_ASSOC-1:0]         req_spm;
    logic [DCACHE_INDEX_WIDTH-1:0]       addr_spm;
    logic                                we_spm;
    logic [(DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH)-1:0]    wdata_spm;
    logic [DCACHE_SET_ASSOC-1:0][(DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH)-1:0]  rdata_spm;
    logic [((DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH+7)/8)-1:0] be_spm;

    logic [DCACHE_SET_ASSOC-1:0]                                                    req_ram;
    logic [DCACHE_SET_ASSOC-1:0][DCACHE_INDEX_WIDTH-1:0]                            addr_ram;
    logic [DCACHE_SET_ASSOC-1:0]                                                    we_ram;
    logic [DCACHE_SET_ASSOC-1:0][(DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH)-1:0]          wdata_ram;
    logic [DCACHE_SET_ASSOC-1:0][(DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH)-1:0]          rdata_ram;
    logic [DCACHE_SET_ASSOC-1:0][((DCACHE_TAG_WIDTH+DCACHE_LINE_WIDTH+7)/8)-1:0]    be_ram;

    // Busy signals
    logic miss_handler_busy;
    assign busy_o = |busy | miss_handler_busy;

    typedef enum logic [2:0] {
        IDLE = 0,
        WAIT_TAG,
        WAIT_CACHE,
        WAIT_SPM,
        WRITE
    } addr_decode_state_t;

    dcache_req_i_t [2:0]    cache_ports_in, spm_ports_in;
    dcache_req_o_t [2:0]    cache_ports_out, spm_ports_out;

    // Debugging - if all cache ways are seeing a request, we want only reads
    logic multi_cache_write;
    assign multi_cache_write = ((req_ram & ~dcache_spm_ways_i) == ~dcache_spm_ways_i) &  we_cache;


    // ------------------
    // Cache vs. SPM split
    // ------------------
    generate
        for (genvar i = 0; i < 3; i++) begin: address_decode

            addr_decode_state_t adec_state_d, adec_state_q;
            logic [DCACHE_INDEX_WIDTH-1:0] addr_idx_d, addr_idx_q;
            logic [DCACHE_TAG_WIDTH-1:0] addr_tag_d, addr_tag_q;
            logic spm_req_d, spm_req_q;

            always_comb begin
                addr_idx_d      = addr_idx_q;
                addr_tag_d      = addr_tag_q;
                adec_state_d    = adec_state_q;
                spm_req_d       = spm_req_q;

                // By default we forward all communication to
                // both targets, minus the ready/valid signals
                spm_ports_in[i]             = req_ports_i[i];
                spm_ports_in[i].data_req    = 1'b0;
                spm_ports_in[i].tag_valid   = 1'b0;
                spm_ports_in[i].kill_req    = 1'b0;

                cache_ports_in[i]           = req_ports_i[i];
                cache_ports_in[i].data_req  = 1'b0;
                cache_ports_in[i].tag_valid = 1'b0;
                cache_ports_in[i].kill_req  = 1'b0;

                // As the answer will most likely come from the
                // cache we forward that by default, also without ready/valid
                req_ports_o[i]              = cache_ports_out[i];
                req_ports_o[i].data_gnt     = 1'b0;
                req_ports_o[i].data_rvalid  = 1'b0;

                unique case (adec_state_q)
                    IDLE: begin
                        spm_req_d = 1'b0;

                        if (req_ports_i[i].data_req) begin
                            addr_idx_d = req_ports_i[i].address_index;

                            if(req_ports_i[i].data_we) begin
                                adec_state_d = WRITE;

                                spm_req_d = &req_ports_i[i].address_tag[DCACHE_TAG_WIDTH-1 -: 36];

                                if(spm_req_d) begin

                                    spm_ports_in[i] = req_ports_i[i];
                                    req_ports_o[i] = spm_ports_out[i];

                                end else begin

                                    cache_ports_in[i] = req_ports_i[i];
                                    req_ports_o[i] = cache_ports_out[i];

                                end

                                // If the write could be acknowledged already
                                // we stay in the idle state
                                if(req_ports_o[i].data_gnt) begin
                                    adec_state_d = IDLE;
                                end

                            end else begin
                                adec_state_d = WAIT_TAG;

                                cache_ports_in[i] = req_ports_i[i];
                                req_ports_o[i] = cache_ports_out[i];

                            end
                        end
                    end

                    WAIT_TAG: begin
                        // By default we forward to the cache
                        cache_ports_in[i] = req_ports_i[i];
                        req_ports_o[i] = cache_ports_out[i];

                        if (req_ports_i[i].tag_valid) begin
                            spm_req_d = &req_ports_i[i].address_tag[DCACHE_TAG_WIDTH-1 -: 36];

                            if(spm_req_d) begin
                                adec_state_d = WAIT_SPM;

                                addr_tag_d = req_ports_i[i].address_tag;

                                spm_ports_in[i] = req_ports_i[i];
                                req_ports_o[i] = spm_ports_out[i];

                                spm_ports_in[i].address_index = addr_idx_q;

                                // All reads have been granted by the cache before,
                                // this is just to signal a pending request to the SPM controller
                                spm_ports_in[i].data_req = 1'b1;

                                // Kill the started cache request as this is a SPM access
                                cache_ports_in[i].kill_req = 1'b1;

                            end else begin
                                adec_state_d = WAIT_CACHE;

                            end

                            // If the request could already be answered in this cycle
                            // we return to the idle state
                            if(req_ports_o[i].data_rvalid) begin
                                adec_state_d = IDLE;
                            end

                            if(req_ports_o[i].data_gnt) begin
                                adec_state_d = WAIT_TAG;
                            end
                        end
                    end

                    WAIT_CACHE: begin

                        cache_ports_in[i] = req_ports_i[i];
                        req_ports_o[i] = cache_ports_out[i];

                        if(req_ports_o[i].data_rvalid) begin
                            adec_state_d = IDLE;
                        end

                        if(req_ports_o[i].data_gnt) begin
                            adec_state_d = WAIT_TAG;
                        end

                    end

                    WAIT_SPM: begin

                        spm_ports_in[i] = req_ports_i[i];
                        req_ports_o[i] = spm_ports_out[i];

                        if(req_ports_o[i].data_rvalid) begin
                            adec_state_d = IDLE;
                        end
                    end

                    WRITE: begin
                        if(spm_req_q) begin

                            spm_ports_in[i] = req_ports_i[i];
                            req_ports_o[i] = spm_ports_out[i];

                        end else begin

                            cache_ports_in[i] = req_ports_i[i];
                            req_ports_o[i] = cache_ports_out[i];

                        end

                        if(req_ports_o[i].data_gnt) begin
                            adec_state_d = IDLE;
                            spm_req_d = 0;
                        end
                    end

                    default: begin
                    end
                endcase

                // If we get a kill request at any point in time,
                // we just return to idle
                if(req_ports_i[i].kill_req) begin
                    adec_state_d = IDLE;
                end

            end

            `FF(addr_idx_q, addr_idx_d, '0, clk_i, rst_ni)
            `FF(addr_tag_q, addr_tag_d, '0, clk_i, rst_ni)
            `FF(adec_state_q, adec_state_d, IDLE, clk_i, rst_ni)
            `FF(spm_req_q, spm_req_d, 1'b0, clk_i, rst_ni)

        end
    endgenerate

    // ------------------
    // SPM Controller
    // ------------------

    spm_ctrl #(
        .NR_PORTS       ( 3                                     ),
        .NR_WAYS        ( DCACHE_SET_ASSOC                      ),
        .LINE_WIDTH     ( DCACHE_LINE_WIDTH                     ),
        .ADDR_WIDTH     ( DCACHE_INDEX_WIDTH                    ),
        .MEMORY_WIDTH   ( DCACHE_TAG_WIDTH + DCACHE_LINE_WIDTH  ),
        .IDX_WIDTH      ( DCACHE_INDEX_WIDTH                    ),
        .NR_WAIT_STAGES ( 1                                     )
    ) i_spm_ctrl (
        .clk_i,
        .rst_ni,

        .active_ways_i      ( dcache_spm_ways_i ),

        // Request ports
        .spm_req_ports_i    ( spm_ports_in  ),
        .spm_req_ports_o    ( spm_ports_out ),

        .req_o              ( req_spm       ),
        .addr_o             ( addr_spm      ),
        .wdata_o            ( wdata_spm     ),
        .we_o               ( we_spm        ),
        .be_o               ( be_spm        ),
        .rdata_i            ( rdata_spm     )
    );


    // ------------------
    // Cache Controller
    // ------------------
    generate
        for (genvar i = 0; i < 3; i++) begin : master_ports
            cache_ctrl  #(
                .ArianeCfg             ( ArianeCfg            )
            ) i_cache_ctrl (
                .bypass_i              ( ~enable_i            ),
                .busy_o                ( busy            [i]  ),
                .stall_i               ( stall_i | flush_i    ),
                // from core
                .req_port_i            ( cache_ports_in  [i]  ),
                .req_port_o            ( cache_ports_out [i]  ),
                // to SRAM array
                .req_o                 ( req            [i+1] ),
                .addr_o                ( addr           [i+1] ),
                .gnt_i                 ( gnt            [i+1] ),
                .data_i                ( rdata                ),
                .tag_o                 ( tag            [i+1] ),
                .data_o                ( wdata          [i+1] ),
                .we_o                  ( we             [i+1] ),
                .be_o                  ( be             [i+1] ),
                // Ensure that only cache owned ways can hit
                .hit_way_i             ( hit_way & ~dcache_spm_ways_i ),

                .miss_req_o            ( miss_req        [i]  ),
                .miss_gnt_i            ( miss_gnt        [i]  ),
                .active_serving_i      ( active_serving  [i]  ),
                .critical_word_i       ( critical_word        ),
                .critical_word_valid_i ( critical_word_valid  ),
                .bypass_gnt_i          ( bypass_gnt      [i]  ),
                .bypass_valid_i        ( bypass_valid    [i]  ),
                .bypass_data_i         ( bypass_data     [i]  ),

                .mshr_addr_o           ( mshr_addr         [i] ),
                .mshr_addr_matches_i   ( mshr_addr_matches [i] ),
                .mshr_index_matches_i  ( mshr_index_matches[i] ),
                .*
            );
        end
    endgenerate

    // ------------------
    // Miss Handling Unit
    // ------------------
    miss_handler #(
        .NR_PORTS               ( 3                    ),
        .AXI_ADDR_WIDTH         ( AXI_ADDR_WIDTH       ),
        .AXI_DATA_WIDTH         ( AXI_DATA_WIDTH       ),
        .AXI_ID_WIDTH           ( AXI_ID_WIDTH         ),
        .axi_req_t              ( axi_req_t            ),
        .axi_rsp_t              ( axi_rsp_t            )
    ) i_miss_handler (
        .busy_o                 ( miss_handler_busy    ),
        .flush_i                ( flush_i              ),
        .busy_i                 ( |busy                ),
        .spm_ways_i             ( dcache_spm_ways_i    ),
        // AMOs
        .amo_req_i              ( amo_req_i            ),
        .amo_resp_o             ( amo_resp_o           ),
        .miss_req_i             ( miss_req             ),
        .miss_gnt_o             ( miss_gnt             ),
        .bypass_gnt_o           ( bypass_gnt           ),
        .bypass_valid_o         ( bypass_valid         ),
        .bypass_data_o          ( bypass_data          ),
        .critical_word_o        ( critical_word        ),
        .critical_word_valid_o  ( critical_word_valid  ),
        .mshr_addr_i            ( mshr_addr            ),
        .mshr_addr_matches_o    ( mshr_addr_matches    ),
        .mshr_index_matches_o   ( mshr_index_matches   ),
        .active_serving_o       ( active_serving       ),
        .req_o                  ( req             [0]  ),
        .addr_o                 ( addr            [0]  ),
        .data_i                 ( rdata                ),
        .be_o                   ( be              [0]  ),
        .data_o                 ( wdata           [0]  ),
        .we_o                   ( we              [0]  ),
        .axi_bypass_o,
        .axi_bypass_i,
        .axi_data_o,
        .axi_data_i,
        .*
    );

    assign tag[0] = '0;

    // --------------
    // Memory Arrays
    // --------------
    for (genvar i = 0; i < DCACHE_SET_ASSOC; i++) begin : sram_block
        always_comb begin
            // Is this cache way SPM owned?
            if(dcache_spm_ways_i[i]) begin
                req_ram[i]      = req_spm[i];
                we_ram[i]       = we_spm;
                addr_ram[i]     = addr_spm;
                wdata_ram[i]    = wdata_spm;
                be_ram[i]       = be_spm;

            // Otherwise the cache accesses it
            end else begin
                req_ram[i]      = req_cache[i];
                we_ram[i]       = we_cache;
                addr_ram[i]     = addr_cache;
                wdata_ram[i]    = {wdata_cache.tag, wdata_cache.data};
                be_ram[i]       = {be_cache.tag, be_cache.data};
            end
        end

        // Read data can be forwarded to both
        assign {rdata_cache[i].tag, rdata_cache[i].data} = rdata_ram[i];
        assign rdata_spm[i] = rdata_ram[i];

        sram #(
            .DATA_WIDTH ( DCACHE_TAG_WIDTH + DCACHE_LINE_WIDTH ),
            .NUM_WORDS  ( DCACHE_NUM_WORDS                  )
        ) spm_sram (
            .req_i   ( req_ram [i]                          ),
            .rst_ni  ( rst_ni                               ),
            .we_i    ( we_ram[i]                            ),
            .addr_i  ( addr_ram[i][DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]  ),
            .wuser_i ( '0                                   ),
            .wdata_i ( wdata_ram[i]                         ),
            .be_i    ( be_ram[i]                            ),
            .ruser_o (                                      ),
            .rdata_o ( rdata_ram[i]                         ),
            .*
        );
    end

    // ----------------
    // Valid/Dirty Regs
    // ----------------

    vldrty_t [DCACHE_SET_ASSOC-1:0] dirty_wdata, dirty_rdata;

    for (genvar i = 0; i < DCACHE_SET_ASSOC; i++) begin
      // If the way is owned by the SPM it's always clean and not valid
      assign dirty_wdata[i]     = (dcache_spm_ways_i[i]) ? '{dirty: {DCACHE_SET_ASSOC{1'b0}}, valid: 1'b0}
                                                         : '{dirty: wdata_cache.dirty, valid: wdata_cache.valid};
      assign rdata_cache[i].dirty = dirty_rdata[i].dirty;
      assign rdata_cache[i].valid = dirty_rdata[i].valid;

      // Always write the zeroes of SPM owned ways to the memory array
      assign be_valid_dirty_ram[i].valid = be_cache.vldrty[i].valid | dcache_spm_ways_i[i];
      assign be_valid_dirty_ram[i].dirty = be_cache.vldrty[i].dirty | {((DCACHE_LINE_WIDTH+7)/8){dcache_spm_ways_i[i]}};
    end

    sram #(
        .USER_WIDTH ( 1                                ),
        .DATA_WIDTH ( DCACHE_SET_ASSOC*$bits(vldrty_t) ),
        .BYTE_WIDTH ( 1                                ),
        .NUM_WORDS  ( DCACHE_NUM_WORDS                 )
    ) valid_dirty_sram (
        .clk_i   ( clk_i                               ),
        .rst_ni  ( rst_ni                              ),
        .req_i   ( |req_cache                            ),
        .we_i    ( we_cache                              ),
        .addr_i  ( addr_cache[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET] ),
        .wuser_i ( '0                                  ),
        .wdata_i ( dirty_wdata                         ),
        .be_i    ( be_valid_dirty_ram                  ),
        .ruser_o (                                     ),
        .rdata_o ( dirty_rdata                         )
    );

    // ------------------------------------------------
    // Tag Comparison and memory arbitration
    // ------------------------------------------------
    tag_cmp #(
        .NR_PORTS           ( 4                  ),
        .ADDR_WIDTH         ( DCACHE_INDEX_WIDTH ),
        .DCACHE_SET_ASSOC   ( DCACHE_SET_ASSOC   )
    ) i_tag_cmp (
        .req_i              ( req         ),
        .gnt_o              ( gnt         ),
        .addr_i             ( addr        ),
        .wdata_i            ( wdata       ),
        .we_i               ( we          ),
        .be_i               ( be          ),
        .rdata_o            ( rdata       ),
        .tag_i              ( tag         ),
        .hit_way_o          ( hit_way     ),

        .req_o              ( req_cache   ),
        .addr_o             ( addr_cache  ),
        .wdata_o            ( wdata_cache ),
        .we_o               ( we_cache    ),
        .be_o               ( be_cache    ),
        .rdata_i            ( rdata_cache ),
        .*
    );


//pragma translate_off
    initial begin
        assert (DCACHE_LINE_WIDTH/AXI_DATA_WIDTH inside {2, 4, 8, 16}) else $fatal(1, "Cache line size needs to be a power of two multiple of AXI_DATA_WIDTH");
    end
//pragma translate_on
endmodule
