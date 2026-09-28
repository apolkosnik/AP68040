// Experimental resident-instruction ID/EX/WB pipeline; opt-in core ownership.
// Optional head-ordered loads/stores use the owner's memory/exception sequencer.
`include "ap040_defs.svh"
module ap040_pipeline_integer #(
    parameter EXTERNAL_STATE = 0,
    parameter ENABLE_LOADS = 0,
    parameter ENABLE_STORES = 0,
    parameter ENABLE_PEA = 0,
    // Brief indexed MOVE loads and stores; legacy public name retained.
    parameter ENABLE_INDEXLOAD = 0,
    parameter ENABLE_SHIFTS = 0,
    parameter ENABLE_DISP_LEA = 0,
    parameter ENABLE_COMPARE = 0,
    parameter ENABLE_BRANCH = 0,
    parameter ENABLE_FAST_READ_RETIRE = 0,
    parameter EXTERNAL_ALU = 0
) (
    input wire clk, nreset, ce, flush,
    // Cancel ID/EX while allowing an accepting WB to commit. A blocked WB
    // is retained. Used when an interrupt is recognized at retirement.
    input wire kill_younger,
    output wire idle,
    // No younger work remains after the accepting final WB edge.
    output wire empty_after_retire,
    // EXTERNAL_STATE uses the owner's one architectural register file and CCR.
    input wire [31:0] external_a, external_b, external_sp, external_dst,
    input wire [4:0] external_ccr,
    // Optional shared core ALU. EX operations and operands remain registered
    // pipeline state; only the combinational datapath is externalized.
    input wire external_alu_selected,
    input wire [31:0] external_alu_result,
    input wire [4:0] external_alu_flags, external_alu_fast_flags,
    input wire external_alu_fast_ok,
    output reg [5:0] ex_op,
    output reg [1:0] ex_size,
    output wire [5:0] shcnt,
    output wire [31:0] src, b,
    output wire [4:0] flags_in,
    output wire [3:0] read_src, read_dst, read_old_dst,
    output wire in_supported,
    // Optional owner admission probe uses the same decoder as real issue.
    input wire [15:0] next_opcode, next_extension,
    input wire next_valid, next_extension_valid,
    output wire next_supported,
    input wire in_valid,
    output wire in_ready,
    input wire [31:0] in_pc,
    input wire [15:0] in_opcode, in_extension,
    input wire in_extension_valid,
    output wire [1:0] in_words,
    input wire retire_ready,
    output wire retire_valid,
    output wire retire_wb_valid, retire_branch_taken,
    output wire [31:0] retire_pc, retire_next_pc,
    output wire [15:0] retire_opcode,
    output wire retire_we,
    output wire [3:0] retire_dst,
    output wire [31:0] retire_data,
    output wire [4:0] retire_ccr,
    // Synchronous head-ordered memory offer. Fields are held after its first
    // clock edge until acknowledgement, including while CE is paused. Cancelled
    // reads drain without a register write. An accepted store is irreversible:
    // the owner handles interrupts after its retirement, or reports its fault.
    // The historical load_* port names now carry reads and writes.
    output wire load_req,
    output wire load_write,
    output wire [31:0] load_wdata,
    output wire [4:0] load_ccr,
    output wire [31:0] load_addr,
    output wire [1:0] load_size,
    output wire [31:0] load_pc,
    output wire [15:0] load_opcode,
    input wire load_ack, load_fault,
    input wire [31:0] load_data,
    output wire retire_fault,
    output wire [31:0] retire_fault_addr,
    output wire fallback_valid,
    output wire [31:0] fallback_pc,
    output wire [15:0] fallback_opcode
);
    reg id_v, ex_v, wb_v;
    reg [3:0] ex_read_dst; // ID-decoded physical register selector for EX port B
    reg ex_load, ex_store, ex_pea, wb_fault;
    reg [15:0] id_extension, ex_extension;
    reg [31:0] id_next_pc, ex_next_pc, wb_next_pc;
    reg load_write_r;
    reg [31:0] load_wdata_r;
    reg [4:0] load_ccr_r;
    reg [31:0] wb_fault_addr;
    reg load_pending, load_done, load_discard, load_error;
    reg [31:0] load_addr_r, load_value, load_pc_r;
    reg [15:0] load_opcode_r;
    reg [1:0] load_size_r;
    // An offer may coincide with older WB retirement. kill_younger prevents
    // issuing past an interrupt at that boundary; stalled WB blocks the offer.
    // Register its fields if not acknowledged, so they remain stable thereafter.
    wire load_issue;
    assign load_req = load_pending || load_issue;
    assign load_write = load_pending ? load_write_r : ex_store;
    assign load_wdata = load_pending ? load_wdata_r : store_value;
    assign load_ccr = load_pending ? load_ccr_r : store_flags;
    assign load_addr = load_pending ? load_addr_r : ex_store ? store_addr : read_addr;
    assign load_size = load_pending ? load_size_r : ex_size;
    assign load_pc = load_pending ? load_pc_r : ex_pc;
    assign load_opcode = load_pending ? load_opcode_r : ex_opcode;
    assign retire_fault = fast_read_retire ? 1'b0 : wb_fault;
    assign retire_fault_addr = wb_fault_addr;
    function automatic is_short_branch(input [15:0] word);
        is_short_branch = ENABLE_BRANCH && word[15:12] == 6 &&
            word[11:8] == 0 && word[7:0] != 0 && !word[0];
    endfunction
    function automatic branch_condition(input [3:0] cc,input [4:0] f);
        case(cc)
        0:branch_condition=1; 1:branch_condition=0;
        2:branch_condition=!f[0]&&!f[2]; 3:branch_condition=f[0]||f[2];
        4:branch_condition=!f[0]; 5:branch_condition=f[0];
        6:branch_condition=!f[2]; 7:branch_condition=f[2];
        8:branch_condition=!f[1]; 9:branch_condition=f[1];
        10:branch_condition=!f[3]; 11:branch_condition=f[3];
        12:branch_condition=f[3]==f[1]; 13:branch_condition=f[3]!=f[1];
        14:branch_condition=!f[2]&&(f[3]==f[1]);
        default:branch_condition=f[2]||(f[3]!=f[1]);
        endcase
    endfunction
    function automatic is_disp_lea(input [15:0] word);
        is_disp_lea = ENABLE_DISP_LEA && (word & 16'hf1f8) == 16'h41e8;
    endfunction
    function automatic is_indexload(input [15:0] word);
        // Brief indexed loads use base and index. Partial Dn destinations
        // also read their old value through the third pipeline RF port.
        is_indexload = ENABLE_INDEXLOAD && word[15:14] == 0 &&
            word[5:3] == 6 && (word[8:6] == 0 || word[8:6] == 1) &&
            word[13:12] != 0 && !(word[13:12] == 1 && word[8:6] == 1);
    endfunction
    function automatic is_indexstore(input [15:0] word);
        is_indexstore = ENABLE_INDEXLOAD && word[15:14] == 0 && word[13:12] != 0 &&
            word[5:4] == 0 && !(word[13:12] == 1 && word[3]) && word[8:6] == 6;
    endfunction
    function automatic is_indexcmp(input [15:0] word);
        is_indexcmp = ENABLE_COMPARE && (word & 16'hf138) == 16'hb030 && word[7:6] != 3;
    endfunction
    function automatic is_indextst(input [15:0] word);
        is_indextst = ENABLE_COMPARE && (word & 16'hff38) == 16'h4a30 && word[7:6] != 3;
    endfunction
    function automatic is_indexmul(input [15:0] word);
        is_indexmul = ENABLE_COMPARE && (word & 16'hf0f8) == 16'hc0f0;
    endfunction
    function automatic is_indexread(input [15:0] word);
        is_indexread = is_indexload(word) || is_indexcmp(word) || is_indextst(word) || is_indexmul(word);
    endfunction
    function automatic is_indirecttst(input [15:0] word);
        is_indirecttst = ENABLE_COMPARE && (word & 16'hff38) == 16'h4a10 && word[7:6] != 3;
    endfunction
    function automatic is_indirectadd(input [15:0] word);
        is_indirectadd = ENABLE_COMPARE && (word & 16'hf138) == 16'hd010 && word[7:6] != 3;
    endfunction
    function automatic is_load(input [15:0] word);
        is_load = is_indirectadd(word) || is_indirecttst(word) || is_indexread(word) || ENABLE_LOADS && word[15:14] == 0 && word[13:12] != 0 &&
                  word[8:6] == 0 && word[5:3] == 3'b010;
    endfunction
    function automatic is_store(input [15:0] word);
        is_store = is_indexstore(word) || ENABLE_STORES && word[15:14] == 0 && word[13:12] != 0 &&
                   word[5:4] == 0 && !(word[13:12] == 1 && word[3]) &&
                   (word[8:6] == 2 || word[8:6] == 3 || word[8:6] == 4);
    endfunction
    function automatic is_pea(input [15:0] word);
        is_pea = ENABLE_PEA && (word & 16'hfff8) == 16'h4870;
    endfunction
    reg [31:0] id_pc, ex_pc, wb_pc;
    reg [15:0] id_opcode, ex_opcode, wb_opcode;
    reg [3:0] ex_src, ex_dst, wb_dst;
    reg ex_imm, ex_we, ex_flags, ex_word_src, wb_we;
    reg [31:0] ex_immediate, wb_data;
    reg [4:0] ccr, wb_ccr;

    // One decode function serves admission and ID. The testbench checks all
    // 65,536 words against a map generated independently from semantic cases.
    function automatic [52:0] decode;
        input [15:0] word;
        reg legal, immediate, writeback, flags, word_src;
        reg [5:0] operation;
        reg [1:0] size;
        reg [3:0] source, destination;
        reg [31:0] literal;
        begin
            legal = 0; immediate = 0; writeback = 1; flags = 1; word_src = 0;
            operation = `AP040_ALU_MOVE; size = `AP040_SZ_L;
            source = {1'b0, word[2:0]}; destination = {1'b0, word[11:9]};
            literal = {{24{word[7]}}, word[7:0]};
            if (is_short_branch(word)) begin
                legal = 1; writeback = 0; flags = 0;
            end else if (is_indirectadd(word)) begin
                legal = 1; writeback = 1; flags = 1; size = word[7:6];
                source = {1'b1, word[2:0]}; destination = {1'b0, word[11:9]};
                operation = `AP040_ALU_ADD;
            end else if (is_indexmul(word)) begin
                legal = 1; writeback = 1; flags = 1; size = `AP040_SZ_W;
                source = {1'b1, word[2:0]}; destination = {1'b0, word[11:9]};
            end else if (is_indexcmp(word) || is_indextst(word) || is_indirecttst(word)) begin
                legal = 1; writeback = 0; flags = 1; size = word[7:6];
                source = {1'b1, word[2:0]}; destination = {1'b0, word[11:9]};
                operation = is_indexcmp(word) ? `AP040_ALU_CMP : `AP040_ALU_TST;
            end else if (ENABLE_COMPARE && (word & 16'hff38) == 16'h4a00 && word[7:6] != 3) begin
                legal = 1; writeback = 0; flags = 1; size = word[7:6];
                source = {1'b0,word[2:0]}; operation = `AP040_ALU_TST;
            end else if (is_disp_lea(word)) begin
                legal = 1; flags = 0;
                source = {1'b1,word[2:0]}; destination = {1'b1,word[11:9]};
            end else if (is_pea(word)) begin
                legal = 1; flags = 0;
                source = {1'b1, word[2:0]}; destination = 4'd15;
            end else if (is_load(word)) begin
                legal = 1;
                source = {1'b1, word[2:0]};
                destination = {word[6], word[11:9]};
                flags = !word[6];
                size = word[13:12] == 1 ? `AP040_SZ_B :
                       word[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
            end else if (is_store(word)) begin
                legal = 1;
                source = {word[3], word[2:0]};
                destination = {1'b1, word[11:9]};
                writeback = word[8:6] == 3 || word[8:6] == 4;
                size = word[13:12] == 1 ? `AP040_SZ_B :
                       word[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
            end else if (ENABLE_SHIFTS && word[15:12] == 4'he && word[7:6] != 3) begin
                legal = 1; size = word[7:6];
                source = {1'b0, word[11:9]}; destination = {1'b0, word[2:0]};
                immediate = !word[5];
                literal = word[11:9] == 0 ? 32'd8 : {29'd0, word[11:9]};
                case (word[4:3])
                    0: operation = word[8] ? `AP040_ALU_ASL1 : `AP040_ALU_ASR1;
                    1: operation = word[8] ? `AP040_ALU_LSL1 : `AP040_ALU_LSR1;
                    2: operation = word[8] ? `AP040_ALU_ROXL1 : `AP040_ALU_ROXR1;
                    3: operation = word[8] ? `AP040_ALU_ROL1 : `AP040_ALU_ROR1;
                endcase
            end else if (word == 16'h4e71) begin
                legal = 1; writeback = 0; flags = 0;
            end else if ((word & 16'hf100) == 16'h7000) begin
                legal = 1; immediate = 1;
            end else if (word[15:14] == 0 && word[13:12] != 0 &&
                         word[8:7] == 0 && word[5:4] == 0 &&
                         !(word[13:12] == 1 && (word[6] || word[3]))) begin
                legal = 1;
                source = {word[3], word[2:0]};
                destination = {word[6], word[11:9]};
                size = word[13:12] == 1 ? `AP040_SZ_B :
                       word[13:12] == 2 ? `AP040_SZ_L : `AP040_SZ_W;
                if (word[6]) begin
                    // MOVEA.W sign-extends before its full-width write.
                    word_src = size == `AP040_SZ_W;
                    size = `AP040_SZ_L; flags = 0;
                end
            end else if (word[15:12] == 5 && word[7:6] != 3 &&
                         word[5:4] == 0 && !(word[7:6] == 0 && word[3])) begin
                legal = 1; immediate = 1;
                literal = word[11:9] == 0 ? 32'd8 : {29'd0, word[11:9]};
                destination = {word[3], word[2:0]};
                operation = word[8] ? `AP040_ALU_SUB : `AP040_ALU_ADD;
                size = word[3] ? `AP040_SZ_L : word[7:6];
                flags = !word[3];
            end else if (word[5:4] == 0 &&
                         (word[15:12] == 9 || word[15:12] == 11 || word[15:12] == 13)) begin
                source = {word[3], word[2:0]};
                operation = word[15:12] == 9 ? `AP040_ALU_SUB :
                            word[15:12] == 13 ? `AP040_ALU_ADD : `AP040_ALU_CMP;
                writeback = word[15:12] != 11;
                if (word[7:6] == 3) begin
                    legal = 1; destination[3] = 1;
                    word_src = !word[8]; size = `AP040_SZ_L;
                    flags = word[15:12] == 11;
                end else if (!word[8] && !(word[7:6] == 0 && word[3])) begin
                    legal = 1; size = word[7:6];
                end else if (word[15:12] == 11 && word[8] && !word[3]) begin
                    legal = 1; size = word[7:6]; writeback = 1;
                    operation = `AP040_ALU_EOR;
                    source = {1'b0, word[11:9]}; destination = {1'b0, word[2:0]};
                end
            end else if (word[5:3] == 0 && word[7:6] != 3 && !word[8] &&
                         (word[15:12] == 8 || word[15:12] == 12)) begin
                legal = 1; size = word[7:6];
                operation = word[15:12] == 8 ? `AP040_ALU_OR : `AP040_ALU_AND;
            end
            decode = {legal, immediate, writeback, flags, word_src, operation,
                      size, source, destination, literal};
        end
    endfunction
    wire legal, dec_imm, dec_we, dec_flags, dec_word_src;
    wire [5:0] dec_op;
    wire [1:0] dec_size;
    wire [3:0] dec_src, dec_dst;
    wire [31:0] dec_immediate;
    wire [52:0] admission = decode(in_opcode);
    wire [52:0] next_admission = decode(next_opcode);
    assign next_supported = next_valid && next_admission[52] &&
        (!(is_pea(next_opcode) || is_indexread(next_opcode) || is_indexstore(next_opcode)) || (next_extension_valid && !next_extension[8])) &&
        (!is_disp_lea(next_opcode) || next_extension_valid);
    assign in_words = (is_pea(in_opcode) || is_indexread(in_opcode) || is_indexstore(in_opcode) || is_disp_lea(in_opcode)) ? 2'd2 : 2'd1;
    assign in_supported = admission[52] &&
        (!(is_pea(in_opcode) || is_indexread(in_opcode) || is_indexstore(in_opcode)) || (in_extension_valid && !in_extension[8])) &&
        (!is_disp_lea(in_opcode) || in_extension_valid);
    // Empty-pipeline admission can decode directly into EX. Outstanding
    // transactions and retained responses always use the existing ID path.
    wire direct_admit = nreset && ce && !flush && !kill_younger &&
        !id_v && !ex_v && !wb_v && !load_pending && !load_done && !load_discard &&
        in_valid && in_supported;
    wire [15:0] ex_input_opcode = direct_admit ? in_opcode : id_opcode;
    wire [15:0] ex_input_extension = direct_admit ? in_extension : id_extension;
    wire [31:0] ex_input_pc = direct_admit ? in_pc : id_pc;
    wire [31:0] ex_input_next_pc = direct_admit ? in_pc + {29'd0, in_words, 1'b0} : id_next_pc;
    assign {legal, dec_imm, dec_we, dec_flags, dec_word_src, dec_op,
            dec_size, dec_src, dec_dst, dec_immediate} =
        direct_admit ? admission : decode(id_opcode);

    wire wb_ready = !wb_v || (retire_ready && !wb_fault);
    wire load_response = load_req && load_ack && !load_discard && !flush && !kill_younger;
    wire ex_complete = !ex_load || load_done || load_response;
    wire ex_ready = !ex_v || (wb_ready && ex_complete);
    wire id_advance = id_v && legal && ex_ready;
    assign idle = !id_v && !ex_v && !wb_v && !load_pending;
    assign in_ready = nreset && ce && !flush && !kill_younger && !load_discard && (!id_v || id_advance);
    wire fast_read_retire = ENABLE_FAST_READ_RETIRE && nreset && ce && !flush &&
        !wb_v && ex_v && ex_load && !ex_store && !is_indexmul(ex_opcode) && load_response && !load_fault && retire_ready;
    assign retire_wb_valid = nreset && ce && !flush && wb_v;
    assign retire_branch_taken = retire_wb_valid && wb_opcode[15:12] == 6 && wb_next_pc != wb_pc + 32'd2;
    assign retire_valid = nreset && ce && !flush && (wb_v || fast_read_retire);
    wire commit = retire_valid && retire_ready;
    assign empty_after_retire = (!id_v && fast_read_retire) || (!id_v && !ex_v && !load_pending &&
                                (!wb_v || (commit && !wb_fault)));
    assign retire_pc = fast_read_retire ? ex_pc : wb_pc;
    assign retire_next_pc = fast_read_retire ? ex_next_pc : wb_next_pc;
    assign retire_opcode = fast_read_retire ? ex_opcode : wb_opcode;
    assign retire_we = fast_read_retire ? ex_we : wb_we && !wb_fault;
    assign retire_dst = fast_read_retire ? ex_dst : wb_dst;
    assign retire_data = fast_read_retire ? (ex_dst[3] ?
        ((ex_size == `AP040_SZ_W) ? {{16{src[15]}},src[15:0]} : src) : merged) : wb_data;
    assign retire_ccr = fast_read_retire ? (ex_flags ? alu_fast_flags : flags_in) : wb_ccr;
    assign fallback_valid = nreset && ce && !flush && id_v && !legal && !ex_v && !wb_v;
    assign fallback_pc = id_pc;
    assign fallback_opcode = id_opcode;

    wire [31:0] rf_a, rf_b, rf_sp, rf_old_dst;
    assign read_old_dst = ex_dst;
    assign read_src = ex_src;
    assign read_dst = ex_read_dst;
    generate if (EXTERNAL_STATE) begin : shared_state
        assign rf_a = external_a;
        assign rf_b = external_b;
        assign rf_sp = external_sp;
        assign rf_old_dst = external_dst;
    end else begin : private_state
    ap040_regfile #(.EXTRA_READS(1)) regfile (
        .clk(clk), .nreset(nreset), .ce(ce), .sr_s(1'b1), .sr_m(1'b0),
        .we(commit && retire_we && !retire_fault), .waddr(retire_dst), .wdata(retire_data),
        .raddr_a(ex_src), .raddr_b(read_dst),
        .rdata_a(rf_a), .rdata_b(rf_b),
        .raddr_c(4'd0), .rdata_c(), .raddr_d(4'd0), .rdata_d(),
        .raddr_e(ex_dst), .rdata_e(rf_old_dst), .raddr_f(4'd0), .rdata_f(),
        .aux_we(1'b0), .aux_sel(2'd0), .aux_wdata(32'd0),
        .usp_q(), .isp_q(), .msp_q(), .dbg_d0(), .dbg_d1(), .dbg_d2(), .dbg_a0(), .dbg_a7(rf_sp)
    );
    end endgenerate
    // EX reads late, forwarding the immediately older WB value, including
    // the upper bytes needed for partial-register writes. The existing RF
    // handles its own delayed MLAB write beneath this bypass.
    wire [31:0] source_full = ex_imm ? ex_immediate :
        (wb_v && wb_we && wb_dst == ex_src) ? wb_data : rf_a;
    assign src = ex_store ? load_wdata : ex_load ? (load_done ? load_value : load_data) : ex_word_src ? {{16{source_full[15]}}, source_full[15:0]} : source_full;
    assign shcnt = source_full[5:0];
    wire [31:0] dst = (wb_v && wb_we && wb_dst == read_dst) ? wb_data : rf_b;
    assign b = is_indexcmp(ex_opcode) ? old_dst : dst;
    wire [31:0] old_dst = wb_v && wb_we && wb_dst == ex_dst ? wb_data : rf_old_dst;
    wire [31:0] merge_dst = is_indexload(ex_opcode) ? old_dst : dst;
    assign flags_in = wb_v ? wb_ccr : (EXTERNAL_STATE ? external_ccr : ccr);
    wire [31:0] alu_result;
    wire [4:0] alu_flags, alu_fast_flags;
    wire alu_fast_ok;
    generate if (EXTERNAL_ALU) begin : shared_alu
        assign alu_result = external_alu_result;
        assign alu_flags = external_alu_flags;
        assign alu_fast_flags = external_alu_fast_flags;
        assign alu_fast_ok = external_alu_fast_ok;
    end else begin : local_alu
        ap040_alu #(.PIPELINE_SUBSET(1)) alu (
            .op(ex_op), .size(ex_size), .shcnt(shcnt), .a(src), .b(b),
            .flags_in(flags_in), .result(alu_result), .flags_out(alu_flags),
            .fast_flags(alu_fast_flags), .fast_ok(alu_fast_ok)
        );
    end endgenerate
    // synthesis translate_off
    always @(posedge clk) if (EXTERNAL_ALU && nreset && ce && ex_v && !external_alu_selected)
        $fatal(1,"pipeline EX work ran without owning the shared ALU");
    always @(posedge clk) if (nreset && ce && !flush && ex_v) begin
        case (ex_op)
            `AP040_ALU_MOVE, `AP040_ALU_TST, `AP040_ALU_ADD, `AP040_ALU_SUB, `AP040_ALU_CMP, `AP040_ALU_AND, `AP040_ALU_OR, `AP040_ALU_EOR, `AP040_ALU_ASL1, `AP040_ALU_ASR1, `AP040_ALU_LSL1, `AP040_ALU_LSR1, `AP040_ALU_ROL1, `AP040_ALU_ROR1, `AP040_ALU_ROXL1, `AP040_ALU_ROXR1: ;
            default: $fatal(1,"pipeline decoder selected an omitted ALU operation");
        endcase
    end
    always @(posedge clk) if (fast_read_retire && ex_flags && !alu_fast_ok)
        $fatal(1,"fast read operation has no bounded flag path");
    // synthesis translate_on
    wire zero_shift = ENABLE_SHIFTS && ex_opcode[15:12] == 4'he && source_full[5:0] == 0;
    wire [31:0] shift_masked = ex_size == `AP040_SZ_B ? {24'd0, dst[7:0]} :
                              ex_size == `AP040_SZ_W ? {16'd0, dst[15:0]} : dst;
    // A word product writes all 32 destination bits. Keep registered WB;
    // never put multiply onto the fast load-retirement path.
    wire signed [16:0] mul_a = {ex_opcode[8] && src[15], src[15:0]};
    wire signed [16:0] mul_b = {ex_opcode[8] && old_dst[15], old_dst[15:0]};
    wire signed [33:0] mul_product = mul_a * mul_b;
    wire [31:0] mul_result = mul_product[31:0];
    wire [4:0] result_flags = is_indexmul(ex_opcode) ?
        {flags_in[4],mul_result[31],mul_result==0,2'b00} : zero_shift ? {flags_in[4],
        ex_size == `AP040_SZ_B ? dst[7] : ex_size == `AP040_SZ_W ? dst[15] : dst[31],
        shift_masked == 0, 1'b0, ex_opcode[4:3] == 2 ? flags_in[4] : 1'b0} : alu_flags;
    wire [31:0] merged = is_indexmul(ex_opcode) ? mul_result : is_disp_lea(ex_opcode) ? source_full + {{16{ex_extension[15]}},ex_extension} : zero_shift ? dst : ex_size == `AP040_SZ_B ? {merge_dst[31:8], alu_result[7:0]} :
                         ex_size == `AP040_SZ_W ? {merge_dst[31:16], alu_result[15:0]} : alu_result;
    wire [31:0] pea_index = ex_extension[11] ? dst : {{16{dst[15]}}, dst[15:0]};
    wire [31:0] pea_value = source_full + (pea_index << ex_extension[10:9]) +
                           {{24{ex_extension[7]}}, ex_extension[7:0]};
    wire [31:0] read_addr = is_indexread(ex_opcode) ? pea_value : source_full;
    wire [31:0] store_value = ex_pea ? pea_value : source_full;
    wire [31:0] stack_value = wb_v && wb_we && wb_dst == 15 ? wb_data : rf_sp;
    wire [31:0] store_step = ex_size == `AP040_SZ_L ? 32'd4 :
                             (ex_size == `AP040_SZ_W || ex_dst == 15) ? 32'd2 : 32'd1;
    wire [31:0] store_addr = is_indexstore(ex_opcode) ? old_dst + (pea_index << ex_extension[10:9]) + {{24{ex_extension[7]}}, ex_extension[7:0]} : ex_pea ? stack_value - 32'd4 : ex_opcode[8:6] == 4 ? dst - store_step : dst;
    wire [31:0] store_update = (ex_pea || ex_opcode[8:6] == 4) ? load_addr : load_addr + store_step;
    // CCR is derived from the actual source before the registered request.
    // MOVE preserves X and clears V/C, independently of address updates.
    wire [4:0] store_flags = ex_pea ? flags_in : {flags_in[4],
        ex_size == `AP040_SZ_B ? source_full[7] : ex_size == `AP040_SZ_W ? source_full[15] : source_full[31],
        ex_size == `AP040_SZ_B ? source_full[7:0] == 0 : ex_size == `AP040_SZ_W ? source_full[15:0] == 0 : source_full == 0,
        2'b00};
    assign load_issue = (ENABLE_LOADS || ENABLE_STORES || ENABLE_PEA || ENABLE_INDEXLOAD || ENABLE_COMPARE) && nreset && ce && !flush && !kill_younger &&
        ex_v && ex_load && wb_ready && !load_pending && !load_done && !load_discard;
    always @(posedge clk) begin
        if (!nreset) begin
            load_pending <= 0; load_done <= 0; load_discard <= 0;
            load_error <= 0; load_addr_r <= 0; load_size_r <= 0; load_value <= 0;
            load_pc_r <= 0; load_opcode_r <= 0;
            load_write_r <= 0; load_wdata_r <= 0; load_ccr_r <= 0;
        end else begin
            if (ce && ex_v && ex_load && ex_complete && wb_ready)
                load_done <= 0;
            if (load_issue) begin
                load_pending <= 1; load_addr_r <= ex_store ? store_addr : read_addr;
                load_write_r <= ex_store; load_wdata_r <= store_value; load_ccr_r <= store_flags;
                load_size_r <= ex_size; load_discard <= 0;
                load_pc_r <= ex_pc; load_opcode_r <= ex_opcode;
            end
            if (ce && (flush || kill_younger)) begin
                load_done <= 0;
                if (load_pending) load_discard <= 1;
            end
            if (load_req && load_ack) begin
                load_pending <= 0;
                load_discard <= 0;
                if (load_discard || (ce && (flush || kill_younger)))
                    load_done <= 0;
                else begin
                    // Forward a response directly into WB when EX can advance;
                    // otherwise retain it, including throughout a CE pause.
                    load_done <= !(ce && ex_v && ex_load && wb_ready);
                    load_value <= load_data; load_error <= load_fault;
                end
            end
        end
    end
    always @(posedge clk) begin
        if (!nreset) begin
            id_v <= 0; ex_v <= 0; wb_v <= 0; ccr <= 0;
            ex_load <= 0; ex_store <= 0; ex_pea <= 0;
            id_extension <= 0; ex_extension <= 0;
            id_next_pc <= 0; ex_next_pc <= 0; wb_next_pc <= 0; wb_fault <= 0; wb_fault_addr <= 0;
            id_pc <= 0; ex_pc <= 0; wb_pc <= 0;
            id_opcode <= 0; ex_opcode <= 0; wb_opcode <= 0;
            ex_op <= 0; ex_size <= 0; ex_src <= 0; ex_dst <= 0; ex_read_dst <= 0;
            ex_word_src <= 0; ex_imm <= 0; ex_we <= 0; ex_flags <= 0; ex_immediate <= 0;
            wb_dst <= 0; wb_we <= 0; wb_data <= 0; wb_ccr <= 0;
        end else if (ce) begin
            if (flush) begin
                id_v <= 0; ex_v <= 0; wb_v <= 0;
            end else begin
                if (commit && !retire_fault) ccr <= retire_ccr;
                if (load_issue && ex_store) ccr <= store_flags;
                if (kill_younger) begin
                    id_v <= 0; ex_v <= 0;
                    if (commit || wb_ready) wb_v <= 0;
                end else begin
                if (wb_ready) begin
                    wb_v <= ex_v && ex_complete && !fast_read_retire;
                    if (ex_v && ex_complete) begin
                        wb_pc <= ex_pc;
                        wb_next_pc <= is_short_branch(ex_opcode) && branch_condition(ex_opcode[11:8],flags_in)
                            ? ex_pc + 32'd2 + {{24{ex_opcode[7]}},ex_opcode[7:0]} : ex_next_pc;
                        wb_opcode <= ex_opcode;
                        wb_fault <= ex_load && (load_done ? load_error : load_fault);
                        wb_fault_addr <= load_addr;
                        wb_dst <= ex_dst; wb_we <= ex_we; wb_data <= ex_store ? store_update : (ex_load && ex_dst[3]) ? ((ex_size == `AP040_SZ_W) ? {{16{src[15]}}, src[15:0]} : src) : merged;
                        wb_ccr <= ex_flags ? result_flags : flags_in;
                    end
                end
                if (ex_ready) begin
                    ex_v <= (id_v || direct_admit) && legal;
                    if ((id_v || direct_admit) && legal) begin
                        ex_pc <= ex_input_pc; ex_next_pc <= ex_input_next_pc; ex_opcode <= ex_input_opcode;
                        ex_extension <= ex_input_extension; ex_pea <= is_pea(ex_input_opcode);
                        ex_load <= is_load(ex_input_opcode) || is_store(ex_input_opcode) || is_pea(ex_input_opcode);
                        ex_store <= is_store(ex_input_opcode) || is_pea(ex_input_opcode);
                        ex_op <= dec_op; ex_size <= dec_size;
                        ex_src <= dec_src; ex_dst <= dec_dst;
                        ex_read_dst <= (is_pea(ex_input_opcode) || is_indexread(ex_input_opcode) ||
                                        is_indexstore(ex_input_opcode)) ? ex_input_extension[15:12] : dec_dst;
                        ex_imm <= dec_imm; ex_immediate <= dec_immediate;
                        ex_we <= dec_we; ex_flags <= dec_flags; ex_word_src <= dec_word_src;
                    end
                end
                if (in_ready) begin
                    id_v <= in_valid && !direct_admit;
                    if (in_valid) begin
                        id_pc <= in_pc; id_next_pc <= in_pc + {29'd0, in_words, 1'b0};
                        id_opcode <= in_opcode; id_extension <= in_extension;
                    end
                end
                end
            end
        end
    end
endmodule
