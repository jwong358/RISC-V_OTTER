`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:  J. Callenes
//
// Create Date: 01/04/2019 04:32:12 PM
// Design Name:
// Module Name: PIPELINED\_OTTER\_CPU
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

typedef enum logic [6:0]{
    LUI      = 7'b0110111,
    AUIPC    = 7'b0010111,
    JAL      = 7'b1101111,
    JALR     = 7'b1100111,
    BRANCH   = 7'b1100011,
    LOAD     = 7'b0000011,
    STORE    = 7'b0100011,
    OP_IMM   = 7'b0010011,
    OP       = 7'b0110011,
    SYSTEM   = 7'b1110011
} opcode_t;

typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(
    input CLK,
    input INTR,
    input RESET,
    input [31:0] IOBUS_IN,
    output [31:0] IOBUS_OUT,
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR
);

    logic [31:0] IR;
        
    wire pcWrite, memRead1, regWrite, memWrite, memRead2, branch, op1_sel, mem_op, IorD, pcWriteCond;
    wire [1:0] rf_sel;
    wire [3:0] alu_fun;
        
    //FETCH
    logic [31:0] next_pc, pc, pc_inc;
    logic [31:0] if_de_pc;
    logic [31:0] if_de_pc_inc;
    logic [31:0] if_de_ir;
    
    //DECODE
    logic [31:0] de_ex_pc;
    logic [31:0] de_ex_pc_inc;
    logic [31:0] de_ex_ir;
    logic EQ, LT, LTU;
    logic [31:0] de_U, de_ex_U;
    logic [31:0] de_I, de_ex_I;
    logic [31:0] de_S, de_ex_S;
    logic [31:0] de_B, de_ex_B;
    logic [31:0] de_J, de_ex_J;
    logic de_aluAsel, de_ex_aluAsel;
    logic [1:0] de_aluBsel, de_ex_aluBsel;
    logic [2:0] de_pcSource, de_ex_pcSource;
    logic [31:0] de_opA, de_ex_opA;
    logic [31:0] de_opB, de_ex_opB;
    logic [31:0] de_rs1, de_rs2;
    logic [31:0] de_ex_rs1, de_ex_rs2;
    logic de_jump, de_branch, de_ex_jump, de_ex_branch;
    
    //EXECUTE
    logic [31:0] ex_mem_pc;
    logic [31:0] ex_mem_pc_inc;
    logic [31:0] ex_mem_ir;
    logic [2:0] ex_pcSource;
    logic [1:0] opB_sel;
    logic opA_sel;
    logic [31:0] ex_aluBin;
    logic [31:0] ex_aluResult, ex_mem_aluResult;
    logic [31:0] ex_mem_rs2;
    logic [31:0] ex_jal;
    logic [31:0] ex_jalr;
    logic [31:0] ex_branch;
    
    //MEMORY
    logic [31:0] mem_wb_pc;
    logic [31:0] mem_wb_pc_inc;
    logic [31:0] mem_dout2, mem_wb_dout2;
    logic [31:0] mem_aluResult, mem_wb_aluResult;
    
    
    //WRITEBACK
    logic [31:0] wb_result;    
    logic [31:0] csr_reg;
    
    // CLOCK DEPENDENT STRUCTS
    instr_t de_ex_inst, ex_mem_inst, mem_wb_inst;
    
    // INTER-STAGE STRUCTS
    instr_t de_inst;
    
//==== Instruction Fetch ===========================================
    
    FourMux PCMUX(
        .SEL(ex_pcSource),
        .ZERO(pc+4),
        .ONE(ex_jalr),
        .TWO(ex_branch),
        .THREE(ex_jal),
        
        .OUT(next_pc)
    );
    
    assign pcWrite = 1'b1;
    assign memRead1 = 1'b1;
    
    // PC reg
    always_ff@(posedge CLK) begin
        if (RESET) begin //If reset is high, set PC output to 0
            pc <= 32'h00000000;
        end
        else if (pcWrite) begin //if Reset is low and D is high,
            pc <= next_pc;              //set PC output to the input!
        end
    end
    
    always_ff @(posedge CLK) begin
        if_de_pc <= pc;
        if_de_pc_inc <= pc+4;
        if_de_ir <= IR;
    end
    
//==== Instruction Decode ===========================================
    
    CU_DCDR DECODER(
        .IR_30(if_de_ir[30]),
        .IR_OPCODE(if_de_ir[6:0]),
        .IR_FUNCT(if_de_ir[14:12]),
        
        .ALU_FUN(alu_fun),
        .ALU_SRCA(de_aluAsel),
        .ALU_SRCB(de_aluBsel),
        .RF_WR_SEL(rf_sel),
        .REG_WRITE(regWrite),
        .MEM_WE2(memWrite),
        .MEM_RDEN2(memRead2),
        .BRANCH(branch)
    );
    
    REG_FILE REGFILE(
        .CLK(CLK),
        .EN(mem_wb_inst.regWrite & (mem_wb_inst.rd_addr != 5'd0)), //1
        .ADR1(de_inst.rs1_addr),
        .ADR2(de_inst.rs2_addr),
        .WA(mem_wb_inst.rd_addr),
        .WD(wb_result),
        
        .RS1(de_rs1),
        .RS2(de_rs2)
    );
        
    ImmediateGenerator ImmedGen(
        .IR(if_de_ir),
        
        .I_TYPE(de_I),
        .S_TYPE(de_S),
        .B_TYPE(de_B),
        .U_TYPE(de_U),
        .J_TYPE(de_J)
    );
    
    TwoMux ALU_A_MUX(
        .SEL(de_aluAsel),
        .ZERO(de_rs1),
        .ONE(de_U),
        
        .OUT(de_opA)
    );
    
    FourMux ALU_B_MUX(
        .SEL(de_aluBsel),
        .ZERO(de_rs2),
        .ONE(de_I),
        .TWO(de_S),
        .THREE(pc),
        
        .OUT(de_opB)
    );
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(if_de_ir[6:0]);
    assign de_inst.opcode = OPCODE;

    assign de_inst.rs1_addr = if_de_ir[19:15];
    assign de_inst.rs2_addr = if_de_ir[24:20];
    assign de_inst.rd_addr = if_de_ir[11:7];
    
    assign de_inst.rs1_used =   de_rs1 != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;  
 
    assign de_inst.mem_type  = if_de_pc[14:12];
    assign de_inst.pc        = if_de_pc;
    // from decoder
    assign de_inst.alu_fun   = alu_fun;
    assign de_inst.regWrite  = regWrite;
    assign de_inst.memWrite  = memWrite;
    assign de_inst.memRead2  = memRead2;
    assign de_inst.rf_wr_sel = rf_sel;
                                    
    always_ff @ (posedge CLK) begin
        de_ex_inst <= de_inst;
//        de_ex_pc <= if_de_pc;
        de_ex_pc_inc <= if_de_pc_inc;
//        de_ex_ir <= if_de_ir;
        de_ex_rs1 <= de_rs1;
        de_ex_rs2 <= de_rs2;
        de_ex_opA <= de_opA;
        de_ex_opB <= de_opB;
        de_ex_I <= de_I;
        de_ex_S <= de_S;
        de_ex_B <= de_B;
        de_ex_J <= de_J;
        
//        de_ex_jump <= de_jump;
//        de_ex_branch <= de_branch;
//        de_ex_aluAsel <= de_aluAsel;
//        de_ex_aluBsel <= de_aluBsel;
    end
    
    
    //==== Execute ======================================================
    
    BAG TargetGen(
        .RS1(de_ex_rs1),    
        .I_TYPE(de_ex_I), 
        .J_TYPE(de_ex_J),
        .B_TYPE(de_ex_B), 
        .FROM_PC(de_ex_inst.pc),
        
        .JAL(ex_jal),  
        .JALR(ex_jalr), 
        .BRANCH(ex_branch)
    );
    
    ALU ALU(
        .SRC_A(de_ex_opA),
        .SRC_B(de_ex_opB),
        .ALU_FUN(de_ex_inst.alu_fun),
        
        .RESULT(ex_aluResult)
    );
    
    logic ex_aluRes = 0;
    logic [31:0] opA_forwarded;
    logic [31:0] opB_forwarded;
    
    always_comb begin
        ex_pcSource = 3'd0; // PC+4
        unique case (de_ex_inst.opcode)
            JAL:    ex_pcSource = 3'd2;
            JALR:   ex_pcSource = 3'd1;
            BRANCH: begin
          // use if_de_ir[14:12] (funct3) with EQ/LT/LTU
                unique case (if_de_ir[14:12])
                    3'b000: ex_pcSource = (EQ ) ? 3'd3 : 3'd0; // BEQ
                    3'b001: ex_pcSource = (!EQ) ? 3'd3 : 3'd0; // BNE
                    3'b100: ex_pcSource = (LT ) ? 3'd3 : 3'd0; // BLT
                    3'b101: ex_pcSource = (!LT) ? 3'd3 : 3'd0; // BGE
                    3'b110: ex_pcSource = (LTU) ? 3'd3 : 3'd0; // BLTU
                    3'b111: ex_pcSource = (!LTU)? 3'd3 : 3'd0; // BGEU
                    default: ;
                endcase
            end
            default: ;
        endcase
    end
    
    always_ff @ (posedge CLK) begin
        ex_mem_inst <= de_ex_inst;
//        ex_mem_pc <= de_ex_pc;
        ex_mem_pc_inc <= de_ex_pc_inc;
//        ex_mem_ir <= de_ex_ir;
        ex_mem_aluResult <= ex_aluResult;
        ex_mem_rs2 <= de_ex_rs2;
    end
    
    //==== Memory ======================================================
    
    assign IOBUS_ADDR = ex_mem_aluResult;
    assign IOBUS_OUT = ex_mem_rs2;
    
    Memory MEMORY(
        .MEM_CLK(CLK),  
        .MEM_RDEN1(memRead1),   
        .MEM_RDEN2(ex_mem_inst.memRead2),
        .MEM_WE2(ex_mem_inst.memWrite),
        .MEM_ADDR1(pc[15:2]),
        .MEM_ADDR2(ex_mem_aluResult),
        .MEM_DIN2(ex_mem_rs2),
        .MEM_SIZE(ex_mem_inst.mem_type[1:0]),
        .MEM_SIGN(ex_mem_inst.mem_type[2]),
        .IO_IN(IOBUS_IN),
        
        .IO_WR(IOBUS_WR),
        .MEM_DOUT1(IR),
        .MEM_DOUT2(mem_dout2)
    ); 
    
    always_ff @ (posedge CLK) begin
        mem_wb_inst <= ex_mem_inst;
//        mem_wb_pc <= ex_mem_pc;
        mem_wb_pc_inc <= ex_mem_pc_inc;
        mem_wb_aluResult <= ex_mem_aluResult;
        mem_wb_dout2 <= mem_dout2;
    end
    
    //==== Write Back ==================================================
    
    FourMux WB_MUX (
        .SEL(mem_wb_inst.rf_wr_sel),
        .ZERO(mem_wb_pc_inc),
        .ONE(csr_reg),
        .TWO(mem_wb_dout2),
        .THREE(mem_wb_aluResult),
        
        .OUT(wb_result)
    );
    
    // =================================================================
    
endmodule
