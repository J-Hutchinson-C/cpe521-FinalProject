`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: OTTER_CPU
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

 typedef enum logic [6:0] {
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
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [4:0] rd;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] func3;  //mem_type  //sign, size //also funct3
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input RESET,
                input INTR,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn;
        
    wire [31:0] mem_data;
    wire [31:0] B_immed;
    wire [31:0] J_immed;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic br_lt,br_eq, br_ltu;
    logic wb_enable;
              
     logic stall_pc;
     logic stall_if;
     logic stall_de;
     //The following stages are not stalled in our first piplined otter  (no interrupts, exceptions, memory delays)
     logic stall_ex=0;
     logic stall_mem=0;
     logic stall_wb=0;
     
     logic if_de_invalid=0;
     logic de_ex_invalid=0;
     logic ex_mem_invalid=0;
     logic mem_wb_invalid=0;
     
     initial     
        $monitor("IF: %4h, DE: %4h (%s)\t EX: %4h (%s)\t MEM: %4h (%s)\t WB: %4h (%0s)", pc,de_inst.pc,de_inst.opcode.name(),de_ex_inst.pc,de_ex_inst.opcode.name(),ex_mem_inst.pc,ex_mem_inst.opcode.name(),mem_wb_inst.pc,mem_wb_inst.opcode.name());         
     
//==== Instruction Fetch ===========================================
     logic [31:0] if_de_pc;
     
     always_ff @(posedge CLK)
            if(!stall_if)
                if_de_pc <= pc;
     
     assign pcWrite = !stall_pc;
     assign memRead1 = !stall_if;
     
    //pc target calculations 
    assign next_pc = pc + 4;    //PC is byte aligned, memory is word aligned
          
    assign opcode = IR[6:0]; // opcode shortcut
    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite), .PC_DIN(pc_value), .PC_COUNT(pc));  
    
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
    Mult4to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, pc_sel, pc_value); 
            
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;
    logic [31:0] de_ex_I_immed;
    logic [31:0] de_ex_J_immed;
    logic [31:0] de_ex_B_immed;
    
    instr_t de_ex_inst, de_inst;
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst.rs1=IR[19:15];
    assign de_inst.rs2=IR[24:20];
    assign de_inst.rd=IR[11:7];
    assign de_inst.opcode=OPCODE;
    assign de_inst.alu_fun=alu_fun;
    assign de_inst.rf_wr_sel=wb_sel;
    assign de_inst.pc = if_de_pc;
    assign de_inst.func3=IR[14:12];
   
    assign de_inst.rs1_used=    de_inst.rs1 != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
    assign de_inst.rs2_used=    de_inst.rs2 != 0
                                && ( de_inst.opcode == BRANCH
                                || de_inst.opcode == STORE
                                || de_inst.opcode == OP);
    assign de_inst.rd_used = de_inst.opcode != BRANCH
                                && de_inst.opcode != STORE;    
                                                         
    assign de_inst.regWrite = de_inst.opcode != BRANCH 
                        && de_inst.opcode != STORE;
    assign de_inst.memWrite = de_inst.opcode == STORE;
    assign de_inst.memRead2 = de_inst.opcode == LOAD;                          
        
    
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(opcode), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_BR_EQ(br_eq),.CU_BR_LT(br_lt),.CU_BR_LTU(br_ltu),//.CU_PCSOURCE(pc_sel),
             .CU_ALU_SRCA(opA_sel),.CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel));
     
    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (B, I_immed, S_immed, de_inst.pc, opB_sel, aluBin);
    
    Mult2to1 ALUAinput (A, U_immed, opA_sel, aluAin);

    // Creates a RISC-V register file
    OTTER_registerFile RF (de_inst.rs1, de_inst.rs2, mem_wb_inst.rd, rfIn, wb_enable, A, B, CLK);

    // Generate immediates
    assign S_immed = {{20{IR[31]}},IR[31:25],IR[11:7]};
    assign I_immed = {{20{IR[31]}},IR[31:20]};
    assign U_immed = {IR[31:12],{12{1'b0}}};
    assign B_immed = {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
    assign J_immed = {{12{IR[31]}}, IR[19:12], IR[20],IR[30:21],1'b0};

    always_ff @(posedge CLK) begin
            if(!stall_de) begin           
                de_ex_inst <= de_inst;
                de_ex_opA <=aluAin;
                de_ex_opB <=aluBin;
                de_ex_rs2 <= B;
                de_ex_I_immed <= I_immed;
                de_ex_J_immed <= J_immed;
                de_ex_B_immed <= B_immed;
            end
     end    
      
    //===== HAZARD DETECTION =================================
    //insert 1 bubble on load-use, stall if and de 
    assign ld_use_hazard = !if_de_invalid && !de_ex_invalid && ((de_ex_inst.memRead2 && (de_inst.rs1 == de_ex_inst.rd && de_inst.rs1_used ||
                                              de_inst.rs2 == de_ex_inst.rd && de_inst.rs2_used))) ;                                           
    assign stall_if = ld_use_hazard;
    assign stall_pc = ld_use_hazard;
    assign stall_de = ld_use_hazard;
    
    //For instruction that is branch/jump, if changes the PC,  
    logic branch_taken;
    assign branch_taken = !de_ex_invalid && (pc_sel != 0);    

    always_ff @ (posedge CLK) begin
        if(RESET) begin
            if_de_invalid <= 1;
            de_ex_invalid <= 1;
            ex_mem_invalid <= 1;
            mem_wb_invalid <= 1;
        end
        else begin      
            if(!stall_if) if_de_invalid <= branch_taken;
            if(!stall_de) de_ex_invalid <= if_de_invalid | branch_taken;
            else if (!stall_ex) de_ex_invalid <= 1;  //insert bubble
            
            if(!stall_ex) ex_mem_invalid <= de_ex_invalid;
            if(!stall_mem) mem_wb_invalid <= ex_mem_invalid;
        end
    end

//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic [31:0] rs2_forwarded;
     logic [31:0] ex_mem_aluRes = 0;
     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     
     // Creates a RISC-V ALU
    OTTER_ALU ALU (de_ex_inst.alu_fun, opA_forwarded, opB_forwarded, aluResult); 
     
    //Branch Condition Generator
    always_comb begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(opA_forwarded) < $signed(opB_forwarded)) br_lt=1;
        if(opA_forwarded==opB_forwarded) br_eq=1;
        if(opA_forwarded<opB_forwarded) br_ltu=1;
    end
    
    logic brn_cond;
    always_comb
            case(de_ex_inst.func3)
                3'b000: brn_cond = br_eq;     //BEQ 
                3'b001: brn_cond = ~br_eq;    //BNE
                3'b100: brn_cond = br_lt;     //BLT
                3'b101: brn_cond = ~br_lt;    //BGE
                3'b110: brn_cond = br_ltu;    //BLTU
                3'b111: brn_cond = ~br_ltu;   //BGEU
                default: brn_cond =0;
            endcase
    always_comb begin
                if(!de_ex_invalid) begin
                case(de_ex_inst.opcode)
                    JAL: pc_sel =2'b11;
                    JALR: pc_sel =2'b01;
                    BRANCH: pc_sel=(brn_cond)?2'b10:2'b00;
                    default: pc_sel=2'b00; 
                endcase 
                end else pc_sel=2'b00;
        end
        
    assign jalr_pc = de_ex_I_immed + opA_forwarded;
    assign branch_pc = de_ex_B_immed + de_ex_inst.pc;   //byte aligned addresses
    assign jump_pc = de_ex_J_immed + de_ex_inst.pc;  
     
      always_ff @(posedge CLK) begin
            if(!stall_ex) begin
                ex_mem_aluRes <=aluResult;
                ex_mem_inst <= de_ex_inst;
                ex_mem_rs2 <= rs2_forwarded;
            end
     end   
     
//==== Memory ======================================================
     
   instr_t mem_wb_inst;
   logic [31:0] mem_wb_aluRes;
   
   logic mem_enable;
   assign mem_Wenable = !ex_mem_invalid && ex_mem_inst.memWrite;
   assign mem_Renable = !ex_mem_invalid && ex_mem_inst.memRead2;
     
   OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc),.MEM_ADDR2(ex_mem_aluRes),.MEM_DIN2(ex_mem_rs2),
                               .MEM_WRITE2(mem_Wenable),.MEM_READ1(memRead1),.MEM_READ2(mem_Renable),
                               .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(ex_mem_inst.func3[1:0]),.MEM_SIGN(ex_mem_inst.func3[2]));     
    
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    always_ff @(posedge CLK) begin
            if(!stall_mem) begin
                mem_wb_inst <= ex_mem_inst;
                mem_wb_aluRes <= ex_mem_aluRes;
            end
     end        
     
//==== Write Back ==================================================
    
    assign wb_enable =  !stall_wb && !mem_wb_invalid && mem_wb_inst.regWrite;
    
    //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (mem_wb_inst.pc+4, csr_reg, mem_data, mem_wb_aluRes, mem_wb_inst.rf_wr_sel, rfIn);

       
 //==== Forwarding Logic ===========================================
     
     logic valid_forward_from_mem;
     logic valid_forward_from_wb;
     
    always_comb begin
        valid_forward_from_mem = ex_mem_inst.regWrite && !ex_mem_invalid;
        valid_forward_from_wb = mem_wb_inst.regWrite && !mem_wb_invalid;
        
        opA_forwarded = de_ex_opA;
        opB_forwarded = de_ex_opB; 
        rs2_forwarded = de_ex_rs2;
        
        if(valid_forward_from_mem && ex_mem_inst.rd == de_ex_inst.rs1 && de_ex_inst.rs1_used) 
            opA_forwarded = ex_mem_aluRes;
        else if(valid_forward_from_wb && mem_wb_inst.rd == de_ex_inst.rs1 && de_ex_inst.rs1_used)
            opA_forwarded = rfIn;
   
        if(valid_forward_from_mem && ex_mem_inst.rd == de_ex_inst.rs2 && de_ex_inst.rs2_used)
            if(de_ex_inst.opcode != STORE) opB_forwarded = ex_mem_aluRes;
            else    rs2_forwarded = ex_mem_aluRes;
        else if(valid_forward_from_wb && mem_wb_inst.rd == de_ex_inst.rs2 && de_ex_inst.rs2_used) begin
            if(de_ex_inst.opcode != STORE) opB_forwarded = rfIn;
            else   rs2_forwarded = rfIn;
        end                        
    end   
            
endmodule
