module TPOC(
    input clk,
    input reset,
    input [31:0] nextPC,
    output reg [31:0] currentPC
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            currentPC <= 32'b0;
        else
            currentPC <= nextPC;
    end
endmodule

module InstructionMemory(
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] memory [0:255];
    
    initial begin
        // Carregar o programa (exemplo hardcoded)
        memory[0] = 32'b00000000000100000000000110010011; // addi x3, x0, 1
        memory[1] = 32'b00000000001000001000001000110011; // add x4, x1, x2
        // Adicionar mais instruções conforme necessário
    end
    
    always @(address) begin
        instruction = memory[address >> 2];
    end
endmodule

module RegisterFile(
    input clk,
    input [4:0] rs1, rs2, rd,
    input [31:0] writeData,
    input regWrite,
    output [31:0] readData1, readData2
);
    reg [31:0] registers [0:31];
    
    // Leitura dos registradores
    assign readData1 = registers[rs1];
    assign readData2 = registers[rs2];
    
    // Escrita nos registradores
    always @(posedge clk) begin
        if (regWrite)
            registers[rd] <= writeData;
    end
endmodule

module ImmGen(
    input [31:0] instruction,
    output reg [31:0] immOut
);
    always @(*) begin
        case (instruction[6:0])
            7'b0010011, // I-type (ADDI, ANDI, ORI)
            7'b0000011, // Load (LW)
            7'b0001011: // Load halfword (LH)
                immOut = {{20{instruction[31]}}, instruction[31:20]};
            7'b0100011: // Store (SW)
                immOut = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b0101011: // Store halfword (SH)
                immOut = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: // Branch (BEQ, BNE)
                immOut = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            7'b1101111: // Jump (JAL)
                immOut = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            default: 
                immOut = 32'b0;
        endcase
    end
endmodule

module ControlUnit(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] aluControl,
    output reg regWrite,
    output reg aluSrc,
    output reg memWrite,
    output reg branch,
    output reg jump
);
    always @(*) begin
        // Definir valores padrão
        aluControl = 4'b0000;
        regWrite = 0;
        aluSrc = 0;
        memWrite = 0;
        branch = 0;
        jump = 0;

        case(opcode)
            7'b0110011: begin // R-type
                regWrite = 1;
                case(funct3)
                    3'b000: aluControl = (funct7 == 7'b0000000) ? 4'b0000 : 4'b0001; // ADD/SUB
                    3'b110: aluControl = 4'b0011; // OR
                    3'b001: aluControl = 4'b0101; // SLL
                endcase
            end
            7'b0010011: begin // I-type (ADDI, ANDI)
                regWrite = 1;
                aluSrc = 1;
                case(funct3)
                    3'b000: aluControl = 4'b0000; // ADDI
                    3'b111: aluControl = 4'b0010; // ANDI
                endcase
            end
            7'b0001011: begin // Load halfword (LH)
                regWrite = 1;
                aluSrc = 1;
                aluControl = 4'b0000; // ADD
            end
            7'b0100011: begin // Store word (SW)
                memWrite = 1;
                aluSrc = 1;
                aluControl = 4'b0000; // ADD
            end
            7'b0101011: begin // Store halfword (SH)
                memWrite = 1;
                aluSrc = 1;
                aluControl = 4'b0000; // ADD
            end
            7'b1100011: begin // Branch (BNE)
                branch = 1;
                case(funct3)
                    3'b001: aluControl = 4'b0001; // BNE
                endcase
            end
            7'b1101111: begin // Jump (JAL)
                jump = 1;
            end
        endcase
    end
endmodule

module ALU(
    input [31:0] srcA,
    input [31:0] srcB,
    input [3:0] aluControl,
    output reg [31:0] aluResult,
    output zero
);
    assign zero = (aluResult == 0);

    always @(*) begin
        case(aluControl)
            4'b0000: aluResult = srcA + srcB; // ADD
            4'b0001: aluResult = srcA - srcB; // SUB
            4'b0010: aluResult = srcA & srcB; // AND
            4'b0011: aluResult = srcA | srcB; // OR
            4'b0101: aluResult = srcA << srcB[4:0]; // SLL
            default: aluResult = 32'b0;
        endcase
    end
endmodule

module DataMemory(
    input clk,
    input memWrite,
    input [31:0] address,
    input [31:0] writeData,
    output reg [31:0] readData
);
    reg [31:0] memory [0:255];
    
    always @(posedge clk) begin
        if (memWrite)
            memory[address >> 2] <= writeData;
    end
    
    always @(*) begin
        readData = memory[address >> 2];
    end
endmodule

module RISCV_Simplified(
    input clk,
    input reset
);
    wire [31:0] currentPC, nextPC, instruction;
    wire [4:0] rs1, rs2, rd;
    wire [31:0] readData1, readData2, writeData, immGen, aluResult, memData;
    wire regWrite, aluSrc, memWrite, branch, jump;
    wire [3:0] aluControl;
    wire zero;

    ProgramCounter pc(
        .clk(clk),
        .reset(reset),
        .nextPC(nextPC),
        .currentPC(currentPC)
    );

    InstructionMemory imem(
        .address(currentPC),
        .instruction(instruction)
    );

    RegisterFile regfile(
        .clk(clk),
        .rs1(instruction[19:15]),
        .rs2(instruction[24:20]),
        .rd(instruction[11:7]),
        .writeData(writeData),
        .regWrite(regWrite),
        .readData1(readData1),
        .readData2(readData2)
    );

    ALU alu(
        .srcA(readData1),
        .srcB(aluSrc ? immGen : readData2),
        .aluControl(aluControl),
        .aluResult(aluResult),
        .zero(zero)
    );

    DataMemory dmem(
        .clk(clk),
        .memWrite(memWrite),
        .address(aluResult),
        .writeData(readData2),
        .readData(memData)
    );

    ControlUnit control(
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .aluControl(aluControl),
        .regWrite(regWrite),
        .aluSrc(aluSrc),
        .memWrite(memWrite),
        .branch(branch),
        .jump(jump)
    );

    ImmGen immgen(
        .instruction(instruction),
        .immOut(immGen)
    );

    assign writeData = memWrite ? memData : aluResult;
    assign nextPC = (branch && !zero) ? (currentPC + immGen) : (jump ? (currentPC + immGen) : (currentPC + 4));
endmodule
