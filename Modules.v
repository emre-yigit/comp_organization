`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU BBF
// Engineer: Emre Yigit, Eren Tasdemir, Berkant Bakisli
// 
// Create Date: 05/09/2022 11:04:59 AM
// Design Name: 
// Module Name: Modules
// Project Name: Computer Organization Project 2
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


module Register8Bit(E, FunSel, I, Q, CLK);
    input wire E, CLK;
    input wire [1:0] FunSel;
    input wire [7:0] I;
    output reg [7:0] Q;
    
    always @(posedge CLK) begin
        if (E) begin
            case (FunSel)
                2'b00: Q <= Q - 1; //Decrement
                2'b01: Q <= Q + 1; //Increment
                2'b10: Q <= I; //Load I
                2'b11: Q <= 8'h00; //Clear
            endcase
        end
    end
endmodule

module Register16Bit(E, FunSel, I, Q, CLK);
    input wire E, CLK;
    input wire [1:0] FunSel;
    input wire [15:0] I;
    output reg [15:0] Q;
    
    always @(posedge CLK) begin
        if (E) begin
            case (FunSel)
                2'b00: Q <= Q - 1; //Decrement
                2'b01: Q <= Q + 1; //Increment
                2'b10: Q <= I; //Load I
                2'b11: Q <= 16'h0000; //Clear
            endcase
        end
    end
endmodule

module RegisterFile(OutASel, OutBSel, FunSel, RegSel, I, OutA, OutB, CLK);
    input wire CLK;
    input wire [1:0] OutASel;
    input wire [1:0] OutBSel;
    input wire [1:0] FunSel;
    input wire [3:0] RegSel;
    input wire [7:0] I;
    output reg [7:0] OutA;
    output reg [7:0] OutB;
    
    wire E1, E2, E3, E4;
    assign E1 = ~RegSel[3];
    assign E2 = ~RegSel[2];
    assign E3 = ~RegSel[1];
    assign E4 = ~RegSel[0];
    
    wire [7:0] Q1, Q2, Q3, Q4;
    
    Register8Bit R1(.E(E1), .FunSel(FunSel), .I(I), .Q(Q1), .CLK(CLK));
    Register8Bit R2(.E(E2), .FunSel(FunSel), .I(I), .Q(Q2), .CLK(CLK));
    Register8Bit R3(.E(E3), .FunSel(FunSel), .I(I), .Q(Q3), .CLK(CLK));
    Register8Bit R4(.E(E4), .FunSel(FunSel), .I(I), .Q(Q4), .CLK(CLK));
    
    always @(*) begin
        case (OutASel)
            2'b00: OutA <= Q1;
            2'b01: OutA <= Q2;
            2'b10: OutA <= Q3;
            2'b11: OutA <= Q4;
        endcase    
    end
    
    always @(*) begin
        case (OutBSel)
            2'b00: OutB <= Q1;
            2'b01: OutB <= Q2;
            2'b10: OutB <= Q3;
            2'b11: OutB <= Q4;
        endcase    
    end
endmodule

module AddressRegisterFile(OutCSel, OutDSel, FunSel, RegSel, I, OutC, OutD, CLK);
    input wire CLK;
    input wire [1:0] OutCSel;
    input wire [1:0] OutDSel;
    input wire [1:0] FunSel;
    input wire [2:0] RegSel;
    input wire [7:0] I;
    output reg [7:0] OutC;
    output reg [7:0] OutD;

    wire E1, E2, E3;
    assign E1 = ~RegSel[2];
    assign E2 = ~RegSel[1];
    assign E3 = ~RegSel[0];

    wire [7:0] Q1, Q2, Q3;

    Register8Bit PC(.E(E1), .FunSel(FunSel), .I(I), .Q(Q1), .CLK(CLK));
    Register8Bit AR(.E(E2), .FunSel(FunSel), .I(I), .Q(Q2), .CLK(CLK));
    Register8Bit SP(.E(E3), .FunSel(FunSel), .I(I), .Q(Q3), .CLK(CLK));

    always @(*) begin
        case (OutCSel)
            2'b00: OutC <= Q1;
            2'b01: OutC <= Q1;
            2'b10: OutC <= Q2;
            2'b11: OutC <= Q3;
        endcase    
    end

    always @(*) begin
        case (OutDSel)
            2'b00: OutD <= Q1;
            2'b01: OutD <= Q1;
            2'b10: OutD <= Q2;
            2'b11: OutD <= Q3;
        endcase    
    end
endmodule

module IR(E, L_H, FunSel, I, IRout, CLK);
    input wire E, CLK;
    input wire L_H;
    input wire [1:0] FunSel;
    input wire [7:0] I;
    output wire [15:0] IRout;
    
    reg [15:0] Iin;
    
    Register16Bit register(.E(E), .FunSel(FunSel), .I(Iin), .Q(IRout), .CLK(CLK));
    
    always @(*) begin
        if (~L_H) begin //Load higher bits
            Iin <= {I, IRout[7:0]};
        end
        else begin //Load lower bits
            Iin <= {IRout[15:8], I};
        end
    end
endmodule

module FlagRegister(Zi, Ci, Ni, Oi, E, Z, C, N, O, CLK);
    input wire CLK;
    input wire [3:0] E;
    input wire Zi, Ci, Ni, Oi;
    output reg Z, C, N, O;
    
    always @(negedge CLK) begin
        if (E[3]) begin
            Z <= Zi;
        end
        if (E[2]) begin
            C <= Ci;
        end
        if (E[1]) begin
            N <= Ni;
        end
        if (E[0]) begin
            O <= Oi;
        end
    end
endmodule

module ALU(A, B, Cin, FunSel, Z, C, N, O, ZCNO_E, OutALU);
    input wire [7:0] A;
    input wire [7:0] B;
    input wire Cin;
    input wire [3:0] FunSel;
    output reg Z, C, N, O;
    output reg [7:0] OutALU;

    reg [8:0] res;
    output reg [3:0] ZCNO_E;

    always @(*) begin
        case (FunSel)
            4'b0000: begin
                OutALU <= A;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b0001: begin
                OutALU <= B;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b0010: begin
                OutALU <= ~A;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b0011: begin
                OutALU <= ~B;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b0100: begin
                res <= A + B;
                OutALU <= res[7:0];
                C <= res[8];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1111;
            end
            4'b0101: begin
                res <= A + B + Cin;
                OutALU <= res[7:0];
                C <= res[8];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1111;
            end
            4'b0110: begin
                res <= A - B;
                OutALU <= res[7:0];
                C <= res[8];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1111;
            end
            4'b0111: begin
                OutALU <= A&B;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b1000: begin
                OutALU <= A|B;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b1001: begin
                OutALU <= A^B;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1010;
            end
            4'b1010: begin
                OutALU <= A<<1;
                C <= A[7];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1110;
            end
            4'b1011: begin
                OutALU <= A>>1;
                C <= A[0];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1110;
            end
            4'b1100: begin
                OutALU <= A<<1;
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1011;
            end
            4'b1101: begin
                OutALU <= {A[7], A[7:1]};
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                ZCNO_E <= 4'b1000;
            end
            4'b1110: begin
                OutALU <= {A[6:0], Cin};
                C <= A[7];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1111;
            end
            4'b1111: begin
                OutALU <= {Cin, A[7:1]};
                C <= A[0];
                Z <= OutALU == 8'h00 ? 1'b1 : 1'b0;
                N <= OutALU[7] == 1'b1 ? 1'b1 : 1'b0;
                O <= (OutALU[7])^(A[7]&B[7]);
                ZCNO_E <= 4'b1111;
            end
        endcase
    end
endmodule

module ALUSystem(RF_OutASel, RF_OutBSel, RF_FunSel, RF_RegSel, ALU_FunSel,
    ARF_OutCSel, ARF_OutDSel, ARF_FunSel, ARF_RegSel, IR_LH, IR_Enable,
    IR_Funsel, Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel, Clock, IROut, ALUOutFlag);
    input wire Clock;
    input wire Mem_WR, Mem_CS;
    input wire [1:0] MuxASel;
    input wire [1:0] MuxBSel;
    input wire MuxCSel;    
    
    input wire [1:0] RF_OutASel;
    input wire [1:0] RF_OutBSel;
    input wire [1:0] RF_FunSel;
    input wire [3:0] RF_RegSel;
    
    input wire [3:0] ALU_FunSel;
    
    input wire [1:0] ARF_OutCSel;
    input wire [1:0] ARF_OutDSel;
    input wire [1:0] ARF_FunSel;
    input wire [2:0] ARF_RegSel;
    
    input wire IR_LH, IR_Enable;
    input wire [1:0] IR_Funsel;

    output wire [15:0] IROut;  
    output wire [3:0] ALUOutFlag;
    
    reg [7:0] MuxAOut;
    wire [7:0] AOut, BOut;
    wire [7:0] ARF_COut, Address;
    reg [7:0] MuxCOut;  
    wire [7:0] ALUOut;
    wire [3:0] ZCNO_Out, ZCNO_E;
    wire [7:0] MemoryOut;
    reg [7:0] MuxBOut;
    
    RegisterFile _RF(
    .OutASel(RF_OutASel),
    .OutBSel(RF_OutBSel), 
    .FunSel(RF_FunSel),
    .RegSel(RF_RegSel),
    .I(MuxAOut),
    .OutA(AOut),
    .OutB(BOut),
    .CLK(Clock));    
    
    always @(*) begin
        case (MuxCSel)
            1'b0: MuxCOut <= ARF_COut;
            1'b1: MuxCOut <= AOut;
        endcase
    end
    
    FlagRegister _FlagRegister(
        .Zi(ZCNO_Out[3]),
        .Ci(ZCNO_Out[2]),
        .Ni(ZCNO_Out[1]),
        .Oi(ZCNO_Out[0]),
        .E(ZCNO_E),
        .Z(ALUOutFlag[3]),
        .C(ALUOutFlag[2]),
        .N(ALUOutFlag[1]),
        .O(ALUOutFlag[0]),
        .CLK(Clock));
        
    ALU _ALU(
    .A(MuxCOut),
    .B(BOut),
    .Cin(ALUOutFlag[2]),
    .Z(ZCNO_Out[3]),
    .C(ZCNO_Out[2]),
    .N(ZCNO_Out[1]),
    .O(ZCNO_Out[0]),
    .ZCNO_E(ZCNO_E),
    .FunSel(ALU_FunSel),
    .OutALU(ALUOut));
    
    Memory _MEM(
        .address(Address),
        .data(ALUOut),
        .wr(Mem_WR),
        .cs(Mem_CS),
        .clock(Clock),
        .o(MemoryOut)
    );
    
    always @(*) begin
        case (MuxBSel)
            2'b00: MuxBOut <= ALUOut;
            2'b01: MuxBOut <= IROut[7:0];
            2'b10: MuxBOut <= MemoryOut;
            2'b11: MuxBOut <= ALUOut;
        endcase
        case (MuxASel)
            2'b00: MuxAOut <= IROut[7:0];
            2'b01: MuxAOut <= MemoryOut;
            2'b10: MuxAOut <= ARF_COut;
            2'b11: MuxAOut <= ALUOut;
        endcase
    end
    
    AddressRegisterFile _ARF(
    .OutCSel(ARF_OutCSel),
    .OutDSel(ARF_OutDSel),
    .FunSel(ARF_FunSel),
    .RegSel(ARF_RegSel),
    .I(MuxBOut),
    .OutC(ARF_COut),
    .OutD(Address),
    .CLK(Clock));    
    
    IR _IR(.E(IR_Enable), .L_H(IR_LH), .FunSel(IR_Funsel), .I(MemoryOut), .IRout(IROut), .CLK(Clock));
endmodule

module System(RESET, Clock);

    input wire RESET, Clock;

    reg Mem_WR, Mem_CS;
    reg [1:0] MuxASel;
    reg [1:0] MuxBSel;
    reg MuxCSel;    
    
    reg [1:0] RF_OutASel;
    reg [1:0] RF_OutBSel;
    reg [1:0] RF_FunSel;
    reg [3:0] RF_RegSel;
    
    reg [3:0] ALU_FunSel;
    
    reg [1:0] ARF_OutCSel;
    reg [1:0] ARF_OutDSel;
    reg [1:0] ARF_FunSel;
    reg [2:0] ARF_RegSel;
    
    reg IR_LH, IR_Enable;
    reg [1:0] IR_FunSel;

    wire [15:0] IROut;
    wire [3:0] ALUOutFlag;
    reg [3:0] LastALUOutFlag;

    reg [3:0] seq_counter;

    //Reset registers
    //NOTE: Reset must be held for one clock cycle (?)
    always@(posedge RESET) begin
        seq_counter <= 4'h0;

        //Reset Register File
        RF_FunSel <= 2'b11;
        RF_RegSel <= 4'b0000;

        //Reset Address Register File
        ARF_FunSel <= 2'b11;
        ARF_RegSel <= 3'b000;

        //Reset Instruction Register
        IR_Enable <= 1'b1;
        IR_FunSel <= 2'b11;
    end

    //Increment sequence counter with every clock cycle, reset if operation is done 
    always @(posedge Clock) begin  
        if (!RESET) begin
            seq_counter <= seq_counter + 1;
        end
    end

    always @(*) begin
        if (!RESET) begin
            if (seq_counter == 4'h0) begin
                LastALUOutFlag <= ALUOutFlag;

                //Fetch: Load LSB of the instruction from RAM to IR
                Mem_CS <= 1'b0;
                Mem_WR <= 1'b0;
                IR_FunSel <= 2'b10;
                IR_Enable <= 1'b1;
                IR_LH <= 1'b1; //Load LSB

                ARF_OutDSel <= 2'b00; //Address is read from Program Counter

                //Increment PC
                ARF_FunSel <= 2'b01;
                ARF_RegSel <= 3'b011;

                //Disable unused registers
                RF_RegSel <= 4'b1111;
            end
            else if (seq_counter == 4'h1) begin
                //Fetch: Load MSB of the instruction from RAM to IR
                Mem_CS <= 1'b0;
                Mem_WR <= 1'b0;
                IR_FunSel <= 2'b10;
                IR_Enable <= 1'b1;
                IR_LH <= 1'b0; //Load MSB

                ARF_OutDSel <= 2'b00; //Address is read from Program Counter

                //Increment PC
                ARF_FunSel <= 2'b01;
                ARF_RegSel <= 3'b011;

                //Disable unused registers
                RF_RegSel <= 4'b1111;
            end
            else begin
                if (seq_counter == 4'h2)
                    IR_Enable <= 1'b0;

                case (IROut[15:12])
                    4'h0: begin
                        // BRA
                        if (seq_counter == 4'h2) begin
                            RF_RegSel <= 4'b1111;
                            ARF_RegSel <= 3'b011;
                            ARF_FunSel <= 2'b10;
                            MuxBSel <= 2'b01;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h1: begin
                        // LD
                        if (seq_counter == 4'h2) begin
                            ARF_RegSel <= 3'b111;
                            case (IROut[9:8])
                                2'b00: RF_RegSel <= 4'b0111;
                                2'b01: RF_RegSel <= 4'b1011;
                                2'b10: RF_RegSel <= 4'b1101;
                                2'b11: RF_RegSel <= 4'b1110;
                            endcase
                            RF_FunSel <= 2'b10;
                            case (IROut[10])
                                1'b0: begin
                                    MuxASel <= 2'b01;
                                    ARF_OutDSel <= 2'b10;
                                end
                                1'b1: MuxASel <= 2'b00;
                            endcase
                            Mem_WR <= 0;
                            Mem_CS <= 0;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h2: begin
                        // ST
                        if (seq_counter == 4'h2) begin
                            RF_RegSel <= 4'b1111;
                            ARF_RegSel <= 3'b111;
                            ARF_OutDSel <= 2'b10;
                            RF_OutBSel <= IROut[9:8];
                            ALU_FunSel <= 4'b0001;
                            Mem_WR <= 1;
                            Mem_CS <= 0;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h3: begin
                        // MOV
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b0000;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h4: begin
                        // AND
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            RF_OutBSel <= IROut[1:0];
                            ALU_FunSel <= 4'b0111;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h5: begin
                        // OR
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            RF_OutBSel <= IROut[1:0];
                            ALU_FunSel <= 4'b1000;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h6: begin
                        // NOT
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b0010;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h7: begin
                        // ADD
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            RF_OutBSel <= IROut[1:0];
                            ALU_FunSel <= 4'b0100;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'h8: begin
                        // SUB
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            RF_OutBSel <= IROut[1:0];
                            ALU_FunSel <= 4'b0110;
                        end
                        // Now we have SRCREG1 - SRCREG2 in DESTREG
                        if (seq_counter == 4'h3) begin
                            MuxCSel <= IROut[10];
                            case (IROut[10])
                                1'b0: ARF_OutCSel <= IROut[9:8];
                                1'b1: RF_OutASel <= IROut[9:8];
                            endcase
                            ALU_FunSel <= 4'b0010;
                        end
                        // 1's Complement of DESTREG --> -DESTREG
                        if (seq_counter == 4'h4) begin
                            case (IROut[10])
                                1'b0: ARF_FunSel <= 2'b01;
                                1'b1: RF_FunSel <= 2'b01;
                            endcase
                        end
                        if (seq_counter == 4'h5) seq_counter <= 0;
                    end
                    4'h9: begin
                        // LSR
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b1011;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'hA: begin
                        // LSL
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b1010;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'hB: begin
                        // PUL
                        if (seq_counter == 4'h2) begin
                            MuxASel <= 2'b01;
                            case (IROut[9:8])
                                2'b00: RF_RegSel <= 4'b0111;
                                2'b01: RF_RegSel <= 4'b1011;
                                2'b10: RF_RegSel <= 4'b1101;
                                2'b11: RF_RegSel <= 4'b1110;
                            endcase
                            ARF_OutDSel <= 2'b11;
                            ARF_FunSel <= 2'b01;
                            ARF_RegSel <= 3'b110;
                            Mem_CS <= 0;
                            Mem_WR <= 0;
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                    4'hC: begin
                        // PSH
                        if (seq_counter == 4'h2) begin
                            ARF_FunSel <= 2'b00;
                            ARF_RegSel <= 3'b110;
                            RF_RegSel <= 4'b1111;
                        end
                        if (seq_counter == 4'h3) begin
                            ALU_FunSel <= 4'b0001;
                            RF_OutBSel <= IROut[9:8];
                            ARF_OutDSel <= 2'b11;
                            ARF_RegSel <= 3'b111;
                            Mem_CS <= 0;
                            Mem_WR <= 1;
                        end
                        if (seq_counter == 4'h4) seq_counter <= 0;
                    end
                    4'hD: begin
                        // INC
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b0000;
                        end
                        if (seq_counter == 4'h3) begin
                            case (IROut[10])
                                1'b0: ARF_FunSel <= 2'b01;
                                1'b1: RF_FunSel <= 2'b01;
                            endcase
                        end
                        if (seq_counter == 4'h4) begin
                            // Additional cycle required to update the ALUOutFlag as this operation is done on the register instead of ALU.
                            ARF_RegSel <= 3'b111;
                            ARF_OutCSel <= IROut[9:8];
                            RF_RegSel <= 4'b1111;
                            RF_OutASel <= IROut[9:8];
                            MuxCSel <= IROut[10];
                            ALU_FunSel <= 4'b0000;
                        end
                        if (seq_counter == 4'h5) seq_counter <= 0;
                    end
                    4'hE: begin
                        // DEC
                        if (seq_counter == 4'h2) begin
                            case (IROut[10])
                                1'b0: begin
                                    RF_RegSel <= 4'b1111;
                                    case (IROut[9:8])
                                        2'b00: ARF_RegSel <= 3'b011;
                                        2'b01: ARF_RegSel <= 3'b011;
                                        2'b10: ARF_RegSel <= 3'b101;
                                        2'b11: ARF_RegSel <= 3'b110;
                                    endcase
                                    ARF_FunSel <= 2'b10;
                                end
                                1'b1: begin
                                    ARF_RegSel <= 3'b111;
                                    case (IROut[9:8])
                                        2'b00: RF_RegSel <= 4'b0111;
                                        2'b01: RF_RegSel <= 4'b1011;
                                        2'b10: RF_RegSel <= 4'b1101;
                                        2'b11: RF_RegSel <= 4'b1110;
                                    endcase
                                    RF_FunSel <= 2'b10;
                                end
                            endcase
                            MuxASel <= 2'b11;
                            MuxBSel <= 2'b11;
                            MuxCSel <= IROut[6];
                            case (IROut[6])
                                1'b0: ARF_OutCSel <= IROut[5:4];
                                1'b1: RF_OutASel <= IROut[5:4];
                            endcase
                            ALU_FunSel <= 4'b0000;
                        end
                        if (seq_counter == 4'h3) begin
                            case (IROut[10])
                                1'b0: ARF_FunSel <= 2'b00;
                                1'b1: RF_FunSel <= 2'b00;
                            endcase
                        end
                        if (seq_counter == 4'h4) begin
                            // Additional cycle required to update the ALUOutFlag as this operation is done on the register instead of ALU.
                            ARF_RegSel <= 3'b111;
                            ARF_OutCSel <= IROut[9:8];
                            RF_RegSel <= 4'b1111;
                            RF_OutASel <= IROut[9:8];
                            MuxCSel <= IROut[10];
                            ALU_FunSel <= 4'b0000;
                        end
                        if (seq_counter == 4'h5) seq_counter <= 0;
                    end
                    4'hF: begin
                        // BNE
                        if (seq_counter == 4'h2) begin
                            RF_RegSel <= 4'b1111;
                            MuxBSel = 2'b01;
                            ARF_FunSel <= 2'b10;
                            if (LastALUOutFlag[3] == 0) begin
                                ARF_RegSel <= 3'b011;
                            end
                            else begin
                                ARF_RegSel <= 3'b111;
                            end
                        end
                        if (seq_counter == 4'h3) seq_counter <= 0;
                    end
                endcase
            end
        end
    end

    ALUSystem _ALUSystem(
    .RF_OutASel(RF_OutASel), 
    .RF_OutBSel(RF_OutBSel), 
    .RF_FunSel(RF_FunSel),
    .RF_RegSel(RF_RegSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutCSel), 
    .ARF_OutDSel(ARF_OutDSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RegSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_FunSel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock),
    .ALUOutFlag(ALUOutFlag),
    .IROut(IROut)
    );

endmodule