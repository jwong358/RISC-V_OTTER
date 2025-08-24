`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly San Luis Obispo
// Engineer: Diego Curiel
// Create Date: 02/09/2023 11:02:37 AM
// Module Name: ImmediateGenerator
//////////////////////////////////////////////////////////////////////////////////

module ImmediateGenerator(
    input logic [31:0] IR,
    output logic [31:0] I_TYPE,
    output logic [31:0] S_TYPE,
    output logic [31:0] B_TYPE,
    output logic [31:0] U_TYPE,
    output logic [31:0] J_TYPE
    );
    
    //Assign each immediate value. 
    assign I_TYPE = {{21{IR[31]}}, IR[30:20]};
    assign S_TYPE = {{21{IR[31]}},IR[30:25],IR[11:7]};
    assign B_TYPE = {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};
    assign U_TYPE = {IR[31:12], 12'b0};
    assign J_TYPE = {{12{IR[31]}},IR[19:12],IR[20],IR[30:21],1'b0};
    
endmodule
