// Luis Pedro Molina Velásquez - Carné 18822
// Electrónica Digital I - Sección 12
// Proyecto Final - Procesador Nibbler de 4 bits


// Módulo de Program Counter

module PC(input wire clk, input wire reset, input wire enable, input wire nonb, input wire [11:0]load, output reg [11:0]exit);

    always @ (posedge clk or posedge reset)begin 
        if (reset == 1)                                             // Empieza en 0
            exit <= 12'b000000000000;
            
        else if(nonb == 1)                                          // Cuando nonb = 1 --> Se carga el valor de load al PC
            exit <= load;

        else if(enable == 1 && ~nonb)                               // Con cada flanco de reloj --> +1
        exit <= exit + 1;

    end

endmodule

// Módulo de Memoria ROM

module ROM(input [11:0]add, output [7:0]information); 

    reg [7:0] Memory [0:4095];                                  // Cantidad de localidades de memoria --> 4096, 8 bits
        
    initial begin
        $readmemh("memory.list", Memory);                       // Initial para leer la información existente en el archivo
    end

    assign information = Memory[add];                           // Asignación de valor a la memoria

endmodule

// Construcción de Flip Flops tipo D de 1, 2 y 4 bits. Estos Flip Flops cuentan con enable y se utilizaron como base para la construcción del módulo Fetch
// Flip Flop tipo D con enable --> 1 bit ;

module FFD1B(input wire clk, input wire reset, input wire enable, input wire D, output reg Q);

    always @ (posedge clk or posedge reset) begin
        if (reset) 
            Q <= 1'b0;
        else if (enable)
            Q <= D;
    end
endmodule

// Flip Flop tipo D con enable --> 2 bits ; Su construcción tiene como base la arquitectura del Flip Flop tipo D de 1 bit con enable

module FFD2B(input wire clk, input wire reset, input wire enable, input wire [1:0]D, output wire [1:0]Q);

    FFD1B ffd1b_0(clk, reset, enable, D[1], Q[1]);
    FFD1B ffd1b_1(clk, reset, enable, D[0], Q[0]);

endmodule

// Flip Flop tipo D con enable --> 4 bits ; Su construcción tiene como base la arquitectura del Flip Flop tipo D de 1 y 2 bits con enable

module FFD4B(input wire clk, input wire reset, input wire enable, input wire [3:0]D, output wire [3:0]Q);

    FFD2B ffd2b_0(clk, reset, enable, D[3:2], Q[3:2]);
    FFD2B ffd2b_1(clk, reset, enable, D[1:0], Q[1:0]);

endmodule

// Módulo de Fetch

module Fetch(input wire clk, input wire reset, input wire En2, input wire [7:0]D4, output wire [3:0]Q1,output wire [3:0]Q2);

    FFD4B fetch_0(clk, reset, En2, D4[7:4], Q1);                         // Primera salida del Fetch
    FFD4B fetch_1(clk, reset, En2, D4[3:0], Q2);                         // Segunda salida del Fetch

endmodule

// Módulo de Flags

module Flags(input wire clk, input wire reset, input wire enable, input wire D1, input wire D2, output wire Ze, C);

    FFD1B flags_0(clk, reset, enable, D1, Ze);
    FFD1B flags_1(clk, reset, enable, D2, C);

endmodule

// Flip Flop tipo T para la construcción del módulo de Phase

module FFT(input wire clk, input wire reset, input wire EN, output wire Q);


    FFD1B phasee(clk, reset, EN, ~Q, Q);                               // Input --> ~Q

endmodule

// Módulo de Bus Driver

module BufferTri(input wire enable, input wire [3:0]entradas, output wire [3:0]salidas);

    assign salidas = (enable) ? entradas:4'bz;

endmodule

// Módulo de Decode

module Decode(input wire [6:0] in, output wire [12:0] outD);
    
    reg [12:0] out;
    
    always @ (in)
        casez(in)
            // any
            7'b????_??0: out <= 13'b1000_000_001000;
            // JC
            7'b0000_1?1: out <= 13'b0100_000_001000;
            7'b0000_0?1: out <= 13'b1000_000_001000;
            // JNC
            7'b0001_1?1: out <= 13'b1000_000_001000;
            7'b0001_0?1: out <= 13'b0100_000_001000;
            // CMPI
            7'b0010_??1: out <= 13'b0001_001_000010;
            // CMPM
            7'b0011_??1: out <= 13'b1001_001_100000;
            // LIT
            7'b0100_??1: out <= 13'b0011_010_000010;
            // IN
            7'b0101_??1: out <= 13'b0011_010_000100;
            // LD
            7'b0110_??1: out <= 13'b1011_010_100000;
            // ST
            7'b0111_??1: out <= 13'b1000_000_111000;
            // JZ
            7'b1000_?11: out <= 13'b0100_000_001000;
            7'b1000_?01: out <= 13'b1000_000_001000;
            // JNZ
            7'b1001_?11: out <= 13'b1000_000_001000;
            7'b1001_?01: out <= 13'b0100_000_001000;
            // ADDI
            7'b1010_??1: out <= 13'b0011_011_000010;
            // ADDM
            7'b1011_??1: out <= 13'b1011_011_100000;
            // JMP
            7'b1100_??1: out <= 13'b0100_000_001000;
            // OUT
            7'b1101_??1: out <= 13'b0000_000_001001;
            // NANDI
            7'b1110_??1: out <= 13'b0011_100_000010;
            // NANDM
            7'b1111_??1: out <= 13'b1011_100_100000;
            default: out <= 13'b1111111111111;
        endcase
        
    assign outD = out;

endmodule

// Módulo de Memoria RAM

module RAM(input wire enable, input wire write, input wire [11:0]addres, output wire [3:0]Data);

    reg [3:0]RAM[0:4095];                                             // Cantidad de localidades de la Memoria RAM
    reg [3:0]dataout;                                                 // Declaración de  salidas de la Memoria RAM
    assign Data = (enable && ~write) ? dataout: 4'bzzzz;              // Condición para mostrar alta impedancia

    always @(enable, write, addres, Data) begin 
        
        if (enable && ~write)
            dataout = RAM[addres];
        if (enable && write)
            RAM[addres] = Data; 
            
        end

endmodule

// Módulo de Acumulador

module ACCU(input wire clk, input wire reset, input wire enable, input wire [3:0]D, output wire [3:0]Q);

    FFD2B accu_0(clk, reset, enable, D[3:2], Q[3:2]);
    FFD2B accu_1(clk, reset, enable, D[1:0], Q[1:0]);

endmodule

// Módulo de ALU

module ALU(input wire [3:0] A, input wire [3:0] B, input [2:0] comm, output carry, output zero, output [3:0] result);
    
    reg [4:0] q;
    
    always @ (A, B, comm)
        case (comm)
            3'b000: q = A;                                      // Dejar pasar A
            3'b001: q = A - B;                                  // Resta
            3'b010: q = B;                                      // Dejar pasar B
            3'b011: q = A + B;                                  // Sumar
            3'b100: q = {1'b0, ~(A & B)};                       // Comparador
            default: q = 5'b10101;
        endcase
    
    assign result = q[3:0];
    assign carry = q[4];
    assign zero = ~(q[3] | q[2] | q[1] | q[0]);
    
endmodule

// Módulo uP --> Unión de módulos para el correcto funcionamiento del Procesador 

module uP(
    input wire clock, reset,
    input wire [3:0]pushbuttons,
    output wire phase, c_flag, z_flag,
    output wire [3:0] instr, oprnd, accu, data_bus, FF_out,
    output wire [7:0] program_byte,
    output wire [11:0] PC, address_RAM);

        wire [3:0] ALU_out;
        wire [12:0]out_decoder;
        wire [6:0]in_decoder;
        wire Ze, C;

        assign address_RAM = {oprnd, program_byte};
        assign in_decoder = {instr, c_flag, z_flag, phase};

        PC ProgramCounter(clock, reset, out_decoder[12], out_decoder[11], address_RAM, PC);
        ROM rom(PC, program_byte);
        Fetch Fetch(clock, reset, ~phase, program_byte, instr, oprnd);
        Flags flags_carry(clock, reset, out_decoder[9], Ze, C, z_flag, c_flag);
        FFT Phase(clock, reset, 1'b1, phase);
        BufferTri BusFetch(out_decoder[1], oprnd, data_bus);
        BufferTri BusALU(out_decoder[3], ALU_out , data_bus);
        BufferTri BusIn(out_decoder[2], pushbuttons, data_bus);
        Decode Decoder(in_decoder, out_decoder);
        RAM ram(out_decoder[5], out_decoder[4], address_RAM, data_bus);
        ACCU Accumulator(clock, reset, out_decoder[10], ALU_out, accu);
        ALU alu(accu, data_bus, out_decoder[8:6], C, Ze, ALU_out);        
        FFD4B OutputsFFD4(clock, reset, out_decoder[0], data_bus, FF_out);
    
endmodule   