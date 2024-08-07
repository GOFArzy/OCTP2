`timescale 1ns/100ps

module RISCV_Simplified_TB;

    reg clk;
    reg reset;

    // Instância do DUT (Device Under Test)
    RISCV_Simplified dut(
        .clk(clk),
        .reset(reset)
    );

    // Gerar o clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // período do clock de 10 unidades de tempo
    end

    // Geração do VCD
    initial begin
        $dumpfile("RISCV_Simplified_TB.vcd"); // Nome do arquivo VCD
        $dumpvars(0, RISCV_Simplified_TB);     // Variáveis a serem registradas
    end

    // Inicialização e teste
    initial begin
        // Condição inicial
        reset = 1;

        // Aguardar alguns ciclos e depois liberar reset
        #20;
        reset = 0;

        // Simulação rodará por 200 ciclos de tempo
        #200 $finish;
    end

endmodule
