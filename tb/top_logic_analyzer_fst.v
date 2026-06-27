`timescale 1ns/1ps

module top_logic_analyzer_fst;
    initial begin
        $dumpfile("sim_build_top_logic_analyzer/top_logic_analyzer.fst");
        $dumpvars(0, top_logic_analyzer);
    end
endmodule
