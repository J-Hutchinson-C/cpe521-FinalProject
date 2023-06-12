module branch_target_buffer_FSM_tb();

    // Test inputs
    logic btb_fsm_clk = 1'b0;
    logic btb_fsm_branch_taken;
    logic [1:0] btb_fsm_prediction;

    // Clock generation
    always #5 btb_fsm_clk = ~btb_fsm_clk;

    // Test Stimulus
    initial begin
        #10 btb_fsm_branch_taken = 1'b0;
        #10 btb_fsm_branch_taken = 1'b1;
        #10 btb_fsm_branch_taken = 1'b1;
        #10 btb_fsm_branch_taken = 1'b0;
        #10 btb_fsm_branch_taken = 1'b0;
        #10 btb_fsm_branch_taken = 1'b1;
        #5 btb_fsm_branch_taken = 1'b0;
        #5 #10 btb_fsm_branch_taken = 1'b1;
        
    end


    // Initialization of DUT
    branch_target_buffer_FSM BTB_FSM(.btb_fsm_clk(btb_fsm_clk), .btb_fsm_branch_taken(btb_fsm_branch_taken), .btb_fsm_prediction(btb_fsm_prediction));

endmodule