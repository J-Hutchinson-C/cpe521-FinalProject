module branch_target_buffer_FSM_tb();

    // Test inputs
    logic CLK = 1'b0;
    logic btb_fsm_branch_taken;
    logic [1:0] btb_fsm_current_prediction;
    //logic [1:0] btb_fsm_new_prediction;

    // Clock generation
    always #5 CLK = ~CLK;

    // Test Stimulus
    initial begin
        btb_fsm_current_prediction = 2'b11;
        #10 btb_fsm_branch_taken = 1'b0;
        btb_fsm_current_prediction = 2'b10;
        #10 btb_fsm_branch_taken = 1'b0;
        btb_fsm_current_prediction = 2'b00;
        #10 btb_fsm_branch_taken = 1'b1;
        btb_fsm_current_prediction = 2'b01;
        #10 btb_fsm_branch_taken = 1'b0;
        btb_fsm_current_prediction = 2'b00;
        #10 btb_fsm_branch_taken = 1'b1;
        btb_fsm_current_prediction = 2'b01;
        #10 btb_fsm_branch_taken = 1'b1;
        btb_fsm_current_prediction = 2'b11;
        #10 btb_fsm_branch_taken = 1'b0;
        
    end


    // Initialization of DUT
    branch_target_buffer_FSM BTB_FSM(.btb_fsm_clk(CLK), .btb_fsm_branch_taken(btb_fsm_branch_taken), .btb_fsm_current_prediction(btb_fsm_current_prediction), .btb_fsm_new_prediction(btb_fsm_current_prediction));

endmodule