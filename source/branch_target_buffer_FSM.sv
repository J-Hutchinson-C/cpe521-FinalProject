module branch_target_buffer_FSM (
    input btb_fsm_clk,
    input btb_fsm_branch_taken,
    //input [1:0] btb_fsm_current_prediction,
    output logic [1:0] btb_fsm_new_prediction
);

    //FSM states
    typedef enum {
        STATE_UNTAKEN_STRONG,
        STATE_UNTAKEN_WEAK,
        STATE_TAKEN_WEAK,
        STATE_TAKEN_STRONG
    } btb_fsm_state_t;

    btb_fsm_state_t BTB_FSM_NS, BTB_FSM_PS;

    //Output assignment

    always_ff @(posedge btb_fsm_clk) begin
        // Acts as the "initial value" since FSM is used for multiple branches
//        case (btb_fsm_current_prediction)
//            2'b00: BTB_FSM_PS <= STATE_UNTAKEN_STRONG;
//            2'b01: BTB_FSM_PS <= STATE_UNTAKEN_WEAK; 
//            2'b10: BTB_FSM_PS <= STATE_TAKEN_WEAK;
//            2'b11: BTB_FSM_PS <= STATE_TAKEN_STRONG; 
//        endcase
        BTB_FSM_PS <= BTB_FSM_NS;
    end
    
    always_comb begin
        case (BTB_FSM_PS)
            STATE_UNTAKEN_STRONG: begin
                // if untaken strong and branch taken then move to untaken weak
                if (btb_fsm_branch_taken) begin
                    btb_fsm_new_prediction = 2'b01;
                    BTB_FSM_NS = STATE_UNTAKEN_WEAK;
                end
                // else branch not taken then still untaken strong
                else begin
                    btb_fsm_new_prediction = 2'b00;
                    BTB_FSM_NS = STATE_UNTAKEN_STRONG;
                end
            end

            STATE_UNTAKEN_WEAK: begin
                // if untaken weak and branch taken then move to taken weak
                if (btb_fsm_branch_taken) begin
                    btb_fsm_new_prediction = 2'b11;
                    BTB_FSM_NS = STATE_TAKEN_STRONG;
                end
                // if untaken weak and branch not taken then move to untaken strong
                else begin
                    btb_fsm_new_prediction = 2'b00;
                    BTB_FSM_NS = STATE_UNTAKEN_STRONG;
                end
            end

            STATE_TAKEN_WEAK: begin
                // if taken weak and branch taken then move to taken strong
                if (btb_fsm_branch_taken) begin
                    btb_fsm_new_prediction = 2'b11;
                    BTB_FSM_NS = STATE_TAKEN_STRONG;
                end
                // if taken weak and branch not taken then move to untaken weak
                else begin
                    btb_fsm_new_prediction = 2'b00;
                    BTB_FSM_NS = STATE_UNTAKEN_STRONG;
                end
            end

            STATE_TAKEN_STRONG: begin
                // if taken strong and branch taken then stay in taken strong
                if (btb_fsm_branch_taken) begin
                    btb_fsm_new_prediction = 2'b11;
                    BTB_FSM_NS = STATE_TAKEN_STRONG;
                end
                // if taken strong and branch not taken then move to taken weak
                else begin
                    btb_fsm_new_prediction = 2'b10;
                    BTB_FSM_NS = STATE_TAKEN_WEAK;
                end
            end

        endcase
        
    
    end

endmodule