#include <cstdint>
#include <iostream>
#include "processor.h"
#include "control.h"
using namespace std;

#ifdef ENABLE_DEBUG
#define DEBUG(x) x
#else
#define DEBUG(x) 
#endif

void Processor::initialize(int level) {
	processor_pc = 0;
	//Initialize control_t
	control = {.reg_dest = 0, 
				.jump = 0,
				.jump_reg = 0,
				.link = 0,
				.shift = 0,
				.branch = 0,
				.bne = 0,
				.mem_read = 0,
				.mem_to_reg = 0,
				.ALU_op = 0,
				.mem_write = 0,
				.halfword = 0,
				.byte = 0,
				.ALU_src = 0,
				.reg_write = 0,
				.zero_extend = 0};
	
	opt_level = level;

	state.fetchDecode = {
		.instruction = 0,
		.pc = 0
	};

	state.decExe = {
		.opcode = 0,
		.rs = 0,
		.rt = 0,
		.rd = 0,
		.shamt = 0,
		.funct = 0,
		.imm = 0,
		.addr = 0,
		.read_data_1 = 0,
		.read_data_2 = 0,
		.pc = 0
	};

	state.exeMem = {
		.imm = 0,
		.read_data_1 = 0,
		.read_data_2 = 0,
		.rd = 0,
		.rt = 0,
		.write_data = 0,
		.alu_zero = 0,
		.alu_result = 0,
		.pc = 0
	};

	state.memWrite = {
		.write_reg = 0,
		.write_data = 0,
		.imm = 0,
		//.addr = 0,
		.alu_zero = 0,
		//.read_data_1 = 0,
		//.read_data_2 = 0
		.pc = 0
	};

	//Initialize prevState to same values
	prevState = state;
	//Optimization level-specific initialization
}

void Processor::advance() {
	switch (opt_level) {
		case 0: single_cycle_processor_advance();
				break;
		case 1: pipelined_processor_advance();
				break;
		//other optimization levels go here
		default: break;
	}
}

void Processor::single_cycle_processor_advance() {
	//fetch
	uint32_t instruction;
	memory->access(regfile.pc, instruction, 0, 1, 0);
	DEBUG(cout << "\nPC: 0x" << std::hex << regfile.pc << std::dec << "\n");
	//increment pc
	regfile.pc += 4;
	
	//decode into contol signals
	control.decode(instruction);
	//extract rs, rt, rd, imm, funct 
	int opcode = (instruction >> 26) & 0x3f;
	int rs = (instruction >> 21) & 0x1f;
	int rt = (instruction >> 16) & 0x1f;
	int rd = (instruction >> 11) & 0x1f;
	int shamt = (instruction >> 6) & 0x1f;
	int funct = instruction & 0x3f;
	uint32_t imm = (instruction & 0xffff);
	int addr = instruction & 0x3ffffff;
	//Variables to read data into
	uint32_t read_data_1 = 0;
	uint32_t read_data_2 = 0;
	
	//Read from reg file
	regfile.access(rs, rt, read_data_1, read_data_2, 0, 0, 0);
	
	//Execution 
	alu.generate_control_inputs(control.ALU_op, funct, opcode);
	
	//Sign Extend Or Zero Extend the immediate
	//Using Arithmetic right shift in order to replicate 1 
	imm = control.zero_extend ? imm : (imm >> 15) ? 0xffff0000 | imm : imm;
	
	//Find operands for the ALU Execution
	//Operand 1 is always R[rs] -> read_data_1, except sll and srl
	//Operand 2 is immediate if ALU_src = 1, for I-type
	uint32_t operand_1 = control.shift ? shamt : read_data_1;
	uint32_t operand_2 = control.ALU_src ? imm : read_data_2;
	uint32_t alu_zero = 0;

	uint32_t alu_result = alu.execute(operand_1, operand_2, alu_zero);
	
	
	uint32_t read_data_mem = 0;
	uint32_t write_data_mem = 0;

	//Memory
	//First read no matter whether it is a load or a store
	memory->access(alu_result, read_data_mem, 0, control.mem_read | control.mem_write, 0);
	//Stores: sb or sh mask and preserve original leftmost bits
	write_data_mem = control.halfword ? (read_data_mem & 0xffff0000) | (read_data_2 & 0xffff) : 
					control.byte ? (read_data_mem & 0xffffff00) | (read_data_2 & 0xff): read_data_2;
	//Write to memory only if mem_write is 1, i.e store
	memory->access(alu_result, read_data_mem, write_data_mem, control.mem_read, control.mem_write);
	//Loads: lbu or lhu modify read data by masking
	read_data_mem &= control.halfword ? 0xffff : control.byte ? 0xff : 0xffffffff;

	int write_reg = control.link ? 31 : control.reg_dest ? rd : rt;

	uint32_t write_data = control.link ? regfile.pc+8 : control.mem_to_reg ? read_data_mem : alu_result;  

	//Write Back
	regfile.access(0, 0, read_data_2, read_data_2, write_reg, control.reg_write, write_data);
	
	//Update PC
	regfile.pc += (control.branch && !control.bne && alu_zero) || (control.bne && !alu_zero) ? imm << 2 : 0; 
	regfile.pc = control.jump_reg ? read_data_1 : control.jump ? (regfile.pc & 0xf0000000) & (addr << 2): regfile.pc;
}

void Processor::pipelined_fetch(){
	cout << "pc: " << processor_pc << "\n";

	if (stall){
		return;	
	}
	
	if (cache_penalty_mem > 0) {
		//state.fetchDecode.instruction = 0;  // NOP
		return;
	}

	bool fetch = memory->access(processor_pc, state.fetchDecode.instruction, 0, 1, 0);
	if (!fetch){
		clear_IF_ID();
		return;
	}

	DEBUG(cout << "\nPC: 0x" << std::hex << regfile.pc << std::dec << "\n");
	
	//increment pc
	state.fetchDecode.pc = processor_pc;
	processor_pc += 4; //standard increment
}

void Processor::pipelined_decode(){
	if (cache_penalty_mem){
		state.fetchDecode = prevState.fetchDecode;
		clear_ID_EX(); //flush state.decExe if hazard detected

		return;
	}
	
	//decode into contol signals (see below)
	uint32_t instruction = prevState.fetchDecode.instruction;
	//DEBUG(control.print());

	control_t new_control;
	new_control.decode(instruction);
	state.decExe.control = new_control; //push generated control signals to next pipeline reg

/*	*	*	*	*	*	*	*	*	*	*	*	*\
*	bool reg_dest;			//0 if rt, 1 if rd
*	bool jump;				//1 if jummp
*	bool jump_reg;			//1 if jr
*	bool link;				//1 if jal
*	bool shift;			  //1 if sll or srl
*	bool branch;			 //1 if branch
*	bool bne;				//1 if bne
*	bool mem_read;			//1 if memory needs to be read
*	bool mem_to_reg;		 //1 if memory needs to written to reg
*	unsigned ALU_op : 2;	 //10 for R-type, 00 for LW/SW, 01 for BEQ/BNE, 11 for others
*	bool mem_write;		  //1 if needs to be written to memory
*	bool halfword;			//1 if loading/storing halfword from memory
*	bool byte;				//1 if loading/storing a byte from memory
*	bool ALU_src;			//0 if second operand is from reg_file, 1 if imm
*	bool reg_write;		  //1 if need to write back to reg file
*	bool zero_extend;		//1 if immediate needs to be zero-extended
*
\*	*	*	*	*	*	*	*	*	*	*	*	*/

	//extract rs, rt, rd, imm, funct 
	int rs = state.decExe.rs = (instruction >> 21) & 0x1f; //store source (important for hazards)
	int rt = state.decExe.rt = (instruction >> 16) & 0x1f; //store target
	state.decExe.rd = (instruction >> 11) & 0x1f; //store destination

	
	if (stall){
		stall = false;
		//state.fetchDecode = prevState.fetchDecode;
		clear_ID_EX(); //flush state.decExe if hazard detected
		return;
	}

	detect_data_hazard(); //use the new rs, rt, rd vals to check for data hazard

	
	//do not push anything to next register unless data hazard is cleared

	state.decExe.opcode = (instruction >> 26) & 0x3f; //store opcode
	state.decExe.shamt = (instruction >> 6) & 0x1f; //?
	state.decExe.funct = instruction & 0x3f; //potential funct bits
	state.decExe.imm = (instruction & 0xffff); //potential immediate bits
	state.decExe.addr = instruction & 0x3ffffff; //address 
	
	//Variables to read data into
	uint32_t read_data_1 = 0;
	uint32_t read_data_2 = 0;
	
	//Read from reg file
	regfile.access(rs, rt, read_data_1, read_data_2, 0, 0, 0);
	// Forward data from writeback stage to decode stage when needed
	if(prevState.memWrite.control.reg_write && prevState.memWrite.write_reg != 0){
    		// Forward to rs if it matches the destination register
    		if (prevState.memWrite.write_reg == rs && rs != 0)
        		read_data_1 = prevState.memWrite.write_data;
    
    		// Forward to rt if it matches the destination register
    		// Only do this for R-type instructions or other instructions that need rt
    		if (prevState.memWrite.write_reg == rt && rt != 0 && !state.decExe.control.ALU_src)
        		read_data_2 = prevState.memWrite.write_data;
	}

/*
	if(prevState.memWrite.write_reg){
		if (prevState.memWrite.write_reg == rs)
			read_data_1 = prevState.memWrite.write_data;
		if (prevState.memWrite.write_reg == rt)
			read_data_2 = prevState.memWrite.write_data;
	}
*/

	state.decExe.read_data_1 = read_data_1;
	state.decExe.read_data_2 = read_data_2; //both of these should have been populated from the reg read
	state.decExe.pc = prevState.fetchDecode.pc;
}

void Processor::pipelined_execute(){
	if (cache_penalty_mem){
		state.decExe = prevState.decExe;
		state.exeMem.control.reset();
		return;
	}

	control_t &ctrl = prevState.decExe.control; //pull control signals from last reg
	state.exeMem.control = ctrl; //...and pass them along

	//Execution 
	alu.generate_control_inputs(ctrl.ALU_op, prevState.decExe.funct, prevState.decExe.opcode);
	
	//Sign Extend Or Zero Extend the immediate
	//Using Arithmetic right shift in order to replicate 1 

	//pull immediate from previous pipeline register, update, either use or send along
	uint32_t imm = prevState.decExe.imm;
	state.exeMem.imm = imm = ctrl.zero_extend ? imm : (imm >> 15) ? 0xffff0000 | imm : imm;
	
	//Find operands for the ALU Execution
	//Operand 1 is always R[rs] -> read_data_1, except sll and srl
	//Operand 2 is immediate if ALU_src = 1, for I-type
	uint32_t read_data_1 = prevState.decExe.read_data_1;
	uint32_t read_data_2 = prevState.decExe.read_data_2;

	//prevState.decExe.forward_a = prevState.decExe.forward_b = 0;
	//detect_data_hazard();

	uint32_t operand_1 = 0;
	uint32_t operand_2 = 0;

	/*
	*  0 = no forward
	*  1 = forward from mem
	*  2 = forward from wb
	*/
	//forwarding logig muxes, recieve input from forwarding unit
	switch(get_forwarding_a()){
		case(0): //raw read data_1, no forwarding
			operand_1 = ctrl.shift ? 
				prevState.decExe.shamt : read_data_1;
			break;
		case(1): //read previous mem data
			//operand_1 = prevState.exeMem.alu_result; 
			operand_1 = prevState.memWrite.write_data; 
			break;
		case(2): //read previous alu_result
			operand_1 = prevState.exeMem.alu_result; 
			//operand_1 = prevState.memWrite.write_data;
			break;
	}
	
	switch(get_forwarding_b()){
		case(0):
			operand_2 = read_data_2;
			//operand_2 = ctrl.ALU_src ? imm : read_data_2;
			break;	
		case(1):
			operand_2 = prevState.memWrite.write_data;	
			break;
		case(2):
			operand_2 = prevState.exeMem.alu_result; 
			break;
	}

	state.exeMem.write_data = operand_2;
	//operand_2 needs to become write_data unconditionally

	if (ctrl.ALU_src) //mux for I type
		operand_2 = imm;
	
	uint32_t alu_zero = 0;

	state.exeMem.alu_result = alu.execute(operand_1, operand_2, alu_zero);


	//logic to take care of updating values read from register in case of forwarding
	//don't remember why this works, but its necessary


	//send updated values down the pipeline
	state.exeMem.rd = prevState.decExe.rd;
	state.exeMem.rt = prevState.decExe.rt;
	state.exeMem.alu_zero = alu_zero;
	state.exeMem.pc = prevState.decExe.pc;
	
	detect_control_hazard(ctrl);
}


void Processor::pipelined_mem(){
	control_t &ctrl = prevState.exeMem.control;
	state.memWrite.control = ctrl; //same as last time

	uint32_t read_data_mem = 0;
	uint32_t write_data_mem = 0;

	if (ctrl.mem_read){
		bool read = memory->access(prevState.exeMem.alu_result, read_data_mem, write_data_mem, ctrl.mem_read, ctrl.mem_write);
		if (stall > 1){
			stall--;
			state = prevState;
			return;
		}
		if (!read){
			stall = 60;
			state = prevState;
			return;
		}
	}
	
	//while(!memory->access(prevState.exeMem.alu_result, read_data_mem, 0, ctrl.mem_read | ctrl.mem_write, 0)){}
	
	//Stores: sb or sh mask and preserve original leftmost bits


	write_data_mem = ctrl.halfword ? (read_data_mem & 0xffff0000) | (prevState.exeMem.write_data & 0xffff) : 
					ctrl.byte ? (read_data_mem & 0xffffff00) | (prevState.exeMem.write_data & 0xff): 
					prevState.exeMem.write_data;  //not populating correctly
	
	//Write to memory only if mem_write is 1, i.e store
	if (ctrl.mem_write){
		bool write = memory->access(prevState.exeMem.alu_result, read_data_mem, write_data_mem, ctrl.mem_read, ctrl.mem_write);
		if (stall > 1){
			stall--;
			state = prevState;
			return;
		}
		if (!write){
			stall = 60;
			state = prevState;
			return;
		}
	}
	//Loads: lbu or lhu modify read data by masking
	read_data_mem &= ctrl.halfword ? 0xffff : ctrl.byte ? 0xff : 0xffffffff;

	state.memWrite.write_reg = ctrl.link ? 31 : ctrl.reg_dest ? prevState.exeMem.rd : prevState.exeMem.rt;

	//state.memWrite.write_data = control.link ? regfile.pc+8 : ctrl.mem_to_reg ? read_data_mem : prevState.exeMem.alu_result;  

	switch(ctrl.mem_read){
		case(false): //standard instruction, no mem access
			state.memWrite.write_data = prevState.exeMem.alu_result; //raw alu result, just normal instruction
			break;
		case(true):
			state.memWrite.write_data = read_data_mem; //memory was accessed
			break;
	}
	
	state.memWrite.pc = prevState.exeMem.pc;
}

void Processor::pipelined_wb(){
	control_t &ctrl = prevState.memWrite.control; //nothing to pass forward this time

	//Write Back
	//imm doesnt do anything, could probably be 0
	regfile.access(0, 0, prevState.memWrite.imm, prevState.memWrite.imm, prevState.memWrite.write_reg, 
			ctrl.reg_write, prevState.memWrite.write_data);

	regfile.pc = prevState.memWrite.pc;
}

void Processor::pipelined_processor_advance(){
	prevState = state;

	pipelined_fetch();
	pipelined_decode();
	pipelined_execute();
	pipelined_mem();	
	pipelined_wb();
}
