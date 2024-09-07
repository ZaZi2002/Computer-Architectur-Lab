
`timescale 1ns/100ps

    // Alu
   `define ADD  0
   `define SUB  1
   `define SLT  2
   `define SLTU 3
   `define AND  4
   `define XOR  5
   `define OR   6
   `define NOR  7
   `define LUI  8 // :)

   // funct
   `define add_funct 32 // :)
   `define sub_funct 34 // :)
   `define addu_funct 33 // :)
   `define subu_funct 35 // :)
   `define and_funct 36 // :)
   `define or_funct 37 // :)
   `define xor_funct 38 //:)
   `define nor_funct 39 // :)
   `define slt_funct 42 //:)
   `define sltu_funct 43 // :)
   `define jr_funct 8 // :)
   `define jalr_funct 9 // :)
   `define multu_funct 25 // :)
   `define mfhi_funct 16 // :)
   `define mflo_funct 18 // :)

   // opcode
   `define beq_opcode 4 // :)
   `define bne_opcode 5 // :)
   `define addi_opcode 8 // :)
   `define addiu_opcode 9 // :)
   `define slti_opcode 10 // :)
   `define sltiu_opcode 11 // :)
   `define andi_opcode 12 // :)
   `define ori_opcode 13 // :)
   `define xori_opcode 14 // :)
   `define lui_opcode 15 // :)
   `define j_opcode 2 // :)
   `define jal_opcode 3 // :)
   `define lw_opcode 35 // :)
   `define sw_opcode 43 // :)
   `define lui_opcode 15 // :)




module multi_cycle_mips(

   input clk,
   input reset,

   // Memory Ports
   output  [31:0] mem_addr,
   input   [31:0] mem_read_data,
   output  [31:0] mem_write_data,
   output         mem_read,
   output         mem_write
);

   // Data Path Registers
   reg MRE, MWE;
   reg [31:0] A, B, PC, IR, MDR, MAR;
   reg [31:0] Hi, Lo; //////////////////////////////////// :)

   // Data Path Control Lines, donot forget, regs are not always regs !!
   reg setMRE, clrMRE, setMWE, clrMWE;
   reg start; /////////////////////////////////// :)
   reg Awrt, Bwrt, RFwrt, PCwrt, IRwrt, MDRwrt, MARwrt;
   reg LOwrt, HIwrt; ///////////////////////////////////// :)

   // Memory Ports Binding
   assign mem_addr = MAR;
   assign mem_read = MRE;
   assign mem_write = MWE;
   assign mem_write_data = B;

   // Mux & ALU Control Lines
   reg [3:0] aluOp; ///////////////////////////////////// :)
   reg [1:0] aluSelB;
   reg [1:0] PCsrc; ///////////////////////////////////// :) 
   reg SgnExt, aluSelA;
   reg [1:0] IorD; ///////////////////////////////////// :)
   reg [1:0] RegDst; ///////////////////////////////////// :)
   reg [2:0] MemtoReg; ////////////////////////////////////// :)

   // Wiring
   wire aluZero;
   wire ready; ////////////////////////////////////////// :)
   wire [31:0] aluResult, rfRD1, rfRD2;
   wire [63:0] multu_product; //////////////////////////////////////// :)

   // Clocked Registers
   always @( posedge clk ) begin
      if( reset )
         PC <= #0.1 32'h00000000;
      else if( PCwrt )
         case (PCsrc) /////////////////////////////////// :)
            0 : PC <= #0.1 aluResult; /////////////////////////////////// :)
            1 : PC <= #0.1 {PC[31:28], IR[25:0], 2'b00}; /////////////////////////////////// :)
            2 : PC <= #0.1 A; /////////////////////////////////// :)
            3 : PC <= #0.1 32'h00000000; /////////////////////////////////// :)
         endcase /////////////////////////////////// :)

      if( Awrt ) A <= #0.1 rfRD1;
      if( Bwrt ) B <= #0.1 rfRD2;

      if( MARwrt ) MAR <= #0.1 IorD == 0 ? PC : 
                                 IorD == 1 ? aluResult :
                                 IorD == 2 ? {PC[31:28], IR[25:0], 2'b00} :
                                 IorD == 3 ? A : 32'h00000000;

      if( IRwrt ) IR <= #0.1 mem_read_data;
      if( MDRwrt ) MDR <= #0.1 mem_read_data;

      if( reset | clrMRE ) MRE <= #0.1 1'b0;
          else if( setMRE) MRE <= #0.1 1'b1;

      if( reset | clrMWE ) MWE <= #0.1 1'b0;
          else if( setMWE) MWE <= #0.1 1'b1;

      if(LOwrt) Lo <= #0.1 multu_product[31:0]; ////////////////////////////////// :)
      if(HIwrt) Hi <= #0.1 multu_product[63:32]; ////////////////////////////////// :)
   end

   // Register File
   reg_file rf(
      .clk( clk ),
      .write( RFwrt ),

      .RR1( IR[25:21] ),
      .RR2( IR[20:16] ),
      .RD1( rfRD1 ),
      .RD2( rfRD2 ),

      .WR( RegDst == 0 ?  IR[20:16] : //////////////////////////////////////// :)
            RegDst == 1 ? IR[15:11] : //////////////////////////////////////// :)
            RegDst == 2 ? 5'b11111 : 5'b00000 //////////////////////////////////////// :)
            ),
      .WD(MemtoReg == 0 ? aluResult : //////////////////////////////////////// :)
            MemtoReg == 1 ? MDR : //////////////////////////////////////// :)
            MemtoReg == 2 ? Lo : //////////////////////////////////////// :)
            MemtoReg == 3 ? Hi :
            MemtoReg == 4 ? PC :  5'b00000 //////////////////////////////////////// :)
          )
   );

   // Sign/Zero Extension
   wire [31:0] SZout = SgnExt ? {{16{IR[15]}}, IR[15:0]} : {16'h0000, IR[15:0]};

   // ALU-A Mux
   wire [31:0] aluA = aluSelA ? A : PC;

   // ALU-B Mux
   reg [31:0] aluB;
   always @(*)
   case (aluSelB)
      2'b00: aluB = B;
      2'b01: aluB = 32'h4;
      2'b10: aluB = SZout;
      2'b11: aluB = SZout << 2;
   endcase


   my_alu alu(
      .A( aluA ),
      .B( aluB ),
      .Op( aluOp ),

      .X( aluResult ),
      .Z( aluZero )
   );

   multiplier1 multiplier(
      .clk (clk),
      .start (start),
      .A (A),
      .B (B),
      .Product (multu_product),
      .ready (ready)
   );


   // Controller Starts Here

   // Controller State Registers
   reg [4:0] state, nxt_state;

   // State Names & Numbers
   localparam
      RESET = 0, FETCH1 = 1, FETCH2 = 2, FETCH3 = 3, DECODE = 4,
      EX_ALU_R = 7, EX_ALU_I = 8,
      EX_LW_1 = 11, EX_LW_2 = 12, EX_LW_3 = 13, EX_LW_4 = 14, EX_LW_5 = 15,
      EX_SW_1 = 21, EX_SW_2 = 22, EX_SW_3 = 23,
      EX_BRA_1 = 25, EX_BRA_2 = 26, 
      EX_MULTU_1 = 27, EX_MULTU_2 = 28, EX_MF_1 = 29, EX_jrORjalr_R = 30; ////////////////////////////////////////// :)

   // State Clocked Register 
   always @(posedge clk)
      if(reset)
         state <= #0.1 RESET;
      else
         state <= #0.1 nxt_state;

   task PrepareFetch;
      begin
         IorD = 0;
         setMRE = 1;
         MARwrt = 1;
         nxt_state = FETCH1;
      end
   endtask

   // State Machine Body Starts Here
   always @( * ) begin

      nxt_state = 'bx;

      PCsrc = 'bx; ////////////////////////////////////////////// :) 
      SgnExt = 'bx; IorD = 'bx;
      MemtoReg = 'bx; RegDst = 'bx;
      aluSelA = 'bx; aluSelB = 'bx; aluOp = 'bx;

      PCwrt = 0;
      Awrt = 0; Bwrt = 0;
      RFwrt = 0; IRwrt = 0;
      MDRwrt = 0; MARwrt = 0;
      setMRE = 0; clrMRE = 0;
      setMWE = 0; clrMWE = 0;
      HIwrt = 0; LOwrt = 0; start = 0; ////////////////////////////////////// :)
      case(state)

         RESET:
            PrepareFetch;

         FETCH1:
            nxt_state = FETCH2;

         FETCH2:
            nxt_state = FETCH3;

         FETCH3: begin
            PCsrc = 0; //////////////////////////////////// :)
            IRwrt = 1;
            PCwrt = 1;
            clrMRE = 1;
            aluSelA = 0;
            aluSelB = 2'b01;
            aluOp = `ADD;
            nxt_state = DECODE;
         end

         DECODE: begin
            Awrt = 1;
            Bwrt = 1;
            case( IR[31:26] )
               6'b000_000:             // R-format
                  case( IR[5:3] )
                     3'b000: ;
                     3'b001: nxt_state = EX_jrORjalr_R; /////////////////////////// :)
                     3'b010: nxt_state = EX_MF_1; /////////////////////////// :)
                     3'b011: nxt_state = EX_MULTU_1; ///////////////////////////// :)
                     3'b100: nxt_state = EX_ALU_R;
                     3'b101: nxt_state = EX_ALU_R;
                     3'b110: ;
                     3'b111: ;
                  endcase

               6'b001_000,             // addi
               6'b001_001,             // addiu
               6'b001_010,             // slti
               6'b001_011,             // sltiu
               6'b001_100,             // andi
               6'b001_101,             // ori
               6'b001_110,             // xori
               6'b001_111:             // lui  ////////////////////////// :)
                  nxt_state = EX_ALU_I;

               6'b100_011: //lw
                  nxt_state = EX_LW_1;

               6'b101_011: //sw
                  nxt_state = EX_SW_1;

               6'b000_100, //beq
               6'b000_101: //bne
                  nxt_state = EX_BRA_1;
                  
                  

               // rest of instructiones should be decoded here
               `j_opcode: 
                  begin
                     PCsrc = 1;
                     PCwrt = 1;
                     IorD = 2;
                     setMRE = 1;
                     MARwrt = 1;
                     nxt_state = FETCH1;
                  end

               `jal_opcode:
                  begin
                        RegDst = 2;
                        MemtoReg = 4;
                        RFwrt = 1;

                        PCsrc = 1;
                        PCwrt = 1;
                        IorD = 2;
                        setMRE = 1;
                        MARwrt = 1;
                        nxt_state = FETCH1;
                  end
   
            endcase
         end

         EX_ALU_R: begin
            aluSelA = 1; 
            aluSelB = 2'b00; 
            case (IR[5:0]) 
                `add_funct: aluOp = `ADD; 
                `sub_funct: aluOp = `SUB; 
                `addu_funct: aluOp = `ADD; 
                `subu_funct: aluOp = `SUB; 
                `and_funct: aluOp = `AND; 
                `or_funct: aluOp = `OR; 
                `xor_funct: aluOp = `XOR; 
                `nor_funct: aluOp = `NOR; 
                `slt_funct: aluOp = `SLT; 
                `sltu_funct: aluOp = `SLTU;
            endcase 
            MemtoReg = 0; 
            RegDst = 1; 
            RFwrt = 1; 
            PrepareFetch; 
         end

         EX_jrORjalr_R: begin
            PCsrc = 2;
            PCwrt = 1;
            IorD = 3;
            setMRE = 1;
            MARwrt = 1;
            if (IR[5:0] == `jalr_funct)
            begin
               RegDst = 2;
               MemtoReg = 4;
               RFwrt = 1;
            end
            nxt_state = FETCH1;
         end

         EX_ALU_I: begin
            aluSelA = 1; 
            aluSelB = 2'b10;
            case (IR[31:26])
                `addi_opcode: 
                begin 
                    aluOp = `ADD; 
                    SgnExt = 1;
                end
                `addiu_opcode:
                begin 
                    aluOp = `ADD;
                    SgnExt = 1;
                end
                `slti_opcode:
                begin 
                    aluOp = `SLT;
                    SgnExt = 1;
                end
                `sltiu_opcode:
                begin 
                    aluOp = `SLTU;
                    SgnExt = 1;
                end
                `andi_opcode:
                begin 
                    aluOp = `AND;
                    SgnExt = 0;
                end
                `ori_opcode:
                begin 
                    aluOp = `OR;
                    SgnExt = 0;
                end 
                `xori_opcode:
                begin 
                    aluOp = `XOR;
                    SgnExt = 0; 
                end
                `lui_opcode:
                begin 
                    aluOp = `LUI;
                    SgnExt = 0; 
                end
            endcase
            MemtoReg = 0;
            RegDst = 0;
            RFwrt = 1;
            PrepareFetch;
         end

         EX_LW_1: begin
            aluSelA = 1; 
            aluSelB = 2'b10;
            SgnExt = 1;
            aluOp = `ADD;
            IorD = 1;
            setMRE = 1;
            MARwrt = 1;
            nxt_state = EX_LW_2;
         end

         EX_LW_2: begin

            nxt_state = EX_LW_3;
         end

         EX_LW_3: begin

            nxt_state = EX_LW_4;
         end

         EX_LW_4: begin
            clrMRE = 1;
            MDRwrt = 1;
            nxt_state = EX_LW_5;
         end

         EX_LW_5: begin
            MemtoReg = 1;
            RegDst = 0;
            RFwrt = 1;
            PrepareFetch;
         end

         EX_SW_1: begin
            aluSelA = 1; 
            aluSelB = 2'b10;
            SgnExt = 1;
            aluOp = `ADD;
            IorD = 1;
            setMWE = 1;
            MARwrt = 1;
            nxt_state = EX_SW_2;
         end

         EX_SW_2: begin
            clrMWE = 1;
            nxt_state = EX_SW_3;
         end

         EX_SW_3: begin
            PrepareFetch;
         end

         EX_BRA_1: begin
            aluSelA = 1;
            aluSelB = 2'b00;
            aluOp = `SUB;
            if (IR[31:26] == `beq_opcode)
            begin
                if (aluZero == 1)
                    nxt_state = EX_BRA_2;
                else
                    PrepareFetch;
            end
            if (IR[31:26] == `bne_opcode)
            begin
                if (aluZero != 1)
                    nxt_state = EX_BRA_2;
                else
                    PrepareFetch;
            end
         end

         EX_BRA_2: begin
            aluSelA = 0;
            aluSelB = 2'b11;
            aluOp = `ADD;
            SgnExt = 1;
            PCsrc = 0;
            PCwrt = 1;
            
            IorD = 1;   
            MARwrt = 1;
            setMRE = 1;
            nxt_state = FETCH1;
         end

         EX_MULTU_1: begin
            start = 1;
            nxt_state = EX_MULTU_2;
         end

         EX_MULTU_2: begin
            start = 0;
            if (ready == 1)
            begin
               HIwrt = 1;
               LOwrt = 1;
               PrepareFetch;
            end
            else
               nxt_state = EX_MULTU_2;
         end

         EX_MF_1: begin
            RegDst = 1;
            RFwrt = 1;
            if (IR[5:0] == `mfhi_funct)
               MemtoReg = 3;
            else if (IR[5:0] == `mflo_funct)
               MemtoReg = 2; 

            PrepareFetch;
         end

      endcase

   end

endmodule

//==============================================================================

module my_alu(
   input [3:0] Op, ///////////////////////////// :)
   input [31:0] A,
   input [31:0] B,

   output [31:0] X,
   output        Z
);

   wire sub = Op != `ADD;

   wire [31:0] bb = sub ? ~B : B;

   wire [32:0] sum = A + bb + sub;

   wire sltu = ! sum[32];

   wire v = sub ? 
        ( A[31] != B[31] && A[31] != sum[31] )
      : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x =   A & B;
         `OR  : x =   A | B;
         `NOR : x = ~(A | B);
         `XOR : x =   A ^ B;
         `LUI : x = B << 16; ////////////////////////////////////// :)
         default : x = 32'hxxxxxxxx;
      endcase

   assign #2 X = x;
   assign #2 Z = x == 32'h00000000;

endmodule

//==============================================================================

module reg_file(
   input clk,
   input write,
   input [4:0] WR,
   input [31:0] WD,
   input [4:0] RR1,
   input [4:0] RR2,
   output [31:0] RD1,
   output [31:0] RD2
);

   reg [31:0] rf_data [0:31];

   assign #2 RD1 = rf_data[ RR1 ];
   assign #2 RD2 = rf_data[ RR2 ];   

   always @( posedge clk ) begin
      if ( write )
         rf_data[ WR ] <= WD;

      rf_data[0] <= 32'h00000000;
   end

endmodule

//==============================================================================

module multiplier1(
//-----------------------Port directions and deceleration
   input clk,  
   input start,
   input [31:0] A, 
   input [31:0] B, 
   output reg [63:0] Product,
   output ready
    );



//------------------------------------------------------

//----------------------------------- register deceleration
reg [63:0] Multiplicand ;
reg [31:0]  Multiplier;
reg [5:0]  counter;
//-------------------------------------------------------

//------------------------------------- wire deceleration
wire product_write_enable;
wire [63:0] adder_output;
//---------------------------------------------------------

//-------------------------------------- combinational logic
assign adder_output = Multiplicand + Product;
assign product_write_enable = Multiplier[0];
assign ready = counter[5];
//---------------------------------------------------------

//--------------------------------------- sequential Logic
always @ (posedge clk)

   if(start) begin
      counter <= 6'b000000 ;
      Multiplier <= B;
      Product <= 64'h0000000000000000;
      Multiplicand <= {32'h00000000, A} ;
   end

   else if(! ready) begin
         counter <= counter + 1;
         Multiplier <= Multiplier >> 1;
         Multiplicand <= Multiplicand << 1;

      if(product_write_enable)
         Product <= adder_output;
   end   

endmodule