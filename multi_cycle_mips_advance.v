
`timescale 1ns/100ps

   `define ADD  3'b000
   `define SUB  3'b001
   `define SLT  3'b010
   `define SLTU 3'b011
   `define AND  3'b100
   `define XOR  3'b101
   `define OR   3'b110
   `define NOR  3'b111

module multi_cycle_mips_advance(

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
   reg [31:0] A, B, PC, IR, MDR, MAR, hi, lo;

   // Data Path Control Lines, donot forget, regs are not always regs !!
   reg setMRE, clrMRE, setMWE, clrMWE, sethi, setlo;
   reg Awrt, Bwrt, RFwrt, PCwrt, IRwrt, MDRwrt, MARwrt, IorD, PCSrc;

   // Memory Ports Binding
   assign mem_addr = MAR;
   assign mem_read = MRE;
   assign mem_write = MWE;
   assign mem_write_data = B;

   // Mux & ALU Control Lines
   reg [2:0] aluOp;
   reg [1:0] aluSelB;
   reg SgnExt;
   reg aluSelA;
   reg mult;
   reg [2:0]MemtoReg;
   reg [1:0]RegDst;

   // Wiring
   wire aluZero;
   wire [31:0] aluResult, rfRD1, rfRD2;
   wire mult_out;
   wire [63:0] mult_result;

   // Clocked Registers
   always @( posedge clk ) begin
      if( reset )
         PC <= #0.1 32'h00000000;
         
      else if( PCwrt )
        if( PCSrc )
          PC <= #0.1 aluResult;
        else
          PC <= #0.1 {{PC[31:28]},{IR[25:0]},2'b00};


      if( Awrt ) A <= #0.1 rfRD1;
      if( Bwrt ) B <= #0.1 rfRD2;

      if( MARwrt ) MAR <= #0.1 IorD ? aluResult : PC;

      if( IRwrt ) IR <= #0.1 mem_read_data;
      if( MDRwrt ) MDR <= #0.1 mem_read_data;

      if( reset | clrMRE ) MRE <= #0.1 1'b0;
          else if( setMRE ) MRE <= #0.1 1'b1;

      if( reset | clrMWE ) MWE <= #0.1 1'b0;
          else if( setMWE ) MWE <= #0.1 1'b1;
            
      if( sethi )
        hi <= #0.1 mult_result[63:32];
      if( setlo )
        lo <= #0.1 mult_result[31:0]; 
        
   end

   // Register File
   reg_file rf(
      .clk( clk ),
      .write( RFwrt ),

      .RR1( IR[25:21] ),
      .RR2( IR[20:16] ),
      .RD1( rfRD1 ),
      .RD2( rfRD2 ),

      .WR( RegDst[1] ? 5'b11111: 
           RegDst[0] ? IR[15:11] : IR[20:16] ),
      .WD( MemtoReg[2] ? ( MemtoReg[0] ? PC : {IR[15:0],16'h0000}) :
           MemtoReg[1] ? ( MemtoReg[0] ? hi : lo ) :
           MemtoReg[0] ? MDR : aluResult )

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

   multiplier mp(
      .clk( clk ),  
      .start( mult ),
      .A( aluA ), 
      .B( aluB ), 
      .Product( mult_result ),
      .ready( mult_out )
  
   );


   // Controller Starts Here

   // Controller State Registers
   reg [5:0] state, nxt_state;

   // State Names & Numbers
   localparam
      RESET = 0, FETCH1 = 1, FETCH2 = 2, FETCH3 = 3, DECODE = 4,
      EX_ALU_R = 7, EX_ALU_I = 8,
      EX_LW_1 = 11, EX_LW_2 = 12, EX_LW_3 = 13, EX_LW_4 = 14, EX_LW_5 = 15,
      EX_SW_1 = 21, EX_SW_2 = 22, EX_SW_3 = 23,
      EX_BRA_1 = 25, EX_BRA_2 = 26,
      EX_J_1 = 27, 
      EX_JAL_1 = 28, 
      EX_JR_1 = 29, 
      EX_JALR_1 = 30,
      EX_JUMPS_2 = 31,
      EX_LUI_1 = 32,
      EX_MULTU_1 = 33, EX_MULTU_2 = 34,
      EX_MFHI_1 = 35,
      EX_MFLO_1 = 36;

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

      aluOp = 'bx; SgnExt = 'bx;
      aluSelA = 'bx; aluSelB = 'bx;
      MemtoReg = 'bx; RegDst = 'bx;

      PCwrt = 0;
      Awrt = 0; Bwrt = 0;
      RFwrt = 0; IRwrt = 0;
      MDRwrt = 0; MARwrt = 0;
      setMRE = 0; clrMRE = 0;
      setMWE = 0; clrMWE = 0;PCSrc = 1;mult = 0;

      case(state)

         RESET:
            PrepareFetch;

         FETCH1:
            nxt_state = FETCH2;

         FETCH2:
            nxt_state = FETCH3;

         FETCH3: begin
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
                     3'b001: 
                     
                        case( IR[2:0] )
                          3'b000:
                            nxt_state = EX_JR_1;
                          3'b001:
                            nxt_state = EX_JALR_1;
                        endcase

                     3'b010: 
                     
                        case( IR[2:0] )
                          3'b000:
                            nxt_state = EX_MFHI_1;
                          3'b010:
                            nxt_state = EX_MFLO_1;
                        endcase
                        
                     3'b011: 
                     
                        case( IR[2:0] )
                          3'b001:
                            nxt_state = EX_MULTU_1;
                        endcase
                        
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
               6'b001_110:             // xori
                  nxt_state = EX_ALU_I;

               6'b100_011:
                  nxt_state = EX_LW_1;

               6'b101_011:
                  nxt_state = EX_SW_1;

               6'b000_100,
               6'b000_101:
                  nxt_state = EX_BRA_1;
                  
               6'b000_010:
                   nxt_state = EX_J_1;
               6'b000_011: 
                   nxt_state = EX_JAL_1;
               6'b001_111: 
                   nxt_state = EX_LUI_1;   

               // rest of instructiones should be decoded here

            endcase
         end

         EX_ALU_R: begin
            aluSelA = 1;
            aluSelB = 2'b00;
            
            case (IR[5:0])
              6'b100_000 : aluOp = `ADD;
              6'b100_001 : aluOp = `ADD;
              6'b100_010 : aluOp = `SUB;
              6'b100_011 : aluOp = `SUB;
              6'b100_100 : aluOp = `AND;
              6'b100_101 : aluOp = `OR;
              6'b100_110 : aluOp = `XOR;
              6'b100_111 : aluOp = `NOR;
              6'b101_010 : aluOp = `SLT;
              6'b101_011 : aluOp = `SLTU;
            endcase
    
            MemtoReg = 3'b000;
            RegDst = 2'b01;
            RFwrt = 1;
            PrepareFetch;
            
         end

         EX_ALU_I: begin
            aluSelA = 1;
            aluSelB = 2'b10;
            
            case (IR[28:26])
              3'b000 : aluOp = `ADD;
              3'b001 : aluOp = `ADD;
              3'b010 : aluOp = `SLT;
              3'b011 : aluOp = `SLTU;
              3'b100 : aluOp = `AND;
              3'b101 : aluOp = `OR;
              3'b110 : aluOp = `XOR;
            endcase
            
        
            RFwrt = 1;
            
            case (IR[28:26])
              3'b001 : SgnExt = 0;
              3'b011 : SgnExt = 0;
              3'b100 : SgnExt = 0;
              3'b101 : SgnExt = 0;
              3'b110 : SgnExt = 0;
              3'b000 : SgnExt = 1;
              3'b010 : SgnExt = 1;
            endcase
            
            MemtoReg = 3'b000;
            RegDst = 2'b00;
            PrepareFetch;
         end

         EX_LW_1: begin
            aluSelA = 1;
            aluSelB = 2'b10;
            aluOp = `ADD;
            IorD = 1;
            setMRE = 1;
            clrMRE = 0;
            MARwrt = 1;
            SgnExt = 1;
            MemtoReg = 3'b001;
            RegDst = 2'b00;
            nxt_state = EX_LW_2;
         end
         
         EX_LW_2: begin
            nxt_state = EX_LW_3;
         end
         
         EX_LW_3: begin
            nxt_state = EX_LW_4;
         end

         EX_LW_4: begin
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 1;
            setMRE = 0;
            clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            MDRwrt = 1;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b01;
            nxt_state = EX_LW_5;
         end
         
         EX_LW_5: begin
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 0;
            //setMRE = 1;
            //clrMRE = 0;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 1;
            //Awrt = 0;
            //Bwrt = 0;
            RFwrt = 1;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 0;
            //SgnExt = 1;
            MemtoReg = 3'b001;
            RegDst = 2'b00;
            PrepareFetch;
            
         end
         
         
         EX_SW_1: begin
            aluSelA = 1;
            aluSelB = 2'b10;
            aluOp = `ADD;
            IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            setMWE = 1;
            clrMWE = 0;
            MARwrt = 1;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 0;
            SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b00;
            nxt_state = EX_SW_2;
         end

         EX_SW_2: begin
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 0;
            //setMRE = 1;
            //clrMRE = 0;
            setMWE = 0;
            clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 0;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b00;
            nxt_state = EX_SW_3;
         end
         
         EX_SW_3: begin
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 0;
            //setMRE = 1;
            //clrMRE = 0;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 1;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 0;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b00;
            PrepareFetch;
         end
         
         EX_BRA_1: begin
            aluSelA = 1;
            aluSelB = 2'b00;
            aluOp = `SUB;
            case( IR[26] )
              1'b1: begin
                if( aluZero != 1 )
                  nxt_state = EX_BRA_2;
                else
                  PrepareFetch;
              end
              1'b0: begin
                if( aluZero == 1 )
                  nxt_state = EX_BRA_2;
                else
                  PrepareFetch;
              end
            endcase
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 0;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b01;
         end

         EX_BRA_2: begin
            aluSelA = 0;
            aluSelB = 2'b11;
            aluOp = `ADD;
            IorD = 1;
            setMRE = 1;
            clrMRE = 0;
            //setMWE = 0;
            //clrMWE = 1;
            MARwrt = 1;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            PCwrt = 1 ; 
            //IRwrt = 0;
            //MDRwrt = 0;
            SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b01;
            nxt_state = FETCH1;
         end
         
         EX_J_1: begin
            PCSrc = 0;
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b01;
            PCwrt = 1;
        
            nxt_state = EX_JUMPS_2;
         end
         
         
         
         EX_JAL_1: begin
            PCSrc = 0;
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //IRwrt = 0;
            RFwrt = 1;
            //MDRwrt = 1;
            PCwrt = 1;
            //SgnExt = 1;
            MemtoReg = 3'b101;
            RegDst = 2'b10;

            nxt_state = EX_JUMPS_2;
            
         end
         
         EX_JR_1: begin
            aluSelA = 1;
            aluSelB = 2'b00;
            aluOp = `ADD;
            PCSrc = 1;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //IRwrt = 0;
            //RFwrt = 1'b1;
            //MDRwrt = 1;
            PCwrt = 1;
            //SgnExt = 1;
            //MemtoReg = 3'b010;
            //RegDst = 2'b10;
            //PCwrt = 1;
            //PCSrc = 1;
            
            
            nxt_state = EX_JUMPS_2;
                
         end
         
         EX_JALR_1: begin
            PCSrc = 1;
            aluSelA = 1;
            aluSelB = 2'b00;
            aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            PCwrt = 1;
            //SgnExt = 1;
            MemtoReg = 3'b101;
            RegDst = 2'b10;
            RFwrt = 1;
            
            nxt_state = EX_JUMPS_2;
            
         end
         
         EX_JUMPS_2: begin
            //PCSrc = 0;
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //RFwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //MemtoReg = 3'b001;
            //RegDst = 2'b01;
            //PCwrt = 1;
            PrepareFetch;
            
         end
                  
         EX_LUI_1: begin
            //PCSrc = 0;
            //aluSelA = 1;
            //aluSelB = 2'b10;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //PCwrt = 1;
            RegDst = 2'b00;
            MemtoReg = 3'b100;
            RFwrt = 1'b1;
            
            PrepareFetch;
            
         end

         EX_MULTU_1: begin
            aluSelA = 1;
            aluSelB = 2'b00;
            mult = 1;
            //PCSrc = 0;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //PCwrt = 1;
            //RegDst = 2'b00;
            //MemtoReg = 3'b011;
            //RFwrt = 1'b1;
            
            nxt_state = EX_MULTU_2;
            
         end

         EX_MULTU_2: begin
            //aluSelA = 1'b1;
            //aluSelB = 2'b00;
            //mult = 1;
            //PCSrc = 0;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //PCwrt = 1;
            //RegDst = 2'b00;
            //MemtoReg = 3'b011;
            //RFwrt = 1'b1;
            if( mult_out == 1'b1 ) begin
                sethi = 1;
                setlo = 1;
                
                PrepareFetch;   
            end
            else
                nxt_state = EX_MULTU_2;
     
         end
         
         EX_MFHI_1: begin
            //aluSelA = 1'b1;
            //aluSelB = 2'b00;
            //mult = 1;
            //PCSrc = 0;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //PCwrt = 1;
            MemtoReg = 3'b011;
            RegDst = 2'b01;
            RFwrt = 1;
            
            PrepareFetch;
         end
         
         EX_MFLO_1: begin
            //aluSelA = 1'b1;
            //aluSelB = 2'b00;
            //mult = 1;
            //PCSrc = 0;
            //aluOp = `ADD;
            //IorD = 1;
            //setMRE = 0;
            //clrMRE = 1;
            //setMWE = 0;
            //clrMWE = 1;
            //MARwrt = 0;
            //Awrt = 0;
            //Bwrt = 0;
            //PCwrt = 0;
            //IRwrt = 0;
            //MDRwrt = 1;
            //SgnExt = 1;
            //PCwrt = 1;
            MemtoReg = 3'b010;
            RegDst = 2'b01;
            RFwrt = 1;
            
            PrepareFetch;
         end


      endcase

   end

endmodule

//==============================================================================

module my_alu(
   input [2:0] Op,
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


module multiplier(
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
reg [5:0]  counter;
reg [31:0] multiplicand;
//-------------------------------------------------------

//------------------------------------- wire deceleration
wire product_write_enable;
wire [32:0] adder_output;
//---------------------------------------------------------

//-------------------------------------- combinational logic
assign adder_output = multiplicand + Product[63:32];
assign product_write_enable = Product[0];
assign ready = counter[5];
//---------------------------------------------------------

//--------------------------------------sequential Logic
always @ (posedge clk) begin

   if(start) begin
      counter <= 6'b000000 ;
      Product <= {32'h00000000,B};
      multiplicand<=A;
   end

   else if(! ready) begin
         counter <= counter + 1;
         Product <= Product >> 1;

      if(product_write_enable)
         Product[63:31] <= adder_output;
   end   
end
endmodule




