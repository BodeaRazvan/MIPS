library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity test_env is
    Port ( clk:  in   STD_LOGIC;
           btn:  in   STD_LOGIC_VECTOR (4 downto 0);
           sw:   in   STD_LOGIC_VECTOR (15 downto 0);
           led:  out  STD_LOGIC_VECTOR (15 downto 0);
           an:   out  STD_LOGIC_VECTOR (3 downto 0);
           cat:  out  STD_LOGIC_VECTOR (6 downto 0));
end test_env;

architecture Behavioral of test_env is
--signal declaration
signal cnt:           STD_LOGIC_VECTOR (3 downto 0) := "0000";
signal cnt_rom:       std_logic_vector(7 downto 0)  := x"00";
signal enable1:       STD_LOgic :='0';
signal enable2:       STD_LOgic :='0';
signal nr1:           std_logic_vector(15 downto 0):=x"0000";
signal nr2:           std_logic_vector(15 downto 0):=x"0000";
signal nr3:           std_logic_vector(15 downto 0):=x"0000";
signal result:        std_logic_vector(15 downto 0):=x"0000";
signal ALUresult:     std_logic_vector(15 downto 0):=x"0000";
signal reset:         std_logic :='0';
signal RD1:           std_logic_vector(15 downto 0):=x"0000";
signal RD2:           std_logic_vector(15 downto 0):=x"0000";
--Instruction fetch data-path
signal PC:            std_logic_vector(15 downto 0):=x"0000";
signal mux1:          std_logic_vector(15 downto 0):=x"0000";
signal mux2:          std_logic_vector(15 downto 0):=x"0000";
signal resetPC:       std_logic :='0';
signal enablePC:      std_logic :='0';
signal PCplusone:     std_logic_vector(15 downto 0):=x"0000";
signal instruction:   std_logic_vector(15 downto 0):=x"0000";
signal pcSrc:         std_logic :='0';
signal JumpAddress:   std_logic_vector(15 downto 0):=x"0000";
signal BranchAddress: std_logic_vector(15 downto 0):=x"0000";

signal opcode:        std_logic_vector(2 downto 0):=instruction(15 downto 13);
signal funct:         std_logic_vector(2 downto 0):=instruction(2 downto 0);
signal sa:            std_logic:=instruction(3);
signal imm:           std_logic_vector(6 downto 0):=instruction(6 downto 0);
signal tAddr:         std_logic_vector(12 downto 0):=instruction(12 downto 0);
signal EXT_Imm:       std_logic_vector(15 downto 0):=x"0000";
signal rs:            std_logic_vector(3 downto 0):=x"0";
signal rt:            std_logic_vector(3 downto 0):=x"0";
signal rd:            std_logic_vector(3 downto 0):=x"0";
signal WD:            std_logic_vector(15 downto 0):=x"0000";
signal WA:            std_logic_vector(3 downto 0):=x"0";
signal MemData:       std_logic_vector(15 downto 0):=x"0000";
signal DisplayResult: std_logic_vector(15 downto 0):=x"0000";
--control signals
signal Zero:     std_logic :='0';
signal RegDst:   std_logic :='0';
signal ExtOp:    std_logic :='0';
signal ALUSrc:   std_logic :='0';
signal Branch:   std_logic :='0';
signal Jump :    std_logic :='0';
signal ALUop:    std_logic_vector(3 downto 0):="0000"; 
signal MemWrite: std_logic:='0';
signal MemtoReg: std_logic:='0';
signal RegWrite: std_logic:='0';
signal Beq:      std_logic:='0';
signal Bneq:     std_logic:='0';

--PIPELINE SIGNALS   (I chose to declare signals for each stage for a clearer understanding and reading)
signal PCplusone_ID:     std_logic_vector(15 downto 0):=x"0000";
signal instruction_ID:   std_logic_vector(15 downto 0):=x"0000";

signal ALUop_EX:    std_logic_vector(3 downto 0):="0000"; 
signal MemWrite_EX: std_logic:='0';
signal MemtoReg_EX: std_logic:='0';
signal RegWrite_EX: std_logic:='0';
signal Beq_EX:      std_logic:='0';
signal Bneq_EX:     std_logic:='0';
signal RegDst_EX:   std_logic :='0';
signal ALUSrc_EX:   std_logic :='0';
signal Branch_EX:   std_logic :='0';
signal PCplusone_EX:     std_logic_vector(15 downto 0):=x"0000";
signal EXT_Imm_EX:       std_logic_vector(15 downto 0):=x"0000";
signal RD1_EX:      std_logic_vector(15 downto 0):=x"0000";  
signal RD2_EX:      std_logic_vector(15 downto 0):=x"0000";
signal rt_ex:            std_logic_vector(3 downto 0):=x"0";
signal rd_ex:            std_logic_vector(3 downto 0):=x"0";
signal sa_ex:            std_logic:='0';

signal Branch_MEM:   std_logic :='0';
signal Beq_MEM:      std_logic:='0';
signal Bneq_MEM:     std_logic:='0';
signal MemWrite_MEM: std_logic:='0';
signal MemtoReg_MEM: std_logic:='0';
signal RegWrite_MEM: std_logic:='0';
signal BranchAddress_MEM: std_logic_vector(15 downto 0):=x"0000";
signal Zero_MEM:     std_logic :='0';
signal ALUresult_MEM:     std_logic_vector(15 downto 0):=x"0000";
signal WA_MEM:            std_logic_vector(3 downto 0):=x"0";
signal RD2_MEM:      std_logic_vector(15 downto 0):=x"0000";

signal ALUresult_WB:     std_logic_vector(15 downto 0):=x"0000";
signal WA_WB:            std_logic_vector(3 downto 0):=x"0";
signal MemtoReg_WB: std_logic:='0';
signal RegWrite_WB: std_logic:='0';
signal MemData_WB:       std_logic_vector(15 downto 0):=x"0000";

--FSM
signal TX_Data: std_logic_vector(7 downto 0):=x"00";
signal TX_EN: std_logic :='0';
signal RST: std_logic :='0';
signal BAUD_EN: std_logic :='0';
signal TX_RDY: std_logic:='0';
signal TX: std_logic:='0';
signal BAUD_COUNT: std_logic_vector(13 downto 0):="00000000000000";
signal TX_STATE: std_logic_vector(1 downto 0):="00";
signal bit_cnt: std_logic_vector(2 downto 0):="000";

signal rx_rdy : std_logic :='0';
signal rx_data : std_logic_vector(7 downto 0) := "00000000";
signal rx : std_logic:='0';
signal rx_state : std_logic_vector(2 downto 0):="000";
signal bit_cnt_rx : std_logic_vector(2 downto 0):="000";
signal baud_cnt_rx: std_logic_vector(3 downto 0):="0000";
signal output_rx: std_logic_vector(7 downto 0):=x"00";

--ROM
type memory is array (0 to 255) of std_logic_vector(15 downto 0);
signal romMem: memory :=(
B"000_001_010_011_0_000", --add
B"000_001_010_011_0_001", --sub
B"000_001_010_011_1_010", --sll
B"000_001_010_011_1_011", --srl
B"000_001_010_011_0_100", --and
B"000_001_010_011_0_101", --or
B"000_001_010_011_0_110", --xor
B"000_001_010_011_0_111", --nor
B"001_001_010_0000000"  , --addi
B"010_001_010_0000000"  , --lw
B"011_001_010_0000000"  , --sw
B"100_001_010_0000000"  , --beq
B"101_001_010_0000000"  , --bne
B"110_001_010_0000000"  , --ori
B"111_0000000000000"    , --jump
others =>x"0000"
);

--component declaration
component MPG is
    Port ( btn : in STD_LOGIC;
           clk : in STD_LOGIC;
           en :  out STD_LOGIC);
end component;
component SSD is
    Port(  digit: in std_logic_vector(15 downto 0);
           clk :  in STD_LOGIC;
           cat :  out STD_LOGIC_VECTOR (6 downto 0);
           an :   out STD_LOGIC_VECTOR (3 downto 0)
    );
end component;
component RF is
     Port (clk:      in std_logic;
           RegWrite: in std_logic;
           RA1:      in std_logic_vector(3 downto 0);
           RA2:      in std_logic_vector(3 downto 0);
           WA:       in std_logic_vector(3 downto 0);
           WD:       in std_logic_vector(15 downto 0);
           RD1:      out std_logic_vector(15 downto 0);
           RD2:      out std_logic_vector(15 downto 0)
         );
end component;
component RAM is
    Port ( clk:   in std_logic;
           RegWr: in std_logic;
           RA:    in std_logic_vector(3 downto 0);
           WA:    in std_logic_vector(3 downto 0);
           WD:    in std_logic_vector(15 downto 0);
           RD:    out std_logic_vector(15 downto 0)
    );
end component;

begin
ssd1: SSD
port map (
digit => DisplayResult,
clk   => clk,
cat   => cat,
an    => an
);
mpg1 : MPG 
port map (
 clk => clk,
 btn => btn(2),
 en  => rst
);
mpg5: MPG
port map (
clk =>clk,
btn =>btn(3),
en => tx_en
);
mpg3: MPG
port map(
clk  => clk,
btn  => btn(0),
en   => resetPC
);
mpg4: MPG
port map(
clk  => clk,
btn => btn(1),
en => enablePC
);
regFile :RF
port map(
 clk      => clk,
 RegWrite => RegWrite_WB,
 RA1      => rs,
 RA2      => rt,
 WA       => WA_WB,
 WD       => WD,
 RD1      => RD1,
 RD2      => RD2
);

ram1 :RAM
port map(
clk     =>clk,
RegWr   =>MemWrite_MEM,
RA      =>ALUresult_MEM(3 downto 0),
WA      =>ALUresult_MEM(3 downto 0),
WD      =>RD2_MEM,
RD      =>MemData
);

--Previous labs
--START CLK
--process(CLK)
--begin
--    if rising_edge(CLK) then
--        if enable1='1' then
--            cnt<=cnt+1;
--        end if;
--    end if;
--end process;

--process(result)
--begin
--    if result=x"0000" then
--        led(7)<='1';
--    else
--        led(7)<='0';
--    end if;
--end process;

--START ROM (Instruction memory)
--process(clk)
--begin
--    if rising_edge(clk) then
--        if enable1='1' then
--            result <= romMem(conv_integer(PC));
--        end if;
--    end if;
--end process;

--START RegFile
--reset <=btn(2);
--result <= RD1+RD2;
--process(clk,reset,enable1)
--begin
--    if rising_edge(clk) then
--        if enable1='1' then
--             cnt<=cnt+1;
--        end if;
--    end if;
--    if reset ='1' then
--        cnt <="0000";
--    end if;
--end process;

--PC & instruction fetch  (Lab5)
process(CLK,resetPC,enablePC,mux2)
begin
    if resetPC='1' then
        PC <=x"0000";
        else
            if rising_edge(CLK) then
                if enablePC='1' then
                    PC <=mux2;
                end if; 
            end if;
    end if;
end process;

process(PCSrc,BranchAddress_MEM,PCplusone)
begin
    if PCSrc='1' then
        mux1 <= BranchAddress_MEM;
        else
            mux1 <= PCplusone;
    end if;
end process;

process(Jump,JumpAddress,mux1)
begin
    if Jump ='1' then
        mux2 <= JumpAddress;
        else
            mux2<=mux1;
    end if;
end process;

PCplusone <= PC + 1;  -- adder implemented with +1
instruction <= romMem(conv_integer(PC));   --fetching the instruction at the address given by the PC


--Instruction Decode Unit (Lab6)
rs<= "0" & instruction_ID(12 downto 10);
rt<= "0" & instruction_ID(9 downto 7);
rd<= "0" & instruction_ID(6 downto 4);

WA <= rd_ex when RegDst_EX='1' else rt_ex when RegDst_EX='0';
sa <= instruction_ID(3);
Funct <= instruction_ID(2 downto 0);
Ext_Imm(6 downto 0)  <= instruction_ID(6 downto 0);
Ext_Imm(15 downto 7) <= (others => instruction_ID(6)) when ExtOp = '1' else (others => '0') when ExtOp = '0';

--Control unit (Lab6)
process(instruction,opcode,funct)
begin
    RegDst<='0'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';Beq<='0';Bneq<='0';
    if opcode="000" then    --Rtype
        case funct is
            when "000" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --add        
            when "001" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0001"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --sub        
            when "010" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0010"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --sll        
            when "011" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0011"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --srl        
            when "100" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0100"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --and        
            when "101" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0101"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --or         
            when "110" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0110"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --xor        
            when "111" =>  RegDst<='1'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0111"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --nor        
            when others=>  RegDst<='0'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';
        end case;                                
    else                                  
        case opcode is                    
            when "001" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --addi  
            when "010" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='1'; RegWrite<='1';     --lw     
            when "011" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='1'; MemtoReg<='0'; RegWrite<='0';     --sw     
            when "100" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='1'; Jump<='0'; ALUop<="1001"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';     --beq    
            when "101" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='1'; Jump<='0'; ALUop<="1010"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';     --bne    
            when "110" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='0'; Jump<='0'; ALUop<="0101"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='1';     --ori   
            when "111" =>  RegDst<='0'; ExtOP<='1'; ALUSrc<='1'; Branch<='0'; Jump<='1'; ALUop<="1000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';     --jump   
            when others=>  RegDst<='0'; ExtOP<='0'; ALUSrc<='0'; Branch<='0'; Jump<='0'; ALUop<="0000"; MemWrite<='0'; MemtoReg<='0'; RegWrite<='0';
        end case;
    end if;
end process;

--ALU  (Lab7)
nr1 <= RD1_EX;
nr2 <= RD2_EX when ALUSrc_EX='0' else EXT_Imm_EX;
BranchAddress <= PCplusone_EX + EXT_Imm_EX;
JumpAddress   <= "000" & instruction_ID(12 downto 0);
process (ALUop_EX,nr1,nr2,ALUresult,sa_ex) 
begin
    case ALUop_EX is
         when "0000"  =>   ALUresult <= nr1+nr2;                      --add  
         when "0001"  =>   ALUresult <= nr1-nr2;                      --sub  
         when "0010"  =>                                              --sll
         if sa_ex='0' then
            ALUresult <= nr1(14 downto 0) & '0';   
         else
            ALUresult <= nr1(14 downto 0) & '1';
         end if;    
         when "0011"  =>                                              --srl  
         if sa_ex='0' then                                         
            ALUresult <= '0' & nr1(15 downto 1);  
         else
            ALUresult <= '1' & nr1(15 downto 1);  
         end if;  
         when "0100"  =>   ALUresult <= nr1 AND nr2;                      --and  
         when "0101"  =>   ALUresult <= nr1 OR  nr2;                      --or   
         when "0110"  =>   ALUresult <= nr1 XOR nr2;                      --xor  
         when "0111"  =>   ALUresult <= nr1 XOR nr2;                      --nor  
         when "1000"  =>   ALUresult <= x"0000";                          --Branch    
         when "1001"  =>                                                  --beq
         if nr1=nr2 then 
            ALUresult <=x"0000";
         else
            ALUresult <=x"1111"; --here we want any value except 0 (to not trigger Zero=1) 
         end if;  
         when "1010"  =>                                                  --bneq
         if nr1/=nr2 then 
            ALUresult <=x"0000";
         else
            ALUresult <=x"1111"; --here we want any value except 0 (to not trigger Zero=1) 
         end if;  
         when others  =>   ALUresult <= x"0000";
    end case;
end process;

Zero  <= '1' when ALUresult=x"0000" else '0';
PCSrc <= '1' when Zero_MEM='1' AND Branch_MEM='1' else '0';

result <=MemData_WB when MemtoReg_WB='1' else ALUresult_WB;
WD <= result;

process(sw(7 downto 5),instruction,PCplusOne,RD1,RD2,Ext_Imm,ALUResult,MemData,WD)
begin
    case(sw(7 downto 5)) is
        when "000"  => DisplayResult <= instruction;
        when "001"  => DisplayResult <= PCplusOne;
        when "010"  => DisplayResult <= RD1;
        when "011"  => DisplayResult <= RD2;
        when "100"  => DisplayResult <= Ext_Imm;
        when "101"  => DisplayResult <= ALUResult;
        when "110"  => DisplayResult <= MemData;
        when "111"  => DisplayResult <= WD;
        when others => DisplayResult <=x"0000";
    end case;
end process;

--PIPELINE REGISTERS

--IF/ID
process(clk)
begin
    if rising_edge(clk) then
        PCplusone_ID   <= PCplusone;
        instruction_ID <= instruction;
    end if;
end process;

--ID/EX
process(clk)
begin
    if rising_edge(clk) then
        ALUop_EX     <=  ALUop;  
        MemWrite_EX  <=  MemWrite;
        MemtoReg_EX  <=  MemtoReg; 
        RegWrite_EX  <=  RegWrite; 
        Beq_EX       <=  Beq;      
        Bneq_EX      <=  Bneq;     
        RegDst_EX    <=  RegDst;   
        ALUSrc_EX    <=  ALUSrc;   
        Branch_EX    <=  Branch;
        EXT_Imm_EX   <=  EXT_Imm;   
        RD1_EX       <=  RD1;
        RD2_EX       <=  RD2;
        rd_ex        <=  rd;
        rt_ex        <=  rt;
        sa_ex        <=  sa;
        PCplusone_EX <= PCplusone_ID;
    end if;
end process;

--EX/MEM
process(clk)
begin
    if rising_edge(clk) then
        Branch_MEM   <=  Branch_EX;
        Beq_MEM      <=  Beq_EX;   
        Bneq_MEM     <=  Bneq_EX;    
        MemWrite_MEM <=  MemWrite_EX;
        MemtoReg_MEM <=  MemtoReg_EX;
        RegWrite_MEM <=  RegWrite_EX;
        BranchAddress_MEM <= BranchAddress;
        Zero_MEM      <= Zero;
        ALUresult_MEM <= ALUresult;
        WA_MEM        <=WA;
        RD2_MEM        <=RD2_EX;
    end if;
end process;

--MEM/WB
process(clk)
begin
    if rising_edge(clk) then
       MemtoReg_WB <= MemtoReg_MEM;
       RegWrite_WB <= RegWrite_MEM;
       ALUresult_WB <= ALUresult_MEM;
       WA_WB <= WA_MEM;
       MemData_WB <= MemData;     
    end if;
end process;


--FSM Serial Communication
--Clock with 100 mhz frequency ==> we need to count 10416 clock cycles to obtain a baud rate of 9600
process(clk)
begin
    if rising_edge(clk) then
        BAUD_EN <='0';
        BAUD_COUNT<=BAUD_COUNT+1;
        if Baud_Count="10100010110000" then
            BAUD_EN <= '1';
            Baud_Count <="00000000000000";
        end if;
    end if;
end process;

process(clk,rst,TX_state,bit_cnt,tx_en)
begin
    if rst='1' then
        TX_state<="00";
    elsif rising_edge(clk) then
      if baud_en = '1' then
        case TX_state is 
            when "00" => 
                if tx_en ='1' then
                    TX_state <="01";
                end if;
            when "01" =>
                TX_state <="10";
            when "10" =>
                if bit_cnt="111" then
                    bit_cnt <="000";
                    TX_state <="11";
                else
                    bit_cnt <= bit_cnt+1;
                end if;
            when "11" =>
                TX_state <="00";
        end case;
      end if;
    end if;
end process;

process(TX_STATE,tx_data,bit_cnt)
begin
    case TX_state is 
        when "00" =>
            TX <= '1';
            TX_RDY <= '1';
        when "01" =>
            TX <= '0';
            TX_RDY <= '0';
        when "10" =>
            TX_RDY <= '0';
            TX <= tx_data(conv_integer(bit_cnt));
        when "11" =>
            Tx <= '1';
            TX_RDY <= '0';
    end case;
end process;


process(clk,rx,rst,rx_state,baud_en,baud_cnt_rx,bit_cnt_rx,output_rx)
begin
    if rst='1' then 
        rx_state <= "000";
    elsif rising_edge(clk) then
      if baud_en <= '1' then
        case rx_state is
            when "000" =>
                if rx='0' then
                    rx_state <= "001";
                end if;
            when "001" =>
                if rx <= '1' then
                    rx_state <="000";
                end if;
                if baud_en = '1' then
                    baud_cnt_rx <= baud_cnt_rx + 1;
                end if;
                if baud_cnt_rx = "111" AND RX = '0' then
                    baud_cnt_rx <="0000";
                    rx_state <= "010";
                end if;
            when "010" =>
                if bit_cnt_rx = "111" then
                    bit_cnt_rx <= "000";
                    baud_cnt_rx <="0000";
                    rx_state <= "011";
                elsif baud_cnt_rx <="1111" then
                    output_rx(conv_integer(bit_cnt)) <= rx;
                    bit_cnt_rx <= bit_cnt_rx +1;
                    baud_cnt_rx <= "0000";
                else
                    baud_cnt_rx <= baud_cnt_rx + 1;
                end if;
            when "011" =>
                if baud_cnt_rx = "1111" then
                    rx_state <= "100";
                    baud_cnt_rx <="0000";
                elsif baud_en ='1' then
                    baud_cnt_rx <= baud_cnt_rx + 1;
                end if;
            when "100" =>
                if baud_cnt_rx ="0111" then
                    rx_state <= "000";
                    baud_cnt_rx <= "0000";
                elsif baud_en = '1' then
                    baud_cnt_rx <= baud_cnt_rx + 1;
                end if;
            when others =>
                rx_state <= "000";
        end case;
      end if;
    end if;
end process;

process(rx_state,output_rx,rx,rx_rdy,rx_data)
begin
    case rx_state is
        when "000" =>
            rx_rdy <= '0';
        when "001" =>
            rx_rdy <='0';
        when "010" =>
            rx_rdy <= '0';
        when "011" =>
            rx_rdy <= '0';
            rx_data <= output_rx;
        when "100" =>
            rx_rdy <= '1';
        when others =>
            rx_rdy <= '0'; 
    end case;
end process;

end Behavioral;