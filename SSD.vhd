library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity SSD is
    Port ( digit: in std_logic_vector(15 downto 0);
           clk :  in STD_LOGIC;
           cat :  out STD_LOGIC_VECTOR (6 downto 0);
           an :   out STD_LOGIC_VECTOR (3 downto 0));
end SSD;

architecture Behavioral of SSD is
signal cnt:  std_logic_vector(15 downto 0) :=x"0000";
signal mux1: std_logic_vector(3 downto 0) :=x"0";
signal mux2: std_logic_vector(3 downto 0) :=x"0";

begin
an <= mux2;

process(clk)
begin
    if rising_edge(CLK) then
        cnt <= cnt+1;
    end if;
end process;

process(cnt(15 downto 14),digit)
begin
    case(cnt (15 downto 14)) is
        when "00" => 
        mux1 <= digit(3 downto 0);
        mux2 <= "1110";
        when "01" =>
        mux1 <= digit(7 downto 4);
        mux2 <= "1101";
        when "10" =>
        mux1 <= digit(11 downto 8);
        mux2 <= "1011";
        when "11" =>
        mux1 <= digit(15 downto 12);
        mux2 <= "0111";
        when others =>
        mux1 <="0000";
        mux2 <="0000";
    end case;
end process;

process (mux1)
begin
    case(mux1) is
         when "0001" => cat <="1111001"; --1
         when "0010" => cat <="0100100"; --2
         when "0011" => cat <="0110000"; --3
         when "0100" => cat <="0011001"; --4
         when "0101" => cat <="0010010"; --5
         when "0110" => cat <="0000010"; --6
         when "0111" => cat <="1111000"; --7
         when "1000" => cat <="0000000"; --8
         when "1001" => cat <="0010000"; --9
         when "1010" => cat <="0001000"; --A
         when "1011" => cat <="0000011"; --b
         when "1100" => cat <="1000110"; --C
         when "1101" => cat <="0100001"; --d
         when "1110" => cat <="0000110"; --E
         when "1111" => cat <="0001110"; --F
         when others => cat <="1000000"; --0
    end case;
end process;
end Behavioral;
