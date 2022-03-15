----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 03/09/2021 08:44:41 PM
-- Design Name: 
-- Module Name: RF - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity RF is
     Port (clk: in std_logic;
           RegWrite: in std_logic;
           RA1: in std_logic_vector(3 downto 0);
           RA2: in std_logic_vector(3 downto 0);
           WA: in std_logic_vector(3 downto 0);
           WD: in std_logic_vector(15 downto 0);
           RD1: out std_logic_vector(15 downto 0);
           RD2: out std_logic_vector(15 downto 0)
         );
end RF;

architecture Behavioral of RF is

type RegFileMem is array (0 to 15) of std_logic_vector(15 downto 0);
signal mem: RegFileMem:=(
x"0001",
x"0010",
x"0100",
x"1000",
x"FFFF",
x"ABCD",
others =>x"0000"
);

begin
process(clk,RegWrite)
begin
    if rising_edge(clk) then
        if RegWrite='1' then
            mem(conv_integer(WA)) <= WD;
        end if;
    end if;
end process;

RD1<= mem(conv_integer(RA1));
RD2<= mem(conv_integer(RA2));

end Behavioral;
