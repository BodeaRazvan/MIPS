----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 03/09/2021 10:02:11 PM
-- Design Name: 
-- Module Name: RAM - Behavioral
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

entity RAM is
    Port ( clk: in std_logic;
           RegWr: in std_logic;
           RA: in std_logic_vector(3 downto 0);
           WA: in std_logic_vector(3 downto 0);
           WD: in std_logic_vector(15 downto 0);
               RD: out std_logic_vector(15 downto 0)
    );
end RAM;

architecture Behavioral of RAM is
type RamMem is array (0 to 15) of std_logic_vector(15 downto 0);
signal RAM :RamMem :=(
x"0001",
x"0010",
x"0011",
x"0100",
others =>x"0000"
);

begin

process(clk,RegWr,RA)     --Synchronous write and asynchronous read
begin
    if rising_edge(clk) then
        if RegWr='1' then
            RAM(conv_integer(WA)) <= WD;
        end if;
    end if;
    RD <= RAM(conv_integer(RA));
end process;

end Behavioral;
