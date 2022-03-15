----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 02/27/2021 06:34:27 PM
-- Design Name: 
-- Module Name: test_new - Behavioral
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

entity test_new is
    Port ( clk : in STD_LOGIC;
           btn : in STD_LOGIC_VECTOR (4 downto 0);
           sw : in STD_LOGIC_VECTOR (15 downto 0);
           led : out STD_LOGIC_VECTOR (15 downto 0);
           an : out STD_LOGIC_VECTOR (3 downto 0);
           cat : out STD_LOGIC_VECTOR (6 downto 0));
end test_new;

architecture Behavioral of test_new is
signal enable: std_logic :='0';
signal cnt: std_logic_vector(2 downto 0) := "000";
signal dcd: std_logic_vector(2 downto 0) := "000";

component MPG is 
    Port ( btn : in STD_LOGIC;
           clk : in STD_LOGIC;
           en : out STD_LOGIC);
end component;

begin

mpg1: MPG
port map (
 clk => clk,
 btn => btn(0),
 en => enable
);

process (clk)
begin
    if rising_edge(clk) then
        if enable='1' then
            cnt <= cnt+1;
        end if;
    end if;
end process;
dcd <= cnt;

process (clk)
begin
    if rising_edge(clk) then
        case dcd is
            when "000" => led(7 downto 0) <="00000001";
            when "001" => led(7 downto 0) <="00000010";
            when "010" => led(7 downto 0) <="00000100";
            when "011" => led(7 downto 0) <="00001000";
            when "100" => led(7 downto 0) <="00010000";
            when "101" => led(7 downto 0) <="00100000";
            when "110" => led(7 downto 0) <="01000000";
            when "111" => led(7 downto 0) <="10000000";
            when others => led(7 downto 0) <="00000000";
        end case;
    end if;
end process;

end Behavioral;
