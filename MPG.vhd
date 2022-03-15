library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity MPG is
    Port ( btn : in STD_LOGIC;
           clk : in STD_LOGIC;
           en :  out STD_LOGIC);
end MPG;

architecture Behavioral of MPG is
signal cnt_int: std_logic_vector(15 downto 0) :=x"0000";
signal q1:      std_logic:='0';
signal q2:      std_logic:='0';
signal q3:      std_logic:='0';
begin
en <= q2 AND (not q3);

process (clk) 
begin
    if rising_edge(clk) then
    cnt_int <= cnt_int+1;
    end if;
end process;

process(clk)
begin
    if rising_edge(clk) then
        if cnt_int = x"1111" then
            q1 <= btn;
        end if;
    end if;
end process;

process(clk)
begin
    if rising_edge(clk) then
        q2 <= q1;
        q3 <= q2;
    end if;
end process;

end Behavioral;
