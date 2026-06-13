-- ============================================================================
-- tb_polar_kin - Verifica la conversión a coordenada polar (casos a mano)
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_polar_kin is
end tb_polar_kin;

architecture sim of tb_polar_kin is
    component polar_kin
        generic (L3 : integer := 63; Z0 : integer := 141);
        port (
            clk    : in  std_logic;
            rst    : in  std_logic;
            start  : in  std_logic;
            theta3 : in  std_logic_vector(7 downto 0);
            d      : in  std_logic_vector(15 downto 0);
            r      : out std_logic_vector(15 downto 0);
            theta  : out std_logic_vector(8 downto 0);
            done   : out std_logic
        );
    end component;

    signal clk    : std_logic := '0';
    signal rst    : std_logic := '1';
    signal start  : std_logic := '0';
    signal theta3 : std_logic_vector(7 downto 0) := (others => '0');
    signal d      : std_logic_vector(15 downto 0) := (others => '0');
    signal r      : std_logic_vector(15 downto 0);
    signal theta  : std_logic_vector(8 downto 0);
    signal done   : std_logic;
    signal simdone: boolean := false;

    procedure run_case(signal clk_s : in std_logic;
                       signal st_s   : out std_logic;
                       signal t3_s   : out std_logic_vector(7 downto 0);
                       signal d_s    : out std_logic_vector(15 downto 0);
                       signal dn_s   : in std_logic;
                       t3v : integer; dv : integer) is
    begin
        t3_s <= std_logic_vector(to_unsigned(t3v, 8));
        d_s  <= std_logic_vector(to_unsigned(dv, 16));
        wait until rising_edge(clk_s);
        st_s <= '1';
        wait until rising_edge(clk_s);
        st_s <= '0';
        wait until dn_s = '1';
        wait until rising_edge(clk_s);
    end procedure;

begin
    dut : polar_kin
        port map (clk => clk, rst => rst, start => start,
                  theta3 => theta3, d => d, r => r, theta => theta, done => done);

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns;
            clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 100 ns; rst <= '0';
        wait until rising_edge(clk);

        -- Caso 1: theta3=45, d=200  -> esperado r≈298, theta≈28
        run_case(clk, start, theta3, d, done, 45, 200);
        report "Caso theta3=45 d=200 -> r=" & integer'image(to_integer(unsigned(r))) &
               " mm, theta=" & integer'image(to_integer(signed(theta))) & " deg (esp r~298, th~28)"
               severity note;

        -- Caso 2: theta3=0, d=200   -> esperado r≈190, theta≈-13
        run_case(clk, start, theta3, d, done, 0, 200);
        report "Caso theta3=0  d=200 -> r=" & integer'image(to_integer(unsigned(r))) &
               " mm, theta=" & integer'image(to_integer(signed(theta))) & " deg (esp r~190, th~-13)"
               severity note;

        -- Caso 3: theta3=45, d=0    -> esperado r≈154, theta≈66
        run_case(clk, start, theta3, d, done, 45, 0);
        report "Caso theta3=45 d=0   -> r=" & integer'image(to_integer(unsigned(r))) &
               " mm, theta=" & integer'image(to_integer(signed(theta))) & " deg (esp r~154, th~66)"
               severity note;

        simdone <= true;
        wait;
    end process;
end sim;
