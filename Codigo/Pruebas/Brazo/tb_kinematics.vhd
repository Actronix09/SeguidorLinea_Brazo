-- ============================================================================
-- tb_kinematics - Valida el motor fusionado FK+IK (kinematics.vhd).
-- ----------------------------------------------------------------------------
-- Caso conocido del pipeline Etapa 1:
--   barrido -> mínimo en theta1*=60, d*=99 mm, phi*=90
--   FK debe dar (r ~168 mm, theta ~-27 deg)
--   IK (con Z_DROP=35, R_TRIM=20, L_GRIP=90) debe dar t1~40, t2~0, t3~58.
--
--   vlib work
--   vcom -2008 kinematics.vhd tb_kinematics.vhd
--   vsim tb_kinematics -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_kinematics is
end tb_kinematics;

architecture sim of tb_kinematics is

    component kinematics
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            start     : in  std_logic;
            in_t1     : in  std_logic_vector(7 downto 0);
            in_d      : in  std_logic_vector(15 downto 0);
            in_phi    : in  std_logic_vector(7 downto 0);
            o_phi     : out std_logic_vector(7 downto 0);
            o_theta1  : out std_logic_vector(7 downto 0);
            o_theta2  : out std_logic_vector(7 downto 0);
            o_theta3  : out std_logic_vector(7 downto 0);
            reachable : out std_logic;
            done      : out std_logic
        );
    end component;

    signal clk   : std_logic := '0';
    signal rst   : std_logic := '1';
    signal start : std_logic := '0';
    signal in_t1 : std_logic_vector(7 downto 0) := (others => '0');
    signal in_d  : std_logic_vector(15 downto 0) := (others => '0');
    signal in_phi: std_logic_vector(7 downto 0) := (others => '0');
    signal o_phi, o_t1, o_t2, o_t3 : std_logic_vector(7 downto 0);
    signal reachable, done : std_logic;
    signal simdone : boolean := false;

    function img(s : std_logic_vector) return string is
    begin
        return integer'image(to_integer(unsigned(s)));
    end function;

begin

    dut : kinematics
        port map (
            clk => clk, rst => rst, start => start,
            in_t1 => in_t1, in_d => in_d, in_phi => in_phi,
            o_phi => o_phi, o_theta1 => o_t1, o_theta2 => o_t2, o_theta3 => o_t3,
            reachable => reachable, done => done
        );

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

        -- Caso de prueba: mínimo del barrido
        in_t1  <= std_logic_vector(to_unsigned(60, 8));
        in_d   <= std_logic_vector(to_unsigned(99, 16));
        in_phi <= std_logic_vector(to_unsigned(90, 8));
        wait until rising_edge(clk);
        start <= '1'; wait until rising_edge(clk); start <= '0';

        wait until done = '1' for 20 us;
        assert done = '1' report "FALLO: kinematics no terminó" severity error;

        report "Resultado: phi=" & img(o_phi) & " t1=" & img(o_t1) &
               " t2=" & img(o_t2) & " t3=" & img(o_t3) &
               " reach=" & std_logic'image(reachable) severity note;

        assert o_phi = std_logic_vector(to_unsigned(90, 8))
            report "FALLO: phi != 90" severity error;
        assert reachable = '1'
            report "FALLO: marcado fuera de alcance" severity error;
        -- Pose esperada con Z_DROP=35, R_TRIM=20: t1~40, t2~0, t3~58
        assert to_integer(unsigned(o_t1)) >= 36 and to_integer(unsigned(o_t1)) <= 44
            report "FALLO: theta1 fuera de ~40" severity error;
        assert to_integer(unsigned(o_t2)) >= 0 and to_integer(unsigned(o_t2)) <= 6
            report "FALLO: theta2 fuera de ~0" severity error;
        assert to_integer(unsigned(o_t3)) >= 54 and to_integer(unsigned(o_t3)) <= 62
            report "FALLO: theta3 fuera de ~58" severity error;

        report "OK: FK+IK fusionadas dan la misma pose que el pipeline de 2 modulos" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
