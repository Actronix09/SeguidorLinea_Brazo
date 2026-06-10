-- ============================================================================
-- tb_grab_ctrl - Ciclo completo del brazo (Etapa 2): barrido -> agarre -> HOLD
--                -> depósito -> REST, con kinematics fusionada.
-- ----------------------------------------------------------------------------
--   vlib work
--   vcom -2008 kinematics.vhd grab_ctrl.vhd tb_grab_ctrl.vhd
--   vsim tb_grab_ctrl -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_grab_ctrl is
end tb_grab_ctrl;

architecture sim of tb_grab_ctrl is

    component grab_ctrl
        generic (MOVE_CYCLES : integer := 125_000_000;
                 GRIP_CYCLES : integer := 40_000_000;
                 DROP_PHI : integer := 90; DROP_T1 : integer := 45;
                 DROP_T2 : integer := 0; DROP_T3 : integer := 0);
        port (
            clk          : in  std_logic;
            rst          : in  std_logic;
            scan_active  : in  std_logic;
            scan_done    : in  std_logic;
            found        : in  std_logic;
            min_t1       : in  std_logic_vector(7 downto 0);
            min_d        : in  std_logic_vector(15 downto 0);
            min_phi      : in  std_logic_vector(7 downto 0);
            cmd_phi      : in  std_logic_vector(7 downto 0);
            cmd_theta1   : in  std_logic_vector(7 downto 0);
            cmd_theta2   : in  std_logic_vector(7 downto 0);
            cmd_theta3   : in  std_logic_vector(7 downto 0);
            cmd_grip     : in  std_logic;
            trigger_drop : in  std_logic;
            phi_out      : out std_logic_vector(7 downto 0);
            theta1_out   : out std_logic_vector(7 downto 0);
            theta2_out   : out std_logic_vector(7 downto 0);
            theta3_out   : out std_logic_vector(7 downto 0);
            grip_out     : out std_logic;
            has_object   : out std_logic;
            arm_ready    : out std_logic;
            reachable    : out std_logic
        );
    end component;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal scan_active : std_logic := '0';
    signal scan_done   : std_logic := '0';
    signal found       : std_logic := '0';
    signal min_t1, min_phi : std_logic_vector(7 downto 0) := (others => '0');
    signal min_d : std_logic_vector(15 downto 0) := (others => '0');
    signal cmd_phi, cmd_t1, cmd_t2, cmd_t3 : std_logic_vector(7 downto 0) := (others => '0');
    signal cmd_grip : std_logic := '0';
    signal trigger_drop : std_logic := '0';
    signal o_phi, o_t1, o_t2, o_t3 : std_logic_vector(7 downto 0);
    signal o_grip, has_object, arm_ready, reachable : std_logic;
    signal simdone : boolean := false;

    function img(s : std_logic_vector) return string is
    begin
        return integer'image(to_integer(unsigned(s)));
    end function;

begin

    dut : grab_ctrl
        generic map (MOVE_CYCLES => 20, GRIP_CYCLES => 10)
        port map (
            clk => clk, rst => rst,
            scan_active => scan_active, scan_done => scan_done, found => found,
            min_t1 => min_t1, min_d => min_d, min_phi => min_phi,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_t1, cmd_theta2 => cmd_t2,
            cmd_theta3 => cmd_t3, cmd_grip => cmd_grip, trigger_drop => trigger_drop,
            phi_out => o_phi, theta1_out => o_t1, theta2_out => o_t2,
            theta3_out => o_t3, grip_out => o_grip,
            has_object => has_object, arm_ready => arm_ready, reachable => reachable
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
        assert arm_ready = '1' report "FALLO: no arranca en REST listo" severity error;

        -- ---- Fase 1: barrido en curso, el MUX pasa cmd_* y garra abierta ----
        scan_active <= '1';
        cmd_phi <= std_logic_vector(to_unsigned(123, 8));
        cmd_t1  <= std_logic_vector(to_unsigned(70, 8));
        cmd_t2  <= std_logic_vector(to_unsigned(20, 8));
        cmd_t3  <= std_logic_vector(to_unsigned(0, 8));
        cmd_grip <= '1';
        wait for 200 ns;
        assert o_phi = cmd_phi and o_t1 = cmd_t1 and o_grip = '1'
            report "FALLO: durante el barrido el MUX no pasa cmd_*/garra" severity error;
        report "Fase1 OK: MUX pasa cmd_* (phi=" & img(o_phi) & " t1=" & img(o_t1) & ")" severity note;

        -- ---- Fase 2: fin del barrido con objeto encontrado ----
        min_t1  <= std_logic_vector(to_unsigned(60, 8));
        min_d   <= std_logic_vector(to_unsigned(99, 16));
        min_phi <= std_logic_vector(to_unsigned(90, 8));
        found   <= '1';
        wait until rising_edge(clk);
        scan_active <= '0';
        scan_done   <= '1';

        -- Espera a que cierre la garra (has_object='1')
        wait until has_object = '1' for 50 us;
        assert has_object = '1' report "FALLO: nunca agarró el objeto" severity error;
        report "Agarre: phi=" & img(o_phi) & " t1=" & img(o_t1) & " t2=" & img(o_t2) &
               " t3=" & img(o_t3) & " grip=" & std_logic'image(o_grip) severity note;
        assert reachable = '1' report "FALLO: objetivo fuera de alcance" severity error;
        assert to_integer(unsigned(o_t1)) >= 36 and to_integer(unsigned(o_t1)) <= 44
            report "FALLO: theta1 de agarre fuera de ~40" severity error;

        -- Espera a llegar a HOLD (acarreo)
        wait until arm_ready = '1' for 50 us;
        assert arm_ready = '1' report "FALLO: no llegó a HOLD" severity error;
        wait for 60 ns;   -- deja asentar los registros de pose (1 ciclo tras arm_ready)
        assert o_phi = std_logic_vector(to_unsigned(180, 8)) and o_t1 = std_logic_vector(to_unsigned(135, 8))
            report "FALLO: pose de acarreo != (180,135)" severity error;
        assert has_object = '1' report "FALLO: perdió el objeto en HOLD" severity error;
        report "Acarreo (HOLD): phi=" & img(o_phi) & " t1=" & img(o_t1) & " has_object=1" severity note;

        -- ---- Fase 3: depósito ----
        trigger_drop <= '1'; wait until rising_edge(clk); trigger_drop <= '0';

        -- Espera a que suelte (has_object='0')
        wait until has_object = '0' for 50 us;
        assert has_object = '0' report "FALLO: no soltó el objeto" severity error;
        assert o_phi = std_logic_vector(to_unsigned(90, 8))
            report "FALLO: no giró a phi=90 al depositar" severity error;
        assert o_grip = '1' report "FALLO: no abrió la garra al depositar" severity error;
        report "Depósito: phi=" & img(o_phi) & " t1=" & img(o_t1) & " grip=" &
               std_logic'image(o_grip) & " has_object=0" severity note;

        -- Espera a volver a REST
        wait until arm_ready = '1' for 50 us;
        wait for 60 ns;   -- deja asentar los registros de pose
        assert o_phi = std_logic_vector(to_unsigned(180, 8)) and o_t1 = std_logic_vector(to_unsigned(90, 8))
            report "FALLO: no volvió a REST (180,90)" severity error;
        report "OK: sigue, escanea, agarra, acarrea (HOLD), deposita y vuelve a REST" severity note;

        simdone <= true;
        wait;
    end process;

end sim;
