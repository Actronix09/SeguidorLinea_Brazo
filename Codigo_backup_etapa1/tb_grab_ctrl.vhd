-- ============================================================================
-- tb_grab_ctrl - Integración de la orquestación de agarre (FSM + polar_ik + MUX).
-- ----------------------------------------------------------------------------
-- 1) Simula el barrido: scan_active='1' -> los servos siguen a cmd_* (MUX).
-- 2) Termina el barrido (scan_done='1') con la coordenada del cubo de prueba
--    (r=168, theta=-27, phi=90) -> debe correr la IK, mover el brazo a
--    (phi=90, t1=46, t2=7, t3=36) [con L_GRIP=90], cerrar la garra y levantar (t1->90).
--
--   vlib work
--   vcom -2008 polar_ik.vhd grab_ctrl.vhd tb_grab_ctrl.vhd
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
                 LIFT_CYCLES : integer := 60_000_000);
        port (
            clk         : in  std_logic;
            rst         : in  std_logic;
            scan_active : in  std_logic;
            scan_done   : in  std_logic;
            in_r        : in  std_logic_vector(15 downto 0);
            in_theta    : in  std_logic_vector(8 downto 0);
            in_phi      : in  std_logic_vector(7 downto 0);
            cmd_phi     : in  std_logic_vector(7 downto 0);
            cmd_theta1  : in  std_logic_vector(7 downto 0);
            cmd_theta2  : in  std_logic_vector(7 downto 0);
            cmd_theta3  : in  std_logic_vector(7 downto 0);
            cmd_grip    : in  std_logic;
            phi_out     : out std_logic_vector(7 downto 0);
            theta1_out  : out std_logic_vector(7 downto 0);
            theta2_out  : out std_logic_vector(7 downto 0);
            theta3_out  : out std_logic_vector(7 downto 0);
            grip_out    : out std_logic;
            grip_closed : out std_logic;
            reachable   : out std_logic;
            done_all    : out std_logic
        );
    end component;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal scan_active : std_logic := '0';
    signal scan_done   : std_logic := '0';
    signal in_r   : std_logic_vector(15 downto 0) := (others => '0');
    signal in_theta : std_logic_vector(8 downto 0) := (others => '0');
    signal in_phi : std_logic_vector(7 downto 0) := (others => '0');
    signal cmd_phi, cmd_t1, cmd_t2, cmd_t3 : std_logic_vector(7 downto 0) := (others => '0');
    signal cmd_grip : std_logic := '0';
    signal o_phi, o_t1, o_t2, o_t3 : std_logic_vector(7 downto 0);
    signal o_grip, grip_closed, reachable, done_all : std_logic;
    signal simdone : boolean := false;

    function img(s : std_logic_vector) return string is
    begin
        return integer'image(to_integer(unsigned(s)));
    end function;

begin

    dut : grab_ctrl
        generic map (MOVE_CYCLES => 20, GRIP_CYCLES => 10, LIFT_CYCLES => 10)
        port map (
            clk => clk, rst => rst,
            scan_active => scan_active, scan_done => scan_done,
            in_r => in_r, in_theta => in_theta, in_phi => in_phi,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_t1, cmd_theta2 => cmd_t2,
            cmd_theta3 => cmd_t3, cmd_grip => cmd_grip,
            phi_out => o_phi, theta1_out => o_t1, theta2_out => o_t2,
            theta3_out => o_t3, grip_out => o_grip,
            grip_closed => grip_closed, reachable => reachable, done_all => done_all
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

        -- ---- Fase 1: barrido en curso, el MUX debe pasar cmd_* ----
        scan_active <= '1'; scan_done <= '0';
        cmd_phi <= std_logic_vector(to_unsigned(123, 8));
        cmd_t1  <= std_logic_vector(to_unsigned(70, 8));
        cmd_t2  <= std_logic_vector(to_unsigned(20, 8));
        cmd_t3  <= std_logic_vector(to_unsigned(0, 8));
        cmd_grip <= '1';                    -- 1 = abierta (convencion TestBrazo)
        wait for 200 ns;
        assert o_phi = cmd_phi and o_t1 = cmd_t1 and o_t2 = cmd_t2 and o_t3 = cmd_t3
            report "FALLO: durante el barrido el MUX no pasa cmd_*" severity error;
        assert o_grip = '1'
            report "FALLO: durante el barrido la garra no esta abierta" severity error;
        report "Fase1 OK: MUX pasa cmd_* (phi=" & img(o_phi) & " t1=" & img(o_t1) & ")" severity note;

        -- ---- Fase 2: fin del barrido con la coordenada del cubo ----
        in_r    <= std_logic_vector(to_unsigned(168, 16));
        in_theta<= std_logic_vector(to_signed(-27, 9));
        in_phi  <= std_logic_vector(to_unsigned(90, 8));
        wait until rising_edge(clk);
        scan_active <= '0';                 -- el LIDAR baja scan_active
        scan_done   <= '1';                 -- y deja scan_done en alto

        -- Espera a que cierre la garra (fin del movimiento de aproximacion)
        wait until grip_closed = '1' for 50 us;
        assert grip_closed = '1' report "FALLO: nunca cerro la garra" severity error;

        report "Pose de agarre: phi=" & img(o_phi) & " t1=" & img(o_t1) &
               " t2=" & img(o_t2) & " t3=" & img(o_t3) & " grip=" & std_logic'image(o_grip)
               severity note;
        assert o_phi = std_logic_vector(to_unsigned(90, 8))
            report "FALLO: phi de agarre != 90" severity error;
        -- Con Z_DROP=35 + R_TRIM=20: pose ~ t1=40, t2=0 (codo en su limite), t3=58
        assert to_integer(unsigned(o_t1)) >= 36 and to_integer(unsigned(o_t1)) <= 44
            report "FALLO: theta1 de agarre fuera de ~40" severity error;
        assert to_integer(unsigned(o_t2)) >= 0 and to_integer(unsigned(o_t2)) <= 5
            report "FALLO: theta2 de agarre fuera de ~0" severity error;
        assert to_integer(unsigned(o_t3)) >= 54 and to_integer(unsigned(o_t3)) <= 62
            report "FALLO: theta3 de agarre fuera de ~58" severity error;
        assert o_grip = '0'
            report "FALLO: la garra no se cerro (0=cerrada)" severity error;

        -- Espera fin de secuencia (levanta el hombro)
        wait until done_all = '1' for 50 us;
        assert done_all = '1' report "FALLO: la secuencia no termino" severity error;
        assert reachable = '1' report "FALLO: el objetivo se marco fuera de alcance" severity error;
        assert o_phi = std_logic_vector(to_unsigned(180, 8))
            report "FALLO: recogida phi != 180" severity error;
        assert o_t1 = std_logic_vector(to_unsigned(135, 8))
            report "FALLO: no fue a la pose de recogida (theta1=135)" severity error;
        assert o_grip = '0'
            report "FALLO: solto el cubo en la recogida (0=cerrada)" severity error;
        report "Recogida: phi=" & img(o_phi) & " theta1=" & img(o_t1) & " grip=" &
               std_logic'image(o_grip) & " done=" & std_logic'image(done_all) severity note;

        report "OK: busca, calcula IK, mueve a la coordenada, cierra la garra y levanta" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
