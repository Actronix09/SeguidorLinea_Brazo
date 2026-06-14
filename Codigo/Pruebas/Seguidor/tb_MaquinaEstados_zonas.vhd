-- ============================================================================
-- tb_MaquinaEstados_zonas - Verifica los 3 MARCADORES de zona (MODO_ZONA 1/2/3) en el
--   esquema de LÍNEA FINA: fondo BLANCO, marcadores NEGROS. Un DUT por modo.
--   Convención del TB: LINE_LVL='1' => sensor '1' = NEGRO (sobre línea), '0' = BLANCO.
--   *_CYCLES reducidos para simular rápido (20 ns/ciclo).
--
--   Comprueba:
--     Modo 2 (RAYAS)  : blanco -> 2 franjas NEGRAS -> blanco  => start_scan.
--     Modo 1 (AJEDREZ): blanco -> 4 alternancias IZQ/DER sobre línea -> blanco => start_scan.
--     Modo 3 (CUADRO) : negro DEMASIADO largo => NO dispara (perdido); luego
--                       negro acotado -> blanco => start_scan.
--
--   vlib work
--   vcom -2008 MaquinaEstados.vhd tb_MaquinaEstados_zonas.vhd
--   vsim tb_MaquinaEstados_zonas -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_MaquinaEstados_zonas is
end tb_MaquinaEstados_zonas;

architecture sim of tb_MaquinaEstados_zonas is

    component MaquinaEstados
        generic (
            USAR_ZONA : boolean; MODO_ZONA : integer;
            N_RAYAS : integer; N_ALTERN : integer;
            ZONA_CYCLES : integer;
            LEAD_CYCLES : integer; W_MIN_CYCLES : integer;
            W_MAX_CYCLES : integer; T_GAP_CYCLES : integer;
            FILTRO_CYCLES : integer; LINE_LVL : std_logic
        );
        port (
            clk, rst : in std_logic;
            sensor_izq, sensor_der : in std_logic;
            motor_a1, motor_a2, motor_b1, motor_b2 : out std_logic;
            start_scan, trigger_drop : out std_logic;
            scan_active, arm_ready, has_object : in std_logic;
            led_estado, led_error : out std_logic
        );
    end component;

    -- Parámetros comunes (rápidos): LEAD=20c(400ns), W_MIN=5c(100ns), W_MAX=60c(1200ns),
    -- T_GAP=400c(8us). El filtro (FILTRO=4c) tarda ~80ns en propagar cada flanco.
    constant C_LEAD  : integer := 20;
    constant C_WMIN  : integer := 5;
    constant C_WMAX  : integer := 60;
    constant C_TGAP  : integer := 400;
    constant C_FILT  : integer := 4;
    constant C_ZONA  : integer := 200;   -- (modo 0; aquí sin uso)

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal scan_active, arm_ready, has_object : std_logic := '0';
    signal simdone : boolean := false;

    -- Sensores por DUT ('1'=NEGRO, '0'=BLANCO). Inician en BLANCO (fondo normal).
    signal si1, sd1, si2, sd2, si3, sd3 : std_logic := '0';
    -- Disparos por DUT
    signal scan1, drop1, scan2, drop2, scan3, drop3 : std_logic;
    signal saw1, saw2, saw3 : boolean := false;
    signal saw2b : boolean := false;   -- 2º disparo modo 2 (prueba reanudar directo)

begin

    -- DUT1: AJEDREZ (modo 1), 4 alternancias
    dut1 : MaquinaEstados
        generic map (USAR_ZONA => true, MODO_ZONA => 1, N_RAYAS => 2, N_ALTERN => 4,
                     ZONA_CYCLES => C_ZONA, LEAD_CYCLES => C_LEAD,
                     W_MIN_CYCLES => C_WMIN, W_MAX_CYCLES => C_WMAX, T_GAP_CYCLES => C_TGAP,
                     FILTRO_CYCLES => C_FILT, LINE_LVL => '1')
        port map (clk => clk, rst => rst, sensor_izq => si1, sensor_der => sd1,
                  motor_a1 => open, motor_a2 => open, motor_b1 => open, motor_b2 => open,
                  start_scan => scan1, trigger_drop => drop1,
                  scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
                  led_estado => open, led_error => open);

    -- DUT2: RAYAS transversales (modo 2), 2 franjas
    dut2 : MaquinaEstados
        generic map (USAR_ZONA => true, MODO_ZONA => 2, N_RAYAS => 2, N_ALTERN => 4,
                     ZONA_CYCLES => C_ZONA, LEAD_CYCLES => C_LEAD,
                     W_MIN_CYCLES => C_WMIN, W_MAX_CYCLES => C_WMAX, T_GAP_CYCLES => C_TGAP,
                     FILTRO_CYCLES => C_FILT, LINE_LVL => '1')
        port map (clk => clk, rst => rst, sensor_izq => si2, sensor_der => sd2,
                  motor_a1 => open, motor_a2 => open, motor_b1 => open, motor_b2 => open,
                  start_scan => scan2, trigger_drop => drop2,
                  scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
                  led_estado => open, led_error => open);

    -- DUT3: CUADRO negro acotado (modo 3)
    dut3 : MaquinaEstados
        generic map (USAR_ZONA => true, MODO_ZONA => 3, N_RAYAS => 2, N_ALTERN => 4,
                     ZONA_CYCLES => C_ZONA, LEAD_CYCLES => C_LEAD,
                     W_MIN_CYCLES => C_WMIN, W_MAX_CYCLES => C_WMAX, T_GAP_CYCLES => C_TGAP,
                     FILTRO_CYCLES => C_FILT, LINE_LVL => '1')
        port map (clk => clk, rst => rst, sensor_izq => si3, sensor_der => sd3,
                  motor_a1 => open, motor_a2 => open, motor_b1 => open, motor_b2 => open,
                  start_scan => scan3, trigger_drop => drop3,
                  scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
                  led_estado => open, led_error => open);

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    mon : process(clk)
    begin
        if rising_edge(clk) then
            if scan1 = '1' then saw1 <= true; end if;
            if scan2 = '1' then saw2 <= true; end if;
            if scan2 = '1' and saw2 then saw2b <= true; end if;  -- 2ª vez
            if scan3 = '1' then saw3 <= true; end if;
        end if;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 200 ns; rst <= '0';
        arm_ready <= '1'; has_object <= '0';   -- brazo libre; sin objeto -> ruta start_scan
        wait until rising_edge(clk);

        -- ===== Modo 2: RAYAS negras (prioritario) ===========================
        si2 <= '0'; sd2 <= '0'; wait for 1 us;       -- blanco de entrada -> ARMA
        si2 <= '1'; sd2 <= '1'; wait for 300 ns;     -- raya 1 (ambos negro)
        si2 <= '0'; sd2 <= '0'; wait for 300 ns;     -- blanco entre rayas
        si2 <= '1'; sd2 <= '1'; wait for 300 ns;     -- raya 2
        si2 <= '0'; sd2 <= '0'; wait for 800 ns;     -- blanco de salida -> dispara
        assert saw2 report "FALLO modo 2 (RAYAS): no disparo start_scan tras 2 franjas" severity error;
        report "Modo 2 OK: 2 rayas -> start_scan" severity note;

        -- completa el ciclo de zona y comprueba que REANUDA la línea directamente (sin escape):
        -- un 2º marcador debe volver a disparar (prueba E_SCAN_INI->E_SCAN_FIN->E_SEGUIR).
        scan_active <= '1'; wait for 200 ns;         -- E_SCAN_INI -> E_SCAN_FIN -> (arm_ready) E_SEGUIR
        scan_active <= '0'; wait for 1 us;           -- reanuda y re-arma con el blanco siguiente
        si2 <= '1'; sd2 <= '1'; wait for 300 ns;     -- 2ª raya 1
        si2 <= '0'; sd2 <= '0'; wait for 300 ns;
        si2 <= '1'; sd2 <= '1'; wait for 300 ns;     -- 2ª raya 2
        si2 <= '0'; sd2 <= '0'; wait for 800 ns;     -- blanco de salida -> dispara otra vez
        assert saw2b report "FALLO modo 2: no reanudo/re-disparo tras la zona (reanudar directo)" severity error;
        report "Modo 2 OK (reanudar directo): 2o marcador vuelve a disparar" severity note;

        -- ===== Modo 1: AJEDREZ (4 alternancias IZQ/DER sobre línea) ==========
        si1 <= '0'; sd1 <= '0'; wait for 1 us;       -- blanco de entrada -> ARMA
        for k in 0 to 4 loop                         -- L,R,L,R,L => 4 cambios de lado
            if (k mod 2) = 0 then
                si1 <= '1'; sd1 <= '0';              -- IZQ sobre línea (1,0)
            else
                si1 <= '0'; sd1 <= '1';              -- DER sobre línea (0,1)
            end if;
            wait for 300 ns;
            si1 <= '0'; sd1 <= '0'; wait for 200 ns; -- blanco entre celdas
        end loop;
        wait for 600 ns;                             -- blanco de salida sostenido -> dispara
        assert saw1 report "FALLO modo 1 (AJEDREZ): no disparo start_scan tras 4 alternancias" severity error;
        report "Modo 1 OK: 4 alternancias -> start_scan" severity note;

        -- ===== Modo 3: CUADRO negro ========================================
        si3 <= '0'; sd3 <= '0'; wait for 1 us;       -- blanco de entrada -> ARMA
        -- (a) negro DEMASIADO largo (> W_MAX) -> NO debe disparar (perdido)
        si3 <= '1'; sd3 <= '1'; wait for 1600 ns;    -- > W_MAX (1200 ns)
        si3 <= '0'; sd3 <= '0'; wait for 800 ns;     -- vuelve a blanco
        assert not saw3 report "FALLO modo 3: negro demasiado largo NO debia disparar" severity error;
        report "Modo 3 OK (a): negro largo no dispara (perdido)" severity note;
        -- (b) cuadro acotado válido -> dispara
        si3 <= '1'; sd3 <= '1'; wait for 600 ns;     -- W_MIN < 600 ns < W_MAX
        si3 <= '0'; sd3 <= '0'; wait for 800 ns;     -- blanco de salida -> dispara
        assert saw3 report "FALLO modo 3 (CUADRO): no disparo start_scan con cuadro valido" severity error;
        report "Modo 3 OK (b): cuadro acotado -> start_scan" severity note;

        report "OK: los 3 marcadores de zona (ajedrez/rayas/cuadro) funcionan" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
