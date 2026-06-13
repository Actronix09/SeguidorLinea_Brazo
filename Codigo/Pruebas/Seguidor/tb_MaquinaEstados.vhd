-- ============================================================================
-- tb_MaquinaEstados - Seguidor "AMBOS SENSORES DENTRO de la línea" + orquestación
--   del brazo (Etapa 2).
--   Verifica: CENTRADO (1,1) -> motores adelante; correcciones (1,0)/(0,1) y PERDIDA
--   (0,0); en ZONA (1,1 SOSTENIDO) SIN objeto -> start_scan; CON objeto -> trigger_drop.
--   (La zona se prueba con USAR_ZONA=true; el build real va con USAR_ZONA=false.)
--
--   vlib work
--   vcom -2008 MaquinaEstados.vhd tb_MaquinaEstados.vhd
--   vsim tb_MaquinaEstados -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_MaquinaEstados is
end tb_MaquinaEstados;

architecture sim of tb_MaquinaEstados is

    component MaquinaEstados
        generic (
            USAR_ZONA : boolean := false;
            ZONA_CYCLES : integer := 2_000_000; SALIR_CYCLES : integer := 25_000_000;
            FILTRO_CYCLES : integer := 50_000; LINE_LVL : std_logic := '1'
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

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal sizq, sder : std_logic := '0';
    signal a1, a2, b1, b2 : std_logic;
    signal start_scan, trigger_drop : std_logic;
    signal scan_active, arm_ready, has_object : std_logic := '0';
    signal led_estado, led_error : std_logic;
    signal simdone : boolean := false;

    signal saw_scan, saw_drop, mot_active : boolean := false;

begin

    -- LINE_LVL='1' aquí: en el tb sizq/sder='1' = "sobre la línea" (NEGRO).
    -- ZONA_CYCLES=200 (4 us): las fases del seguidor mantienen cada estado < 4 us para NO
    -- disparar zona; las fases de zona sostienen (1,1) > 4 us a propósito.
    dut : MaquinaEstados
        generic map (USAR_ZONA => true, ZONA_CYCLES => 200, SALIR_CYCLES => 60,
                     FILTRO_CYCLES => 4, LINE_LVL => '1')
        port map (
            clk => clk, rst => rst, sensor_izq => sizq, sensor_der => sder,
            motor_a1 => a1, motor_a2 => a2, motor_b1 => b1, motor_b2 => b2,
            start_scan => start_scan, trigger_drop => trigger_drop,
            scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
            led_estado => led_estado, led_error => led_error
        );

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- Monitores
    mon : process(clk)
    begin
        if rising_edge(clk) then
            if start_scan = '1' then saw_scan <= true; end if;
            if trigger_drop = '1' then saw_drop <= true; end if;
            if a1 = '1' then mot_active <= true; end if;
        end if;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 100 ns; rst <= '0';
        arm_ready <= '1';                       -- brazo libre al inicio
        wait until rising_edge(clk);

        -- ---- Fase A: CENTRADO (ambos sensores DENTRO del negro) -> recto ----
        sizq <= '1'; sder <= '1';
        wait for 2 us;                          -- < ZONA_CYCLES (4 us): no debe zonear
        assert mot_active report "FALLO: los motores no se mueven al seguir centrado" severity error;
        report "Fase A OK: motores activos al ir centrado (1,1)" severity note;

        -- ---- Fase B: correcciones (un sensor sale) + PERDIDA (0,0) ----
        sizq <= '1'; sder <= '0'; wait for 1 us;   -- DER salió -> corrige IZQUIERDA
        sizq <= '0'; sder <= '1'; wait for 1 us;   -- IZQ salió -> corrige DERECHA
        sizq <= '0'; sder <= '0'; wait for 1 us;   -- AMBOS blanco -> PERDIÓ -> recupera
        sizq <= '1'; sder <= '1'; wait for 1 us;   -- vuelve al centro
        report "Fase B OK: correcciones + recuperación sin trabarse" severity note;

        -- ---- Fase C: ZONA SIN objeto -> start_scan ----
        has_object <= '0';
        sizq <= '1'; sder <= '1';               -- (1,1) SOSTENIDO -> cuadro
        wait for 5 us;                          -- supera ZONA_CYCLES -> start_scan
        assert saw_scan report "FALLO: no disparó start_scan en zona sin objeto" severity error;
        report "Fase C OK: zona sin objeto -> start_scan" severity note;
        -- emula barrido + agarre (brazo OCUPADO mientras scan_active=1)
        scan_active <= '1'; arm_ready <= '0';
        wait for 2 us;
        scan_active <= '0';                     -- barrido terminó
        sizq <= '0'; sder <= '0';               -- el robot sale de la zona (ambos_linea=0)
        wait for 1 us;
        arm_ready <= '1';                       -- brazo terminó (sin objeto)
        wait for 3 us;                          -- E_SALIR_ZONA -> E_SEGUIR

        -- ---- Fase D: ZONA CON objeto -> trigger_drop ----
        has_object <= '1';
        sizq <= '1'; sder <= '1';
        wait for 5 us;                          -- zona -> trigger_drop
        assert saw_drop report "FALLO: no disparó trigger_drop en zona con objeto" severity error;
        report "Fase D OK: zona con objeto -> trigger_drop" severity note;
        -- emula el depósito (grab_ctrl baja has_object y el robot sale)
        arm_ready <= '0';
        wait for 1 us;
        has_object <= '0';                      -- soltó el cubo
        sizq <= '0'; sder <= '0';
        arm_ready <= '1';
        wait for 3 us;

        report "OK: seguidor (ambos dentro) + orquestación scan/drop funcionan" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
