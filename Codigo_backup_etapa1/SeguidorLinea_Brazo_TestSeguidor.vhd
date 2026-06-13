-- ============================================================================
-- Testbench: SeguidorLinea_Brazo_TestSeguidor
-- Proposito: Simulacion en VHDL de solo la parte del seguidor de linea,
--            sin brazo robotico, para verificar las nuevas rutinas de 
--            reduccion gradual de velocidad para curvas y correcciones.
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity SeguidorLinea_Brazo_TestSeguidor is
    -- Entidad top-level vacia (testbench)
end SeguidorLinea_Brazo_TestSeguidor;

architecture Behavioral of SeguidorLinea_Brazo_TestSeguidor is

    -- =========================================================================
    -- Senales de entrada para la simulacion
    -- =========================================================================
    signal clk          : std_logic := '0';
    signal rst          : std_logic := '0';   -- Active low
    signal sensor_izq   : std_logic := '0';   -- 1 = sobre negro
    signal sensor_der   : std_logic := '0';   -- 1 = sobre negro
    
    -- =========================================================================
    -- Senales de salida del modulo MaquinaEstados (instanciado aqui)
    -- =========================================================================
    signal motor1_in1   : std_logic;
    signal motor1_in2   : std_logic;
    signal motor2_in1   : std_logic;
    signal motor2_in2   : std_logic;
    signal motor1_pwm   : std_logic;
    signal motor2_pwm   : std_logic;
    signal estado_actual: std_logic_vector(3 downto 0);
    signal error_flag   : std_logic;

    -- =========================================================================
    -- Constantes del testbench
    -- =========================================================================
    constant CLK_PERIOD : time := 20 ns;  -- 50 MHz

    -- =========================================================================
    -- Senales simuladas para LIDAR y brazo (no utilizadas en este testbench)
    -- =========================================================================
    signal lidar_complete  : std_logic := '0';
    signal lidar_phi       : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_theta     : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_dist      : std_logic_vector(7 downto 0) := (others => '0');

    -- =========================================================================
    -- Señal de estado para control de simulación
    -- =========================================================================
    signal end_sim : boolean := false;

begin

    -- =========================================================================
    -- Instancia del modulo MaquinaEstados
    -- =========================================================================
    uut: entity work.MaquinaEstados
        port map (
            clk                => clk,
            rst                => rst,
            sensor_izq         => sensor_izq,
            sensor_der         => sensor_der,
            motor1_in1         => motor1_in1,
            motor1_in2         => motor1_in2,
            motor2_in1         => motor2_in1,
            motor2_in2         => motor2_in2,
            motor1_pwm         => motor1_pwm,
            motor2_pwm         => motor2_pwm,
            lidar_start        => open,
            lidar_complete     => lidar_complete,
            lidar_phi          => lidar_phi,
            lidar_theta        => lidar_theta,
            lidar_dist         => lidar_dist,
            brazo_garra_abrir  => open,
            brazo_garra_cerrar => open,
            brazo_mover        => open,
            brazo_home         => open,
            estado_actual      => estado_actual,
            error_flag         => error_flag
        );

    -- =========================================================================
    -- Generador de reloj
    -- =========================================================================
    clk_gen : process
    begin
        while not end_sim loop
            clk <= '0';
            wait for CLK_PERIOD / 2;
            clk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process;

    -- =========================================================================
    -- Proceso de estimulos: robot sobre linea recta, luego curva a la izquierda
    -- =========================================================================
    stim_proc : process
    begin
        -- -------------------------------------------------------------------
        -- 1) Reset
        -- -------------------------------------------------------------------
        rst <= '0';
        wait for 100 ns;
        rst <= '1';
        wait for 100 ns;

        -- -------------------------------------------------------------------
        -- 2) Linea recta: ambos sensores en blanco (0,0)
        --    Esperamos 5 ms
        -- -------------------------------------------------------------------
        sensor_izq <= '0';
        sensor_der <= '0';
        wait for 5 ms;

        -- -------------------------------------------------------------------
        -- 3) Curva a la izquierda: sensor izquierdo en negro (1,0)
        --    Esperamos 3 ms para observar la reduccion gradual
        -- -------------------------------------------------------------------
        sensor_izq <= '1';
        sensor_der <= '0';
        wait for 3 ms;

        -- -------------------------------------------------------------------
        -- 4) Volvemos a recto
        -- -------------------------------------------------------------------
        sensor_izq <= '0';
        sensor_der <= '0';
        wait for 2 ms;

        -- -------------------------------------------------------------------
        -- 5) Curva a la derecha: sensor derecho en negro (0,1)
        -- -------------------------------------------------------------------
        sensor_izq <= '0';
        sensor_der <= '1';
        wait for 3 ms;

        -- -------------------------------------------------------------------
        -- 6) Recto de nuevo
        -- -------------------------------------------------------------------
        sensor_izq <= '0';
        sensor_der <= '0';
        wait for 5 ms;

        -- -------------------------------------------------------------------
        -- 7) Perdida de linea: ambos en blanco prolongado (0,0)
        --    Vemos como el robot busca segun el ultimo giro recuerde
        -- -------------------------------------------------------------------
        wait for 10 ms;

        -- -------------------------------------------------------------------
        -- 8) Finalizar simulacion
        -- -------------------------------------------------------------------
        end_sim <= true;
        wait;
    end process;

end Behavioral;