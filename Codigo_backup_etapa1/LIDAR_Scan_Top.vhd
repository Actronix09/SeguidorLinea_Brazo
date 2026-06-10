-- ============================================================================
-- LIDAR_Scan_Top - Top de PRUEBA del escáner LIDAR (ETAPA 1, con servos reales)
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- A diferencia de LIDAR_Top (que usaba los pines de servo como bus de debug),
-- este top mueve los SERVOS REALES para probar el barrido completo:
--
--   LIDAR (escáner) --cmd_*--> polarPWM --pwm_*--> 5 pines de servo
--   LIDAR <--I2C--> VL53L0X físico (pines 142/136)
--
-- DISPARO: el barrido se lanza AUTOMÁTICAMENTE ~1.5 s después de soltar el
-- reset (no requiere un botón extra). Para repetir el barrido: pulsar reset.
--
-- LEDs de la FPGA (activo-bajo / pulldown: '0' enciende):
--   led_1(PIN_3): scan_active  (encendido mientras barre)
--   led_2(PIN_7): scan_done    (encendido cuando hay coordenada lista)
--   led_3(PIN_9): heartbeat de medición (conmuta con cada lectura del sensor)
--
-- RESULTADO (r, theta, phi) -> se observa por SignalTap (proyecto: signals.stp).
--   Nodos sugeridos: u_lidar|out_r_r, u_lidar|out_theta_r, u_lidar|out_phi_r,
--   u_lidar|min_d, u_lidar|min_phi, u_lidar|min_t3, u_lidar|st.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity LIDAR_Scan_Top is
    port (
        clk          : in    std_logic;                 -- PIN_17, 50 MHz
        reset        : in    std_logic;                 -- PIN_144, activo bajo
        i2c_scl      : out   std_logic;                 -- PIN_142
        i2c_sda      : inout std_logic;                 -- PIN_136
        -- 5 pines de servo (PWM real)
        servo_phi    : out   std_logic;                 -- PIN_118
        servo_theta1 : out   std_logic;                 -- PIN_122
        servo_theta2 : out   std_logic;                 -- PIN_126
        servo_theta3 : out   std_logic;                 -- PIN_132
        servo_gripper: out   std_logic;                 -- PIN_134
        -- 3 LEDs de la FPGA, activo-bajo
        led_1        : out   std_logic;                 -- scan_active
        led_2        : out   std_logic;                 -- scan_done
        led_3        : out   std_logic                  -- heartbeat de medición
    );
end LIDAR_Scan_Top;

architecture Behavioral of LIDAR_Scan_Top is

    component LIDAR
        generic (
            CLK_FREQ_HZ     : integer := 50_000_000;
            I2C_FREQ_HZ     : integer := 100_000;
            PWRUP_CYCLES    : integer := 500_000;
            SETTLE_CYCLES   : integer := 50_000_000;   -- 1 s (asentamiento del tambaleo)
            N_AVG           : integer := 8;
            COARSE_PHI_STEP : integer := 10;
            COARSE_T1_STEP  : integer := 9;
            FINE_PHI_STEP   : integer := 3;
            FINE_T1_STEP    : integer := 3
        );
        port (
            clk         : in    std_logic;
            rst         : in    std_logic;
            start_scan  : in    std_logic;
            i2c_scl     : out   std_logic;
            i2c_sda     : inout std_logic;
            scan_active : out   std_logic;
            cmd_phi     : out   std_logic_vector(7 downto 0);
            cmd_theta1  : out   std_logic_vector(7 downto 0);
            cmd_theta2  : out   std_logic_vector(7 downto 0);
            cmd_theta3  : out   std_logic_vector(7 downto 0);
            cmd_grip    : out   std_logic;
            out_r       : out   std_logic_vector(15 downto 0);
            out_theta   : out   std_logic_vector(8 downto 0);
            out_phi     : out   std_logic_vector(7 downto 0);
            scan_done   : out   std_logic;
            dbg_meas_tick : out std_logic
        );
    end component;

    component polarPWM
        port (
            clk         : in  std_logic;
            rst         : in  std_logic;
            phi_in      : in  std_logic_vector(7 downto 0);
            theta1_in   : in  std_logic_vector(7 downto 0);
            theta2_in   : in  std_logic_vector(7 downto 0);
            theta3_in   : in  std_logic_vector(7 downto 0);
            grip_cmd    : in  std_logic;
            pwm_phi     : out std_logic;
            pwm_theta1  : out std_logic;
            pwm_theta2  : out std_logic;
            pwm_theta3  : out std_logic;
            pwm_gripper : out std_logic
        );
    end component;

    constant START_DELAY : integer := 150_000_000;       -- ~3 s @ 50 MHz (deja llegar a reposo antes de barrer)

    signal reset_int  : std_logic;
    signal start_scan : std_logic := '0';
    signal armed      : std_logic := '1';
    signal dly        : integer range 0 to START_DELAY := 0;

    signal scan_active : std_logic;
    signal scan_done   : std_logic;
    signal meas_tick   : std_logic;

    signal cmd_phi    : std_logic_vector(7 downto 0);
    signal cmd_theta1 : std_logic_vector(7 downto 0);
    signal cmd_theta2 : std_logic_vector(7 downto 0);
    signal cmd_theta3 : std_logic_vector(7 downto 0);
    signal cmd_grip   : std_logic;

    signal out_r     : std_logic_vector(15 downto 0);
    signal out_theta : std_logic_vector(8 downto 0);
    signal out_phi   : std_logic_vector(7 downto 0);

    -- theta1 compensado: el servo theta1 está montado INVERTIDO (igual que theta3),
    -- se corrige aquí (180 - theta1) justo antes de polarPWM.
    signal theta1_pwm : std_logic_vector(7 downto 0);

begin

    reset_int <= not reset;   -- reset externo activo bajo -> activo alto interno

    theta1_pwm <= std_logic_vector(to_unsigned(180 - to_integer(unsigned(cmd_theta1)), 8));

    -- ----------------------------------------------------------------
    -- Disparo automático: 1 pulso de start_scan ~1.5 s tras soltar reset
    -- ----------------------------------------------------------------
    arm_proc : process(clk, reset_int)
    begin
        if reset_int = '1' then
            armed      <= '1';
            dly        <= 0;
            start_scan <= '0';
        elsif rising_edge(clk) then
            start_scan <= '0';
            if armed = '1' then
                if dly >= START_DELAY-1 then
                    start_scan <= '1';
                    armed      <= '0';
                else
                    dly <= dly + 1;
                end if;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Escáner LIDAR (genéricos por defecto = tiempos reales de 50 MHz)
    -- ----------------------------------------------------------------
    u_lidar : LIDAR
        port map (
            clk           => clk,
            rst           => reset_int,
            start_scan    => start_scan,
            i2c_scl       => i2c_scl,
            i2c_sda       => i2c_sda,
            scan_active   => scan_active,
            cmd_phi       => cmd_phi,
            cmd_theta1    => cmd_theta1,
            cmd_theta2    => cmd_theta2,
            cmd_theta3    => cmd_theta3,
            cmd_grip      => cmd_grip,
            out_r         => out_r,
            out_theta     => out_theta,
            out_phi       => out_phi,
            scan_done     => scan_done,
            dbg_meas_tick => meas_tick
        );

    -- ----------------------------------------------------------------
    -- Control de los 5 servos (recibe los comandos del escáner)
    -- ----------------------------------------------------------------
    u_pwm : polarPWM
        port map (
            clk         => clk,
            rst         => reset_int,
            phi_in      => cmd_phi,
            theta1_in   => theta1_pwm,
            theta2_in   => cmd_theta2,
            theta3_in   => cmd_theta3,
            grip_cmd    => cmd_grip,
            pwm_phi     => servo_phi,
            pwm_theta1  => servo_theta1,
            pwm_theta2  => servo_theta2,
            pwm_theta3  => servo_theta3,
            pwm_gripper => servo_gripper
        );

    -- ----------------------------------------------------------------
    -- LEDs de la FPGA (activo-bajo: '0' enciende)
    -- ----------------------------------------------------------------
    led_1 <= not scan_active;     -- barriendo
    led_2 <= not scan_done;       -- coordenada lista
    led_3 <= not meas_tick;       -- heartbeat de medición

end Behavioral;
