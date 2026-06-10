-- ============================================================================
-- Robot - Módulo Principal del Sistema
-- FPGA: Cyclone IV EP4CE6E22C8 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ============================================================================
-- Hardware conectado:
--   - 5 servomotores (phi, theta1, theta2, theta3, gripper)
--   - 2 sensores QRD1114 (izquierdo, derecho)
--   - 2 motores DC con L293 (PWM directo en IN, sin pin enable)
--   - Sensor VL53L0X por I2C (solo SDA + SCL, sin GPIO)
--   - 3 LEDs: led_estado (parpadeo 1 Hz), led_error (fallo FSM)
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY
-- ============================================================================
entity SeguidorLinea_Brazo is
    Port (
        -- Control
        clk          : in  std_logic;
        reset        : in  std_logic;        -- activo bajo
        start_scan   : in  std_logic;        -- dispara escaneo LIDAR

        -- Servomotores (5 ejes)
        servo_phi    : out std_logic;
        servo_theta1 : out std_logic;
        servo_theta2 : out std_logic;
        servo_theta3 : out std_logic;
        servo_gripper: out std_logic;

        -- Sensores QRD1114 (solo 2)
        sensor_izq   : in  std_logic;
        sensor_der   : in  std_logic;

        -- Motores DC L293 (PWM directo en IN, sin enable)
        motor1_in1   : out std_logic;
        motor1_in2   : out std_logic;
        motor2_in1   : out std_logic;
        motor2_in2   : out std_logic;
        motor1_pwm   : out std_logic;
        motor2_pwm   : out std_logic;

        -- VL53L0X I2C (solo SDA y SCL)
        i2c_scl      : out std_logic;
        i2c_sda      : inout std_logic;

        -- LEDs
        led_estado   : out std_logic;        -- parpadea 1 Hz en operación
        led_error    : out std_logic         -- activo en estado ERROR
    );
end SeguidorLinea_Brazo;

-- ============================================================================
-- ARCHITECTURE
-- ============================================================================
architecture Behavioral of SeguidorLinea_Brazo is

    -- -------------------------------------------------------------------------
    -- Componente: polarPWM
    -- -------------------------------------------------------------------------
    component polarPWM
        Port (
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

    -- -------------------------------------------------------------------------
    -- Componente: LIDAR (sin i2c_gpio)
    -- -------------------------------------------------------------------------
    component LIDAR
        Port (
            clk           : in  std_logic;
            rst           : in  std_logic;
            start_scan    : in  std_logic;
            i2c_scl       : out std_logic;
            i2c_sda       : inout std_logic;
            best_phi      : out std_logic_vector(7 downto 0);
            best_theta    : out std_logic_vector(7 downto 0);
            best_distance : out std_logic_vector(7 downto 0);
            scan_complete : out std_logic;
            current_state : out std_logic_vector(3 downto 0)
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Componente: MaquinaEstados (2 sensores, sin enable de motor)
    -- -------------------------------------------------------------------------
    component MaquinaEstados
        Port (
            clk                : in  std_logic;
            rst                : in  std_logic;
            sensor_izq         : in  std_logic;
            sensor_der         : in  std_logic;
            motor1_in1         : out std_logic;
            motor1_in2         : out std_logic;
            motor2_in1         : out std_logic;
            motor2_in2         : out std_logic;
            motor1_pwm         : out std_logic;
            motor2_pwm         : out std_logic;
            lidar_start        : out std_logic;
            lidar_complete     : in  std_logic;
            lidar_phi          : in  std_logic_vector(7 downto 0);
            lidar_theta        : in  std_logic_vector(7 downto 0);
            lidar_dist         : in  std_logic_vector(7 downto 0);
            brazo_garra_abrir  : out std_logic;
            brazo_garra_cerrar : out std_logic;
            brazo_mover        : out std_logic;
            brazo_home         : out std_logic;
            estado_actual      : out std_logic_vector(3 downto 0);
            error_flag         : out std_logic
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Señales internas: brazo
    -- Posición HOME: phi=0°, theta1=90°, theta2=0°, theta3=0°, grip=cerrado
    -- -------------------------------------------------------------------------
    signal phi_in    : std_logic_vector(7 downto 0) := x"00";
    signal theta1_in : std_logic_vector(7 downto 0) := x"5A";
    signal theta2_in : std_logic_vector(7 downto 0) := x"00";
    signal theta3_in : std_logic_vector(7 downto 0) := x"00";
    signal grip_cmd  : std_logic := '1';

    -- Señales internas: LIDAR
    signal lidar_phi      : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_theta    : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_distance : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_complete : std_logic := '0';

    -- Señales internas: FSM
    signal error_me       : std_logic := '0';
    signal brazo_abrir    : std_logic := '0';
    signal brazo_cerrar   : std_logic := '0';
    signal brazo_home_sig : std_logic := '0';

    -- Reset interno
    signal reset_int : std_logic := '0';

    -- LED 1 Hz
    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 49_999_999 := 0;

begin

    reset_int <= not reset;

    -- =========================================================================
    -- Generador 1 Hz para led_estado
    -- =========================================================================
    gen_1hz : process(clk, reset)
    begin
        if reset = '0' then
            cnt_1s <= 0;
            clk_1s <= '0';
        elsif rising_edge(clk) then
            if cnt_1s = 49_999_999 then
                cnt_1s <= 0;
                clk_1s <= not clk_1s;
            else
                cnt_1s <= cnt_1s + 1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Instancia: polarPWM
    -- =========================================================================
    u_polarPWM : polarPWM
        Port Map (
            clk         => clk,
            rst         => reset_int,
            phi_in      => phi_in,
            theta1_in   => theta1_in,
            theta2_in   => theta2_in,
            theta3_in   => theta3_in,
            grip_cmd    => grip_cmd,
            pwm_phi     => servo_phi,
            pwm_theta1  => servo_theta1,
            pwm_theta2  => servo_theta2,
            pwm_theta3  => servo_theta3,
            pwm_gripper => servo_gripper
        );

    -- =========================================================================
    -- Instancia: LIDAR (sin i2c_gpio)
    -- =========================================================================
    u_LIDAR : LIDAR
        Port Map (
            clk           => clk,
            rst           => reset_int,
            start_scan    => start_scan,
            i2c_scl       => i2c_scl,
            i2c_sda       => i2c_sda,
            best_phi      => lidar_phi,
            best_theta    => lidar_theta,
            best_distance => lidar_distance,
            scan_complete => lidar_complete,
            current_state => open
        );

    -- =========================================================================
    -- Instancia: MaquinaEstados
    -- =========================================================================
    u_MaquinaEstados : MaquinaEstados
        Port Map (
            clk                => clk,
            rst                => reset_int,
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
            lidar_dist         => lidar_distance,
            brazo_garra_abrir  => brazo_abrir,
            brazo_garra_cerrar => brazo_cerrar,
            brazo_mover        => open,
            brazo_home         => brazo_home_sig,
            estado_actual      => open,
            error_flag         => error_me
        );

    -- =========================================================================
    -- Control de posición del brazo
    -- =========================================================================
    control_brazo : process(clk, reset)
    begin
        if reset = '0' then
            phi_in    <= x"00";
            theta1_in <= x"5A";
            theta2_in <= x"00";
            theta3_in <= x"00";
            grip_cmd  <= '1';
        elsif rising_edge(clk) then
            if brazo_home_sig = '1' then
                phi_in    <= x"00";
                theta1_in <= x"5A";
                theta2_in <= x"00";
                theta3_in <= x"00";
                grip_cmd  <= '1';
            elsif brazo_cerrar = '1' then
                grip_cmd  <= '1';
                phi_in    <= lidar_phi;
                theta1_in <= lidar_theta;
            elsif brazo_abrir = '1' then
                grip_cmd  <= '0';
            end if;
        end if;
    end process;

    -- =========================================================================
    -- LEDs
    -- =========================================================================
    led_estado <= clk_1s;
    led_error  <= error_me;

end Behavioral;