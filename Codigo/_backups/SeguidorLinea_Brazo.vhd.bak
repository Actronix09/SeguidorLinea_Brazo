-- ============================================================================
-- Robot - Módulo Principal del Sistema
-- FPGA: Cyclone IV EP4CE6E22C8 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- Integrantes: [Nombres]
-- ============================================================================
-- Descripción: Módulo principal que integra los subsistemas del robot:
--   - Brazo robótico de 4 ejes (servomotores)
--   - Seguidor de línea con sensores QRD1114
--   - Sensor de distancia VL6180X (LIDAR)
--   - Control de motores DC con puente H L293
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY: SeguidorLinea_Brazo
-- ============================================================================
entity SeguidorLinea_Brazo is
    Port (
        -- Señales de control
        clk             : in  std_logic;
        reset           : in  std_logic;
        start_scan      : in  std_logic;
        start_nav       : in  std_logic;
        
        -- Servomotores (4 ejes)
        servo_phi       : out std_logic;
        servo_theta1    : out std_logic;
        servo_theta2    : out std_logic;
        servo_theta3    : out std_logic;
        servo_gripper   : out std_logic;
        
        -- Sensores QRD1114
        sensor_izq      : in  std_logic;
        sensor_der      : in  std_logic;
        sensor_cent     : in  std_logic;
        sensor_del      : in  std_logic;
        sensor_tras     : in  std_logic;
        
        -- Motores DC (L293)
        motor1_in1      : out std_logic;
        motor1_in2      : out std_logic;
        motor1_en       : out std_logic;
        motor2_in1      : out std_logic;
        motor2_in2      : out std_logic;
        motor2_en       : out std_logic;
        
        -- LIDAR (I2C)
        i2c_scl         : out std_logic;
        i2c_sda         : inout std_logic;
        i2c_gpio        : in  std_logic;
        
        -- Debug
        led_estado      : out std_logic;
        led_error       : out std_logic;
        debug_out       : out std_logic_vector(15 downto 0);
        
        -- Configuración
        sw_mode         : in  std_logic_vector(2 downto 0);
        sw_vel          : in  std_logic_vector(1 downto 0);
        sw_test         : in  std_logic
    );
end SeguidorLinea_Brazo;

-- ============================================================================
-- ARCHITECTURE: Behavioral
-- ============================================================================
architecture Behavioral of Robot is

    -- Componente: polarPWM (control de servos)
    component polarPWM
        Port (
            clk         : in  std_logic;
            rst         : in  std_logic;
            phi_in      : in  std_logic_vector(7 downto 0);
            theta_in    : in  std_logic_vector(7 downto 0);
            radio_in    : in  std_logic_vector(7 downto 0);
            gripper_in  : in  std_logic_vector(7 downto 0);
            pwm_phi     : out std_logic;
            pwm_theta1  : out std_logic;
            pwm_theta2  : out std_logic;
            pwm_theta3  : out std_logic
        );
    end component;
    
    -- Componente: LIDAR (sensor VL6180X)
    component LIDAR
        Port (
            clk           : in  std_logic;
            rst           : in  std_logic;
            start_scan    : in  std_logic;
            i2c_scl       : out std_logic;
            i2c_sda       : inout std_logic;
            i2c_gpio      : in  std_logic;
            best_phi      : out std_logic_vector(7 downto 0);
            best_theta    : out std_logic_vector(7 downto 0);
            best_distance : out std_logic_vector(7 downto 0);
            scan_complete : out std_logic;
            current_state : out std_logic_vector(3 downto 0);
            debug_data    : out std_logic_vector(15 downto 0)
        );
    end component;
    
    -- Componente: MaquinaEstados (seguidor de línea)
    component MaquinaEstados
        Port (
            clk                : in  std_logic;
            rst                : in  std_logic;
            sensor_izq         : in  std_logic;
            sensor_der         : in  std_logic;
            sensor_centro      : in  std_logic;
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

    -- Señales internas: brazo robótico
    signal phi_in         : std_logic_vector(7 downto 0) := (others => '0');
    signal theta_in       : std_logic_vector(7 downto 0) := (others => '0');
    signal radio_in       : std_logic_vector(7 downto 0) := (others => '0');
    signal gripper_in     : std_logic_vector(7 downto 0) := (others => '0');
    
    -- Señales internas: LIDAR
    signal lidar_phi      : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_theta    : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_distance : std_logic_vector(7 downto 0) := (others => '0');
    signal lidar_complete : std_logic := '0';
    signal lidar_debug    : std_logic_vector(15 downto 0) := (others => '0');
    
    -- Señales internas: máquina de estados
    signal error_me       : std_logic := '0';
    signal brazo_abrir    : std_logic := '0';
    signal brazo_cerrar   : std_logic := '0';
    signal brazo_home_sig : std_logic := '0';
    
    -- Señales internas: motores
    signal motor1_in1_sig : std_logic := '0';
    signal motor1_in2_sig : std_logic := '0';
    signal motor2_in1_sig : std_logic := '0';
    signal motor2_in2_sig : std_logic := '0';
    signal motor1_pwm_sig : std_logic := '0';
    signal motor2_pwm_sig : std_logic := '0';
    
    -- Temporizador 1Hz
    signal reset_int : std_logic := '0';
    signal clk_1s    : std_logic := '0';
    signal cnt_1s    : integer range 0 to 49999999 := 0;

begin

    -- Reset activo bajo
    reset_int <= not reset;

    -- Generador de señal 1Hz (debug)
    gen_1hz : process(clk, reset)
    begin
        if reset = '0' then
            cnt_1s <= 0;
            clk_1s <= '0';
        elsif rising_edge(clk) then
            if cnt_1s = 49999999 then
                cnt_1s <= 0;
                clk_1s <= not clk_1s;
            else
                cnt_1s <= cnt_1s + 1;
            end if;
        end if;
    end process;

    -- polarPWM: control de servomotores
    u_polarPWM : polarPWM
        Port Map (
            clk         => clk,
            rst         => reset_int,
            phi_in      => phi_in,
            theta_in    => theta_in,
            radio_in    => radio_in,
            gripper_in  => gripper_in,
            pwm_phi     => servo_phi,
            pwm_theta1  => servo_theta1,
            pwm_theta2  => servo_theta2,
            pwm_theta3  => servo_theta3
        );

    -- LIDAR: sensor de distancia VL6180X
    u_LIDAR : LIDAR
        Port map (
            clk           => clk,
            rst           => reset_int,
            start_scan    => start_scan,
            i2c_scl       => i2c_scl,
            i2c_sda       => i2c_sda,
            i2c_gpio      => i2c_gpio,
            best_phi      => lidar_phi,
            best_theta    => lidar_theta,
            best_distance => lidar_distance,
            scan_complete => lidar_complete,
            current_state => open,
            debug_data    => lidar_debug
        );

    -- MaquinaEstados: control del seguidor de línea
    u_MaquinaEstados : MaquinaEstados
        Port map (
            clk                => clk,
            rst                => reset_int,
            sensor_izq         => sensor_izq,
            sensor_der         => sensor_der,
            sensor_centro      => sensor_cent,
            motor1_in1         => motor1_in1_sig,
            motor1_in2         => motor1_in2_sig,
            motor2_in1         => motor2_in1_sig,
            motor2_in2         => motor2_in2_sig,
            motor1_pwm         => motor1_pwm_sig,
            motor2_pwm         => motor2_pwm_sig,
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

    -- Asignación de salidas de motores
    motor1_in1 <= motor1_in1_sig;
    motor1_in2 <= motor1_in2_sig;
    motor2_in1 <= motor2_in1_sig;
    motor2_in2 <= motor2_in2_sig;
    motor1_en  <= motor1_pwm_sig;
    motor2_en  <= motor2_pwm_sig;

    -- Control de posición del brazo
    control_brazo : process(clk, reset)
    begin
        if reset = '0' then
            phi_in <= (others => '0');
            theta_in <= (others => '0');
            radio_in <= (others => '0');
            gripper_in <= (others => '0');
            servo_gripper <= '0';
        elsif rising_edge(clk) then
            if brazo_home_sig = '1' then
                phi_in <= x"00";
                theta_in <= x"00";
                radio_in <= x"00";
                gripper_in <= x"00";
                servo_gripper <= '0';
            elsif brazo_cerrar = '1' then
                gripper_in <= x"FF";
                servo_gripper <= '1';
            elsif brazo_abrir = '1' then
                gripper_in <= x"00";
                servo_gripper <= '0';
            end if;
        end if;
    end process;

    -- Salidas de debug
    led_estado <= clk_1s;
    led_error  <= error_me;
    debug_out  <= lidar_debug;

end Behavioral;