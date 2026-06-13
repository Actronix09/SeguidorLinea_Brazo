-- ============================================================================
-- LIDAR_Top - Top de PRUEBA/DEBUG del driver VL53L0X (ETAPA 1)
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Permite depurar la comunicación con el sensor ANTES de integrarlo al top
-- completo. Reutiliza los 5 pines de servo como bus de debug de 5 bits y los
-- 3 LEDs de la FPGA como indicadores.
--
-- Bus de debug (5 pines de servo, LEDs externos ACTIVO-ALTO, LSB->MSB):
--   servo_phi(PIN_118)=bit0, servo_theta1(122)=bit1, servo_theta2(126)=bit2,
--   servo_theta3(132)=bit3, servo_gripper(134)=bit4.
--   - Si hay error (led_2 encendido): muestra el código de error en binario.
--   - Si NO hay error: muestra la distancia en cm (0..31, saturada).
--
-- LEDs de la FPGA (ACTIVO-BAJO / pulldown: '0' enciende):
--   led_1(PIN_3): heartbeat de comunicación (conmuta cada medición).
--   led_2(PIN_7): encendido si hay error.
--   led_3(PIN_9): encendido si la distancia < 5 cm (objeto cerca).
--
-- Códigos de error (ver vl53l0x_pkg):
--   00001 NACK de dirección (sensor ausente / sin pull-ups)
--   00010 MODEL_ID != 0xEE
--   00011 NACK durante init/tuning
--   00100 timeout esperando data-ready
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.vl53l0x_pkg.all;

entity LIDAR_Top is
    port (
        clk          : in    std_logic;                 -- PIN_17, 50 MHz
        reset        : in    std_logic;                 -- PIN_144, activo bajo
        i2c_scl      : out   std_logic;                 -- PIN_142
        i2c_sda      : inout std_logic;                 -- PIN_136
        -- 5 pines de servo reutilizados como bus de debug (activo-alto)
        servo_phi    : out   std_logic;  -- bit0 (LSB)
        servo_theta1 : out   std_logic;  -- bit1
        servo_theta2 : out   std_logic;  -- bit2
        servo_theta3 : out   std_logic;  -- bit3
        servo_gripper: out   std_logic;  -- bit4 (MSB)
        -- 3 LEDs de la FPGA, activo-bajo
        led_1        : out   std_logic;  -- heartbeat de comunicación
        led_2        : out   std_logic;  -- hay error
        led_3        : out   std_logic   -- distancia < 5 cm
    );
end LIDAR_Top;

architecture Behavioral of LIDAR_Top is

    component VL53L0X
        generic (
            CLK_FREQ_HZ  : integer := 50_000_000;
            I2C_FREQ_HZ  : integer := 100_000;
            PWRUP_CYCLES : integer := 500_000
        );
        port (
            clk         : in    std_logic;
            rst         : in    std_logic;
            i2c_scl     : out   std_logic;
            i2c_sda     : inout std_logic;
            distance_mm : out   std_logic_vector(15 downto 0);
            data_valid  : out   std_logic;
            sensor_ok   : out   std_logic;
            err_code    : out   std_logic_vector(4 downto 0);
            meas_tick   : out   std_logic
        );
    end component;

    signal reset_int   : std_logic;
    signal distance_s  : std_logic_vector(15 downto 0);
    signal data_valid_s: std_logic;
    signal err_code_s  : std_logic_vector(4 downto 0);
    signal meas_tick_s : std_logic;

    signal cm_int : integer range 0 to 31;
    signal dbg5   : std_logic_vector(4 downto 0);

begin

    reset_int <= not reset;   -- reset externo activo bajo -> activo alto interno

    u_VL53L0X : VL53L0X
        port map (
            clk         => clk,
            rst         => reset_int,
            i2c_scl     => i2c_scl,
            i2c_sda     => i2c_sda,
            distance_mm => distance_s,
            data_valid  => data_valid_s,
            sensor_ok   => open,
            err_code    => err_code_s,
            meas_tick   => meas_tick_s
        );

    -- Distancia en cm (saturada a 31 = 0b11111)
    cm_proc : process(distance_s)
        variable d : integer;
    begin
        d := to_integer(unsigned(distance_s));
        if d >= 310 then
            cm_int <= 31;
        else
            cm_int <= d / 10;
        end if;
    end process;

    -- Bus de debug de 5 bits: código de error o distancia en cm
    dbg5 <= err_code_s when err_code_s /= ERR_NONE
            else std_logic_vector(to_unsigned(cm_int, 5));

    -- Mapeo a los pines de servo (activo-alto), orden LSB phi -> MSB grip
    servo_phi     <= dbg5(0);
    servo_theta1  <= dbg5(1);
    servo_theta2  <= dbg5(2);
    servo_theta3  <= dbg5(3);
    servo_gripper <= dbg5(4);

    -- LEDs de la FPGA (activo-bajo: '0' enciende)
    led_1 <= not meas_tick_s;                                    -- heartbeat
    led_2 <= '0' when err_code_s /= ERR_NONE else '1';           -- error
    led_3 <= '0' when (data_valid_s = '1' and unsigned(distance_s) < 50)
             else '1';                                           -- < 5 cm

end Behavioral;
