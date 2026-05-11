-- ============================================================================
-- polarPWM - Conversión de Coordenadas Polares a PWM
-- FPGA: Cyclone IV EP4CE6E22C8 | Brazo Robótico 4 Ejes
-- ============================================================================
-- Descripción: Convierte coordenadas polares (phi, theta, radio) a señales
-- PWM para 4 servomotores. Cada servo tiene 180° de libertad.
-- Especificaciones: L1=100mm, L2=100mm, L3=77.6mm, Pinza=25.269mm
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY: polarPWM
-- ============================================================================
entity polarPWM is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        phi_in     : in  std_logic_vector(7 downto 0);
        theta_in   : in  std_logic_vector(7 downto 0);
        radio_in   : in  std_logic_vector(7 downto 0);
        gripper_in : in  std_logic_vector(7 downto 0);
        pwm_phi    : out std_logic;
        pwm_theta1 : out std_logic;
        pwm_theta2 : out std_logic;
        pwm_theta3 : out std_logic
    );
end polarPWM;

-- ============================================================================
-- ARCHITECTURE: Behavioral
-- ============================================================================
architecture Behavioral of polarPWM is

    -- Constantes: periodo PWM (20ms = 50Hz estándar para servos)
    constant PWM_PERIOD : integer := 1000000;  -- 50MHz * 20ms
    constant PWM_MIN    : integer := 25000;    -- 0.5ms (0°)
    constant PWM_MAX    : integer := 100000;   -- 2.0ms (180°)
    
    -- Tabla de conversión: ángulo (0-180) -> duty cycle PWM
    type angle_to_pwm is array (0 to 180) of integer;
    constant ANGLE_PWM : angle_to_pwm := (
        25000, 2751, 3002, 3253, 3505, 3756, 4008, 4259, 4511, 4762,
        5013, 5265, 5516, 5767, 6019, 6270, 6522, 6773, 7024, 7276,
        7527, 7778, 8030, 8281, 8532, 8784, 9035, 9286, 9537, 9789,
        10040, 10291, 10543, 10794, 11045, 11296, 11548, 11799, 12050, 12301,
        12553, 12804, 13055, 13306, 13558, 13809, 14060, 14311, 14562, 14814,
        15065, 15316, 15567, 15819, 16070, 16321, 16572, 16824, 17075, 17326,
        17577, 17829, 18080, 18331, 18582, 18834, 19085, 19336, 19587, 19838,
        20090, 20341, 20592, 20843, 21094, 21346, 21597, 21848, 22099, 22350,
        22602, 22853, 23104, 23355, 23606, 23857, 24109, 24360, 24611, 24862,
        25113, 25365, 25616, 25867, 26118, 26369, 26620, 26872, 27123, 27374,
        27625, 27877, 28128, 28379, 28630, 28881, 29133, 29384, 29635, 29886,
        30137, 30389, 30640, 30891, 31142, 31393, 31645, 31896, 32147, 32398,
        32649, 32901, 33152, 33403, 33654, 33905, 34157, 34408, 34659, 34910,
        35161, 35413, 35664, 35915, 36166, 36417, 36669, 36920, 37171, 37422,
        37673, 37925, 38176, 38427, 38678, 38929, 39181, 39432, 39683, 39934,
        40185, 40437, 40688, 40939, 41190, 41441, 41693, 41944, 42195, 42446,
        42697, 42949, 43200, 43451, 43702, 43953, 44205, 44456, 44707, 44958,
        45209, 45461, 45712, 45963, 46214, 46465, 46717, 46968, 47219, 47470,
        47722
    );

    -- Ángulos calculados
    signal angulo_phi   : integer range 0 to 180 := 90;
    signal angulo_t1    : integer range 0 to 180 := 90;
    signal angulo_t2    : integer range 0 to 180 := 90;
    signal angulo_t3    : integer range 0 to 180 := 90;
    
    -- Contador PWM
    signal cuenta_pwm   : integer range 0 to PWM_PERIOD-1 := 0;
    signal pwm_phi_sig  : std_logic := '0';
    signal pwm_t1_sig   : std_logic := '0';
    signal pwm_t2_sig   : std_logic := '0';
    signal pwm_t3_sig   : std_logic := '0';

begin

    -- Conversión: coordenadas polares -> ángulos
    calculo : process(phi_in, theta_in)
    begin
        angulo_phi   <= to_integer(unsigned(phi_in));
        angulo_t1    <= to_integer(unsigned(theta_in));
        angulo_t2    <= 180 - to_integer(unsigned(theta_in));
        angulo_t3    <= to_integer(unsigned(theta_in));
    end process;

    -- Generación de PWM para 4 servos
    gen_pwm : process(clk, rst)
    begin
        if rst = '0' then
            cuenta_pwm <= 0;
            pwm_phi_sig <= '0';
            pwm_t1_sig <= '0';
            pwm_t2_sig <= '0';
            pwm_t3_sig <= '0';
        elsif rising_edge(clk) then
            -- Contador de periodo (20ms)
            if cuenta_pwm = PWM_PERIOD-1 then
                cuenta_pwm <= 0;
            else
                cuenta_pwm <= cuenta_pwm + 1;
            end if;
            
            -- Generación de duty cycle para cada servo
            pwm_phi_sig <= '1' when cuenta_pwm < ANGLE_PWM(angulo_phi) else '0';
            pwm_t1_sig  <= '1' when cuenta_pwm < ANGLE_PWM(angulo_t1) else '0';
            pwm_t2_sig  <= '1' when cuenta_pwm < ANGLE_PWM(angulo_t2) else '0';
            pwm_t3_sig  <= '1' when cuenta_pwm < ANGLE_PWM(angulo_t3) else '0';
        end if;
    end process;

    -- Salidas PWM
    pwm_phi    <= pwm_phi_sig;
    pwm_theta1 <= pwm_t1_sig;
    pwm_theta2 <= pwm_t2_sig;
    pwm_theta3 <= pwm_t3_sig;

end Behavioral;