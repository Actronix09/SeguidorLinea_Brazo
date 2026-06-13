-- ============================================================================
-- Robot - MÃ³dulo Principal - Todos los Servos a 90Â°
-- FPGA: Cyclone IV EP4CE6E22C8 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ============================================================================
-- DescripciÃ³n:
--   Mantiene los 5 servos fijos en 90Â° usando UNA sola instancia de
--   polarPWM, que ahora expone el puerto pwm_gripper como 5.Âª salida.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY
-- ============================================================================
entity SeguidorLinea_Brazo_Servos90 is
    Port (
        clk           : in  std_logic;   -- 50 MHz
        reset         : in  std_logic;   -- activo bajo (botÃ³n KEY)
        servo_phi     : out std_logic;
        servo_theta1  : out std_logic;
        servo_theta2  : out std_logic;
        servo_theta3  : out std_logic;
        servo_gripper : out std_logic
    );
end SeguidorLinea_Brazo_Servos90;

-- ============================================================================
-- ARCHITECTURE
-- ============================================================================
architecture Behavioral of SeguidorLinea_Brazo_Servos90 is

    -- =========================================================================
    -- DeclaraciÃ³n del componente polarPWM (ahora con pwm_gripper)
    -- =========================================================================
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
            pwm_theta3  : out std_logic;
            pwm_gripper : out std_logic    -- â† 5.Âª salida
        );
    end component;

    -- =========================================================================
    -- Constante: 90Â° = 90 decimal
    -- =========================================================================
    constant POS_90 : std_logic_vector(7 downto 0) :=
                      std_logic_vector(to_unsigned(90, 8));

begin

    -- =========================================================================
    -- Instancia Ãºnica de polarPWM â€” todos los ejes a 90Â°
    -- =========================================================================
    u_pwm : polarPWM
        Port Map (
            clk         => clk,
            rst         => reset,
            phi_in      => POS_90,
            theta_in    => POS_90,
            radio_in    => POS_90,
            gripper_in  => POS_90,
            pwm_phi     => servo_phi,
            pwm_theta1  => servo_theta1,
            pwm_theta2  => servo_theta2,
            pwm_theta3  => servo_theta3,
            pwm_gripper => servo_gripper   -- â† conectado correctamente
        );

end Behavioral;
