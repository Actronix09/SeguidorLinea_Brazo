-- ============================================================================
-- LIDAR_Grab_Top - Top de PRUEBA: BUSCAR + AGARRAR un cubo (con servos reales)
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Secuencia completa, autodisparada ~1.5 s tras soltar el reset:
--   1) LIDAR escanea el área y localiza el cubo -> coordenada polar (r,theta,phi).
--   2) grab_ctrl: polar_ik convierte la coordenada a ángulos de servo y mueve el
--      brazo a esos ángulos (garra abierta), CIERRA la garra y levanta.
--
--   LIDAR --cmd_*/out_*--> grab_ctrl --(MUX)--> polarPWM --> 5 servos
--
-- LEDs (activo-bajo: '0' enciende):
--   led_1(PIN_3): scan_active (barriendo)
--   led_2(PIN_7): grip_closed (cubo agarrado)
--   led_3(PIN_9): done_all    (secuencia completa)
-- Observar la coordenada/ángulos por SignalTap (signals.stp).
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity LIDAR_Grab_Top is
    port (
        clk          : in    std_logic;                 -- PIN_17, 50 MHz
        reset        : in    std_logic;                 -- PIN_144, activo bajo
        i2c_scl      : out   std_logic;                 -- PIN_142
        i2c_sda      : inout std_logic;                 -- PIN_136
        servo_phi    : out   std_logic;                 -- PIN_118
        servo_theta1 : out   std_logic;                 -- PIN_122
        servo_theta2 : out   std_logic;                 -- PIN_126
        servo_theta3 : out   std_logic;                 -- PIN_132
        servo_gripper: out   std_logic;                 -- PIN_134
        led_1        : out   std_logic;                 -- scan_active
        led_2        : out   std_logic;                 -- grip_closed
        led_3        : out   std_logic                  -- done_all
    );
end LIDAR_Grab_Top;

architecture Behavioral of LIDAR_Grab_Top is

    component LIDAR
        port (
            clk           : in    std_logic;
            rst           : in    std_logic;
            start_scan    : in    std_logic;
            i2c_scl       : out   std_logic;
            i2c_sda       : inout std_logic;
            scan_active   : out   std_logic;
            cmd_phi       : out   std_logic_vector(7 downto 0);
            cmd_theta1    : out   std_logic_vector(7 downto 0);
            cmd_theta2    : out   std_logic_vector(7 downto 0);
            cmd_theta3    : out   std_logic_vector(7 downto 0);
            cmd_grip      : out   std_logic;
            out_r         : out   std_logic_vector(15 downto 0);
            out_theta     : out   std_logic_vector(8 downto 0);
            out_phi       : out   std_logic_vector(7 downto 0);
            scan_done     : out   std_logic;
            dbg_meas_tick : out   std_logic
        );
    end component;

    component grab_ctrl
        generic (
            MOVE_CYCLES : integer := 125_000_000;
            GRIP_CYCLES : integer := 40_000_000;
            LIFT_CYCLES : integer := 60_000_000
        );
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

    constant START_DELAY : integer := 150_000_000;   -- ~1.5 s: deja que el brazo llegue a REPOSO (phi=180) antes de barrer

    signal reset_int  : std_logic;

    -- Auto-disparo del barrido
    signal start_scan : std_logic := '0';
    signal armed      : std_logic := '1';
    signal dly        : integer range 0 to START_DELAY := 0;

    -- LIDAR
    signal scan_active : std_logic;
    signal scan_done   : std_logic;
    signal meas_tick   : std_logic;
    signal cmd_phi     : std_logic_vector(7 downto 0);
    signal cmd_theta1  : std_logic_vector(7 downto 0);
    signal cmd_theta2  : std_logic_vector(7 downto 0);
    signal cmd_theta3  : std_logic_vector(7 downto 0);
    signal cmd_grip    : std_logic;
    signal out_r       : std_logic_vector(15 downto 0);
    signal out_theta   : std_logic_vector(8 downto 0);
    signal out_phi     : std_logic_vector(7 downto 0);

    -- grab_ctrl
    signal grip_closed : std_logic;
    signal reachable   : std_logic;
    signal done_all    : std_logic;
    signal phi_in      : std_logic_vector(7 downto 0);
    signal theta1_in   : std_logic_vector(7 downto 0);
    signal theta2_in   : std_logic_vector(7 downto 0);
    signal theta3_in   : std_logic_vector(7 downto 0);
    signal grip_in     : std_logic;

    -- theta1 compensado: el servo theta1 está montado INVERTIDO (igual que theta3),
    -- se corrige aquí (180 - theta1) para barrido Y agarre, justo antes de polarPWM.
    signal theta1_pwm  : std_logic_vector(7 downto 0);

begin

    reset_int <= not reset;

    theta1_pwm <= std_logic_vector(to_unsigned(180 - to_integer(unsigned(theta1_in)), 8));

    -- Auto-disparo: 1 pulso de start_scan ~1.5 s tras soltar reset
    arm_proc : process(clk, reset_int)
    begin
        if reset_int = '1' then
            armed <= '1'; dly <= 0; start_scan <= '0';
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

    u_lidar : LIDAR
        port map (
            clk => clk, rst => reset_int, start_scan => start_scan,
            i2c_scl => i2c_scl, i2c_sda => i2c_sda,
            scan_active => scan_active,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_theta1, cmd_theta2 => cmd_theta2,
            cmd_theta3 => cmd_theta3, cmd_grip => cmd_grip,
            out_r => out_r, out_theta => out_theta, out_phi => out_phi,
            scan_done => scan_done, dbg_meas_tick => meas_tick
        );

    u_grab : grab_ctrl
        port map (
            clk => clk, rst => reset_int,
            scan_active => scan_active, scan_done => scan_done,
            in_r => out_r, in_theta => out_theta, in_phi => out_phi,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_theta1, cmd_theta2 => cmd_theta2,
            cmd_theta3 => cmd_theta3, cmd_grip => cmd_grip,
            phi_out => phi_in, theta1_out => theta1_in, theta2_out => theta2_in,
            theta3_out => theta3_in, grip_out => grip_in,
            grip_closed => grip_closed, reachable => reachable, done_all => done_all
        );

    u_pwm : polarPWM
        port map (
            clk => clk, rst => reset_int,
            phi_in => phi_in, theta1_in => theta1_pwm, theta2_in => theta2_in,
            theta3_in => theta3_in, grip_cmd => grip_in,
            pwm_phi => servo_phi, pwm_theta1 => servo_theta1, pwm_theta2 => servo_theta2,
            pwm_theta3 => servo_theta3, pwm_gripper => servo_gripper
        );

    -- LEDs (activo-bajo: '0' enciende)
    led_1 <= not scan_active;     -- barriendo
    led_2 <= not grip_closed;     -- cubo agarrado
    led_3 <= not done_all;        -- secuencia completa

end Behavioral;
