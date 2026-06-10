-- ============================================================================
-- SeguidorLinea_Brazo - TOP de Sísifo (ETAPA 2, integración final)
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Sísifo sigue una pista cerrada en loop y, en cada zona (cuadrado negro con
-- línea blanca), alterna según el estado de acarreo:
--   - SIN objeto -> dispara el LIDAR (buscar + agarrar) y queda en HOLD.
--   - CON objeto -> deposita (gira la base a phi=90, extiende, abre garra).
--
-- Cableado (ver plan Etapa 2):
--   QRD izq/der -> MaquinaEstados --(PWM A1/A2/B1/B2)--> L293 -> 2 motores DC
--                       │  ▲ has_object, arm_ready
--          start_scan ──┘  │  trigger_drop
--                       ▼  │
--     LIDAR (escáner) --cmd_*/min_*/found/scan_done/scan_active--> grab_ctrl
--          │ I2C VL53L0X                                              │
--          ▼                                                         ▼
--                  grab_ctrl --(phi,t1,t2,t3,grip muxeados)--> [180−t1] -> polarPWM -> 5 servos
--
-- El top sólo aplica la inversión de theta1 (servo montado al revés) antes de
-- polarPWM y cablea motores / sensores / LEDs. reset activo BAJO (PIN_144).
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity SeguidorLinea_Brazo is
    port (
        -- Control
        clk          : in    std_logic;                 -- PIN_17, 50 MHz
        reset        : in    std_logic;                 -- PIN_144, activo bajo

        -- VL53L0X I2C
        i2c_scl      : out   std_logic;                 -- PIN_142
        i2c_sda      : inout std_logic;                 -- PIN_136

        -- Servomotores (5 ejes)
        servo_phi    : out   std_logic;                 -- PIN_118
        servo_theta1 : out   std_logic;                 -- PIN_122
        servo_theta2 : out   std_logic;                 -- PIN_126
        servo_theta3 : out   std_logic;                 -- PIN_132
        servo_gripper: out   std_logic;                 -- PIN_134

        -- Sensores de línea QRD1114
        sensor_izq   : in    std_logic;
        sensor_der   : in    std_logic;

        -- Motores DC (PWM directo en las 4 entradas del L293, enables fijos en HW)
        motor_a1     : out   std_logic;                 -- PIN_21  (IZQ adelante)
        motor_a2     : out   std_logic;                 -- PIN_8   (IZQ atrás, =0)
        motor_b1     : out   std_logic;                 -- PIN_26  (DER adelante)
        motor_b2     : out   std_logic;                 -- PIN_24  (DER atrás, =0)

        -- LEDs de la placa (activo-bajo: '0' enciende)
        led_1        : out   std_logic;                 -- vida (1 Hz)
        led_2        : out   std_logic;                 -- has_object (lleva cubo)
        led_3        : out   std_logic                  -- scan_active (escaneando)
    );
end SeguidorLinea_Brazo;

architecture Behavioral of SeguidorLinea_Brazo is

    -- -------------------------------------------------------------------------
    -- Componente: LIDAR (escáner, entrega el mínimo crudo + found)
    -- -------------------------------------------------------------------------
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
            min_t1        : out   std_logic_vector(7 downto 0);
            min_d         : out   std_logic_vector(15 downto 0);
            min_phi       : out   std_logic_vector(7 downto 0);
            found         : out   std_logic;
            scan_done     : out   std_logic;
            dbg_meas_tick : out   std_logic
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Componente: grab_ctrl (orquestador del brazo: REST/agarre/HOLD/depósito)
    -- -------------------------------------------------------------------------
    component grab_ctrl
        generic (
            MOVE_CYCLES : integer := 125_000_000;
            GRIP_CYCLES : integer := 40_000_000;
            DROP_PHI    : integer := 90; DROP_T1 : integer := 45;
            DROP_T2     : integer := 0;  DROP_T3 : integer := 0
        );
        port (
            clk          : in  std_logic;
            rst          : in  std_logic;
            scan_active  : in  std_logic;
            scan_done    : in  std_logic;
            found        : in  std_logic;
            min_t1       : in  std_logic_vector(7 downto 0);
            min_d        : in  std_logic_vector(15 downto 0);
            min_phi      : in  std_logic_vector(7 downto 0);
            cmd_phi      : in  std_logic_vector(7 downto 0);
            cmd_theta1   : in  std_logic_vector(7 downto 0);
            cmd_theta2   : in  std_logic_vector(7 downto 0);
            cmd_theta3   : in  std_logic_vector(7 downto 0);
            cmd_grip     : in  std_logic;
            trigger_drop : in  std_logic;
            phi_out      : out std_logic_vector(7 downto 0);
            theta1_out   : out std_logic_vector(7 downto 0);
            theta2_out   : out std_logic_vector(7 downto 0);
            theta3_out   : out std_logic_vector(7 downto 0);
            grip_out     : out std_logic;
            has_object   : out std_logic;
            arm_ready    : out std_logic;
            reachable    : out std_logic
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Componente: MaquinaEstados (seguidor de línea + orquestación)
    -- -------------------------------------------------------------------------
    component MaquinaEstados
        generic (
            ZONA_CYCLES      : integer := 2_000_000; SALIR_CYCLES  : integer := 10_000_000;
            FILTRO_CYCLES    : integer := 50_000;    LINE_LVL      : std_logic := '1'
        );
        port (
            clk          : in  std_logic;
            rst          : in  std_logic;
            sensor_izq   : in  std_logic;
            sensor_der   : in  std_logic;
            motor_a1     : out std_logic;
            motor_a2     : out std_logic;
            motor_b1     : out std_logic;
            motor_b2     : out std_logic;
            start_scan   : out std_logic;
            trigger_drop : out std_logic;
            scan_active  : in  std_logic;
            arm_ready    : in  std_logic;
            has_object   : in  std_logic;
            led_estado   : out std_logic;
            led_error    : out std_logic
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Componente: polarPWM (5 servos)
    -- -------------------------------------------------------------------------
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

    -- Reset interno (activo alto para todos los submódulos)
    signal reset_int : std_logic;

    -- LIDAR <-> grab_ctrl / MaquinaEstados
    signal start_scan  : std_logic;
    signal scan_active : std_logic;
    signal scan_done   : std_logic;
    signal found       : std_logic;
    signal meas_tick   : std_logic;
    signal cmd_phi     : std_logic_vector(7 downto 0);
    signal cmd_theta1  : std_logic_vector(7 downto 0);
    signal cmd_theta2  : std_logic_vector(7 downto 0);
    signal cmd_theta3  : std_logic_vector(7 downto 0);
    signal cmd_grip    : std_logic;
    signal min_t1      : std_logic_vector(7 downto 0);
    signal min_d       : std_logic_vector(15 downto 0);
    signal min_phi     : std_logic_vector(7 downto 0);

    -- grab_ctrl -> polarPWM / MaquinaEstados
    signal trigger_drop : std_logic;
    signal has_object   : std_logic;
    signal arm_ready    : std_logic;
    signal reachable    : std_logic;
    signal phi_in       : std_logic_vector(7 downto 0);
    signal theta1_in    : std_logic_vector(7 downto 0);
    signal theta2_in    : std_logic_vector(7 downto 0);
    signal theta3_in    : std_logic_vector(7 downto 0);
    signal grip_in      : std_logic;

    -- theta1 compensado: el servo theta1 está montado INVERTIDO; se corrige aquí
    -- (180 - theta1) para barrido Y agarre, justo antes de polarPWM.
    signal theta1_pwm  : std_logic_vector(7 downto 0);

    -- LEDs
    signal me_led_estado : std_logic;

begin

    reset_int  <= not reset;
    theta1_pwm <= std_logic_vector(to_unsigned(180 - to_integer(unsigned(theta1_in)), 8));

    -- =========================================================================
    -- LIDAR (escáner): la MaquinaEstados lo dispara con start_scan
    -- =========================================================================
    u_lidar : LIDAR
        port map (
            clk => clk, rst => reset_int, start_scan => start_scan,
            i2c_scl => i2c_scl, i2c_sda => i2c_sda,
            scan_active => scan_active,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_theta1, cmd_theta2 => cmd_theta2,
            cmd_theta3 => cmd_theta3, cmd_grip => cmd_grip,
            min_t1 => min_t1, min_d => min_d, min_phi => min_phi,
            found => found, scan_done => scan_done, dbg_meas_tick => meas_tick
        );

    -- =========================================================================
    -- grab_ctrl: ciclo del brazo (REST -> agarre -> HOLD -> depósito -> REST)
    -- =========================================================================
    u_grab : grab_ctrl
        -- Pose de DEPÓSITO al soltar el objeto: phi=90, theta1=90, theta2=0, theta3=0.
        generic map (
            DROP_PHI => 90, DROP_T1 => 90,
            DROP_T2 => 0,   DROP_T3 => 0
        )
        port map (
            clk => clk, rst => reset_int,
            scan_active => scan_active, scan_done => scan_done, found => found,
            min_t1 => min_t1, min_d => min_d, min_phi => min_phi,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_theta1, cmd_theta2 => cmd_theta2,
            cmd_theta3 => cmd_theta3, cmd_grip => cmd_grip,
            trigger_drop => trigger_drop,
            phi_out => phi_in, theta1_out => theta1_in, theta2_out => theta2_in,
            theta3_out => theta3_in, grip_out => grip_in,
            has_object => has_object, arm_ready => arm_ready, reachable => reachable
        );

    -- =========================================================================
    -- MaquinaEstados: seguidor de línea + orquestación por estado de acarreo
    -- =========================================================================
    u_me : MaquinaEstados
        -- QRD físico: '1' en BLANCO, '0' en NEGRO -> "sobre la línea negra" = '0'.
        -- Velocidad/calibración: constantes DUTY_* dentro de MaquinaEstados.vhd.
        generic map (
            LINE_LVL      => '0',
            FILTRO_CYCLES => 15_000,   -- antirrebote sensores ~300 us. Equilibrio: filtra el
                                       -- ruido del LM393 (se salía random en recta) PERO no
                                       -- tanto que borre la pista direccional al entrar a una
                                       -- curva (un sensor pisa negro antes que el otro). Subir
                                       -- si tiembla en recta; bajar si pierde curvas cerradas.
            ZONA_CYCLES   => 15_000_000 -- CONFIRMAR zona = 1 s de negro-doble CONTINUO (viniendo
                                        -- de RECTO). El robot avanza recto durante ese segundo,
                                        -- así que el cuadro debe ser grande / el robot lento para
                                        -- estar 1 s encima; si lo cruza en <1 s NO lo detecta ->
                                        -- bajar este valor. Subir si una curva dispara zona falsa.
        )
        port map (
            clk => clk, rst => reset_int,
            sensor_izq => sensor_izq, sensor_der => sensor_der,
            motor_a1 => motor_a1, motor_a2 => motor_a2,
            motor_b1 => motor_b1, motor_b2 => motor_b2,
            start_scan => start_scan, trigger_drop => trigger_drop,
            scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
            led_estado => me_led_estado, led_error => open
        );

    -- =========================================================================
    -- polarPWM: 5 servos (theta1 ya invertido)
    -- =========================================================================
    u_pwm : polarPWM
        port map (
            clk => clk, rst => reset_int,
            phi_in => phi_in, theta1_in => theta1_pwm, theta2_in => theta2_in,
            theta3_in => theta3_in, grip_cmd => grip_in,
            pwm_phi => servo_phi, pwm_theta1 => servo_theta1, pwm_theta2 => servo_theta2,
            pwm_theta3 => servo_theta3, pwm_gripper => servo_gripper
        );

    -- =========================================================================
    -- LEDs de la placa (activo-bajo: '0' enciende)
    -- =========================================================================
    led_1 <= not me_led_estado;   -- vida (parpadeo 1 Hz)
    led_2 <= not has_object;      -- lleva el cubo
    led_3 <= not scan_active;     -- escaneando

end Behavioral;
