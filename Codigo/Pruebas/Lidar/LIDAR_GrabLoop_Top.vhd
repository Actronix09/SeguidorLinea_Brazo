-- ============================================================================
-- LIDAR_GrabLoop_Top - Top de PRUEBA: LOOP CONTINUO de BUSCAR + RECOGER.
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Ejercita SOLO el brazo (LIDAR + grab_ctrl + 5 servos), SIN motores DC.
-- Auto-dispara la secuencia en BUCLE infinito:
--   1) start_scan -> LIDAR barre y localiza el cubo (min_t1/min_d/min_phi, found).
--   2) grab_ctrl: IK -> mueve el brazo -> CIERRA garra -> HOLD (has_object=1).
--   3) trigger_drop -> deposita (DROP_PHI/T1) -> ABRE -> REST.
--   4) pausa breve y vuelve a 1). Si NO encuentra nada, re-escanea igual.
-- Si el sensor no responde, el watchdog del LIDAR aborta el barrido (scan_fault)
-- y el bucle sigue intentando (no se cuelga).
--
--   LIDAR --cmd_*/min_*/found--> grab_ctrl --(MUX)--> [180-t1] -> polarPWM -> 5 servos
--
-- LEDs (activo-bajo: '0' enciende):
--   led_1(PIN_3): scan_active (barriendo)
--   led_2(PIN_7): has_object  (cubo agarrado)
--   led_3(PIN_9): scan_fault  (el sensor VL53L0X no respondió en el último barrido)
--
-- Generics: defaults = HARDWARE; un tb los baja para simular rápido.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity LIDAR_GrabLoop_Top is
    generic (
        START_DELAY   : integer := 150_000_000;  -- ~3 s: deja el brazo en REPOSO antes del 1er barrido
        PAUSE_CYCLES  : integer := 50_000_000;    -- ~1 s entre ciclos del bucle (observable)
        -- Pasan a los submódulos (default = su valor de hardware)
        T_PWRUP       : integer := 500_000;       -- LIDAR/VL53L0X powerup
        T_SETTLE      : integer := 12_500_000;    -- LIDAR asentamiento por punto
        T_WDOG        : integer := 100_000_000;   -- LIDAR watchdog (~2 s)
        T_MOVE        : integer := 125_000_000;   -- grab_ctrl: mover el brazo
        T_GRIP        : integer := 75_000_000     -- grab_ctrl: abrir/cerrar garra (~1.5 s; cierre real ~1 s)
    );
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
        led_2        : out   std_logic;                 -- has_object
        led_3        : out   std_logic                  -- scan_fault (error de sensor)
    );
end LIDAR_GrabLoop_Top;

architecture Behavioral of LIDAR_GrabLoop_Top is

    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    constant TMR_MAX : integer := imax(START_DELAY, PAUSE_CYCLES);

    signal reset_int : std_logic;

    -- Bucle de control (autodisparo)
    type lst_t is (L_INIT, L_DECIDE, L_SCAN, L_SCANGO, L_DROP, L_DROPGO, L_BUSY, L_PAUSE);
    signal lst : lst_t := L_INIT;
    signal tmr : integer range 0 to TMR_MAX := 0;
    signal start_scan   : std_logic := '0';
    signal trigger_drop : std_logic := '0';

    -- LIDAR
    signal scan_active : std_logic;
    signal scan_done   : std_logic;
    signal found       : std_logic;
    signal scan_fault  : std_logic;
    signal cmd_phi     : std_logic_vector(7 downto 0);
    signal cmd_theta1  : std_logic_vector(7 downto 0);
    signal cmd_theta2  : std_logic_vector(7 downto 0);
    signal cmd_theta3  : std_logic_vector(7 downto 0);
    signal cmd_grip    : std_logic;
    signal min_t1      : std_logic_vector(7 downto 0);
    signal min_d       : std_logic_vector(15 downto 0);
    signal min_phi     : std_logic_vector(7 downto 0);

    -- grab_ctrl
    signal has_object  : std_logic;
    signal arm_ready   : std_logic;
    signal reachable   : std_logic;
    signal phi_in      : std_logic_vector(7 downto 0);
    signal theta1_in   : std_logic_vector(7 downto 0);
    signal theta2_in   : std_logic_vector(7 downto 0);
    signal theta3_in   : std_logic_vector(7 downto 0);
    signal grip_in     : std_logic;

    -- theta1 montado INVERTIDO: se corrige (180 - theta1) antes de polarPWM (igual que el top real)
    signal theta1_pwm  : std_logic_vector(7 downto 0);

begin

    reset_int  <= not reset;
    theta1_pwm <= std_logic_vector(to_unsigned(180 - to_integer(unsigned(theta1_in)), 8));

    -- ------------------------------------------------------------------------
    -- Bucle: escanear -> (si agarró) depositar -> pausa -> repetir
    -- ------------------------------------------------------------------------
    loop_proc : process(clk, reset_int)
    begin
        if reset_int = '1' then
            lst <= L_INIT; tmr <= 0;
            start_scan <= '0'; trigger_drop <= '0';
        elsif rising_edge(clk) then
            start_scan   <= '0';   -- pulsos por defecto a '0'
            trigger_drop <= '0';

            case lst is
                -- Espera inicial: el brazo llega a REPOSO antes del primer barrido.
                when L_INIT =>
                    if tmr >= START_DELAY-1 then tmr <= 0; lst <= L_DECIDE;
                    else tmr <= tmr + 1; end if;

                -- ¿Lleva cubo? -> depositar; si no -> escanear.
                when L_DECIDE =>
                    if has_object = '1' then lst <= L_DROP;
                    else                     lst <= L_SCAN; end if;

                -- Dispara un barrido (1 pulso).
                when L_SCAN =>
                    start_scan <= '1';
                    lst <= L_SCANGO;

                -- Espera a que el barrido arranque de verdad.
                when L_SCANGO =>
                    if scan_active = '1' then lst <= L_BUSY; end if;

                -- Dispara el depósito (1 pulso).
                when L_DROP =>
                    trigger_drop <= '1';
                    lst <= L_DROPGO;

                -- Espera a que el depósito arranque (el brazo deja de estar listo).
                when L_DROPGO =>
                    if arm_ready = '0' then lst <= L_BUSY; end if;

                -- Espera a que la acción (barrido+agarre o depósito) termine.
                when L_BUSY =>
                    if arm_ready = '1' then tmr <= 0; lst <= L_PAUSE; end if;

                -- Pausa observable y vuelve a empezar.
                when L_PAUSE =>
                    if tmr >= PAUSE_CYCLES-1 then tmr <= 0; lst <= L_DECIDE;
                    else tmr <= tmr + 1; end if;
            end case;
        end if;
    end process;

    -- ------------------------------------------------------------------------
    -- Brazo: LIDAR -> grab_ctrl -> polarPWM (instanciación directa por entidad)
    -- ------------------------------------------------------------------------
    u_lidar : entity work.LIDAR
        generic map (PWRUP_CYCLES => T_PWRUP, SETTLE_CYCLES => T_SETTLE, WDOG_CYCLES => T_WDOG)
        port map (
            clk => clk, rst => reset_int, start_scan => start_scan,
            i2c_scl => i2c_scl, i2c_sda => i2c_sda,
            scan_active => scan_active,
            cmd_phi => cmd_phi, cmd_theta1 => cmd_theta1, cmd_theta2 => cmd_theta2,
            cmd_theta3 => cmd_theta3, cmd_grip => cmd_grip,
            min_t1 => min_t1, min_d => min_d, min_phi => min_phi,
            found => found, scan_done => scan_done, scan_fault => scan_fault,
            dbg_meas_tick => open
        );

    u_grab : entity work.grab_ctrl
        -- DROP_* NO se fijan aquí: se toman del default de grab_ctrl.vhd (único mando)
        generic map (
            MOVE_CYCLES => T_MOVE, GRIP_CYCLES => T_GRIP
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

    u_pwm : entity work.polarPWM
        port map (
            clk => clk, rst => reset_int,
            phi_in => phi_in, theta1_in => theta1_pwm, theta2_in => theta2_in,
            theta3_in => theta3_in, grip_cmd => grip_in,
            pwm_phi => servo_phi, pwm_theta1 => servo_theta1, pwm_theta2 => servo_theta2,
            pwm_theta3 => servo_theta3, pwm_gripper => servo_gripper
        );

    -- LEDs (activo-bajo: '0' enciende)
    led_1 <= not scan_active;   -- barriendo
    led_2 <= not has_object;    -- cubo agarrado
    led_3 <= not scan_fault;    -- el sensor no respondió (watchdog abortó el barrido)

end Behavioral;
