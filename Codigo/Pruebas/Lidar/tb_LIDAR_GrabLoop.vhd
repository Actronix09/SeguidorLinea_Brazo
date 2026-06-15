-- ============================================================================
-- tb_LIDAR_GrabLoop - Verifica que LIDAR_GrabLoop_Top RE-DISPARA el barrido en
--   bucle. Sin esclavo I2C el VL53L0X no entrega medición -> el watchdog del LIDAR
--   aborta cada barrido (found=0, scan_fault=1) y el bucle debe volver a escanear.
--   Se cuenta cuántas veces arranca el barrido (led_1 baja = scan_active sube).
--   Generics chicos para simular rápido.
--
--   vcom -2008 vl53l0x_pkg VL53L0X kinematics polarPWM grab_ctrl LIDAR
--             LIDAR_GrabLoop_Top tb_LIDAR_GrabLoop
--   vsim tb_LIDAR_GrabLoop -do "run -all"
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_LIDAR_GrabLoop is
end tb_LIDAR_GrabLoop;

architecture sim of tb_LIDAR_GrabLoop is
    signal clk : std_logic := '0';
    signal reset : std_logic := '0';   -- activo BAJO: '0' = en reset
    signal scl : std_logic;
    signal sda : std_logic;
    signal s_phi, s_t1, s_t2, s_t3, s_grip : std_logic;
    signal led_1, led_2, led_3 : std_logic;
    signal simdone : boolean := false;

    -- Cuenta de arranques de barrido (flanco de bajada de led_1 = scan_active sube).
    signal led1_prev : std_logic := '1';
    signal n_scans   : integer := 0;
begin
    sda <= 'H';   -- pull-up; sin esclavo => toda lectura I2C es NACK

    dut : entity work.LIDAR_GrabLoop_Top
        generic map (
            START_DELAY => 1000, PAUSE_CYCLES => 1000,
            T_PWRUP => 200, T_SETTLE => 200, T_WDOG => 2000,
            T_MOVE => 500, T_GRIP => 500
        )
        port map (
            clk => clk, reset => reset, i2c_scl => scl, i2c_sda => sda,
            servo_phi => s_phi, servo_theta1 => s_t1, servo_theta2 => s_t2,
            servo_theta3 => s_t3, servo_gripper => s_grip,
            led_1 => led_1, led_2 => led_2, led_3 => led_3
        );

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- Cuenta arranques de barrido (led_1: 1 -> 0).
    cnt : process(clk)
    begin
        if rising_edge(clk) then
            if led_1 = '0' and led1_prev = '1' then
                n_scans <= n_scans + 1;
            end if;
            led1_prev <= led_1;
        end if;
    end process;

    stim : process
    begin
        reset <= '0'; wait for 1 us; reset <= '1';   -- suelta el reset (activo bajo)

        -- Cada ciclo del bucle: start(1000)+settle(200)+watchdog(2000)+pausa(1000) ~4.5k ciclos.
        -- En 1 ms (50k ciclos) deben caber varios barridos.
        wait for 1 ms;

        assert n_scans >= 2
            report "FALLO: el bucle no re-disparo el barrido (n_scans=" &
                   integer'image(n_scans) & ")" severity error;
        report "OK: bucle continuo de barridos, n_scans=" & integer'image(n_scans) severity note;

        -- Con el sensor sin responder, el watchdog debe marcar scan_fault (led_3 encendido='0').
        assert led_3 = '0'
            report "FALLO: sin sensor, led_3 (scan_fault) deberia encender" severity error;
        report "OK: sin sensor -> led_3 (scan_fault) encendido" severity note;

        report "OK: LIDAR_GrabLoop_Top hace loop de busqueda/recogida" severity note;
        simdone <= true;
        wait;
    end process;
end sim;
