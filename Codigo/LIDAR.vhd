-- ============================================================================
-- LIDAR - Escáner 2D con el brazo: localiza el objeto más cercano (cubo) y
--         entrega el PUNTO CRUDO del mínimo (theta1*, d*, phi*) + 'found'.
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Patrón de barrido (al recibir start_scan):
--   1) Pose: theta3 fijo = 0 (haz vertical hacia ABAJO), grip ABIERTO.
--   2) Barrido GRUESO 2D: phi 45->135 y theta1 90->45 (con theta2 = 90-theta1
--      ACOPLADO, L2 horizontal -> el haz baja vertical y se TRASLADA sobre la mesa).
--      Serpentina; en cada punto promedia N_AVG mediciones y guarda el de MENOR
--      distancia (= cima del cubo).
--   3) Barrido FINO alrededor del mínimo (theta1 SIEMPRE en el mismo sentido).
--   4) Latchea el mínimo crudo (best_t1, best_d, best_phi) -> salidas min_*, marca
--      found = (min_d < FOUND_TH) y pulsa scan_done. La cinemática (FK+IK) la hace
--      'kinematics' aguas abajo (en grab_ctrl), NO aquí.
--
-- Comanda ÁNGULOS ABSOLUTOS de servo (0-180) -> van a polarPWM. rst activo ALTO.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity LIDAR is
    generic (
        CLK_FREQ_HZ     : integer := 50_000_000;
        I2C_FREQ_HZ     : integer := 100_000;
        PWRUP_CYCLES    : integer := 500_000;
        SETTLE_CYCLES   : integer := 12_500_000;  -- 1 s de asentamiento/punto (el brazo se tambalea tras moverse)
        N_AVG           : integer := 8;           -- mediciones promediadas por punto
        COARSE_PHI_STEP : integer := 10;          -- paso phi grueso (45..135 -> 10 pts)
        COARSE_T1_STEP  : integer := 9;           -- paso theta1 grueso (45..90 -> 6 pts)
        FINE_PHI_STEP   : integer := 3;           -- paso phi fino
        FINE_T1_STEP    : integer := 3;           -- paso theta1 fino
        FOUND_TH        : integer := 200          -- mm: min_d < FOUND_TH => hay objeto
    );
    port (
        clk         : in    std_logic;
        rst         : in    std_logic;                      -- activo ALTO
        start_scan  : in    std_logic;                      -- pulso: inicia barrido
        i2c_scl     : out   std_logic;
        i2c_sda     : inout std_logic;
        -- comandos de servo (ángulos absolutos 0-180) durante el barrido
        scan_active : out   std_logic;                      -- '1' mientras escanea (MUX del top)
        cmd_phi     : out   std_logic_vector(7 downto 0);
        cmd_theta1  : out   std_logic_vector(7 downto 0);   -- barre 90..45
        cmd_theta2  : out   std_logic_vector(7 downto 0);   -- = 90 - theta1 (L2 horizontal)
        cmd_theta3  : out   std_logic_vector(7 downto 0);   -- = 0 (haz hacia abajo)
        cmd_grip    : out   std_logic;                      -- = 1 (abierto; 1=abre, 0=cierra como TestBrazo)
        -- resultado: punto más cercano del barrido (crudo, para kinematics)
        min_t1      : out   std_logic_vector(7 downto 0);    -- theta1* del mínimo
        min_d       : out   std_logic_vector(15 downto 0);   -- distancia* (mm)
        min_phi     : out   std_logic_vector(7 downto 0);    -- phi* del mínimo
        found       : out   std_logic;                       -- '1' si min_d < FOUND_TH (hay objeto)
        scan_done   : out   std_logic;
        dbg_meas_tick : out std_logic                       -- conmuta cada medición (LED de vida)
    );
end LIDAR;

architecture rtl of LIDAR is

    -- Límites mecánicos del barrido
    constant PHI_MIN : integer := 45;
    constant PHI_MAX : integer := 135;
    constant T1_MIN  : integer := 45;        -- theta1 mínimo (brazo extendido al frente)
    constant T1_MAX  : integer := 90;        -- theta1 máximo (eslabón L1 vertical)
    constant THETA3_POSE : integer := 0;     -- theta3 fijo: haz vertical hacia abajo

    -- --------------------------------------------------------------------
    -- Componente del driver del sensor
    -- --------------------------------------------------------------------
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

    -- Señales del driver
    signal distance_mm : std_logic_vector(15 downto 0);
    signal meas_tick   : std_logic;

    -- FSM del escáner
    type st_t is (S_IDLE, S_MOVE, S_SETTLE, S_AVG, S_NEXT, S_DONE);
    signal st : st_t := S_IDLE;

    -- Rejilla actual (límites/pasos cambian entre grueso y fino)
    signal phi_cur  : integer range 0 to 180 := 90;
    signal t1_cur   : integer range 0 to 180 := T1_MAX;
    signal phi_lo   : integer range 0 to 180 := PHI_MIN;
    signal phi_hi   : integer range 0 to 180 := PHI_MAX;
    signal t1_lo    : integer range 0 to 180 := T1_MIN;
    signal t1_hi    : integer range 0 to 180 := T1_MAX;
    signal phi_step : integer range 1 to 180 := COARSE_PHI_STEP;
    signal t1_step  : integer range 1 to 180 := COARSE_T1_STEP;
    signal t1_dir   : integer range -1 to 1 := 1;
    signal fine_ph  : std_logic := '0';

    -- Mínimo (punto más cercano encontrado)
    signal best_d   : unsigned(15 downto 0) := (others => '1');
    signal best_phi : integer range 0 to 180 := 90;
    signal best_t1  : integer range 0 to 180 := T1_MAX;

    -- Asentamiento y promediado
    signal settle_cnt : integer range 0 to SETTLE_CYCLES := 0;
    signal acc        : unsigned(23 downto 0) := (others => '0');
    signal navg       : integer range 0 to 255 := 0;
    signal tick_prev  : std_logic := '0';
    signal discard1   : std_logic := '1';
    signal avg_val    : unsigned(15 downto 0) := (others => '0');

    -- Salidas registradas (resultado del barrido, latcheadas al terminar)
    signal scan_done_r : std_logic := '0';
    signal min_t1_r    : std_logic_vector(7 downto 0)  := (others => '0');
    signal min_d_r     : std_logic_vector(15 downto 0) := (others => '0');
    signal min_phi_r   : std_logic_vector(7 downto 0)  := (others => '0');
    signal found_r     : std_logic := '0';

    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    function imin(a, b : integer) return integer is
    begin
        if a < b then return a; else return b; end if;
    end function;

begin

    -- --------------------------------------------------------------------
    -- Instancia del driver del sensor
    -- --------------------------------------------------------------------
    u_VL53L0X : VL53L0X
        generic map (
            CLK_FREQ_HZ  => CLK_FREQ_HZ,
            I2C_FREQ_HZ  => I2C_FREQ_HZ,
            PWRUP_CYCLES => PWRUP_CYCLES
        )
        port map (
            clk         => clk,
            rst         => rst,
            i2c_scl     => i2c_scl,
            i2c_sda     => i2c_sda,
            distance_mm => distance_mm,
            data_valid  => open,
            sensor_ok   => open,
            err_code    => open,
            meas_tick   => meas_tick
        );

    -- --------------------------------------------------------------------
    -- Salidas combinacionales
    -- --------------------------------------------------------------------
    cmd_phi    <= std_logic_vector(to_unsigned(phi_cur, 8));
    cmd_theta1 <= std_logic_vector(to_unsigned(t1_cur, 8));
    cmd_theta2 <= std_logic_vector(to_unsigned(90 - t1_cur, 8));  -- L2 horizontal
    cmd_theta3 <= std_logic_vector(to_unsigned(THETA3_POSE, 8));  -- haz hacia abajo
    cmd_grip   <= '1';                                            -- siempre ABIERTO en el escáner (1=abre)
    scan_active <= '0' when st = S_IDLE else '1';

    min_t1    <= min_t1_r;
    min_d     <= min_d_r;
    min_phi   <= min_phi_r;
    found     <= found_r;
    scan_done <= scan_done_r;
    dbg_meas_tick <= meas_tick;

    -- --------------------------------------------------------------------
    -- FSM del escáner
    -- --------------------------------------------------------------------
    process(clk, rst)
        variable s     : unsigned(23 downto 0);
        variable cphi  : integer range 0 to 180;
        variable ct1   : integer range 0 to 180;
        variable nlo   : integer range 0 to 180;
        variable nhi   : integer range 0 to 180;
    begin
        if rst = '1' then
            st          <= S_IDLE;
            phi_cur     <= 90;
            t1_cur      <= T1_MAX;
            t1_dir      <= 1;
            fine_ph     <= '0';
            best_d      <= (others => '1');
            best_phi    <= 90;
            best_t1     <= T1_MAX;
            settle_cnt  <= 0;
            acc         <= (others => '0');
            navg        <= 0;
            discard1    <= '1';
            avg_val     <= (others => '0');
            scan_done_r <= '0';
            min_t1_r    <= (others => '0');
            min_d_r     <= (others => '0');
            min_phi_r   <= (others => '0');
            found_r     <= '0';

        elsif rising_edge(clk) then

            case st is

                -- Espera disparo; prepara rejilla gruesa
                when S_IDLE =>
                    if start_scan = '1' then
                        scan_done_r <= '0';
                        best_d   <= (others => '1');
                        best_phi <= 90;
                        best_t1  <= T1_MAX;
                        phi_lo   <= PHI_MIN;  phi_hi <= PHI_MAX;
                        t1_lo    <= T1_MIN;   t1_hi  <= T1_MAX;
                        phi_step <= COARSE_PHI_STEP;
                        t1_step  <= COARSE_T1_STEP;
                        -- theta1 arranca en 90 (L1 vertical = HOME, sin salto) y BAJA a 45
                        phi_cur  <= PHI_MIN;  t1_cur <= T1_MAX;
                        t1_dir   <= -1;
                        fine_ph  <= '0';
                        st       <= S_MOVE;
                    end if;

                -- Punto fijado: arranca temporizador de asentamiento
                when S_MOVE =>
                    settle_cnt <= 0;
                    st         <= S_SETTLE;

                -- Espera que el servo llegue y se estabilice
                when S_SETTLE =>
                    if settle_cnt >= SETTLE_CYCLES-1 then
                        acc       <= (others => '0');
                        navg      <= 0;
                        discard1  <= '1';
                        tick_prev <= meas_tick;
                        st        <= S_AVG;
                    else
                        settle_cnt <= settle_cnt + 1;
                    end if;

                -- Promedia N_AVG mediciones (1 por flanco de meas_tick)
                when S_AVG =>
                    if meas_tick /= tick_prev then
                        tick_prev <= meas_tick;
                        if discard1 = '1' then
                            discard1 <= '0';             -- descarta la 1ª (puede ser de transición)
                        else
                            s := acc + resize(unsigned(distance_mm), 24);
                            acc <= s;
                            if navg = N_AVG-1 then
                                avg_val <= resize(s / N_AVG, 16);
                                st      <= S_NEXT;
                            else
                                navg <= navg + 1;
                            end if;
                        end if;
                    end if;

                -- Actualiza mínimo y avanza la rejilla
                when S_NEXT =>
                    if avg_val < best_d then
                        cphi := phi_cur;  ct1 := t1_cur;
                        best_d   <= avg_val;
                        best_phi <= phi_cur;
                        best_t1  <= t1_cur;
                    else
                        cphi := best_phi;  ct1 := best_t1;
                    end if;

                    if t1_dir = 1 and (t1_cur + t1_step) <= t1_hi then
                        t1_cur <= t1_cur + t1_step;
                        st     <= S_MOVE;
                    elsif t1_dir = -1 and (t1_cur - t1_step) >= t1_lo then
                        t1_cur <= t1_cur - t1_step;
                        st     <= S_MOVE;
                    else
                        -- columna terminada: avanza phi
                        if (phi_cur + phi_step) <= phi_hi then
                            phi_cur <= phi_cur + phi_step;
                            if fine_ph = '1' then
                                -- FINO: barre SIEMPRE en el mismo sentido (theta1 t1_lo->t1_hi,
                                -- "de adelante a atras"); el flyback de theta1 se asienta en el
                                -- settle. Evita que theta2 (acoplado) deba recuperarse de la
                                -- reversion de la serpentina.
                                t1_cur <= t1_lo;
                                t1_dir <= 1;
                            else
                                t1_dir <= -t1_dir;   -- GRUESO: serpentina (mas rapido)
                            end if;
                            st      <= S_MOVE;
                        else
                            -- rejilla terminada
                            if fine_ph = '0' then
                                -- prepara rejilla FINA centrada en el mínimo (variables: sin carrera)
                                nlo := imax(PHI_MIN, cphi - COARSE_PHI_STEP);
                                nhi := imin(PHI_MAX, cphi + COARSE_PHI_STEP);
                                phi_lo <= nlo;  phi_hi <= nhi;  phi_cur <= nlo;
                                nlo := imax(T1_MIN, ct1 - COARSE_T1_STEP);
                                nhi := imin(T1_MAX, ct1 + COARSE_T1_STEP);
                                t1_lo  <= nlo;  t1_hi  <= nhi;  t1_cur <= nlo;
                                phi_step <= FINE_PHI_STEP;
                                t1_step  <= FINE_T1_STEP;
                                t1_dir   <= 1;
                                fine_ph  <= '1';
                                st       <= S_MOVE;
                            else
                                st <= S_DONE;
                            end if;
                        end if;
                    end if;

                -- Latchea el mínimo crudo a las salidas; marca found y scan_done
                when S_DONE =>
                    min_t1_r  <= std_logic_vector(to_unsigned(best_t1, 8));
                    min_d_r   <= std_logic_vector(best_d);
                    min_phi_r <= std_logic_vector(to_unsigned(best_phi, 8));
                    if best_d < to_unsigned(FOUND_TH, 16) then
                        found_r <= '1';
                    else
                        found_r <= '0';
                    end if;
                    scan_done_r <= '1';
                    st          <= S_IDLE;

            end case;
        end if;
    end process;

end rtl;
