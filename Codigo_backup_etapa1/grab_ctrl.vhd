-- ============================================================================
-- grab_ctrl - Orquesta el AGARRE tras el barrido del LIDAR.
-- FPGA: Cyclone II EP2C5T144C7 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Espera a que el barrido termine (scan_done), lanza la cinemática inversa
-- (polar_ik) sobre la coordenada del cubo (in_r,in_theta,in_phi), mueve el brazo
-- a esos ángulos (garra abierta), CIERRA la garra y levanta el hombro.
-- Multiplexa los ángulos hacia polarPWM: durante el barrido pasa los del LIDAR
-- (cmd_*); después, los de la fase de agarre.
--
-- Los tiempos son genéricos para poder simular rápido (la testbench los reduce).
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity grab_ctrl is
    generic (
        MOVE_CYCLES : integer := 125_000_000;   -- ~2.5 s para que el brazo llegue
        GRIP_CYCLES : integer := 40_000_000;     -- ~0.8 s para cerrar la garra
        LIFT_CYCLES : integer := 60_000_000      -- ~1.2 s para levantar
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;                      -- activo alto
        -- del LIDAR
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
        -- a polarPWM (multiplexado)
        phi_out     : out std_logic_vector(7 downto 0);
        theta1_out  : out std_logic_vector(7 downto 0);
        theta2_out  : out std_logic_vector(7 downto 0);
        theta3_out  : out std_logic_vector(7 downto 0);
        grip_out    : out std_logic;
        -- estado
        grip_closed : out std_logic;                      -- garra cerrada (cubo tomado)
        reachable   : out std_logic;                      -- el objetivo estaba en alcance
        done_all    : out std_logic                       -- secuencia completa
    );
end grab_ctrl;

architecture rtl of grab_ctrl is

    component polar_ik
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            start     : in  std_logic;
            r         : in  std_logic_vector(15 downto 0);
            theta     : in  std_logic_vector(8 downto 0);
            phi       : in  std_logic_vector(7 downto 0);
            o_phi     : out std_logic_vector(7 downto 0);
            o_theta1  : out std_logic_vector(7 downto 0);
            o_theta2  : out std_logic_vector(7 downto 0);
            o_theta3  : out std_logic_vector(7 downto 0);
            reachable : out std_logic;
            done      : out std_logic
        );
    end component;

    function imax3(a, b, c : integer) return integer is
        variable m : integer;
    begin
        m := a;
        if b > m then m := b; end if;
        if c > m then m := c; end if;
        return m;
    end function;
    constant TMR_MAX : integer := imax3(MOVE_CYCLES, GRIP_CYCLES, LIFT_CYCLES);

    signal ik_start : std_logic := '0';
    signal ik_done  : std_logic;
    signal reach    : std_logic;
    signal gphi, gt1, gt2, gt3 : std_logic_vector(7 downto 0);

    type gst_t is (G_IDLE, G_IK, G_IK_WAIT, G_MOVE, G_GRIP, G_RETRIEVE, G_DONE);
    signal gst : gst_t := G_IDLE;
    signal tmr : integer range 0 to TMR_MAX := 0;

    -- Pose de REPOSO/HOME: phi=180, theta1=90, theta2=0, theta3=0, garra CERRADA
    signal grab_phi : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(180, 8));
    signal grab_t1  : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(90, 8));
    signal grab_t2  : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(0, 8));
    signal grab_t3  : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(0, 8));
    signal grab_grip   : std_logic := '0';   -- reposo = CERRADA (0=cierra, 1=abre como TestBrazo)
    signal grip_cl_r   : std_logic := '0';
    signal reach_latch : std_logic := '0';
    signal done_r      : std_logic := '0';

begin

    u_ik : polar_ik
        port map (
            clk => clk, rst => rst, start => ik_start,
            r => in_r, theta => in_theta, phi => in_phi,
            o_phi => gphi, o_theta1 => gt1, o_theta2 => gt2, o_theta3 => gt3,
            reachable => reach, done => ik_done
        );

    grab_proc : process(clk, rst)
    begin
        if rst = '1' then
            gst <= G_IDLE; tmr <= 0;
            grab_phi <= std_logic_vector(to_unsigned(180, 8));   -- reposo: phi=180
            grab_t1  <= std_logic_vector(to_unsigned(90, 8));
            grab_t2  <= std_logic_vector(to_unsigned(0, 8));
            grab_t3  <= std_logic_vector(to_unsigned(0, 8));
            grab_grip <= '0'; grip_cl_r <= '0';   -- reposo: garra CERRADA (0=cierra)
            reach_latch <= '0'; done_r <= '0'; ik_start <= '0';
        elsif rising_edge(clk) then
            ik_start <= '0';
            case gst is

                when G_IDLE =>
                    if scan_done = '1' then
                        gst <= G_IK;
                    end if;

                when G_IK =>
                    ik_start  <= '1';
                    grab_grip <= '1';   -- abre la garra al terminar el barrido (mantener abierta hasta el agarre)
                    gst <= G_IK_WAIT;

                when G_IK_WAIT =>
                    if ik_done = '1' then
                        grab_phi <= gphi; grab_t1 <= gt1;
                        grab_t2  <= gt2;  grab_t3 <= gt3;
                        reach_latch <= reach;
                        grab_grip <= '1';                 -- garra ABIERTA para acercarse (1=abre)
                        tmr <= 0;
                        gst <= G_MOVE;
                    end if;

                when G_MOVE =>
                    if tmr >= MOVE_CYCLES-1 then
                        tmr <= 0; gst <= G_GRIP;
                    else
                        tmr <= tmr + 1;
                    end if;

                when G_GRIP =>
                    grab_grip <= '0'; grip_cl_r <= '1';   -- CIERRA sobre el cubo (0=cierra)
                    if tmr >= GRIP_CYCLES-1 then
                        tmr <= 0; gst <= G_RETRIEVE;
                    else
                        tmr <= tmr + 1;
                    end if;

                -- RECOGIDA: con el cubo agarrado, va a phi=180, theta1=135, resto 0 (garra CERRADA)
                when G_RETRIEVE =>
                    grab_phi <= std_logic_vector(to_unsigned(180, 8));
                    grab_t1  <= std_logic_vector(to_unsigned(135, 8));
                    grab_t2  <= std_logic_vector(to_unsigned(0, 8));
                    grab_t3  <= std_logic_vector(to_unsigned(0, 8));
                    if tmr >= LIFT_CYCLES-1 then
                        tmr <= 0; gst <= G_DONE;
                    else
                        tmr <= tmr + 1;
                    end if;

                when G_DONE =>
                    done_r <= '1';

            end case;
        end if;
    end process;

    -- MUX: barrido -> LIDAR; agarre -> ángulos de la IK
    phi_out    <= cmd_phi    when scan_active = '1' else grab_phi;
    theta1_out <= cmd_theta1 when scan_active = '1' else grab_t1;
    theta2_out <= cmd_theta2 when scan_active = '1' else grab_t2;
    theta3_out <= cmd_theta3 when scan_active = '1' else grab_t3;
    grip_out   <= cmd_grip   when scan_active = '1' else grab_grip;

    grip_closed <= grip_cl_r;
    reachable   <= reach_latch;
    done_all    <= done_r;

end rtl;
