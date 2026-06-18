-- ============================================================================
-- grab_ctrl - Orquestador del BRAZO para Sísifo (Etapa 2).
-- FPGA: Cyclone II EP2C5T144C7 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Ciclo de vida del brazo, multiplexando los ángulos hacia polarPWM:
--   REST (sin objeto, siguiendo línea: phi=180,t1=90,resto0, garra CERRADA)
--     -> al iniciarse un barrido (scan_active) espera a scan_done
--     -> si 'found' y la IK es alcanzable: agarra (kinematics -> mover -> cerrar)
--        y queda en HOLD (acarreo: phi=180,t1=135, garra cerrada, has_object=1)
--     -> con 'trigger_drop': DROP (gira phi=90, extiende, ABRE garra) -> REST
--   Si no hay objeto o no es alcanzable, vuelve a REST sin agarrar.
--
-- 'arm_ready' = 1 en REST(sin barrido) y en HOLD (la FSM espera este flanco).
-- La cinemática FK+IK la hace 'kinematics' (un CORDIC compartido).
-- Convención de garra (validada en TestBrazo): grip '1'=ABRE, '0'=CIERRA.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity grab_ctrl is
    generic (
        MOVE_CYCLES : integer := 125_000_000;
        GRIP_CYCLES : integer := 75_000_000;
        DROP_PHI    : integer := 100;
        DROP_T1     : integer := 45;
        DROP_T2     : integer := 45;
        DROP_T3     : integer := 0
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
end grab_ctrl;

architecture rtl of grab_ctrl is

    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    constant TMR_MAX : integer := imax(MOVE_CYCLES, GRIP_CYCLES);

    signal ik_start : std_logic := '0';
    signal ik_done  : std_logic;
    signal reach    : std_logic;
    signal gphi, gt1, gt2, gt3 : std_logic_vector(7 downto 0);

    type gst_t is (G_REST, G_SCANWAIT, G_IK, G_IK_WAIT, G_MOVE, G_GRIP, G_HOLD, G_DROP, G_RELEASE);
    signal gst : gst_t := G_REST;
    signal tmr : integer range 0 to TMR_MAX := 0;

    signal grab_phi  : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(180, 8));
    signal grab_t1   : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(100, 8));
    signal grab_t2   : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(10, 8));
    signal grab_t3   : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned(0, 8));
    signal grab_grip : std_logic := '0';
    signal has_obj_r   : std_logic := '0';
    signal reach_latch : std_logic := '0';

begin

    u_kin : entity work.kinematics
        port map (
            clk => clk, rst => rst, start => ik_start,
            in_t1 => min_t1, in_d => min_d, in_phi => min_phi,
            o_phi => gphi, o_theta1 => gt1, o_theta2 => gt2, o_theta3 => gt3,
            reachable => reach, done => ik_done
        );

    grab_proc : process(clk, rst)
    begin
        if rst = '1' then
            gst <= G_REST; tmr <= 0;
            grab_phi <= std_logic_vector(to_unsigned(180, 8));
            grab_t1  <= std_logic_vector(to_unsigned(90, 8));
            grab_t2  <= std_logic_vector(to_unsigned(0, 8));
            grab_t3  <= std_logic_vector(to_unsigned(0, 8));
            grab_grip <= '0'; has_obj_r <= '0'; reach_latch <= '0'; ik_start <= '0';
        elsif rising_edge(clk) then
            ik_start <= '0';
            case gst is

                when G_REST =>
                    grab_phi  <= std_logic_vector(to_unsigned(180, 8));
                    grab_t1   <= std_logic_vector(to_unsigned(100, 8));
                    grab_t2   <= std_logic_vector(to_unsigned(0, 8));
                    grab_t3   <= std_logic_vector(to_unsigned(0, 8));
                    grab_grip <= '0';
                    if scan_active = '1' then
                        gst <= G_SCANWAIT;
                    end if;

                when G_SCANWAIT =>
                    grab_grip <= '1';
                    if scan_done = '1' then
                        if found = '1' then
                            gst <= G_IK;
                        else
                            gst <= G_REST;
                        end if;
                    end if;

                when G_IK =>
                    ik_start <= '1';
                    gst <= G_IK_WAIT;

                when G_IK_WAIT =>
                    if ik_done = '1' then
                        if reach = '1' then
                            grab_phi  <= gphi; grab_t1 <= gt1;
                            grab_t2   <= gt2;  grab_t3 <= gt3;
                            grab_grip <= '1';
                            reach_latch <= '1';
                            tmr <= 0;
                            gst <= G_MOVE;
                        else
                            reach_latch <= '0';
                            gst <= G_REST;
                        end if;
                    end if;

                when G_MOVE =>
                    if tmr >= MOVE_CYCLES-1 then
                        tmr <= 0; gst <= G_GRIP;
                    else
                        tmr <= tmr + 1;
                    end if;

                when G_GRIP =>
                    grab_grip <= '0'; has_obj_r <= '1';
                    if tmr >= GRIP_CYCLES-1 then
                        tmr <= 0; gst <= G_HOLD;
                    else
                        tmr <= tmr + 1;
                    end if;

                when G_HOLD =>
                    grab_phi  <= std_logic_vector(to_unsigned(180, 8));
                    grab_t1   <= std_logic_vector(to_unsigned(100, 8));
                    grab_t2   <= std_logic_vector(to_unsigned(10, 8));
                    grab_t3   <= std_logic_vector(to_unsigned(0, 8));
                    grab_grip <= '0';
                    if trigger_drop = '1' then
                        tmr <= 0; gst <= G_DROP;
                    end if;

                when G_DROP =>
                    grab_phi  <= std_logic_vector(to_unsigned(DROP_PHI, 8));
                    grab_t1   <= std_logic_vector(to_unsigned(DROP_T1, 8));
                    grab_t2   <= std_logic_vector(to_unsigned(DROP_T2, 8));
                    grab_t3   <= std_logic_vector(to_unsigned(DROP_T3, 8));
                    grab_grip <= '0';
                    if tmr >= MOVE_CYCLES-1 then
                        tmr <= 0; gst <= G_RELEASE;
                    else
                        tmr <= tmr + 1;
                    end if;

                when G_RELEASE =>
                    grab_grip <= '1'; has_obj_r <= '0';
                    if tmr >= GRIP_CYCLES-1 then
                        tmr <= 0; gst <= G_REST;
                    else
                        tmr <= tmr + 1;
                    end if;

            end case;
        end if;
    end process;

    -- MUX: durante scan_active los servos los manda el LIDAR
    phi_out    <= cmd_phi    when scan_active = '1' else grab_phi;
    theta1_out <= cmd_theta1 when scan_active = '1' else grab_t1;
    theta2_out <= cmd_theta2 when scan_active = '1' else grab_t2;
    theta3_out <= cmd_theta3 when scan_active = '1' else grab_t3;
    grip_out   <= cmd_grip   when scan_active = '1' else grab_grip;

    has_object <= has_obj_r;
    reachable  <= reach_latch;
    arm_ready  <= '1' when (gst = G_REST and scan_active = '0') or gst = G_HOLD else '0';

end rtl;
