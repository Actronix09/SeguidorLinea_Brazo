-- ============================================================================
-- kinematics - Cinemática FUSIONADA del brazo (reemplaza polar_kin + polar_ik).
-- FPGA: Cyclone II EP2C5T144C7 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- De golpe, a partir del punto más cercano del barrido (theta1*, d*, phi*) calcula
-- los ángulos de servo (theta1, theta2, theta3) que llevan la GARRA al objeto con
-- aproximación vertical hacia ABAJO. Internamente hace, en secuencia:
--
--   (1) DIRECTA (FK): (theta1*, d*) -> coordenada polar (r, theta) respecto al eje
--       theta1.  dradial = L1*cos(t1)+L2 ;  dz = L1*sin(t1) - (L3+d)
--       r = hypot(dradial,dz) ; theta = atan2(dz,dradial)
--   (2) INVERSA (IK): (r, theta) -> ángulos.  rw = r*cos(theta) - R_TRIM ;
--       zw = r*sin(theta) - Z_DROP + L_GRIP ;  D = hypot(rw,zw) ;
--       beta = atan2(zw,rw) ; psi = arccos(D/200) ; theta1 = beta+psi (codo arriba) ;
--       alfa2 = atan2(zw-L1*sin(t1), rw-L1*cos(t1)) ;
--       theta2 = alfa2 - theta1 + 90 ; theta3 = ALFA3_TGT - alfa2 + 90.
--   phi se pasa directo (azimut).
--
-- ÁREA: UN solo CORDIC vectoring (datapath signed 24b, NITER=14) compartido por las
-- TRES pasadas (FK + 2 de la IK), reutilizado vía 'cordic_ret'. UNA sola ROM cos/sin
-- (-90..180) y una ROM arccos, con SALIDA REGISTRADA -> Quartus las mapea a M4K.
-- Sustituye a polar_kin (que traía su propio CORDIC + LUT cos/sin) y a polar_ik.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity kinematics is
    generic (
        L1        : integer := 100;     -- mm (eje theta1 -> eje theta2)
        L2        : integer := 100;     -- mm (eje theta2 -> eje theta3)
        L3        : integer := 43;      -- mm (eje theta3 -> sensor)
        L_GRIP    : integer := 90;      -- mm (eje theta3 -> punta de la garra)
        ALFA3_TGT : integer := -90;     -- grados: orientacion absoluta de la garra (-90=abajo)
        Z_DROP    : integer := 60;      -- mm: descenso bajo la cima del objeto
        R_TRIM    : integer := 15;      -- mm: recorte radial del objetivo
        PHI_TRIM  : integer := -1       -- grados: correccion offset lateral del sensor
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        start     : in  std_logic;
        in_t1     : in  std_logic_vector(7 downto 0);
        in_d      : in  std_logic_vector(15 downto 0);
        in_phi    : in  std_logic_vector(7 downto 0);
        o_phi     : out std_logic_vector(7 downto 0);
        o_theta1  : out std_logic_vector(7 downto 0);
        o_theta2  : out std_logic_vector(7 downto 0);
        o_theta3  : out std_logic_vector(7 downto 0);
        reachable : out std_logic;
        done      : out std_logic
    );
end kinematics;

architecture rtl of kinematics is

    constant NITER    : integer := 14;
    constant INV_GAIN : integer := 2487;     -- 1/(16*K) en Q16 (K = ganancia CORDIC)
    constant REACH    : integer := L1 + L2;
    constant ANG_LO   : integer := -90;
    constant ANG_HI   : integer := 180;
    constant ANG_N    : integer := ANG_HI - ANG_LO;

    type trig_rom_t is array(0 to ANG_N) of signed(13 downto 0);
    function build_cos return trig_rom_t is
        variable l : trig_rom_t;
    begin
        for i in 0 to ANG_N loop
            l(i) := to_signed(integer(round(cos(real(i+ANG_LO)*MATH_PI/180.0)*4096.0)), 14);
        end loop;
        return l;
    end function;
    function build_sin return trig_rom_t is
        variable l : trig_rom_t;
    begin
        for i in 0 to ANG_N loop
            l(i) := to_signed(integer(round(sin(real(i+ANG_LO)*MATH_PI/180.0)*4096.0)), 14);
        end loop;
        return l;
    end function;
    constant COS_ROM : trig_rom_t := build_cos;
    constant SIN_ROM : trig_rom_t := build_sin;

    type acos_rom_t is array(0 to REACH) of unsigned(7 downto 0);
    function build_acos return acos_rom_t is
        variable l : acos_rom_t;
    begin
        for d in 0 to REACH loop
            l(d) := to_unsigned(integer(round(arccos(real(d)/real(REACH))*180.0/MATH_PI)), 8);
        end loop;
        return l;
    end function;
    constant ACOS_ROM : acos_rom_t := build_acos;

    type atan_t is array(0 to NITER-1) of integer;
    function build_atan return atan_t is
        variable l : atan_t;
    begin
        for i in 0 to NITER-1 loop
            l(i) := integer(round(arctan(2.0**(-i)) * 180.0/MATH_PI * 256.0));
        end loop;
        return l;
    end function;
    constant ATAN_LUT : atan_t := build_atan;

    function idx_of(a : integer) return integer is
        variable i : integer;
    begin
        i := a - ANG_LO;
        if i < 0 then i := 0; elsif i > ANG_N then i := ANG_N; end if;
        return i;
    end function;
    function clamp180(a : integer) return integer is
    begin
        if a < 0 then return 0; elsif a > 180 then return 180; else return a; end if;
    end function;
    function q8_round(a : integer) return integer is
    begin
        if a >= 0 then return (a + 128) / 256; else return -(((-a) + 128) / 256); end if;
    end function;

    signal trig_idx  : integer range 0 to ANG_N := 0;
    signal acos_idx  : integer range 0 to REACH := 0;
    signal cosrom_q  : signed(13 downto 0) := (others => '0');
    signal sinrom_q  : signed(13 downto 0) := (others => '0');
    signal acosrom_q : unsigned(7 downto 0) := (others => '0');

    type st_t is (S_IDLE,
                  S_FK_W, S_FK, S_FK_POST,
                  S_IK_SET, S_IK_W, S_IK, S_ACOS_SET, S_ACOS_W,
                  S_THETA1, S_LUT2_W, S_ELBOW, S_POST2,
                  S_CORDIC, S_DONE);
    signal st         : st_t := S_IDLE;
    signal cordic_ret : st_t := S_FK_POST;

    signal xi : signed(23 downto 0) := (others => '0');
    signal yi : signed(23 downto 0) := (others => '0');
    signal zi : signed(23 downto 0) := (others => '0');
    signal it : integer range 0 to NITER := 0;

    signal r_fk : integer range -2048 to 2048 := 0;
    signal th_fk: integer range -360 to 360 := 0;
    signal rw   : integer range -1024 to 1024 := 0;
    signal zw   : integer range -1024 to 1024 := 0;
    signal t1d  : integer range -360 to 360 := 0;
    signal phi_l: std_logic_vector(7 downto 0) := (others => '0');

    signal o_t1_r : std_logic_vector(7 downto 0) := (others => '0');
    signal o_t2_r : std_logic_vector(7 downto 0) := (others => '0');
    signal o_t3_r : std_logic_vector(7 downto 0) := (others => '0');
    signal reach_r: std_logic := '1';
    signal dn     : std_logic := '0';

begin

    o_phi     <= phi_l;
    o_theta1  <= o_t1_r;
    o_theta2  <= o_t2_r;
    o_theta3  <= o_t3_r;
    reachable <= reach_r;
    done      <= dn;

    -- ROM con salida registrada (M4K): cos/sin compartidas + arccos
    rom_proc : process(clk)
    begin
        if rising_edge(clk) then
            cosrom_q  <= COS_ROM(trig_idx);
            sinrom_q  <= SIN_ROM(trig_idx);
            acosrom_q <= ACOS_ROM(acos_idx);
        end if;
    end process;

    fsm : process(clk, rst)
        variable M       : integer;
        variable dradial : integer;
        variable dzv     : integer;
        variable rt, zt, zwv : integer;
        variable Dmm, dcl    : integer;
        variable a1_q8       : integer;
        variable alfa2       : integer;
        variable ex, ey      : integer;
        variable dx, dy      : integer;
        variable rr          : signed(39 downto 0);
        variable xv, yv      : signed(23 downto 0);
        variable sx, sy      : signed(23 downto 0);
    begin
        if rst = '1' then
            st <= S_IDLE; dn <= '0'; it <= 0;
            xi <= (others => '0'); yi <= (others => '0'); zi <= (others => '0');
            o_t1_r <= (others => '0'); o_t2_r <= (others => '0'); o_t3_r <= (others => '0');
            reach_r <= '1'; trig_idx <= 0; acos_idx <= 0;
        elsif rising_edge(clk) then
            case st is

                when S_IDLE =>
                    dn <= '0';
                    if start = '1' then
                        phi_l    <= std_logic_vector(to_unsigned(
                                        clamp180(to_integer(unsigned(in_phi)) + PHI_TRIM), 8));
                        reach_r  <= '1';
                        trig_idx <= idx_of(to_integer(unsigned(in_t1)));
                        st <= S_FK_W;
                    end if;

                when S_FK_W =>                  -- burbuja: espera la ROM
                    st <= S_FK;

                when S_FK =>
                    M       := to_integer(unsigned(in_d)) + L3;
                    dradial := (L1 * to_integer(cosrom_q)) / 4096 + L2;
                    dzv     := (L1 * to_integer(sinrom_q)) / 4096 - M;
                    xi <= to_signed(dradial * 16, 24);
                    yi <= to_signed(dzv * 16, 24);
                    zi <= (others => '0');
                    it <= 0;
                    cordic_ret <= S_FK_POST;
                    st <= S_CORDIC;

                when S_FK_POST =>
                    rr   := xi * to_signed(INV_GAIN, 16);
                    r_fk  <= to_integer(shift_right(rr, 16));
                    th_fk <= q8_round(to_integer(zi));
                    st <= S_IK_SET;

                when S_IK_SET =>
                    trig_idx <= idx_of(th_fk);
                    st <= S_IK_W;

                when S_IK_W =>                  -- burbuja: espera la ROM
                    st <= S_IK;

                when S_IK =>
                    rt  := (r_fk * to_integer(cosrom_q)) / 4096 - R_TRIM;
                    zt  := (r_fk * to_integer(sinrom_q)) / 4096;
                    zwv := zt - Z_DROP + L_GRIP;
                    rw <= rt;  zw <= zwv;
                    xi <= to_signed(rt  * 16, 24);
                    yi <= to_signed(zwv * 16, 24);
                    zi <= (others => '0');
                    it <= 0;
                    cordic_ret <= S_ACOS_SET;
                    st <= S_CORDIC;

                when S_ACOS_SET =>
                    rr  := xi * to_signed(INV_GAIN, 16);
                    Dmm := to_integer(shift_right(rr, 16));
                    if Dmm > REACH then
                        dcl := REACH;  reach_r <= '0';
                    elsif Dmm < 0 then
                        dcl := 0;
                    else
                        dcl := Dmm;
                    end if;
                    acos_idx <= dcl;
                    st <= S_ACOS_W;

                when S_ACOS_W =>                -- burbuja: espera la ROM
                    st <= S_THETA1;

                when S_THETA1 =>
                    a1_q8 := to_integer(zi) + to_integer(acosrom_q) * 256;
                    t1d   <= q8_round(a1_q8);
                    trig_idx <= idx_of(q8_round(a1_q8));
                    st <= S_LUT2_W;

                when S_LUT2_W =>                -- burbuja: espera la ROM
                    st <= S_ELBOW;

                when S_ELBOW =>
                    ex := (L1 * to_integer(cosrom_q)) / 4096;
                    ey := (L1 * to_integer(sinrom_q)) / 4096;
                    dx := rw - ex;
                    dy := zw - ey;
                    xi <= to_signed(dx * 16, 24);
                    yi <= to_signed(dy * 16, 24);
                    zi <= (others => '0');
                    it <= 0;
                    cordic_ret <= S_POST2;
                    st <= S_CORDIC;

                when S_POST2 =>
                    alfa2 := q8_round(to_integer(zi));
                    o_t1_r <= std_logic_vector(to_unsigned(clamp180(t1d), 8));
                    o_t2_r <= std_logic_vector(to_unsigned(clamp180(alfa2 - t1d + 90), 8));
                    o_t3_r <= std_logic_vector(to_unsigned(clamp180(ALFA3_TGT - alfa2 + 90), 8));
                    st <= S_DONE;

                when S_CORDIC =>
                    xv := xi;  yv := yi;
                    sx := shift_right(xv, it);
                    sy := shift_right(yv, it);
                    if yv >= 0 then
                        xi <= xv + sy;  yi <= yv - sx;
                        zi <= zi + to_signed(ATAN_LUT(it), 24);
                    else
                        xi <= xv - sy;  yi <= yv + sx;
                        zi <= zi - to_signed(ATAN_LUT(it), 24);
                    end if;
                    if it = NITER-1 then
                        st <= cordic_ret;
                    else
                        it <= it + 1;
                    end if;

                when S_DONE =>
                    dn <= '1';
                    st <= S_IDLE;

            end case;
        end if;
    end process;

end rtl;
