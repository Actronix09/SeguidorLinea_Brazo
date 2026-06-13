-- ============================================================================
-- polar_ik - Cinemática INVERSA: de la coordenada polar del objeto (r, theta, phi)
--            respecto al eje theta1, calcula los ángulos de servo (theta1, theta2,
--            theta3) que llevan la GARRA al objeto con aproximación hacia ABAJO
--            (eslabón final vertical, alfa3 = -90°). phi se pasa directo (azimut).
-- FPGA: Cyclone II EP2C5T144C7 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Geometría (brazo planar de 2 eslabones L1,L2 + muñeca L_grip; origen = eje theta1):
--   Punto objetivo (cubo):  radial_t = r*cos(theta);  z_t = r*sin(theta)
--   Garra vertical hacia abajo -> la muñeca (eje theta3) queda L_grip ARRIBA del
--   objetivo:  rw = radial_t ;  zw = z_t + L_grip
--   2 eslabones iguales (L1=L2):  D = |muñeca| = sqrt(rw^2+zw^2)
--     beta = atan2(zw, rw)
--     psi  = arccos(D / (2*L1)) = arccos(D/200)          (triángulo isósceles)
--     theta1 = beta + psi                                 (codo arriba)
--     alfa2  = atan2(zw - L1*sin(theta1), rw - L1*cos(theta1))   (ángulo abs. de L2)
--     theta2 = alfa2 - theta1 + 90
--     theta3 = ALFA3_TGT - alfa2 + 90        (ALFA3_TGT = -90 -> garra hacia abajo)
-- atan2 por CORDIC vectoring (signed, iterativo, reutilizado 2 veces).
-- ----------------------------------------------------------------------------
-- ÁREA: las LUT cos/sin (rango -90..180) y arccos se implementan como ROM con
-- SALIDA REGISTRADA -> Quartus las mapea a bloques M4K (no a LEs). Una sola ROM
-- cos/sin compartida, direccionada por el FSM (se consulta 2 veces, en distintos
-- estados), en vez de 4 multiplexores ROM en lógica.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity polar_ik is
    generic (
        L1        : integer := 100;     -- mm (eje theta1 -> eje theta2)
        L2        : integer := 100;     -- mm (eje theta2 -> eje theta3); IK asume L1=L2
        L_GRIP    : integer := 90;      -- mm (eje theta3 -> punta de la garra): 62.7 (sensor) + 27.5 (offset sensor->pinza)
        ALFA3_TGT : integer := -90;     -- grados: orientación absoluta de la garra (-90 = abajo)
        Z_DROP    : integer := 35;      -- mm: baja la garra bajo la cima del cubo (quedaba ~1 cm arriba con 25)
        R_TRIM    : integer := 20       -- mm: recorta el radial del objetivo (la garra quedaba ~2 cm adelantada)
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;                      -- activo alto
        start     : in  std_logic;                      -- pulso: inicia cálculo
        r         : in  std_logic_vector(15 downto 0);  -- mm desde el eje theta1
        theta     : in  std_logic_vector(8 downto 0);   -- elevación, grados (signed)
        phi       : in  std_logic_vector(7 downto 0);   -- azimut, grados
        o_phi     : out std_logic_vector(7 downto 0);   -- = phi
        o_theta1  : out std_logic_vector(7 downto 0);   -- grados servo 0..180
        o_theta2  : out std_logic_vector(7 downto 0);
        o_theta3  : out std_logic_vector(7 downto 0);
        reachable : out std_logic;                      -- '0' si el objetivo está fuera de alcance
        done      : out std_logic
    );
end polar_ik;

architecture rtl of polar_ik is

    constant NITER    : integer := 14;
    constant INV_GAIN : integer := 2487;     -- 1/(16*K) en Q16 (K = ganancia CORDIC)
    constant REACH    : integer := L1 + L2;  -- alcance máximo de la muñeca (=200)
    constant ANG_LO   : integer := -90;
    constant ANG_HI   : integer := 180;
    constant ANG_N    : integer := ANG_HI - ANG_LO;   -- 270

    -- ROM cos/sin (Q12) para ángulos -90..180; índice = ángulo - ANG_LO
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

    -- ROM arccos(D/REACH) en grados enteros (0..90), índice D = 0..REACH
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

    -- LUT atan(2^-i) en grados Q8 (x256) para el CORDIC (pequeña, se deja en lógica)
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

    -- ROM con salida registrada -> M4K
    signal trig_idx  : integer range 0 to ANG_N := 0;
    signal acos_idx  : integer range 0 to REACH := 0;
    signal cosrom_q  : signed(13 downto 0) := (others => '0');
    signal sinrom_q  : signed(13 downto 0) := (others => '0');
    signal acosrom_q : unsigned(7 downto 0) := (others => '0');

    type st_t is (I_IDLE, I_PREP_W, I_PREP, I_CORDIC, I_ACOS_SET, I_ACOS_W,
                  I_THETA1, I_LUT2_W, I_ELBOW, I_POST2, I_DONE);
    signal st        : st_t := I_IDLE;
    signal cordic_ret: st_t := I_ACOS_SET;

    signal xi : signed(23 downto 0) := (others => '0');
    signal yi : signed(23 downto 0) := (others => '0');
    signal zi : signed(23 downto 0) := (others => '0');
    signal it : integer range 0 to NITER := 0;

    signal rw   : integer range -1024 to 1024 := 0;   -- muñeca radial (mm)
    signal zw   : integer range -1024 to 1024 := 0;   -- muñeca z (mm)
    signal t1d  : integer range -360 to 360 := 0;     -- theta1 (grados)
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
            st <= I_IDLE; dn <= '0'; it <= 0;
            xi <= (others => '0'); yi <= (others => '0'); zi <= (others => '0');
            o_t1_r <= (others => '0'); o_t2_r <= (others => '0'); o_t3_r <= (others => '0');
            reach_r <= '1'; trig_idx <= 0; acos_idx <= 0;
        elsif rising_edge(clk) then
            case st is

                when I_IDLE =>
                    dn <= '0';
                    if start = '1' then
                        phi_l   <= phi;
                        reach_r <= '1';
                        trig_idx <= idx_of(to_integer(signed(theta)));   -- pide cos/sin(theta)
                        st <= I_PREP_W;
                    end if;

                when I_PREP_W =>                       -- burbuja: espera la ROM
                    st <= I_PREP;

                when I_PREP =>                         -- cos/sin(theta) listos
                    rt  := (to_integer(unsigned(r)) * to_integer(cosrom_q)) / 4096 - R_TRIM;  -- radial recortado
                    zt  := (to_integer(unsigned(r)) * to_integer(sinrom_q)) / 4096;
                    zwv := zt - Z_DROP + L_GRIP;        -- muñeca L_grip arriba de la punta
                    rw <= rt;  zw <= zwv;
                    xi <= to_signed(rt  * 16, 24);
                    yi <= to_signed(zwv * 16, 24);
                    zi <= (others => '0');
                    it <= 0;
                    cordic_ret <= I_ACOS_SET;
                    st <= I_CORDIC;

                when I_CORDIC =>                       -- CORDIC vectoring compartido
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

                when I_ACOS_SET =>                     -- D -> pide arccos(D/REACH)
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
                    st <= I_ACOS_W;

                when I_ACOS_W =>                       -- burbuja: espera la ROM
                    st <= I_THETA1;

                when I_THETA1 =>                       -- theta1 = beta + psi; pide cos/sin(theta1)
                    a1_q8 := to_integer(zi) + to_integer(acosrom_q) * 256;
                    t1d   <= q8_round(a1_q8);
                    trig_idx <= idx_of(q8_round(a1_q8));
                    st <= I_LUT2_W;

                when I_LUT2_W =>                       -- burbuja: espera la ROM
                    st <= I_ELBOW;

                when I_ELBOW =>                        -- codo -> arranca CORDIC #2 (alfa2)
                    ex := (L1 * to_integer(cosrom_q)) / 4096;
                    ey := (L1 * to_integer(sinrom_q)) / 4096;
                    dx := rw - ex;
                    dy := zw - ey;
                    xi <= to_signed(dx * 16, 24);
                    yi <= to_signed(dy * 16, 24);
                    zi <= (others => '0');
                    it <= 0;
                    cordic_ret <= I_POST2;
                    st <= I_CORDIC;

                when I_POST2 =>                        -- alfa2 -> theta2, theta3
                    alfa2 := q8_round(to_integer(zi));
                    o_t1_r <= std_logic_vector(to_unsigned(clamp180(t1d), 8));
                    o_t2_r <= std_logic_vector(to_unsigned(clamp180(alfa2 - t1d + 90), 8));
                    o_t3_r <= std_logic_vector(to_unsigned(clamp180(ALFA3_TGT - alfa2 + 90), 8));
                    st <= I_DONE;

                when I_DONE =>
                    dn <= '1';
                    st <= I_IDLE;

            end case;
        end if;
    end process;

end rtl;
