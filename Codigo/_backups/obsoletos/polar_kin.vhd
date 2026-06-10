-- ============================================================================
-- polar_kin - Convierte (theta1*, d*) del punto más cercano a coordenada polar
--             (r, theta) respecto al eje de theta1. Se ejecuta UNA vez al final
--             del barrido del escáner LIDAR.
-- FPGA: Cyclone II EP2C5T144C7 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Patrón de barrido: theta1 (90->45) y theta2 (0->45) ACOPLADOS (theta2=90-theta1)
-- para mantener el eslabón L2 (theta2->theta3) HORIZONTAL. theta3 queda FIJO con
-- el haz apuntando hacia ABAJO (alfa3 = -90°), así que el haz baja vertical sobre
-- la mesa y se traslada con theta1.
--
-- Modelo (origen = eje de theta1; L1=L2=100 mm, L3=63 mm; M = L3 + d):
--   dradial = L1*cos(theta1) + L2                 (sólo depende de theta1; cos(alfa3)=0)
--   dz      = L1*sin(theta1) - M                  (signed, negativo: objeto bajo el eje)
--   r       = sqrt(dradial^2 + dz^2)              (distancia desde el eje theta1)
--   theta   = atan2(dz, dradial)                  (elevación, negativa hacia abajo)
-- cos/sin por LUT (Q12); r y theta por CORDIC vectoring (datapath de 24 bits).
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity polar_kin is
    generic (
        L1 : integer := 100;    -- mm (eje theta1 -> eje theta2)
        L2 : integer := 100;    -- mm (eje theta2 -> eje theta3)
        L3 : integer := 63      -- mm (eje theta3 -> cara sensor), 62.7 redondeado
    );
    port (
        clk    : in  std_logic;
        rst    : in  std_logic;                      -- activo alto
        start  : in  std_logic;                      -- pulso: inicia cálculo
        theta1 : in  std_logic_vector(7 downto 0);   -- grados 45..90 (variable de barrido)
        d      : in  std_logic_vector(15 downto 0);  -- mm (distancia promediada, haz vertical)
        r      : out std_logic_vector(15 downto 0);  -- mm desde el eje theta1
        theta  : out std_logic_vector(8 downto 0);   -- elevación en grados (signed)
        done   : out std_logic
    );
end polar_kin;

architecture rtl of polar_kin is

    constant NITER : integer := 14;

    -- LUT cos/sin de theta1, Q12 (x4096), calculadas en elaboración (0..90 grados)
    type lut12_t is array(0 to 90) of integer;
    function build_cos return lut12_t is
        variable l : lut12_t;
    begin
        for t in 0 to 90 loop
            l(t) := integer(round(cos(real(t)*MATH_PI/180.0)*4096.0));
        end loop;
        return l;
    end function;
    function build_sin return lut12_t is
        variable l : lut12_t;
    begin
        for t in 0 to 90 loop
            l(t) := integer(round(sin(real(t)*MATH_PI/180.0)*4096.0));
        end loop;
        return l;
    end function;
    constant COS_LUT : lut12_t := build_cos;
    constant SIN_LUT : lut12_t := build_sin;

    -- LUT atan(2^-i) en grados Q8 (x256) para el CORDIC
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

    -- 1/(16*K) en Q16, K = ganancia CORDIC (~1.64676)  -> 0.037954*65536 ≈ 2487
    constant INV_GAIN : integer := 2487;

    type st_t is (K_IDLE, K_ITER, K_POST, K_DONE);
    signal st   : st_t := K_IDLE;
    signal xi   : signed(23 downto 0) := (others => '0');
    signal yi   : signed(23 downto 0) := (others => '0');
    signal zi   : signed(23 downto 0) := (others => '0');
    signal it   : integer range 0 to NITER := 0;
    signal r_o  : std_logic_vector(15 downto 0) := (others => '0');
    signal th_o : std_logic_vector(8 downto 0)  := (others => '0');
    signal dn   : std_logic := '0';

begin

    r     <= r_o;
    theta <= th_o;
    done  <= dn;

    process(clk, rst)
        variable ti      : integer;
        variable M       : integer;
        variable dradial : integer;
        variable dz      : integer;
        variable xv, yv  : signed(23 downto 0);
        variable sx, sy  : signed(23 downto 0);
        variable rr      : signed(39 downto 0);
    begin
        if rst = '1' then
            st   <= K_IDLE;
            dn   <= '0';
            it   <= 0;
            xi   <= (others => '0');
            yi   <= (others => '0');
            zi   <= (others => '0');
            r_o  <= (others => '0');
            th_o <= (others => '0');
        elsif rising_edge(clk) then
            case st is
                when K_IDLE =>
                    dn <= '0';
                    if start = '1' then
                        ti := to_integer(unsigned(theta1));
                        if ti > 90 then ti := 90; end if;     -- clamp al rango de la LUT
                        M       := to_integer(unsigned(d)) + L3;
                        -- haz vertical (alfa3 = -90): cos(alfa3)=0, sin(alfa3)=-1
                        dradial := (L1 * COS_LUT(ti)) / 4096 + L2;
                        dz      := (L1 * SIN_LUT(ti)) / 4096 - M;
                        -- escala Q4 (x16) para precisión del CORDIC
                        xi <= to_signed(dradial * 16, 24);
                        yi <= to_signed(dz * 16, 24);
                        zi <= (others => '0');
                        it <= 0;
                        st <= K_ITER;
                    end if;

                when K_ITER =>
                    xv := xi;
                    yv := yi;
                    sx := shift_right(xv, it);
                    sy := shift_right(yv, it);
                    if yv >= 0 then
                        xi <= xv + sy;
                        yi <= yv - sx;
                        zi <= zi + to_signed(ATAN_LUT(it), 24);
                    else
                        xi <= xv - sy;
                        yi <= yv + sx;
                        zi <= zi - to_signed(ATAN_LUT(it), 24);
                    end if;
                    if it = NITER-1 then
                        st <= K_POST;
                    else
                        it <= it + 1;
                    end if;

                when K_POST =>
                    -- r = x_final * (1/(16*K))  ->  (xi * INV_GAIN) >> 16
                    rr   := xi * to_signed(INV_GAIN, 16);
                    r_o  <= std_logic_vector(resize(shift_right(rr, 16), 16));
                    -- theta = z(gradosQ8) -> grados con redondeo
                    th_o <= std_logic_vector(resize(shift_right(zi + to_signed(128, 24), 8), 9));
                    st   <= K_DONE;

                when K_DONE =>
                    dn <= '1';
                    st <= K_IDLE;
            end case;
        end if;
    end process;

end rtl;
