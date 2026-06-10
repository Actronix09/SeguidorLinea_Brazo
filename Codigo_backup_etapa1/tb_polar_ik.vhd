-- ============================================================================
-- tb_polar_ik - Verifica la cinemática inversa por IDA Y VUELTA:
--   IK convierte (r, theta, phi) -> (theta1, theta2, theta3); la testbench aplica
--   la cinemática DIRECTA a esos ángulos y comprueba que la punta de la garra
--   cae sobre el punto objetivo (rt = r*cos theta, zt = r*sin theta), y que todos
--   los ángulos quedan en el rango de servo [0,180].
--
--   vlib work
--   vcom -2008 polar_ik.vhd tb_polar_ik.vhd
--   vsim tb_polar_ik -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

entity tb_polar_ik is
end tb_polar_ik;

architecture sim of tb_polar_ik is

    component polar_ik
        generic (L1 : integer := 100; L2 : integer := 100; L_GRIP : integer := 90;
                 ALFA3_TGT : integer := -90; Z_DROP : integer := 0);
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

    constant L1 : real := 100.0;
    constant L2 : real := 100.0;
    constant LG : real := 90.0;   -- = L_GRIP del DUT (62.7 sensor + 27.5 offset)

    signal clk   : std_logic := '0';
    signal rst   : std_logic := '1';
    signal start : std_logic := '0';
    signal r     : std_logic_vector(15 downto 0) := (others => '0');
    signal theta : std_logic_vector(8 downto 0)  := (others => '0');
    signal phi   : std_logic_vector(7 downto 0)  := (others => '0');
    signal o_phi : std_logic_vector(7 downto 0);
    signal o_t1  : std_logic_vector(7 downto 0);
    signal o_t2  : std_logic_vector(7 downto 0);
    signal o_t3  : std_logic_vector(7 downto 0);
    signal reach : std_logic;
    signal done  : std_logic;
    signal simdone : boolean := false;

    procedure run_case(signal clk_s   : in  std_logic;
                       signal st_s     : out std_logic;
                       signal r_s      : out std_logic_vector(15 downto 0);
                       signal th_s     : out std_logic_vector(8 downto 0);
                       signal phi_s    : out std_logic_vector(7 downto 0);
                       signal dn_s     : in  std_logic;
                       rv : integer; thv : integer; phv : integer) is
    begin
        r_s   <= std_logic_vector(to_unsigned(rv, 16));
        th_s  <= std_logic_vector(to_signed(thv, 9));
        phi_s <= std_logic_vector(to_unsigned(phv, 8));
        wait until rising_edge(clk_s);
        st_s <= '1';
        wait until rising_edge(clk_s);
        st_s <= '0';
        wait until dn_s = '1';
        wait until rising_edge(clk_s);
    end procedure;

begin

    dut : polar_ik
        port map (clk => clk, rst => rst, start => start,
                  r => r, theta => theta, phi => phi,
                  o_phi => o_phi, o_theta1 => o_t1, o_theta2 => o_t2,
                  o_theta3 => o_t3, reachable => reach, done => done);

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns;
            clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    stim : process
        variable t1, t2, t3 : integer;
        variable a1, a2, a3 : real;
        variable ex, ey, wx, wy, tipx, tipz : real;
        variable rt, zt, err : real;

        procedure check(rv, thv, phv : integer; tol : real; name : string) is
        begin
            run_case(clk, start, r, theta, phi, done, rv, thv, phv);
            t1 := to_integer(unsigned(o_t1));
            t2 := to_integer(unsigned(o_t2));
            t3 := to_integer(unsigned(o_t3));
            -- cinemática directa de los ángulos resultantes
            a1 := real(t1) * MATH_PI/180.0;
            a2 := real(t1 + t2 - 90) * MATH_PI/180.0;
            a3 := real(t1 + t2 + t3 - 180) * MATH_PI/180.0;
            ex := L1*cos(a1);              ey := L1*sin(a1);
            wx := ex + L2*cos(a2);         wy := ey + L2*sin(a2);
            tipx := wx + LG*cos(a3);       tipz := wy + LG*sin(a3);
            rt := real(rv) * cos(real(thv)*MATH_PI/180.0);
            zt := real(rv) * sin(real(thv)*MATH_PI/180.0);
            err := sqrt((tipx-rt)**2 + (tipz-zt)**2);
            report name & ": (r=" & integer'image(rv) & ", th=" & integer'image(thv) &
                   ", phi=" & integer'image(phv) & ") -> t1=" & integer'image(t1) &
                   " t2=" & integer'image(t2) & " t3=" & integer'image(t3) &
                   " | objetivo=(" & integer'image(integer(rt)) & "," & integer'image(integer(zt)) &
                   ") punta=(" & integer'image(integer(tipx)) & "," & integer'image(integer(tipz)) &
                   ") err=" & integer'image(integer(err)) & " mm, reach=" & std_logic'image(reach)
                   severity note;
            assert o_phi = std_logic_vector(to_unsigned(phv, 8))
                report name & ": o_phi != phi" severity error;
            assert (t1 >= 0 and t1 <= 180 and t2 >= 0 and t2 <= 180 and t3 >= 0 and t3 <= 180)
                report name & ": algún ángulo fuera de [0,180]" severity error;
            assert err < tol
                report name & ": FK(IK) lejos del objetivo (err=" & integer'image(integer(err)) & " mm)"
                severity error;
        end procedure;
    begin
        rst <= '1'; wait for 100 ns; rst <= '0';
        wait until rising_edge(clk);

        -- Casos dentro de la región del barrido (cubo sobre la mesa, al frente)
        check(168, -27,  90, 7.0, "Caso1");   -- el cubo del tb del escáner
        check(185, -32, 120, 7.0, "Caso2");
        check(150, -20,  60, 7.0, "Caso3");

        -- Objetivo fuera de alcance (debe bajar reachable)
        run_case(clk, start, r, theta, phi, done, 400, 0, 90);
        assert reach = '0'
            report "FALLO: objetivo lejano no marcó fuera de alcance" severity error;
        report "Caso4: r=400 -> reachable=" & std_logic'image(reach) & " (esperado '0')" severity note;

        report "OK: la cinemática inversa cierra (FK(IK)=objetivo) y respeta [0,180]" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
