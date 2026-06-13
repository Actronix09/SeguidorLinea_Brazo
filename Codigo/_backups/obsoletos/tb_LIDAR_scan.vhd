-- ============================================================================
-- tb_LIDAR_scan - Testbench autocomprobante del escáner LIDAR.
-- ----------------------------------------------------------------------------
-- Modelo de esclavo I2C del VL53L0X cuya distancia devuelta DEPENDE de los
-- ángulos que el escáner comanda (cmd_phi, cmd_theta1, leídos como puertos del
-- DUT). El barrido mueve theta1 (con theta2=90-theta1, L2 horizontal) y theta3
-- queda FIJO=0 (haz hacia abajo). Se sintetiza un "cuenco" con mínimo conocido
-- en (phi0=90, t10=60):
--      raw = 50 (offset) + 100 + 4*|phi-90| + 2*|theta1-60|   [mm]
-- El driver resta el offset de 50 mm -> distancia reportada mínima = 100 mm.
--
-- Verifica que el barrido grueso+fino encuentra ese punto y que la conversión
-- polar da:  out_phi = 90,  out_r ≈ 168 mm,  out_theta ≈ -27°.
--   (dradial = 100*cos60 + 100 = 150; dz = 100*sin60 - 163 = -77;
--    r = sqrt(150^2 + 77^2) = 168.6; theta = atan2(-77,150) = -27.2°)
--
-- Simulación (ModelSim-Altera):
--   vlib work
--   vcom -2008 vl53l0x_pkg.vhd VL53L0X.vhd polar_kin.vhd LIDAR.vhd tb_LIDAR_scan.vhd
--   vsim tb_LIDAR_scan -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_LIDAR_scan is
end tb_LIDAR_scan;

architecture sim of tb_LIDAR_scan is

    component LIDAR
        generic (
            CLK_FREQ_HZ     : integer := 50_000_000;
            I2C_FREQ_HZ     : integer := 100_000;
            PWRUP_CYCLES    : integer := 500_000;
            SETTLE_CYCLES   : integer := 7_500_000;
            N_AVG           : integer := 8;
            COARSE_PHI_STEP : integer := 10;
            COARSE_T1_STEP  : integer := 9;
            FINE_PHI_STEP   : integer := 3;
            FINE_T1_STEP    : integer := 3
        );
        port (
            clk         : in    std_logic;
            rst         : in    std_logic;
            start_scan  : in    std_logic;
            i2c_scl     : out   std_logic;
            i2c_sda     : inout std_logic;
            scan_active : out   std_logic;
            cmd_phi     : out   std_logic_vector(7 downto 0);
            cmd_theta1  : out   std_logic_vector(7 downto 0);
            cmd_theta2  : out   std_logic_vector(7 downto 0);
            cmd_theta3  : out   std_logic_vector(7 downto 0);
            cmd_grip    : out   std_logic;
            out_r       : out   std_logic_vector(15 downto 0);
            out_theta   : out   std_logic_vector(8 downto 0);
            out_phi     : out   std_logic_vector(7 downto 0);
            scan_done   : out   std_logic
        );
    end component;

    signal clk        : std_logic := '0';
    signal rst        : std_logic := '1';
    signal start_scan : std_logic := '0';
    signal scl        : std_logic;
    signal sda        : std_logic;
    signal sda_clean  : std_logic;
    signal sda_drv    : std_logic := 'Z';

    signal scan_active : std_logic;
    signal cmd_phi     : std_logic_vector(7 downto 0);
    signal cmd_theta1  : std_logic_vector(7 downto 0);
    signal cmd_theta2  : std_logic_vector(7 downto 0);
    signal cmd_theta3  : std_logic_vector(7 downto 0);
    signal cmd_grip    : std_logic;
    signal out_r       : std_logic_vector(15 downto 0);
    signal out_theta   : std_logic_vector(8 downto 0);
    signal out_phi     : std_logic_vector(7 downto 0);
    signal scan_done   : std_logic;

    signal sim_done  : boolean := false;
    signal synth_raw : unsigned(15 downto 0) := to_unsigned(150, 16);

    constant SLAVE_ADDR7 : std_logic_vector(6 downto 0) := "0101001"; -- 0x29

begin

    -- Bus I2C: pull-up en SDA, SCL conducido por el master (DUT)
    sda       <= 'H';
    sda       <= sda_drv;
    sda_clean <= '0' when sda = '0' else '1';

    -- ----------------------------------------------------------------
    -- DUT: escáner con rejilla y tiempos reducidos para simular rápido
    -- ----------------------------------------------------------------
    dut : LIDAR
        generic map (
            CLK_FREQ_HZ     => 2_000_000,
            I2C_FREQ_HZ     => 100_000,    -- QUARTER = 5 ciclos
            PWRUP_CYCLES    => 50,
            SETTLE_CYCLES   => 30,
            N_AVG           => 4,
            COARSE_PHI_STEP => 45,         -- phi   : 45,90,135
            COARSE_T1_STEP  => 15,         -- theta1: 45,60,75,90
            FINE_PHI_STEP   => 15,
            FINE_T1_STEP    => 9
        )
        port map (
            clk         => clk,
            rst         => rst,
            start_scan  => start_scan,
            i2c_scl     => scl,
            i2c_sda     => sda,
            scan_active => scan_active,
            cmd_phi     => cmd_phi,
            cmd_theta1  => cmd_theta1,
            cmd_theta2  => cmd_theta2,
            cmd_theta3  => cmd_theta3,
            cmd_grip    => cmd_grip,
            out_r       => out_r,
            out_theta   => out_theta,
            out_phi     => out_phi,
            scan_done   => scan_done
        );

    -- Reloj
    clk_proc : process
    begin
        while not sim_done loop
            clk <= '0'; wait for 10 ns;
            clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- ----------------------------------------------------------------
    -- "Cuenco" de distancia: mínimo en (phi=90, theta1=60)
    --   raw = 50 + 100 + 4*|phi-90| + 2*|theta1-60|
    -- ----------------------------------------------------------------
    synth_proc : process(cmd_phi, cmd_theta1)
        variable p, t, val : integer;
    begin
        p   := to_integer(unsigned(cmd_phi));
        t   := to_integer(unsigned(cmd_theta1));
        val := 50 + 100 + 4*abs(p - 90) + 2*abs(t - 60);
        synth_raw <= to_unsigned(val, 16);
    end process;

    -- ----------------------------------------------------------------
    -- Modelo conductual de esclavo I2C del VL53L0X
    --   * 0xC0 -> 0xEE (MODEL_ID)
    --   * 0x13 -> 0x04, 0x14 -> 0x01 (siempre listo)
    --   * 0x1E/0x1F -> bytes alto/bajo de synth_raw (rango dinámico)
    -- ----------------------------------------------------------------
    slave : process(scl, sda_clean)
        constant PH_ADDR : integer := 0;
        constant PH_WR   : integer := 1;
        constant PH_RD   : integer := 2;
        type mem_t is array(0 to 255) of std_logic_vector(7 downto 0);
        variable mem     : mem_t := (16#C0# => x"EE",
                                     16#C2# => x"10",
                                     16#13# => x"04",
                                     16#14# => x"01",
                                     others => x"00");
        variable phase   : integer := PH_ADDR;
        variable bcnt    : integer := 0;
        variable shin    : std_logic_vector(7 downto 0) := (others => '0');
        variable shout   : std_logic_vector(7 downto 0) := (others => '0');
        variable ptr     : integer range 0 to 255 := 0;
        variable first_wr: boolean := true;
        variable is_read : std_logic := '0';
        variable active  : boolean := false;

        impure function rd_byte(a : integer) return std_logic_vector is
        begin
            if a = 16#1E# then
                return std_logic_vector(synth_raw(15 downto 8));
            elsif a = 16#1F# then
                return std_logic_vector(synth_raw(7 downto 0));
            else
                return mem(a);
            end if;
        end function;
    begin
        if scl = '1' and sda_clean'event then
            if sda_clean = '0' then               -- START / repeated START
                active   := true;
                phase    := PH_ADDR;
                bcnt     := 0;
                first_wr := true;
                sda_drv  <= 'Z';
            else                                  -- STOP
                active  := false;
                phase   := PH_ADDR;
                bcnt    := 0;
                sda_drv <= 'Z';
            end if;

        elsif rising_edge(scl) then
            if active then
                if bcnt < 8 then
                    if phase = PH_ADDR or phase = PH_WR then
                        shin(7 - bcnt) := sda_clean;
                    end if;
                    bcnt := bcnt + 1;
                else
                    if phase = PH_ADDR then
                        if is_read = '1' then
                            phase := PH_RD;
                            shout := rd_byte(ptr);
                        else
                            phase    := PH_WR;
                            first_wr := true;
                        end if;
                        bcnt := 0;
                    elsif phase = PH_WR then
                        bcnt := 0;
                    else  -- PH_RD: ACK/NACK del master
                        if sda_clean = '0' then        -- ACK -> más bytes
                            ptr   := (ptr + 1) mod 256;
                            shout := rd_byte(ptr);
                        else                            -- NACK -> soltar el bus
                            sda_drv <= 'Z';
                            phase   := PH_ADDR;
                        end if;
                        bcnt := 0;
                    end if;
                end if;
            end if;

        elsif falling_edge(scl) then
            if active then
                if bcnt = 8 then
                    if phase = PH_RD then
                        sda_drv <= 'Z';
                    elsif phase = PH_ADDR then
                        if shin(7 downto 1) = SLAVE_ADDR7 then
                            sda_drv <= '0';            -- ACK dirección
                            is_read := shin(0);
                        else
                            sda_drv <= 'Z';            -- NACK
                        end if;
                    else  -- PH_WR
                        sda_drv <= '0';                -- ACK dato
                        if first_wr then
                            ptr      := to_integer(unsigned(shin));
                            first_wr := false;
                        else
                            mem(ptr) := shin;
                            ptr      := (ptr + 1) mod 256;
                        end if;
                    end if;
                else
                    if phase = PH_RD then
                        if shout(7 - bcnt) = '0' then sda_drv <= '0'; else sda_drv <= 'Z'; end if;
                    else
                        sda_drv <= 'Z';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Estímulo y comprobaciones
    -- ----------------------------------------------------------------
    stim : process
        variable rr  : integer;
        variable thh : integer;
        variable phh : integer;
    begin
        rst <= '1';
        wait for 500 ns;
        rst <= '0';
        wait for 500 ns;

        -- Dispara el barrido
        wait until rising_edge(clk);
        start_scan <= '1';
        wait until rising_edge(clk);
        start_scan <= '0';

        -- Comprueba que se activa el escaneo
        wait until scan_active = '1' for 1 ms;
        assert scan_active = '1'
            report "FALLO: scan_active no se activo" severity error;
        assert cmd_theta3 = std_logic_vector(to_unsigned(0, 8))
            report "FALLO: theta3 de pose != 0 (el haz no apunta hacia abajo)" severity error;
        assert (to_integer(unsigned(cmd_theta1)) + to_integer(unsigned(cmd_theta2))) = 90
            report "FALLO: theta1+theta2 != 90 (L2 no esta horizontal)" severity error;
        assert cmd_grip = '0'
            report "FALLO: el grip no esta abierto durante el escaneo" severity error;

        -- Espera fin del barrido (timeout de seguridad amplio)
        wait until scan_done = '1' for 300 ms;
        assert scan_done = '1'
            report "FALLO: el barrido no termino (timeout)" severity error;

        phh := to_integer(unsigned(out_phi));
        rr  := to_integer(unsigned(out_r));
        thh := to_integer(signed(out_theta));

        report "Resultado: out_phi=" & integer'image(phh) &
               "  out_r=" & integer'image(rr) & " mm" &
               "  out_theta=" & integer'image(thh) & " deg" severity note;

        assert phh = 90
            report "FALLO: out_phi != 90 (minimo no localizado en phi)" severity error;
        assert (rr >= 163 and rr <= 174)
            report "FALLO: out_r fuera de rango esperado (~168 mm)" severity error;
        assert (thh >= -31 and thh <= -23)
            report "FALLO: out_theta fuera de rango esperado (~-27 deg)" severity error;

        assert scan_active = '0'
            report "FALLO: scan_active no bajo tras terminar" severity error;

        report "OK: barrido localizo el minimo en phi=90 y convirtio a polar correctamente"
            severity note;

        sim_done <= true;
        wait;
    end process;

end sim;
