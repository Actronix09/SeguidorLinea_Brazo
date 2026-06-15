-- ============================================================================
-- VL53L0X - Driver VHDL del sensor ToF VL53L0X (equivalente a Adafruit begin()
--           + readRange()). Incluye el motor I2C (master) integrado.
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Uso: se instancia este componente y se lee 'distance_mm' (modo continuo: el
-- driver mide en bucle y deja siempre el último valor válido). 'i2c_scl' /
-- 'i2c_sda' se conectan físicamente a los pines del bus (pull-ups externos
-- obligatorios; el breakout Adafruit ya los trae).
--
-- Estructura interna (3 capas):
--   1) Motor I2C (bit/byte): genera SCL/SDA a I2C_FREQ desde CLK_FREQ.
--   2) Secuenciador de transacciones: reg_write / reg_read8 / reg_read16.
--   3) FSM de aplicación: verifica ID -> init (DataInit+StaticInit mínimo +
--      tuning) -> bucle de medición single-shot. Errores en 'err_code'.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.vl53l0x_pkg.all;

entity VL53L0X is
    generic (
        CLK_FREQ_HZ  : integer := 50_000_000;
        I2C_FREQ_HZ  : integer := 100_000;
        PWRUP_CYCLES : integer := 500_000   -- retardo de arranque (~10 ms @50 MHz)
    );
    port (
        clk         : in    std_logic;
        rst         : in    std_logic;                      -- activo ALTO
        i2c_scl     : out   std_logic;                      -- push-pull
        i2c_sda     : inout std_logic;                      -- open-drain ('0'/'Z')
        distance_mm : out   std_logic_vector(15 downto 0);  -- último rango en mm
        data_valid  : out   std_logic;                      -- '1' tras 1ª medida
        sensor_ok   : out   std_logic;                      -- init OK e ID correcto
        err_code    : out   std_logic_vector(4 downto 0);   -- 0=OK; ver vl53l0x_pkg
        meas_tick   : out   std_logic                       -- conmuta cada medición
    );
end VL53L0X;

architecture rtl of VL53L0X is

    -- Cuarto de periodo de SCL (4 fases por bit)
    constant QUARTER  : integer := CLK_FREQ_HZ / (I2C_FREQ_HZ * 4);
    constant POLL_MAX : integer := 255;     -- nº máx de sondeos data-ready
    constant ERR_HARD : integer := 4;       -- errores ligeros seguidos antes de la recuperación DURA (bus + re-init)

    -- Corrección de offset del sensor en mm (lectura cruda = real + offset).
    -- Ajustar si el part-to-part offset de tu unidad es distinto.
    constant OFFSET_MM : integer := 50;

    -- --------------------------------------------------------------------
    -- Capa 1: motor I2C (bit/byte)
    -- --------------------------------------------------------------------
    constant CMD_START : std_logic_vector(1 downto 0) := "00";
    constant CMD_STOP  : std_logic_vector(1 downto 0) := "01";
    constant CMD_WRITE : std_logic_vector(1 downto 0) := "10";
    constant CMD_READ  : std_logic_vector(1 downto 0) := "11";

    -- ES_RECOVER/ES_RSTOP: recuperación del bus tras un reset (ver engine).
    type eng_state_t is (ES_RECOVER, ES_RSTOP, ES_IDLE, ES_START, ES_STOP, ES_WRITE, ES_READ, ES_FIN);
    signal e_st    : eng_state_t := ES_RECOVER;
    signal e_q     : integer range 0 to QUARTER-1 := 0;
    signal e_phase : integer range 0 to 3 := 0;
    signal e_bit   : integer range 0 to 8 := 0;
    signal e_shift : std_logic_vector(7 downto 0) := (others => '0');
    signal e_scl   : std_logic := '1';
    signal e_sda   : std_logic := '1';
    signal sda_in  : std_logic;

    signal eng_cmd     : std_logic_vector(1 downto 0) := CMD_START;
    signal eng_start   : std_logic := '0';
    signal eng_done    : std_logic := '0';
    signal eng_wr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal eng_rd_data : std_logic_vector(7 downto 0) := (others => '0');
    signal eng_rd_ack  : std_logic := '1';   -- ack que envía el master tras leer
    signal eng_ack_in  : std_logic := '1';   -- ack recibido tras escribir ('0'=ACK)

    -- --------------------------------------------------------------------
    -- Capa 2: secuenciador de transacciones
    -- --------------------------------------------------------------------
    constant TXN_WRITE  : std_logic_vector(1 downto 0) := "00";
    constant TXN_READ8  : std_logic_vector(1 downto 0) := "01";
    constant TXN_READ16 : std_logic_vector(1 downto 0) := "10";

    type txn_state_t is (T_IDLE, T_ISSUE, T_WAIT, T_DONE);
    signal txn_state  : txn_state_t := T_IDLE;
    signal txn_step   : integer range 0 to 7 := 0;
    signal txn_kind   : std_logic_vector(1 downto 0) := TXN_WRITE;  -- pedido (app)
    signal txn_reg    : std_logic_vector(7 downto 0) := (others => '0');
    signal txn_wdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal txn_start  : std_logic := '0';
    signal txn_done   : std_logic := '0';
    signal txn_nack   : std_logic := '0';
    signal txn_rdata  : std_logic_vector(15 downto 0) := (others => '0');
    signal txn_kind_r : std_logic_vector(1 downto 0) := TXN_WRITE;  -- latcheado
    signal txn_reg_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal txn_wdata_r: std_logic_vector(7 downto 0) := (others => '0');
    signal msb_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal lsb_r      : std_logic_vector(7 downto 0) := (others => '0');

    -- --------------------------------------------------------------------
    -- Capa 3: FSM de aplicación
    -- --------------------------------------------------------------------
    -- Tras el init se ejecuta la calibración de referencia (VHV + fase),
    -- imprescindible para que el sensor entregue rangos válidos.
    type app_state_t is (A_PWRUP, A_EXEC, A_WAIT, A_RMW_WR, A_RMW_WAIT,
                         A_TUNE, A_TUNE_WAIT,
                         A_CAL_BEGIN, A_CAL_BEGIN_W, A_CAL_GO, A_CAL_GO_W,
                         A_CAL_POLL, A_CAL_POLL_W, A_CAL_CLR, A_CAL_CLR_W,
                         A_CAL_STOP, A_CAL_STOP_W, A_CAL_RESTORE, A_CAL_RESTORE_W,
                         A_AFTER_INIT,
                         A_POLL, A_POLL_WAIT, A_RANGE, A_RANGE_WAIT,
                         A_CLEAR, A_CLEAR_WAIT, A_UPDATE,
                         A_RECLR, A_RECLR_W,            -- recuperación ligera (post-init)
                         A_ERROR);
    signal app_state : app_state_t := A_PWRUP;
    signal idx       : integer range 0 to 31 := 0;
    signal tune_idx  : integer range 0 to 127 := 0;
    signal rom_sel   : std_logic := '0';     -- '0'=INIT_ROM, '1'=MEAS_ROM
    signal op_r      : opcode_t := OP_WR;
    signal reg_r2    : std_logic_vector(7 downto 0) := (others => '0');
    signal val_r2    : std_logic_vector(7 downto 0) := (others => '0');
    signal rmw_val   : std_logic_vector(7 downto 0) := (others => '0');
    signal sv_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal range_r   : std_logic_vector(15 downto 0) := (others => '0');
    signal poll_cnt  : integer range 0 to POLL_MAX := 0;
    signal pwr_cnt   : integer range 0 to PWRUP_CYCLES-1 := 0;
    signal tick_r    : std_logic := '0';
    signal cal_phase : integer range 0 to 1 := 0;  -- 0 = VHV, 1 = fase
    signal inited    : std_logic := '0';           -- '1' tras el init: un error de medición se recupera ligero (no re-init)
    signal err_streak: integer range 0 to ERR_HARD := 0;  -- errores ligeros consecutivos (0 tras una medición OK)
    signal recover_req: std_logic := '0';          -- app -> motor I2C: pide limpiar el bus (pulsos SCL) antes de re-init

begin

    -- Salidas físicas del bus (SCL push-pull, SDA open-drain)
    i2c_scl <= e_scl;
    i2c_sda <= '0' when e_sda = '0' else 'Z';
    sda_in  <= '0' when i2c_sda = '0' else '1';

    meas_tick <= tick_r;

    -- ====================================================================
    -- CAPA 1: Motor I2C (bit/byte)
    -- ====================================================================
    engine : process(clk, rst)
        variable tick : boolean;
    begin
        if rst = '1' then
            -- Arranca en RECUPERACIÓN del bus (no en IDLE) para desatascar un
            -- esclavo que quedó a media transacción si el reset se pulsó a mitad
            -- de una lectura I2C (mantenía SDA en bajo esperando reloj).
            e_st    <= ES_RECOVER;
            e_q     <= 0;
            e_phase <= 0;
            e_bit   <= 0;
            e_shift <= (others => '0');
            e_scl   <= '1';
            e_sda   <= '1';
            eng_done    <= '0';
            eng_ack_in  <= '1';
            eng_rd_data <= (others => '0');
        elsif rising_edge(clk) then
            eng_done <= '0';

            -- Generación del "tick" de cuarto de periodo
            tick := false;
            if e_st = ES_IDLE then
                e_q <= 0;
            elsif e_q = QUARTER-1 then
                e_q  <= 0;
                tick := true;
            else
                e_q <= e_q + 1;
            end if;

            case e_st is
                -- Recuperación del bus: con SDA liberado, pulsa SCL hasta 9 veces
                -- para que un esclavo colgado a media transacción termine su byte y
                -- suelte SDA. Se ejecuta una vez tras cada reset, antes de IDLE.
                when ES_RECOVER =>
                    case e_phase is
                        when 0      => e_scl <= '0'; e_sda <= '1';
                        when 1      => e_scl <= '1';
                        when 2      => e_scl <= '1';
                        when others => e_scl <= '0';
                    end case;
                    if tick then
                        if e_phase = 3 then
                            e_phase <= 0;
                            if e_bit = 8 then        -- 9 pulsos de SCL hechos
                                e_bit <= 0;
                                e_st  <= ES_RSTOP;
                            else
                                e_bit <= e_bit + 1;
                            end if;
                        else
                            e_phase <= e_phase + 1;
                        end if;
                    end if;

                -- STOP de cierre tras la recuperación (SDA sube con SCL alto) -> bus libre.
                when ES_RSTOP =>
                    case e_phase is
                        when 0      => e_sda <= '0'; e_scl <= '0';
                        when 1      => e_sda <= '0'; e_scl <= '1';
                        when others => e_sda <= '1'; e_scl <= '1';
                    end case;
                    if tick then
                        if e_phase = 3 then e_st <= ES_IDLE; else e_phase <= e_phase + 1; end if;
                    end if;

                when ES_IDLE =>
                    if recover_req = '1' then
                        -- La app pide recuperar el bus (pulsos SCL) para desatascar
                        -- un esclavo trabado a media transacción.
                        e_q <= 0; e_phase <= 0; e_bit <= 0;
                        e_scl <= '1'; e_sda <= '1';
                        e_st  <= ES_RECOVER;
                    elsif eng_start = '1' then
                        e_q     <= 0;
                        e_phase <= 0;
                        e_bit   <= 0;
                        e_shift <= eng_wr_data;
                        if    eng_cmd = CMD_START then e_st <= ES_START;
                        elsif eng_cmd = CMD_STOP  then e_st <= ES_STOP;
                        elsif eng_cmd = CMD_WRITE then e_st <= ES_WRITE;
                        else                            e_st <= ES_READ;
                        end if;
                    end if;

                when ES_START =>
                    -- 0:sda1/scl0  1:sda1/scl1  2:sda0/scl1 (START)  3:sda0/scl0
                    case e_phase is
                        when 0      => e_sda <= '1'; e_scl <= '0';
                        when 1      => e_sda <= '1'; e_scl <= '1';
                        when 2      => e_sda <= '0'; e_scl <= '1';
                        when others => e_sda <= '0'; e_scl <= '0';
                    end case;
                    if tick then
                        if e_phase = 3 then e_st <= ES_FIN; else e_phase <= e_phase + 1; end if;
                    end if;

                when ES_STOP =>
                    -- 0:sda0/scl0  1:sda0/scl1  2:sda1/scl1 (STOP)  3:sda1/scl1
                    case e_phase is
                        when 0      => e_sda <= '0'; e_scl <= '0';
                        when 1      => e_sda <= '0'; e_scl <= '1';
                        when others => e_sda <= '1'; e_scl <= '1';
                    end case;
                    if tick then
                        if e_phase = 3 then e_st <= ES_FIN; else e_phase <= e_phase + 1; end if;
                    end if;

                when ES_WRITE =>
                    -- bits 0..7 = dato (MSB first); bit 8 = lectura de ACK
                    -- (e_scl/e_sda son idempotentes; el muestreo/avance va en 'tick')
                    case e_phase is
                        when 0 =>
                            e_scl <= '0';
                            if e_bit < 8 then e_sda <= e_shift(7); else e_sda <= '1'; end if;
                        when 1      => e_scl <= '1';
                        when 2      => e_scl <= '1';
                        when others => e_scl <= '0';
                    end case;
                    if tick then
                        if e_phase = 2 and e_bit = 8 then
                            eng_ack_in <= sda_in;        -- ACK del esclavo (1 muestra)
                        end if;
                        if e_phase = 3 then
                            e_phase <= 0;
                            if e_bit = 8 then
                                e_st <= ES_FIN;
                            else
                                if e_bit < 8 then e_shift <= e_shift(6 downto 0) & '0'; end if;
                                e_bit <= e_bit + 1;
                            end if;
                        else
                            e_phase <= e_phase + 1;
                        end if;
                    end if;

                when ES_READ =>
                    -- bits 0..7 = dato leído; bit 8 = ACK/NACK que envía el master
                    case e_phase is
                        when 0 =>
                            e_scl <= '0';
                            if e_bit < 8 then e_sda <= '1'; else e_sda <= eng_rd_ack; end if;
                        when 1      => e_scl <= '1';
                        when 2      => e_scl <= '1';
                        when others => e_scl <= '0';
                    end case;
                    if tick then
                        if e_phase = 2 and e_bit < 8 then
                            e_shift <= e_shift(6 downto 0) & sda_in;   -- 1 muestra/bit
                        end if;
                        if e_phase = 3 then
                            e_phase <= 0;
                            if e_bit = 8 then
                                eng_rd_data <= e_shift;
                                e_st <= ES_FIN;
                            else
                                e_bit <= e_bit + 1;
                            end if;
                        else
                            e_phase <= e_phase + 1;
                        end if;
                    end if;

                when ES_FIN =>
                    eng_done <= '1';
                    e_st     <= ES_IDLE;
            end case;
        end if;
    end process;

    -- ====================================================================
    -- CAPA 2: Secuenciador de transacciones (WRITE / READ8 / READ16)
    -- ====================================================================
    txn : process(clk, rst)
    begin
        if rst = '1' then
            txn_state   <= T_IDLE;
            txn_step    <= 0;
            txn_done    <= '0';
            txn_nack    <= '0';
            txn_rdata   <= (others => '0');
            eng_start   <= '0';
            eng_cmd     <= CMD_START;
            eng_wr_data <= (others => '0');
            eng_rd_ack  <= '1';
            txn_kind_r  <= TXN_WRITE;
            txn_reg_r   <= (others => '0');
            txn_wdata_r <= (others => '0');
            msb_r       <= (others => '0');
            lsb_r       <= (others => '0');
        elsif rising_edge(clk) then
            eng_start <= '0';
            txn_done  <= '0';

            case txn_state is
                when T_IDLE =>
                    if txn_start = '1' then
                        txn_kind_r  <= txn_kind;
                        txn_reg_r   <= txn_reg;
                        txn_wdata_r <= txn_wdata;
                        txn_step    <= 0;
                        txn_nack    <= '0';
                        txn_state   <= T_ISSUE;
                    end if;

                when T_ISSUE =>
                    eng_start <= '1';
                    if txn_kind_r = TXN_WRITE then
                        case txn_step is
                            when 0      => eng_cmd <= CMD_START;
                            when 1      => eng_cmd <= CMD_WRITE; eng_wr_data <= VL53_ADDR_W;
                            when 2      => eng_cmd <= CMD_WRITE; eng_wr_data <= txn_reg_r;
                            when 3      => eng_cmd <= CMD_WRITE; eng_wr_data <= txn_wdata_r;
                            when others => eng_cmd <= CMD_STOP;
                        end case;
                    elsif txn_kind_r = TXN_READ8 then
                        case txn_step is
                            when 0      => eng_cmd <= CMD_START;
                            when 1      => eng_cmd <= CMD_WRITE; eng_wr_data <= VL53_ADDR_W;
                            when 2      => eng_cmd <= CMD_WRITE; eng_wr_data <= txn_reg_r;
                            when 3      => eng_cmd <= CMD_START;
                            when 4      => eng_cmd <= CMD_WRITE; eng_wr_data <= VL53_ADDR_R;
                            when 5      => eng_cmd <= CMD_READ;  eng_rd_ack  <= '1';  -- NACK
                            when others => eng_cmd <= CMD_STOP;
                        end case;
                    else  -- TXN_READ16
                        case txn_step is
                            when 0      => eng_cmd <= CMD_START;
                            when 1      => eng_cmd <= CMD_WRITE; eng_wr_data <= VL53_ADDR_W;
                            when 2      => eng_cmd <= CMD_WRITE; eng_wr_data <= txn_reg_r;
                            when 3      => eng_cmd <= CMD_START;
                            when 4      => eng_cmd <= CMD_WRITE; eng_wr_data <= VL53_ADDR_R;
                            when 5      => eng_cmd <= CMD_READ;  eng_rd_ack  <= '0';  -- ACK
                            when 6      => eng_cmd <= CMD_READ;  eng_rd_ack  <= '1';  -- NACK
                            when others => eng_cmd <= CMD_STOP;
                        end case;
                    end if;
                    txn_state <= T_WAIT;

                when T_WAIT =>
                    if eng_done = '1' then
                        if txn_kind_r = TXN_WRITE then
                            if (txn_step = 1 or txn_step = 2 or txn_step = 3)
                               and eng_ack_in = '1' then
                                txn_nack <= '1';
                            end if;
                            if txn_step = 4 then
                                txn_state <= T_DONE;
                            else
                                txn_step  <= txn_step + 1;
                                txn_state <= T_ISSUE;
                            end if;
                        elsif txn_kind_r = TXN_READ8 then
                            if (txn_step = 1 or txn_step = 2 or txn_step = 4)
                               and eng_ack_in = '1' then
                                txn_nack <= '1';
                            end if;
                            if txn_step = 5 then lsb_r <= eng_rd_data; end if;
                            if txn_step = 6 then
                                txn_state <= T_DONE;
                            else
                                txn_step  <= txn_step + 1;
                                txn_state <= T_ISSUE;
                            end if;
                        else  -- TXN_READ16
                            if (txn_step = 1 or txn_step = 2 or txn_step = 4)
                               and eng_ack_in = '1' then
                                txn_nack <= '1';
                            end if;
                            if txn_step = 5 then msb_r <= eng_rd_data; end if;
                            if txn_step = 6 then lsb_r <= eng_rd_data; end if;
                            if txn_step = 7 then
                                txn_state <= T_DONE;
                            else
                                txn_step  <= txn_step + 1;
                                txn_state <= T_ISSUE;
                            end if;
                        end if;
                    end if;

                when T_DONE =>
                    txn_done <= '1';
                    if txn_kind_r = TXN_READ16 then
                        txn_rdata <= msb_r & lsb_r;
                    else
                        txn_rdata <= x"00" & lsb_r;
                    end if;
                    txn_state <= T_IDLE;
            end case;
        end if;
    end process;

    -- ====================================================================
    -- CAPA 3: FSM de aplicación (init + medición continua)
    -- ====================================================================
    app : process(clk, rst)
        variable cur : cmd_t;
    begin
        if rst = '1' then
            app_state   <= A_PWRUP;
            idx         <= 0;
            tune_idx    <= 0;
            rom_sel     <= '0';
            poll_cnt    <= 0;
            pwr_cnt     <= 0;
            cal_phase   <= 0;
            sv_r        <= (others => '0');
            range_r     <= (others => '0');
            distance_mm <= (others => '0');
            data_valid  <= '0';
            sensor_ok   <= '0';
            inited      <= '0';
            err_streak  <= 0;
            recover_req <= '0';
            err_code    <= ERR_NONE;
            tick_r      <= '0';
            txn_start   <= '0';
            txn_kind    <= TXN_WRITE;
            txn_reg     <= (others => '0');
            txn_wdata   <= (others => '0');
        elsif rising_edge(clk) then
            txn_start   <= '0';   -- pulso por defecto a '0'
            recover_req <= '0';   -- pulso por defecto a '0'

            case app_state is
                when A_PWRUP =>
                    if pwr_cnt = PWRUP_CYCLES-1 then
                        -- No arrancar el init hasta que el motor I2C esté libre (por si
                        -- venimos de una recuperación del bus en curso); evita perder eng_start.
                        if e_st = ES_IDLE then
                            rom_sel   <= '0';
                            idx       <= 0;
                            app_state <= A_EXEC;
                        end if;
                    else
                        pwr_cnt <= pwr_cnt + 1;
                    end if;

                when A_EXEC =>
                    if rom_sel = '0' then cur := INIT_ROM(idx); else cur := MEAS_ROM(idx); end if;
                    op_r   <= cur.op;
                    reg_r2 <= cur.reg;
                    val_r2 <= cur.val;
                    case cur.op is
                        when OP_END =>
                            if rom_sel = '0' then
                                cal_phase <= 0;          -- init listo -> calibrar
                                app_state <= A_CAL_BEGIN;
                            else
                                app_state <= A_POLL;     -- MEAS_ROM listo -> medir
                            end if;
                        when OP_TUNING =>
                            tune_idx  <= 0;
                            app_state <= A_TUNE;
                        when OP_WR =>
                            txn_kind  <= TXN_WRITE; txn_reg <= cur.reg; txn_wdata <= cur.val;
                            txn_start <= '1';       app_state <= A_WAIT;
                        when OP_WR_SV =>
                            txn_kind  <= TXN_WRITE; txn_reg <= cur.reg; txn_wdata <= sv_r;
                            txn_start <= '1';       app_state <= A_WAIT;
                        when others =>  -- OP_RD_SV, OP_CHK_ID, OP_OR, OP_AND
                            txn_kind  <= TXN_READ8; txn_reg <= cur.reg;
                            txn_start <= '1';       app_state <= A_WAIT;
                    end case;

                when A_WAIT =>
                    if txn_done = '1' then
                        if txn_nack = '1' then
                            if    op_r = OP_CHK_ID then err_code <= ERR_NACK_ADDR;
                            elsif rom_sel = '0'    then err_code <= ERR_NACK_INIT;
                            else                        err_code <= ERR_NACK_ADDR;
                            end if;
                            app_state <= A_ERROR;
                        else
                            case op_r is
                                when OP_WR | OP_WR_SV =>
                                    idx <= idx + 1; app_state <= A_EXEC;
                                when OP_RD_SV =>
                                    sv_r <= txn_rdata(7 downto 0);
                                    idx  <= idx + 1; app_state <= A_EXEC;
                                when OP_CHK_ID =>
                                    if txn_rdata(7 downto 0) /= val_r2 then
                                        err_code  <= ERR_MODEL_ID;
                                        app_state <= A_ERROR;
                                    else
                                        idx <= idx + 1; app_state <= A_EXEC;
                                    end if;
                                when OP_OR =>
                                    rmw_val   <= txn_rdata(7 downto 0) or val_r2;
                                    app_state <= A_RMW_WR;
                                when OP_AND =>
                                    rmw_val   <= txn_rdata(7 downto 0) and val_r2;
                                    app_state <= A_RMW_WR;
                                when others =>
                                    idx <= idx + 1; app_state <= A_EXEC;
                            end case;
                        end if;
                    end if;

                when A_RMW_WR =>
                    txn_kind  <= TXN_WRITE; txn_reg <= reg_r2; txn_wdata <= rmw_val;
                    txn_start <= '1';       app_state <= A_RMW_WAIT;

                when A_RMW_WAIT =>
                    if txn_done = '1' then
                        if txn_nack = '1' then
                            err_code  <= ERR_NACK_INIT;
                            app_state <= A_ERROR;
                        else
                            idx <= idx + 1; app_state <= A_EXEC;
                        end if;
                    end if;

                when A_TUNE =>
                    txn_kind  <= TXN_WRITE;
                    txn_reg   <= TUNING_ROM(tune_idx).reg;
                    txn_wdata <= TUNING_ROM(tune_idx).val;
                    txn_start <= '1';
                    app_state <= A_TUNE_WAIT;

                when A_TUNE_WAIT =>
                    if txn_done = '1' then
                        if txn_nack = '1' then
                            err_code  <= ERR_NACK_INIT;
                            app_state <= A_ERROR;
                        elsif tune_idx = TUNING_ROM'high then
                            idx <= idx + 1; app_state <= A_EXEC;  -- continúa INIT_ROM
                        else
                            tune_idx <= tune_idx + 1; app_state <= A_TUNE;
                        end if;
                    end if;

                -- ----------------------------------------------------------
                -- Calibración de referencia: VHV (cal_phase=0) y fase (=1).
                -- Equivale a VL53L0X_PerformRefCalibration: sin esto el sensor
                -- entrega rangos inválidos/máximos aunque el init "funcione".
                -- ----------------------------------------------------------
                when A_CAL_BEGIN =>
                    txn_kind <= TXN_WRITE; txn_reg <= REG_SEQ_CONFIG;
                    if cal_phase = 0 then txn_wdata <= x"01"; else txn_wdata <= x"02"; end if;
                    txn_start <= '1'; app_state <= A_CAL_BEGIN_W;

                when A_CAL_BEGIN_W =>
                    if txn_done = '1' then app_state <= A_CAL_GO; end if;

                when A_CAL_GO =>
                    -- SYSRANGE_START = 0x01 | vhv_byte (0x40 para VHV, 0x00 para fase)
                    txn_kind <= TXN_WRITE; txn_reg <= REG_SYSRANGE_START;
                    if cal_phase = 0 then txn_wdata <= x"41"; else txn_wdata <= x"01"; end if;
                    txn_start <= '1'; poll_cnt <= 0; app_state <= A_CAL_GO_W;

                when A_CAL_GO_W =>
                    if txn_done = '1' then app_state <= A_CAL_POLL; end if;

                when A_CAL_POLL =>
                    txn_kind <= TXN_READ8; txn_reg <= REG_INT_STATUS;
                    txn_start <= '1'; app_state <= A_CAL_POLL_W;

                when A_CAL_POLL_W =>
                    if txn_done = '1' then
                        if txn_rdata(2 downto 0) /= "000" then   -- interrupt listo
                            app_state <= A_CAL_CLR;
                        elsif poll_cnt = POLL_MAX then
                            err_code  <= ERR_TIMEOUT; app_state <= A_ERROR;
                        else
                            poll_cnt  <= poll_cnt + 1; app_state <= A_CAL_POLL;
                        end if;
                    end if;

                when A_CAL_CLR =>
                    txn_kind <= TXN_WRITE; txn_reg <= REG_INT_CLEAR; txn_wdata <= x"01";
                    txn_start <= '1'; app_state <= A_CAL_CLR_W;

                when A_CAL_CLR_W =>
                    if txn_done = '1' then app_state <= A_CAL_STOP; end if;

                when A_CAL_STOP =>
                    txn_kind <= TXN_WRITE; txn_reg <= REG_SYSRANGE_START; txn_wdata <= x"00";
                    txn_start <= '1'; app_state <= A_CAL_STOP_W;

                when A_CAL_STOP_W =>
                    if txn_done = '1' then
                        if cal_phase = 0 then
                            cal_phase <= 1; app_state <= A_CAL_BEGIN;  -- fase
                        else
                            app_state <= A_CAL_RESTORE;
                        end if;
                    end if;

                when A_CAL_RESTORE =>
                    -- restaurar SYSTEM_SEQUENCE_CONFIG = 0xE8 (medición normal)
                    txn_kind <= TXN_WRITE; txn_reg <= REG_SEQ_CONFIG; txn_wdata <= x"E8";
                    txn_start <= '1'; app_state <= A_CAL_RESTORE_W;

                when A_CAL_RESTORE_W =>
                    if txn_done = '1' then app_state <= A_AFTER_INIT; end if;

                when A_AFTER_INIT =>
                    sensor_ok <= '1';
                    inited    <= '1';     -- a partir de aquí los errores se recuperan ligero
                    rom_sel   <= '1';
                    idx       <= 0;
                    app_state <= A_EXEC;

                when A_POLL =>
                    txn_kind  <= TXN_READ8; txn_reg <= REG_RANGE_STATUS;
                    txn_start <= '1';       app_state <= A_POLL_WAIT;

                when A_POLL_WAIT =>
                    if txn_done = '1' then
                        if txn_nack = '1' then
                            err_code  <= ERR_NACK_ADDR; app_state <= A_ERROR;
                        elsif txn_rdata(0) = '1' then
                            poll_cnt  <= 0;             app_state <= A_RANGE;
                        elsif poll_cnt = POLL_MAX then
                            err_code  <= ERR_TIMEOUT;   app_state <= A_ERROR;
                        else
                            poll_cnt  <= poll_cnt + 1;  app_state <= A_POLL;
                        end if;
                    end if;

                when A_RANGE =>
                    txn_kind  <= TXN_READ16; txn_reg <= REG_RANGE_MM;
                    txn_start <= '1';        app_state <= A_RANGE_WAIT;

                when A_RANGE_WAIT =>
                    if txn_done = '1' then
                        if txn_nack = '1' then
                            err_code  <= ERR_NACK_ADDR; app_state <= A_ERROR;
                        else
                            range_r   <= txn_rdata;     app_state <= A_CLEAR;
                        end if;
                    end if;

                when A_CLEAR =>
                    txn_kind  <= TXN_WRITE; txn_reg <= REG_INT_CLEAR; txn_wdata <= x"01";
                    txn_start <= '1';       app_state <= A_CLEAR_WAIT;

                when A_CLEAR_WAIT =>
                    if txn_done = '1' then
                        app_state <= A_UPDATE;   -- (se ignora NACK al limpiar)
                    end if;

                when A_UPDATE =>
                    -- Corregir offset constante del sensor (saturando a 0)
                    if unsigned(range_r) > to_unsigned(OFFSET_MM, 16) then
                        distance_mm <= std_logic_vector(unsigned(range_r) - OFFSET_MM);
                    else
                        distance_mm <= (others => '0');
                    end if;
                    data_valid  <= '1';
                    tick_r      <= not tick_r;
                    err_streak  <= 0;            -- medición OK: limpia la racha de errores
                    rom_sel     <= '1';
                    idx         <= 0;
                    app_state   <= A_EXEC;       -- siguiente medición (continuo)

                -- ----------------------------------------------------------
                -- Recuperación LIGERA (sensor ya inicializado): limpia la
                -- interrupción que quedó levantada y re-arma una medición. NO
                -- re-inicializa (un re-init completo en caliente corrompe el
                -- sensor y la calibración se cuelga -> error constante).
                -- ----------------------------------------------------------
                when A_RECLR =>
                    txn_kind  <= TXN_WRITE; txn_reg <= REG_INT_CLEAR; txn_wdata <= x"01";
                    txn_start <= '1';       app_state <= A_RECLR_W;

                when A_RECLR_W =>
                    if txn_done = '1' then
                        poll_cnt  <= 0;
                        rom_sel   <= '1'; idx <= 0;   -- re-arma single-shot (MEAS_ROM)
                        app_state <= A_EXEC;
                    end if;

                when A_ERROR =>
                    -- Recuperación ESCALADA:
                    --  * ya midiendo + pocos errores seguidos -> LIGERA (limpia int + re-arma).
                    --  * demasiados errores seguidos, o error en el init -> DURA: libera el
                    --    bus (pulsos SCL) y re-inicializa. Rompe atascos profundos (bus
                    --    trabado / sensor confundido) que la ligera no resuelve.
                    if inited = '1' and err_streak < ERR_HARD then
                        err_streak <= err_streak + 1;
                        app_state  <= A_RECLR;
                    else
                        err_streak  <= 0;
                        inited      <= '0';
                        sensor_ok   <= '0';
                        data_valid  <= '0';
                        recover_req <= '1';      -- el motor I2C limpia el bus antes del re-init
                        pwr_cnt     <= 0;
                        rom_sel     <= '0';
                        idx         <= 0;
                        app_state   <= A_PWRUP;
                    end if;
            end case;
        end if;
    end process;

end rtl;
