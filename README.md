# Robot Seguidor de Línea con Brazo Robótico - Documentación

## Proyecto por Adrian Damas Garnica

**Semestre:** 2026/2  
**Asignatura:** Diseño de Sistemas Digitales
**Institución:** ESCOM - IPN

---

## Índice
1. [Resumen del Proyecto](#resumen-del-proyecto)
2. [Vistas del Sistema](#vistas-del-sistema)
3. [Arquitectura del Sistema](#arquitectura-del-sistema)
4. [Módulos de VHDL](#módulos-de-vhdl)
5. [Máquina de Estados](#máquina-de-estados)
6. [Conversión Coordenadas Polares a PWM](#conversión-coordenadas-polares-a-pwm)
7. [Asignación de Pines](#asignación-de-pines)
8. [Conclusiones](#conclusiones)

---

## Resumen del Proyecto

El presente proyecto consiste en el diseño e implementación de un robot seguidor de línea autónomo equipado con un brazo robótico de 4 grados de libertad. El sistema utiliza una FPGA Cyclone IV como unidad de procesamiento principal, integrando sensores infrarrojos QRD1114 para el seguimiento de línea, un sensor VL6180X para detección de objetos mediante LIDAR, y un sistema de control PWM para el posicionamiento preciso del brazo robótico.

**Características principales:**
- Navegación autónoma por pista de línea negra
- Detección y localización de objetos mediante sensor de distancia óptico
- Brazo robótico de 4 ejes con control de posición preciso
- Sistema de adquisición y depósito de objetos
- Retroalimentación visual mediante LEDs de estado

---

## Vistas del Sistema

### Esquemático Eléctrico

![Esquemático V3](Esquematico%20V3.png)

*Figura 1: Diagrama esquemático del sistema mostrando la interconexión de sensores QRD1114, controlador L293D, reguladores de voltaje y conexiones a la FPGA.*

**Componentes principales:**
- **Sensores QRD1114:** Detectan la línea negra mediante reflexión infrarroja
- **LM393:** Comparadores para señal digital de sensores
- **L293D:** Puente H para control de motores DC
- **VL6180X:** Sensor de distancia por tiempo de vuelo (ToF)
- **Reguladores:** LM317 para 5V y 3.3V estables

### PCB Diseñado

![PCB V3](PCB%20V3.png)

*Figura 2: Diseño de la PCB mostrando la distribución de componentes y ruteo de pistas. Dimensiones: 100mm x 84mm.*

**Características del PCB:**
- **Capas:** 2 capas (superior e inferior)
- **Conectores:** Headers de 2.54mm para fácil conexión
- **Alimentación:** Jack DC barrel + reguladores lineales
- **Sensores:** 3 módulos QRD1114 configurables
- **Motores:** Conectores para 2 motores DC con reductora

### Modelo 3D del Robot

![3D Robot](3D%20Robot.png)

*Figura 3: Renderizado 3D del robot mostrando la distribución de componentes y grados de libertad del brazo robótico.*

**Dimensiones del brazo:**
- **Eje 1 (φ):** Base rotativa - 180°
- **Eje 2 (θ₁):** Primer segmento - 100mm
- **Eje 3 (θ₂):** Segundo segmento - 100mm  
- **Eje 4 (θ₃):** Tercer segmento con pinza - 77.6mm
- **Pinza:** Apertura de 25.269mm

---

## Arquitectura del Sistema

```
                    ┌─────────────────────────────────────────┐
                    │           FPGA Cyclone IV               │
                    │                                         │
    ┌──────────┐    │  ┌──────────┐  ┌──────────┐             │
    │ Sensores │───►│  │ LIDAR    │  │ polarPWM │             │
    │ QRD1114  │    │  │ Module   │  │ Module   │             │
    └──────────┘    │  └──────────┘  └─────┬────┘             │
                    │                      │                  │
    ┌──────────┐    │  ┌──────────┐        │                  │
    │  LIDAR   │───►│  │ Maquina  │        ▼                  │
    │ VL6180X  │    │  │ Estados  │  ┌──────────┐             │
    └──────────┘    │  │          │  │ Servos   │             │
                    │  │          │  │ (4 ejes) │             │
    ┌──────────┐    │  │          │  └──────────┘             │
    │ Motores  │◄───┤  │          │                           │
    │   L293D  │    │  └──────────┘                           │
    └──────────┘    │                                         │
                    └─────────────────────────────────────────┘
```

**Flujo de operación:**
1. Los sensores QRD1114 detectan la línea negra
2. La máquina de estados decide la dirección de movimiento
3. El LIDAR escanea el área cuando se detecta zona de búsqueda
4. El módulo polarPWM posiciona el brazo para tomar el objeto
5. El robot regresa al inicio y deposita el objeto

---

## Módulos de VHDL

### 1. SeguidorLinea_Brazo (Módulo Principal)

**Archivo:** `SeguidorLinea_Brazo.vhd`

**Descripción:** Módulo de nivel superior que integra todos los subsistemas del robot.

**Funciones principales:**
- Instanciación de los módulos hijos (polarPWM, LIDAR, MaquinaEstados)
- Interconexión de señales entre módulos
- Generación de señal de 1Hz para debug
- Control básico del brazo robótico

**Señales de entrada/salida:**
| Señal | Tipo | Descripción |
|-------|------|-------------|
| `clk` | in | Reloj principal de 50 MHz |
| `reset` | in | Reset global (activo bajo) |
| `servo_*` | out | Señales PWM para servomotores |
| `sensor_*` | in | Lectura de sensores QRD1114 |
| `motor*_in*` | out | Control de dirección de motores |
| `i2c_*` | in/out | Comunicación I2C con VL6180X |

**Código resumido:**
```vhdl
entity SeguidorLinea_Brazo is
Port (
  clk, reset : in std_logic;
  servo_phi, servo_theta1, ... : out std_logic;
  sensor_izq, sensor_der, ... : in std_logic;
  motor1_in1, motor1_in2, ... : out std_logic;
  i2c_scl, i2c_sda : inout std_logic;
  ...
);
end SeguidorLinea_Brazo;
```

---

### 2. polarPWM (Control de Servomotores)

**Archivo:** `polarPWM.vhd`

**Descripción:** Convierte coordenadas polares (φ, θ, radio) en señales PWM para controlar 4 servomotores del brazo robótico.

**Especificaciones técnicas:**
- **Frecuencia PWM:** 50Hz (periodo de 20ms)
- **Rango de duty cycle:** 0.5ms - 2.0ms
- **Resolución angular:** 0° - 180° (mapeado a 8 bits)
- **Tabla LUT:** 181 valores precalculados para conversión lineal

**Fórmula de conversión:**
```
Duty Cycle = PWM_MIN + (ángulo × PWM_RANGE / 180)
donde:
  PWM_MIN = 25000 (0.5ms a 50MHz)
  PWM_MAX = 100000 (2.0ms a 50MHz)
  PWM_RANGE = PWM_MAX - PWM_MIN
```

**Estructura del módulo:**
```vhdl
entity polarPWM is
    Port (
        clk, rst : in std_logic;
        phi_in, theta_in : in std_logic_vector(7 downto 0);
        gripper_in : in std_logic_vector(7 downto 0);
        pwm_phi, pwm_theta1, ... : out std_logic
    );
end polarPWM;
```

**Proceso de generación PWM:**
```vhdl
-- Contador de periodo (20ms)
if cuenta_pwm = PWM_PERIOD-1 then
    cuenta_pwm <= 0;
else
    cuenta_pwm <= cuenta_pwm + 1;
end if;

-- Generación de duty cycle
pwm_phi_sig <= '1' when cuenta_pwm < ANGLE_PWM(angulo_phi) else '0';
```

---

### 3. LIDAR (Sensor VL6180X)

**Archivo:** `LIDAR.vhd`

**Descripción:** Controla el sensor de distancia óptico VL6180X mediante comunicación I2C para escanear el área de -45° a +45° y detectar el objeto más cercano.

**Características:**
- **Rango de escaneo:** -45° a +45° (19 puntos)
- **Resolución angular:** 5° por paso
- **Protocolo:** I2C a 100 kHz
- **Algoritmo:** Búsqueda del mínimo + promedio ponderado

**Máquina de estados (LIDAR):**
1. **IDLE:** Espera de inicio
2. **INIT:** Inicialización del sensor
3. **STARTING:** Configuración inicial
4. **WAIT_M:** Espera de medición
5. **READ_M:** Lectura de distancia
6. **NEXT_PT:** Siguiente punto de escaneo
7. **REFINE:** Refinamiento de búsqueda
8. **CALC:** Cálculo de mejor ángulo
9. **OUTPUT:** Entrega de resultados
10. **COMPLETE:** Fin del escaneo

**Código de escaneo:**
```vhdl
when READ_M =>
    measured_dist <= to_integer(unsigned(i2c_read_data));
    distances(scan_idx) <= measured_dist;
    phi_values(scan_idx) <= phi_angle;
    
    if measured_dist > 0 and measured_dist < min_dist then
        min_dist <= measured_dist;
        min_idx <= scan_idx;
    end if;
    state <= NEXT_PT;
```

---

### 4. MaquinaEstados (Seguidor de Línea)

**Archivo:** `MaquinaEstados.vhd`

**Descripción:** Controla la navegación autónoma del robot siguiendo la línea negra, detectando zonas de búsqueda y coordinando la captura de objetos.

**Estados principales:**

| Estado | Descripción | Acción |
|--------|-------------|--------|
| `INICIO` | Inicialización | Reset de variables |
| `SEGUIR_LINEA` | Seguimiento normal | Ajuste de dirección según sensores |
| `DETECTA_ZONA_NEGRA` | Zona de búsqueda detectada | Detener y preparar escaneo |
| `EXPLORAR_ZONA` | Escaneo con LIDAR | Activar sensor VL6180X |
| `BUSCAR_OBJETO` | Acercamiento al objeto | Mover hacia coordenadas |
| `AGARRAR_OBJETO` | Captura | Cerrar pinza |
| `CALCULAR_RETORNO` | Planear regreso | Invertir dirección |
| `RETORNAR_INICIO` | Vuelta a la base | Seguir línea de retorno |
| `DEJAR_OBJETO` | Descarga | Abrir pinza en zona segura |
| `MEMORIZAR_ZONA` | Registro | Guardar zona explorada |
| `CONTINUAR_PISTA` | Reanudar | Volver a seguimiento |
| `ZONA_BLANCA` | Final de pista | Detener completamente |
| `ERROR` | Manejo de fallos | Parada de emergencia |

**Lógica de seguimiento:**
```vhdl
-- Ambos sensores en blanco: perdió línea
if sensor_izq_reg = '0' and sensor_der_reg = '0' then
    -- Buscar última posición conocida
    if girando_derecha = '1' then
        motor1_in1 <= '1'; motor1_in2 <= '0';
        motor2_in1 <= '0'; motor2_in2 <= '1';
    else
        motor1_in1 <= '0'; motor1_in2 <= '1';
        motor2_in1 <= '1'; motor2_in2 <= '0';
    end if;
    
-- Sensor izquierdo detecta línea: girar izquierda
elsif sensor_izq_reg = '1' then
    motor1_in1 <= '0'; motor1_in2 <= '1';
    motor2_in1 <= '0'; motor2_in2 <= '1';
    
-- Sensor derecho detecta línea: girar derecha
elsif sensor_der_reg = '1' then
    motor1_in1 <= '1'; motor1_in2 <= '0';
    motor2_in1 <= '1'; motor2_in2 <= '0';
    
-- Ambos en línea: avanzar recto
else
    motor1_in1 <= '1'; motor1_in2 <= '0';
    motor2_in1 <= '1'; motor2_in2 <= '0';
end if;
```

---

## Máquina de Estados - Diagrama Detallado

```
                                    ┌──────────────┐
                           ┌───────►│  ZONA_BLANCA │
                           │        └──────────────┘
                           │                 ▲
┌─────────┐      ┌────────────────┐          │
│  INICIO │─────►│ SEGUIR_LINEA   │──────────┘
└─────────┘      └──────┬─────────┘     (zona blanca)
                        │
         ┌──────────────┼──────────────┐
         │              │              │
         ▼              ▼              ▼
┌─────────────────┐ ┌─────────────────┐ ┌──────────────┐
│ DETECTA_ZONA    │ │  (ambos=1 y     │ │  (ambos=0)   │
│     NEGRA       │ │   centro=1)     │ │  Búsqueda    │
└────────┬────────┘ └─────────────────┘ └──────────────┘
         │
         ▼
┌─────────────────┐
│  EXPLORAR_ZONA  │
│   (LIDAR on)    │
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
    ▼         ▼
┌─────────┐ ┌──────────────┐
│ BUSCAR  │ │ MEMORIZAR    │
│ OBJETO  │ │   ZONA       │
└────┬────┘ └──────────────┘
     │
     ▼
┌──────────────┐
│ AGARRAR      │
│ OBJETO       │
└────┬─────────┘
     │
     ▼
┌──────────────┐
│ CALCULAR     │
│ RETORNO      │
└────┬─────────┘
     │
     ▼
┌──────────────┐
│ RETORNAR     │
│   INICIO     │
└────┬─────────┘
     │
     ▼
┌──────────────┐
│  DEJAR       │
│   OBJETO     │
└────┬─────────┘
     │
     ▼
┌──────────────┐
│ CONTINUAR    │
│   PISTA      │
└──────────────┘
```

**Condiciones de transición:**
- **INICIO → SEGUIR_LINEA:** Inicialización completada
- **SEGUIR_LINEA → DETECTA_ZONA_NEGRA:** `sensor_izq = '1' AND sensor_der = '1' AND sensor_centro = '1'`
- **EXPLORAR_ZONA → BUSCAR_OBJETO:** `lidar_complete = '1' AND distancia < 200`
- **EXPLORAR_ZONA → MEMORIZAR_ZONA:** `lidar_complete = '1' AND distancia >= 200`
- **BUSCAR_OBJETO → AGARRAR_OBJETO:** `distancia < 50`
- **RETORNAR_INICIO → DEJAR_OBJETO:** Detección de zona de inicio
- **RETORNAR_INICIO → ERROR:** Timeout de retorno

---

## Conversión Coordenadas Polares a PWM

### Fundamento Matemático

El sistema de coordenadas polares describe la posición de un punto mediante:
- **Radio (r):** Distancia desde el origen
- **Ángulo (φ):** Medida angular desde el eje de referencia

Para el brazo robótico de 4 ejes:
1. **Eje 1 (φ):** Rotación horizontal de la base
2. **Eje 2 (θ₁):** Primer segmento vertical
3. **Eje 3 (θ₂):** Segundo segmento (ángulo complementario)
4. **Eje 4 (θ₃):** Tercer segmento con pinza

### Proceso de Conversión

**Paso 1: Entrada de coordenadas polares**
```
Entrada: φ (0-180°), θ (0-180°), radio (0-255)
Resolución: 8 bits (256 niveles)
```

**Paso 2: Cálculo de ángulos individuales**
```vhdl
angulo_phi   <= to_integer(unsigned(phi_in));      -- 0-180°
angulo_t1    <= to_integer(unsigned(theta_in));    -- 0-180°
angulo_t2    <= 180 - angulo_t1;                    -- Complementario
angulo_t3    <= angulo_t1;                          -- Mismo que θ₁
```

**Paso 3: Mapeo lineal a duty cycle**
```
Fórmula: PWM = PWM_MIN + (ángulo × PWM_RANGE / 180)

Ejemplo para 90°:
PWM = 25000 + (90 × 75000 / 180)
PWM = 25000 + 37500 = 62500

Equivalencia en tiempo:
62500 / 50000000 = 1.25ms (posición central)
```

**Paso 4: Generación de señal PWM**
```vhdl
-- Tabla de lookup (LUT) para 181 valores
type angle_to_pwm is array (0 to 180) of integer;
constant ANGLE_PWM : angle_to_pwm := (
    25000, 2751, 3002, ..., 47722  -- 181 valores
);

-- Generación de PWM
pwm_sig <= '1' when cuenta_pwm < ANGLE_PWM(angulo) else '0';
```

### Tabla de Conversión

| Ángulo (°) | Duty Cycle | Tiempo (ms) | Posición |
|------------|------------|-------------|----------|
| 0 | 25000 | 0.50 | Extremo izquierdo |
| 45 | 43750 | 0.875 | 25% del recorrido |
| 90 | 62500 | 1.25 | Centro |
| 135 | 81250 | 1.625 | 75% del recorrido |
| 180 | 100000 | 2.00 | Extremo derecho |

### Consideraciones de Implementación

**Ventajas de usar LUT:**
- Cálculo en tiempo constante (1 ciclo de reloj)
- Sin operaciones de punto flotante
- Precisión garantizada

**Precisión del sistema:**
- Resolución angular: 180° / 256 = 0.703° por LSB
- Resolución temporal: 20ns (periodo de reloj de 50MHz)
- Error máximo de cuantización: ±0.35°

---

## Asignación de Pines

### Tabla de Asignación de Pines FPGA

| Componente | Señal | Pin FPGA | Dirección | Notas |
|------------|-------|----------|-----------|-------|
| **Reloj** | clk_50 | | IN | 50 MHz |
| **Reset** | reset | | IN | Activo bajo |
| **Sensores Línea** | | | | |
| QRD1114-1 | sensor_izq | | IN | Izquierda |
| QRD1114-2 | sensor_der | | IN | Derecha |
| QRD1114-3 | sensor_cent | | IN | Central |
| QRD1114-4 | sensor_del | | IN | Delantero |
| QRD1114-5 | sensor_tras | | IN | Trasero |
| **Motores DC** | | | | |
| Motor 1 | motor1_in1 | | OUT | Dirección A |
| Motor 1 | motor1_in2 | | OUT | Dirección B |
| Motor 1 | motor1_en | | OUT | PWM Velocidad |
| Motor 2 | motor2_in1 | | OUT | Dirección A |
| Motor 2 | motor2_in2 | | OUT | Dirección B |
| Motor 2 | motor2_en | | OUT | PWM Velocidad |
| **Servomotores** | | | | |
| Eje 1 (φ) | servo_phi | | OUT | Base |
| Eje 2 (θ₁) | servo_theta1 | | OUT | Segmento 1 |
| Eje 3 (θ₂) | servo_theta2 | | OUT | Segmento 2 |
| Eje 4 (θ₃) | servo_theta3 | | OUT | Pinza |
| **LIDAR VL6180X** | | | | |
| I2C Clock | i2c_scl | | OUT | 100 kHz |
| I2C Data | i2c_sda | | INOUT | Bidireccional |
| I2C GPIO | i2c_gpio | | IN | Opcional |
| **Debug** | | | | |
| LED Estado | led_estado | | OUT | Parpadeo 1Hz |
| LED Error | led_error | | OUT | Indicador fallo |
| Debug [15:0] | debug_out | | OUT | Datos depuración |
| **Configuración** | | | | |
| Modo [2:0] | sw_mode | | IN | Select modo |
| Velocidad [1:0] | sw_vel | | IN | Select velocidad |
| Test | sw_test | | IN | Modo test |

### Notas de Asignación:

1. **Pines de alimentación:**
   - VCC: 3.3V (lógica FPGA)
   - VCC_5V: 5V (motores y servos)
   - GND: Tierra común

2. **Pines de sensores QRD1114:**
   - Salida digital (0 = blanco, 1 = negro)
   - Umbrales ajustados con potenciómetros

3. **Pines de servomotores:**
   - Frecuencia: 50Hz
   - Rango: 1-2ms de duty cycle
   - Voltaje: 5V

4. **Pines de motores DC:**
   - Control mediante L293D
   - PWM para velocidad
   - IN1/IN2 para dirección

5. **Pines I2C:**
   - Resistencias de pull-up: 4.7kΩ
   - Velocidad: 100 kHz (estándar)

---

## Conclusiones

### Logros del Proyecto

1. **Integración completa:** Se logró integrar exitosamente todos los subsistemas (seguimiento de línea, brazo robótico, detección LIDAR) en una única FPGA.

2. **Control preciso:** El uso de tablas LUT para la conversión polar-PWM permite un control preciso y en tiempo real de los 4 servomotores.

3. **Navegación autónoma:** La máquina de estados implementada permite al robot navegar de forma autónoma, detectar objetos y realizar tareas de pick-and-place.

4. **Diseño modular:** La arquitectura en módulos VHDL facilita el mantenimiento, pruebas y futuras ampliaciones del sistema.

### Trabajo Futuro

- Implementar algoritmos de visión por cámara para mayor precisión
- Agregar comunicación inalámbrica (Bluetooth/WiFi)
- Mejorar la cinemática inversa del brazo
- Implementar filtrado de Kalman para el LIDAR

### Recursos Utilizados

- **FPGA:** Cyclone IV EP4CE6E22C8
- **Herramientas:** Quartus Prime Lite 23.1
- **Lenguaje:** VHDL
- **Sensores:** QRD1114 (3x), VL6180X (1x)
- **Actuadores:** Servomotores (4x), Motores DC con reductora (2x)

---

## Anexos

### A. Archivos del Proyecto

```
SeguidorLinea_Brazo/
├── SeguidorLinea_Brazo.vhd # Módulo principal
├── polarPWM.vhd # Control de servos
├── LIDAR.vhd # Sensor VL6180X
├── MaquinaEstados.vhd # Seguidor de línea
├── Practica5.vhd # Referencia (opcional)
├── SeguidorLinea_Brazo.qpf # Proyecto Quartus
├── SeguidorLinea_Brazo.qsf # Asignaciones
└── DOCUMENTACION.md # Este archivo
```

### B. Comandos de Compilación

```bash
# Compilación completa en Quartus
quartus_map SeguidorLinea_Brazo -c SeguidorLinea_Brazo
quartus_fit SeguidorLinea_Brazo
quartus_asm SeguidorLinea_Brazo
quartus_sta SeguidorLinea_Brazo

# Programación de FPGA
quartus_pgm -c "USB-Blaster" -o p,SeguidorLinea_Brazo.sof
```

### C. Referencias Bibliográficas

1. Altera Corporation. "Cyclone IV Device Handbook." 2023.
2. Pololu Corporation. "QRD1114 Reflective Optical Sensor." Datasheet.
3. STMicroelectronics. "VL6180X Time-of-Flight Distance Sensor." Datasheet.
4. Texas Instruments. "L293D Quadruple Half-H Driver." Datasheet.
5. IEEE Standard 1076-2008. "VHDL Language Reference Manual."

---

**Fecha de elaboración:** 10 de Mayo del 2026  
**Versión:** 3.0  
**Última actualización:** 10 de Mayo del 2026
