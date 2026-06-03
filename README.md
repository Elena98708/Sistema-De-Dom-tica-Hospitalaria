# ============================================================
# Sistema de Domotica Hospitalaria
# ESP32 / WOKWI / MicroPython
# MPU6050 usando GIROSCOPIO + ACELEROMETRO
# Sin servo
# ============================================================

import machine
import time
import dht
import network
import urequests
import struct
import json
import gc
import micropython

micropython.alloc_emergency_exception_buf(100)
gc.collect()

# ----------------------------
# CONFIGURACION DEL NODO
# ----------------------------
NODE_ID = "habitacion101"

SSID = "Wokwi-GUEST"
PASSWORD = ""

SERVER_URL = "http://xxxxx.ngrok-free.app"

# ----------------------------
# UMBRALES BASE
# ----------------------------
TEMP_FRIA = 27
TEMP_ALTA = 31
GAS_ALERTA = 500
GAS_CRITICO = 650
LUZ_OSCURA = 3800

RIESGO_ALERTA = 0.45
RIESGO_CRITICO = 0.70

# ----------------------------
# CONFIGURACION MPU6050
# ----------------------------
ANGULO_PUERTA_ABIERTA = 35.0
ANGULO_PUERTA_MAX = 90.0

GYRO_SENSIBILIDAD = 131.0      # grados/seg para rango ±250 dps
ACCEL_SENSIBILIDAD = 16384.0   # g para rango ±2g
ZONA_MUERTA_GYRO = 1.5

angulo_puerta = 0.0
ultimo_tiempo_gyro = time.time()

# ----------------------------
# SENSORES
# ----------------------------
dht_sensor = dht.DHT22(machine.Pin(25))

mq135 = machine.ADC(machine.Pin(34))
mq135.atten(machine.ADC.ATTN_11DB)

sensor_luz = machine.ADC(machine.Pin(35))
sensor_luz.atten(machine.ADC.ATTN_11DB)

boton_panico = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_DOWN)

i2c = machine.I2C(0, sda=machine.Pin(21), scl=machine.Pin(22))
MPU_ADDR = 0x68

# ----------------------------
# ACTUADORES
# ----------------------------
buzzer_pin = machine.Pin(19, machine.Pin.OUT)
buzzer = machine.PWM(buzzer_pin)
buzzer.duty(0)

rgb_r = machine.Pin(14, machine.Pin.OUT)
rgb_g = machine.Pin(12, machine.Pin.OUT)
rgb_b = machine.Pin(15, machine.Pin.OUT)

vent_1 = machine.Pin(27, machine.Pin.OUT)
vent_2 = machine.Pin(23, machine.Pin.OUT)

luz_1 = machine.Pin(32, machine.Pin.OUT)
luz_2 = machine.Pin(13, machine.Pin.OUT)

calef_1 = machine.Pin(26, machine.Pin.OUT)
calef_2 = machine.Pin(4, machine.Pin.OUT)

# ----------------------------
# VARIABLES GLOBALES
# ----------------------------
estado = {
    "node_id": NODE_ID,
    "t": 0,
    "h": 0,
    "gas": 0,
    "luz": 0,

    "puerta": "CERRADA",
    "puerta_posicion": "CERRADA",
    "puerta_seg": 0,
    "puerta_ang": 0.0,

    "accel_x": 0.0,
    "accel_y": 0.0,
    "accel_z": 0.0,

    "gyro_x": 0.0,
    "gyro_y": 0.0,
    "gyro_z": 0.0,

    "riesgo": 0.0,
    "sistema": "NORMAL",
    "modo_manual": False,

    "vent": False,
    "luz_act": False,
    "calef": False,

    "panico": False
}

boton_presionado = False
inicio_puerta_abierta = None

# ----------------------------
# INTERRUPCION BOTON PANICO
# ----------------------------
def isr_boton(pin):
    global boton_presionado
    boton_presionado = True

boton_panico.irq(trigger=machine.Pin.IRQ_RISING, handler=isr_boton)

# ----------------------------
# FUNCIONES BASICAS
# ----------------------------
def conectar_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Conectando a WiFi Wokwi...")
        wlan.connect(SSID, PASSWORD)

        intentos = 0

        while not wlan.isconnected() and intentos < 20:
            print("Esperando WiFi...", intentos)
            time.sleep(0.5)
            intentos += 1

    if wlan.isconnected():
        print("WiFi conectado")
        print("IP:", wlan.ifconfig()[0])
        return True

    print("No se pudo conectar a WiFi, continuo sin WiFi")
    return False


def buzzer_on():
    buzzer.freq(1000)
    buzzer.duty(512)


def buzzer_off():
    buzzer.duty(0)


def set_rgb(estado_sistema):
    rgb_r.off()
    rgb_g.off()
    rgb_b.off()

    if estado_sistema == "NORMAL":
        rgb_g.on()

    elif estado_sistema == "ALERTA":
        rgb_r.on()
        rgb_g.on()

    elif estado_sistema == "CRITICO":
        rgb_r.on()

    else:
        rgb_b.on()


def activar_motor(p1, p2, activo):
    if activo:
        p1.on()
        p2.off()
    else:
        p1.off()
        p2.off()


def aplicar_actuadores():
    activar_motor(vent_1, vent_2, estado["vent"])
    activar_motor(luz_1, luz_2, estado["luz_act"])
    activar_motor(calef_1, calef_2, estado["calef"])

# ----------------------------
# LOGICA DIFUSA
# ----------------------------
def clamp(x, minimo=0.0, maximo=1.0):
    return max(minimo, min(maximo, x))


def rampa_subida(x, a, b):
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    return (x - a) / (b - a)


def rampa_bajada(x, a, b):
    if x <= a:
        return 1.0
    if x >= b:
        return 0.0
    return (b - x) / (b - a)


def triangular(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


def calcular_riesgo_difuso(temp, gas, puerta_seg):
    temp_baja = rampa_bajada(temp, 20, TEMP_FRIA)
    temp_media = triangular(temp, TEMP_FRIA - 2, 29, TEMP_ALTA + 2)
    temp_alta = rampa_subida(temp, TEMP_ALTA, 36)

    gas_bajo = rampa_bajada(gas, 250, GAS_ALERTA)
    gas_medio = triangular(gas, 400, GAS_ALERTA, GAS_CRITICO)
    gas_alto = rampa_subida(gas, GAS_ALERTA, GAS_CRITICO)

    puerta_corta = rampa_bajada(puerta_seg, 0, 5)
    puerta_media = triangular(puerta_seg, 3, 8, 15)
    puerta_larga = rampa_subida(puerta_seg, 8, 15)

    reglas = []

    reglas.append((temp_baja, 0.20))
    reglas.append((temp_media, 0.45))
    reglas.append((temp_alta, 0.65))
    reglas.append((gas_medio, 0.65))
    reglas.append((gas_alto, 1.00))
    reglas.append((min(temp_alta, puerta_larga), 0.85))
    reglas.append((min(gas_alto, puerta_media), 1.00))
    reglas.append((puerta_larga, 0.55))
    reglas.append((min(gas_bajo, puerta_corta, temp_media), 0.15))

    numerador = 0.0
    denominador = 0.0

    for grado, peso in reglas:
        numerador += grado * peso
        denominador += grado

    if denominador == 0:
        return 0.0

    return clamp(numerador / denominador)


def clasificar_sistema(riesgo):
    if riesgo >= RIESGO_CRITICO:
        return "CRITICO"
    elif riesgo >= RIESGO_ALERTA:
        return "ALERTA"
    else:
        return "NORMAL"

# ----------------------------
# MPU6050 - ACELEROMETRO + GIROSCOPIO
# ----------------------------
def clasificar_posicion_puerta(angulo):
    if angulo <= 5:
        return "CERRADA"
    elif angulo < ANGULO_PUERTA_ABIERTA:
        return "ENTREABIERTA"
    elif angulo < ANGULO_PUERTA_MAX:
        return "ABIERTA"
    else:
        return "TOTALMENTE ABIERTA"


def leer_mpu6050():
    global angulo_puerta
    global ultimo_tiempo_gyro

    try:
        # Despertar MPU6050
        i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')

        # Leer acelerometro desde 0x3B:
        # 0x3B/0x3C = Accel X
        # 0x3D/0x3E = Accel Y
        # 0x3F/0x40 = Accel Z
        accel_data = i2c.readfrom_mem(MPU_ADDR, 0x3B, 6)
        accel_x_raw, accel_y_raw, accel_z_raw = struct.unpack(">hhh", accel_data)

        accel_x = accel_x_raw / ACCEL_SENSIBILIDAD
        accel_y = accel_y_raw / ACCEL_SENSIBILIDAD
        accel_z = accel_z_raw / ACCEL_SENSIBILIDAD

        # Leer giroscopio desde 0x43:
        # 0x43/0x44 = Gyro X
        # 0x45/0x46 = Gyro Y
        # 0x47/0x48 = Gyro Z
        gyro_data = i2c.readfrom_mem(MPU_ADDR, 0x43, 6)
        gyro_x_raw, gyro_y_raw, gyro_z_raw = struct.unpack(">hhh", gyro_data)

        gyro_x = gyro_x_raw / GYRO_SENSIBILIDAD
        gyro_y = gyro_y_raw / GYRO_SENSIBILIDAD
        gyro_z = gyro_z_raw / GYRO_SENSIBILIDAD

        # Para la puerta usamos el giro en Z
        velocidad_puerta = gyro_z

        ahora = time.time()
        dt = ahora - ultimo_tiempo_gyro
        ultimo_tiempo_gyro = ahora

        if abs(velocidad_puerta) < ZONA_MUERTA_GYRO:
            velocidad_puerta = 0.0

        angulo_puerta = angulo_puerta + (velocidad_puerta * dt)

        if angulo_puerta < 0:
            angulo_puerta = 0.0

        if angulo_puerta > ANGULO_PUERTA_MAX:
            angulo_puerta = ANGULO_PUERTA_MAX

        posicion = clasificar_posicion_puerta(angulo_puerta)

        return {
            "ok": True,

            "accel_x": round(accel_x, 2),
            "accel_y": round(accel_y, 2),
            "accel_z": round(accel_z, 2),

            "gyro_x": round(gyro_x, 2),
            "gyro_y": round(gyro_y, 2),
            "gyro_z": round(gyro_z, 2),

            "angulo": round(angulo_puerta, 1),
            "posicion": posicion
        }

    except Exception as e:
        print("Error MPU6050:", e)

        return {
            "ok": False,

            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,

            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,

            "angulo": -1,
            "posicion": "ERROR"
        }

# ----------------------------
# LECTURA DE SENSORES
# ----------------------------
def leer_sensores():
    global inicio_puerta_abierta
    global boton_presionado

    try:
        dht_sensor.measure()
        estado["t"] = dht_sensor.temperature()
        estado["h"] = dht_sensor.humidity()
    except Exception as e:
        print("Error DHT22:", e)

    estado["gas"] = mq135.read()
    estado["luz"] = sensor_luz.read()

    datos_mpu = leer_mpu6050()

    estado["accel_x"] = datos_mpu["accel_x"]
    estado["accel_y"] = datos_mpu["accel_y"]
    estado["accel_z"] = datos_mpu["accel_z"]

    estado["gyro_x"] = datos_mpu["gyro_x"]
    estado["gyro_y"] = datos_mpu["gyro_y"]
    estado["gyro_z"] = datos_mpu["gyro_z"]

    estado["puerta_ang"] = datos_mpu["angulo"]
    estado["puerta_posicion"] = datos_mpu["posicion"]

    puerta_abierta = (
        datos_mpu["angulo"] > 5 or
        datos_mpu["angulo"] == -1
    )

    if puerta_abierta:
        estado["puerta"] = "ABIERTA"

        if inicio_puerta_abierta is None:
            inicio_puerta_abierta = time.time()

        estado["puerta_seg"] = int(time.time() - inicio_puerta_abierta)

    else:
        estado["puerta"] = "CERRADA"
        inicio_puerta_abierta = None
        estado["puerta_seg"] = 0

    if boton_presionado:
        estado["panico"] = True
        boton_presionado = False

# ----------------------------
# CONTROL LOCAL AUTOMATICO
# ----------------------------
def control_automatico():
    estado["riesgo"] = calcular_riesgo_difuso(
        estado["t"],
        estado["gas"],
        estado["puerta_seg"]
    )

    if estado["panico"]:
        estado["riesgo"] = 1.0

    estado["sistema"] = clasificar_sistema(estado["riesgo"])

    if not estado["modo_manual"]:
        if estado["sistema"] == "NORMAL":
            estado["vent"] = estado["t"] > TEMP_ALTA or estado["gas"] > GAS_ALERTA
            estado["calef"] = estado["t"] < TEMP_FRIA
            estado["luz_act"] = estado["luz"] < LUZ_OSCURA and estado["puerta"] == "ABIERTA"

        elif estado["sistema"] == "ALERTA":
            estado["vent"] = estado["gas"] > GAS_ALERTA or estado["t"] > TEMP_ALTA
            estado["calef"] = False
            estado["luz_act"] = True

        elif estado["sistema"] == "CRITICO":
            estado["vent"] = True
            estado["calef"] = False
            estado["luz_act"] = True

    aplicar_actuadores()

    if estado["sistema"] == "CRITICO":
        buzzer_on()
    else:
        buzzer_off()

    set_rgb(estado["sistema"])

# ----------------------------
# COMUNICACION CON SERVIDOR
# ----------------------------
def enviar_datos_servidor():
    try:
        url = SERVER_URL + "/api/data"
        headers = {"Content-Type": "application/json"}
        respuesta = urequests.post(url, headers=headers, data=json.dumps(estado))
        respuesta.close()
    except Exception as e:
        print("Error enviando datos:", e)


def recibir_comandos_servidor():
    try:
        url = SERVER_URL + "/api/commands/" + NODE_ID
        respuesta = urequests.get(url)
        datos = respuesta.json()
        respuesta.close()

        if datos.get("has_command"):
            comando = datos.get("command")

            if comando == "modo_auto":
                estado["modo_manual"] = False

            elif comando == "modo_manual":
                estado["modo_manual"] = True

            elif comando == "reset":
                estado["panico"] = False
                estado["modo_manual"] = False
                estado["vent"] = False
                estado["calef"] = False
                estado["luz_act"] = False

            elif comando == "vent_toggle":
                estado["modo_manual"] = True
                estado["vent"] = not estado["vent"]

            elif comando == "luz_toggle":
                estado["modo_manual"] = True
                estado["luz_act"] = not estado["luz_act"]

            elif comando == "calef_toggle":
                estado["modo_manual"] = True
                estado["calef"] = not estado["calef"]

            aplicar_actuadores()

    except Exception as e:
        print("Error recibiendo comandos:", e)

# ----------------------------
# PROGRAMA PRINCIPAL
# ----------------------------
wifi_ok = conectar_wifi()

timer_sensores = 0
timer_envio = 0
timer_comandos = 0
timer_wifi = 0

while True:
    try:
        ahora = time.time()

        if ahora - timer_sensores >= 1:
            leer_sensores()
            control_automatico()

            print("================================")
            print("Nodo: " + str(NODE_ID))
            print("WiFi: " + ("CONECTADO" if wifi_ok else "SIN WIFI"))

            print("Temp: " + str(estado["t"]) + " C | Hum: " + str(estado["h"]) + " %")
            print("Gas: " + str(estado["gas"]) + " | Luz: " + str(estado["luz"]))

            print("Puerta: " + str(estado["puerta"]))
            print("Posicion: " + str(estado["puerta_posicion"]))
            print("Angulo puerta: " + str(estado["puerta_ang"]) + " grados")
            print("Tiempo abierta: " + str(estado["puerta_seg"]) + " s")

            print("Acelerometro X: " + str(estado["accel_x"]) + " g")
            print("Acelerometro Y: " + str(estado["accel_y"]) + " g")
            print("Acelerometro Z: " + str(estado["accel_z"]) + " g")

            print("Giroscopio X: " + str(estado["gyro_x"]) + " grados/seg")
            print("Giroscopio Y: " + str(estado["gyro_y"]) + " grados/seg")
            print("Giroscopio Z: " + str(estado["gyro_z"]) + " grados/seg")

            print("Riesgo: " + str(round(estado["riesgo"], 2)) + " | Estado: " + str(estado["sistema"]))
            print("Ventilador: " + ("ON" if estado["vent"] else "OFF"))
            print("Luz habitacion: " + ("ON" if estado["luz_act"] else "OFF"))
            print("Calefactor: " + ("ON" if estado["calef"] else "OFF"))
            print("Buzzer: " + ("ON" if estado["sistema"] == "CRITICO" else "OFF"))
            print("================================")

            timer_sensores = ahora

        # Comunicacion con servidor desactivada para Wokwi
        # if ahora - timer_envio >= 2:
        #     enviar_datos_servidor()
        #     timer_envio = ahora

        # if ahora - timer_comandos >= 2:
        #     recibir_comandos_servidor()
        #     timer_comandos = ahora

        # Verificacion WiFi desactivada para que Wokwi no se bloquee
        # if ahora - timer_wifi >= 20:
        #     verificar_wifi()
        #     timer_wifi = ahora

        gc.collect()
        time.sleep(0.05)

    except Exception as e:
        print("Error en loop principal:", e)
        time.sleep(1)
