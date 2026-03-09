import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configurazione Porta Seriale
try:
    ser = serial.Serial('COM7', 9600, timeout=1)
    # Pulizia buffer iniziale per evitare residui di avvio
    ser.reset_input_buffer() 
except:
    print("Errore: Impossibile aprire COM7. Verifica la connessione.")
    exit()

# Contenitori dati
data_lists = [[] for _ in range(11)] 
buffer = []
start_time = None

# Soglie temporali (secondi)
LIMITS = {'15m': 15*60, '1h': 60*60, '8h': 8*60*60, '24h': 24*60*60}

# Setup Figura e Assi (stessa configurazione tua)
fig = plt.figure(figsize=(12, 10))
gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1.8, 1.8])

ax_temp = fig.add_subplot(gs[0, 0]); ax_temp.set_title("Temperatura (°C)")
ax_hum  = fig.add_subplot(gs[0, 1]); ax_hum.set_title("Umidità (%)")
ax_aqi  = fig.add_subplot(gs[1, 0]); ax_aqi.set_title("AQI")
ax_voc  = fig.add_subplot(gs[1, 1]); ax_voc.set_title("VOC (mg/m³)")
ax_eco2 = fig.add_subplot(gs[1, 2]); ax_eco2.set_title("eCO2 / ETH")
ax_co   = fig.add_subplot(gs[2:4, :]); ax_co.set_title("Monitoraggio CO")

# Linee Grafiche
l_temp, = ax_temp.plot([], [], 'r-')
l_hum,  = ax_hum.plot([], [], 'b-')
l_aqi,  = ax_aqi.plot([], [], 'g-')
l_voc,  = ax_voc.plot([], [], 'm-')
l_eth,  = ax_eco2.plot([], [], label="ETH")
l_eco2, = ax_eco2.plot([], [], label="eCO2 (20m)")

l_co_ist, = ax_co.plot([], [], label="Istantanea", color='green')
l_co_15m, = ax_co.plot([], [], label="Media 15 min", color='blue', visible=False)
l_co_1h,  = ax_co.plot([], [], label="Media 1 ora", color='orange', visible=False)
l_co_8h,  = ax_co.plot([], [], label="Media 8 ore", color='red', visible=False)
l_co_24h, = ax_co.plot([], [], label="Media 24 ore", color='darkred', visible=False)

# Soglie CO
th_instant = ax_co.axhline(100, color='green', ls='--', alpha=0.6, visible=True)
th_15m = ax_co.axhline(87, color='blue', ls='--', alpha=0.6, visible=False)
th_1h  = ax_co.axhline(35, color='orange', ls='--', alpha=0.6, visible=False)
th_8h  = ax_co.axhline(9, color='red', ls='--', alpha=0.6, visible=False)
th_24h = ax_co.axhline(6, color='darkred', ls='--', alpha=0.6, visible=False)

ax_eco2.legend(fontsize='x-small')
ax_co.legend(loc='upper right', fontsize='small', ncol=2)

def is_valid_data(b):
    """
    Controlla se i dati sono realistici.
    b[0] è la temperatura, b[2] è l'AQI.
    Se temp è 0 o AQI è esattamente 0 o 50 (valore di default ENS160 in warmup), scartiamo.
    """
    temp_val = b[0]
    aqi_val = b[2]
    # Se la temperatura è 0.0 o l'AQI è 0 (non inizializzato), i dati non sono validi
    if temp_val == 0.0 or aqi_val == 0:
        return False
    return True

def update(frame):
    global buffer, start_time

    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        if not line: continue
        try:
            val = float(line)
            buffer.append(val)
        except ValueError: continue

        if len(buffer) == 11:
            # VALIDAZIONE: Se i dati sono "vuoti" o di warmup, svuota il buffer e ignora
            if not is_valid_data(buffer):
                buffer = []
                continue

            if start_time is None: start_time = time.time()
            elapsed = time.time() - start_time
            
            # 0:temp, 1:hum, 2:aqi, 3:mgm3, 4:eth, 5:eco2_20m, 6:co_ist
            for i in range(7):
                data_lists[i].append(buffer[i])
            
            # Logica CO temporizzata (15m, 1h, 8h, 24h)
            for i, key in enumerate(['15m', '1h', '8h', '24h'], start=7):
                if elapsed >= LIMITS[key]:
                    data_lists[i].append(buffer[i])
                    # Attiva visibilità
                    if key == '15m': l_co_15m.set_visible(True); th_15m.set_visible(True)
                    if key == '1h':  l_co_1h.set_visible(True);  th_1h.set_visible(True)
                    if key == '8h':  l_co_8h.set_visible(True);  th_8h.set_visible(True)
                    if key == '24h': l_co_24h.set_visible(True); th_24h.set_visible(True)
                else:
                    data_lists[i].append(np.nan)

            buffer = []

    # Aggiornamento grafici
    x = range(len(data_lists[0]))
    if len(x) > 0:
        l_temp.set_data(x, data_lists[0])
        l_hum.set_data(x, data_lists[1])
        l_aqi.set_data(x, data_lists[2])
        l_voc.set_data(x, data_lists[3])
        l_eth.set_data(x, data_lists[4])
        l_eco2.set_data(x, data_lists[5])
        l_co_ist.set_data(x, data_lists[6])
        l_co_15m.set_data(x, data_lists[7])
        l_co_1h.set_data(x, data_lists[8])
        l_co_8h.set_data(x, data_lists[9])
        l_co_24h.set_data(x, data_lists[10])

        for ax in [ax_temp, ax_hum, ax_aqi, ax_voc, ax_eco2, ax_co]:
            ax.relim()
            ax.autoscale_view()
        ax_co.legend(loc='upper right', fontsize='small', ncol=2)

ani = FuncAnimation(fig, update, interval=500, cache_frame_data=False)
plt.tight_layout()
plt.show()