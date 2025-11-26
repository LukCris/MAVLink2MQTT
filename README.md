# MAVLink2MQTT
Questo repository contiene un testbed sperimentale per la valutazione delle prestazioni di 
un collegamento UAV–GCS basato su MQTT tramite TLS, integrato con:
- ArduPilot / DroneKit lato drone (SITL o flight controller reale)
- Ground Control Station (GCS) testuale via CLI
- logging estensivo di:
  - latenza dei comandi MQTT
  - banda TCP/UDP (iperf3)
  - ping ICMP
  - RSSI Wi-Fi
  - telemetria di batteria

---

## Architettura
- Drone / SITL
  - `drone_mqtt.py`
  - si connette al veicolo
  - si connette al broker MQTT TLS (mosquitto) come client `drone-bridge-<uuid>`
  - riceve comandi high-level via topic `uav/<UAV_ID>/cmd`
  - pubblica:
    1. stato: uav/<UAV_ID>/status
    2. telemetria batteria: uav/<UAV_ID>/telemetry/battery
    3. telemetria altitudine (fase di decollo/atterraggio): uav/<UAV_ID>/telemetry/altitude
    4. messaggi di sistema/statustext: `uav/<UAV_ID>/telemetry/sys`
- GCS
  - `ground_station.py`
    - client MQTT TLS
    - CLI interattiva (prompt sitl>)
    - invia comandi JSON sul topic uav/<UAV_ID>/cmd
    - riceve ACK sul topic uav/<UAV_ID>/ack
    - riceve e stampa:
      1. stato UAV (`status`)
      2. batteria (`telemetry/battery`)
      3. altitudine (`telemetry/altitude`)
      4. messaggi STATUSTEXT inoltrati (`telemetry/sys`)
    - logga latenza dei comandi su `latency_metrics.csv` tramite `metrics_logger.py`
- Broker MQTT (mosquitto) con TLS abilitato (CA, certificati client/server)

---

## Esecuzione di uno scenario
1. Drone
```bash
  ./run_drone_scenario.sh dist-5m_tls-on_qos1
```
2. GCS
```bash
  ./run_gcs_scenario.sh dist-5m_tls-on_qos1 10.42.0.20 wlan0
```
Per eseguire anche i test di banda:
```bash
  ./run_iperf_scenario.sh dist-5m_tls-on_qos1 10.42.0.20
```

---

## Produzione grafici
### Lato GCS
Lo script `plot_gcs_metrics.py`:
- crea `./plots` se non esiste;
- se trova i file, genera i grafici:
- Ping ICMP
  - `ping_rtt_vs_time.png` → RTT vs tempo relativo
  - `ping_rtt_vs_seq.png` → RTT vs numero di sequenza
- Latenza MQTT
  - `latency_rtt_vs_time.png` → RTT comandi vs tempo relativo
  - `latency_rtt_hist.png` → distribuzione degli RTT
- Iperf TCP
  - `iperf_tcp_throughput.png` → throughput [Mbps] vs tempo
  - `iperf_tcp_rtt.png` → RTT TCP vs tempo (se disponibile)
  - `iperf_tcp_retransmits.png` → ritrasmissioni per intervallo (se > 0)
- Iperf UDP 10M
  - `iperf_udp_10M_throughput.png` → throughput vs tempo, con jitter e perdita media nel titolo
- Wi-Fi RSSI
  - `wifi_rssi_vs_time.png` → RSSI [dBm] vs tempo

Se un file non esiste o non contiene dati validi, lo script stampa un messaggio e salta il relativo grafico.

### Lato Drone
Lo script `plot_drone_metrics.py`, una volta ricevuto in input il valore di QoS usato:
- legge battery_metrics.csv e forza i tipi numerici;
- calcola il tempo relativo (t_rel_s);
- genera:
  - `battery_voltage.png`: Voltaggio [V] vs tempo
  - `battery_current.png`: Corrente [A] vs tempo
  - `battery_mAh.png`: Energia consumata [mAh] vs tempo
  - `battery_percentage.png`: Livello batteria [%] vs tempo (0–100%)

---

## Requisiti
- Python 3.x
- Virtualenv (o equivalente)
- Librerie Python:
  - paho-mqtt
  - dronekit
  - pymavlink
  - pandas
  - matplotlib
- iperf3 installato sul sistema (lato drone e GCS)
- mosquitto (broker MQTT) con TLS configurato
- ArduPilot SITL (o drone reale compatibile con DroneKit)
- Strumenti di rete:
  - ping
  - iw (per la lettura RSSI Wi-Fi)

---

## Troubleshooting 
- Installare `future` se non è rilevato il modulo `past` nel file `__init__.py` di dronekit
- Modificare la classe Parameters nel file `__init__.py` di dronekit in:
  ```python
  class Parameters(collections.abc.MutableMapping, HasObservers):
  ```
