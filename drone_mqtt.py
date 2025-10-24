# drone_bridge_min.py — MQTT -> serial Pixhawk (solo comandi) con TLS e verifica cert
import paho.mqtt.client as mqtt
from pymavlink import mavutil
import argparse, ssl, sys

CERT_CA = "/etc/mosquitto/ca_certificates/ca.crt"
CERT_FILE = "/etc/mosquitto/certs/client.crt"
KEY_FILE = "/etc/mosquitto/certs/client.key"

parser = argparse.ArgumentParser()
parser.add_argument("--broker", default="mqtt.local")          # usa un NOME che sia nel cert del broker
parser.add_argument("--port", type=int, default=None)          # auto: 8883 se --tls, altrimenti 1883
parser.add_argument("--tls", action="store_true")              # abilita TLS
parser.add_argument("--ca", help="CA file (obbligatorio con --tls)", default=CERT_CA)
parser.add_argument("--cert", help="client cert (se richiesto dal broker)", default=CERT_FILE)
parser.add_argument("--key", help="client key  (se richiesto dal broker)", default=KEY_FILE)
parser.add_argument("--allow-insecure", action="store_true",   # sconsigliato: disabilita hostname check
                    help="(solo test) disabilita verifica hostname")
parser.add_argument("--topic_in", default="uav/alpha/1/1/mav/out/raw")
parser.add_argument("--serial", help="Se si vuole usare un FC reale", default="/dev/ttyACM0")
parser.add_argument("--baud", type=int, default=57600)
parser.add_argument("--sitl", help="Se si vuole usare SITL", default="127.0.0.1:14550")
args = parser.parse_args()

# Porta di default in base a TLS
port = args.port if args.port is not None else (8883 if args.tls else 1883)

# === Connessione alla Pixhawk ===
link = args.udp_fcu if args.udp_fcu else f"serial:{args.serial}:{args.baud}"
master = mavutil.mavlink_connection(link, autoreconnect=True)

def on_message(client, userdata, msg):
    master.write(msg.payload)   # bytes MAVLink → FCU

client = mqtt.Client(client_id="drone-bridge", clean_session=True)

# Auth (se il broker usa username/password)
if args.username:
    client.username_pw_set(args.username, args.password or None)

if args.tls:
    if not args.ca:
        sys.exit("Errore: con --tls devi passare anche --ca <CA file>")
    # Abilita TLS con verifica del certificato server
    client.tls_set(
        ca_certs=args.ca,
        certfile=args.cert,
        keyfile=args.key,
        tls_version=ssl.PROTOCOL_TLSv1_2,
        cert_reqs=ssl.CERT_REQUIRED,
    )
    # Verifica hostname: deve rimanere False (cioè verifica ATTIVA)
    client.tls_insecure_set(args.allow_insecure)
    if args.allow_insecure:
        print("[WARN] Hostname verification DISABILITATA (solo per test).")

client.on_message = on_message
client.connect(args.broker, port, 60)
client.subscribe(args.topic_in, qos=0)
client.loop_forever()
