import paho.mqtt.client as mqtt
import json
import time

MQTT_BROKER = "localhost"
TOPIC_DATA = "iot/sensor/data"
TOPIC_ACK = "iot/sensor/ack"

def on_connect(client, userdata, flags, rc):
    print(" Server connesso. In ascolto dei dati...")
    client.subscribe(TOPIC_DATA)

def on_message(client, userdata, msg):
    t2 = int(time.time() * 1000) 
    
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        t1 = payload.get("t1")
        valore = payload.get("avg", payload.get("freq_hz", 0))
        
        print(f" Ricevuto {valore:.2f} Hz. Invio ACK all'ESP32...")
        
        # t3: Istante esatto in cui il PC ha finito e sta per rispondere
        t3 = int(time.time() * 1000) 
        
        # Prepara la risposta con t1, t2 e t3
        ack_payload = json.dumps({"t1": t1, "t2": t2, "t3": t3})
        client.publish(TOPIC_ACK, ack_payload)
        
    except Exception as e:
        print("Errore:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
