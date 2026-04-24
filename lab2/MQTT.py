import paho.mqtt.client as mqtt
import time

# Define callback functions
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    client.subscribe("test/topic") # Subscribe upon connection

def on_message(client, userdata, msg):
    print(f"Received '{msg.payload.decode()}' on topic '{msg.topic}'")

# Initialize client with V2 API
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# Connect to EE-284A's broker (All Right Now)
client.username_pw_set("eestudent", "arnbiarn")
client.connect("10.136.92.10", 1883, 60)

# Start network loop
client.loop_start()

# Publish a message
client.publish("test/topic/", payload="Hello Julia", qos=1)

# Keep script running
time.sleep(2)
client.disconnect()
client.loop_stop()