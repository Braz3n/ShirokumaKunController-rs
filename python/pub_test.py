import paho.mqtt.client as mqtt

def read_secret(file_name: str):
    SECRETS_PATH = "../secrets/"
    SECRETS_SUFFIX = ".txt"
    filename = f"{SECRETS_PATH}{file_name}{SECRETS_SUFFIX}"
    with open(filename, 'r') as f:
        return f.readline().strip()

# Define event callbacks
def on_connect(client, userdata, flags, rc):
    print("rc: " + str(rc))

def on_message(client, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_publish(client, obj, mid):
    print("mid: " + str(mid))

def on_subscribe(client, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
    print(string)

mqttc = mqtt.Client()
mqttc.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
# Assign event callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe

# Uncomment to enable debug messages
mqttc.on_log = on_log

broker_user = read_secret("broker_user")
broker_pass = read_secret("broker_pass")
broker_url  = read_secret("broker_url")
broker_port = 8883

topic = 'aircon/mode'

# Connect
mqttc.username_pw_set(username=broker_user, password=broker_pass)
rc = mqttc.connect(broker_url, broker_port)

if rc != 0:
    print("Failed to connect!")
    exit(1)

# Publish a message
mqttc.publish(topic, "OFF")
if rc != 0:
    print("Failed to publish!")
    exit(1)

# Continue the network loop, exit when an error occurs
rc = 0
while rc == 0:
    rc = mqttc.loop()
print("rc: " + str(rc))
