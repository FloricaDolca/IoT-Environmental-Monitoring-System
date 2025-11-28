import argparse
import configparser
import sys
import time
import paho.mqtt.client as mqtt
import logging
import signal
import json

from pydantic import ValidationError

from schemas import SensorData

import os
from dotenv import load_dotenv

from sensor_data_writer import SensorDataWriter, InfluxDBHandler


class MQTTSubscriber:
    def __init__(self, config: configparser.ConfigParser, args, sensor_data_writer: SensorDataWriter):
        self.setup_configurations(config, args)

        # Setup logging
        log_level = args.log_level or config['logging']['level'].upper()
        self.logger = self.setup_logging(log_level)

        # Setup the MQTT client
        self.client = self.setup_client()

        # Setup the sensor data writer
        self.sensor_data_writer = sensor_data_writer

    def setup_logging(self, log_level):
        logging.basicConfig(format='%(asctime)s - %(levelname)s: %(message)s',
                            level=log_level)
        return logging.getLogger(__name__)

    def setup_client(self):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=self.client_id,  # ensure it is unique when i provide it
                             clean_session=not self.disable_clean_session)
        client.username_pw_set(self.username, self.password)

        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_subscribe = self.on_subscribe
        client.on_unsubscribe = self.on_unsubscribe
        client.on_disconnect = self.on_disconnect

        return client

    def setup_configurations(self, config: configparser.ConfigParser, args):
        self.broker_url = args.broker_url if args.broker_url else config['mqtt']['broker_url']
        print(self.broker_url)
        self.broker_port = int(args.broker_port or config['mqtt']['broker_port'])
        self.topic = args.topic or config['mqtt']['topic']
        self.username = args.username or config['mqtt']['username']
        self.password = args.password or config['mqtt']['password']
        self.client_id = args.client_id or config['mqtt']['client_id']
        self.qos = int(args.qos or config['mqtt']['qos'])
        self.keepalive = int(args.keepalive or config['mqtt']['keepalive'])
        self.disable_clean_session = args.disable_clean_session or config['mqtt']['disable_clean_session']

    def on_connect(self, client, userdata, flags, reason_code, properties):
        CONNECTION_SUCCESS = 0
        if self.client.is_connected() and reason_code == CONNECTION_SUCCESS:
            self.logger.info("Connected to the MQTT broker successfully")
            self.client.subscribe(self.topic, qos=self.qos)
        else:
            self.logger.error(f"Failed to connect to the MQTT broker. Reason code: {reason_code}")

    def on_message(self, client, userdata, message):
        try:
            self.logger.info(f"Received message on {message.topic}:")
            data = json.loads(message.payload.decode())
            data = SensorData(**data)  # validate the payload
            self.logger.info("Writting data to InfluxDB...")
            if self.sensor_data_writer.write_data(data) != 0:
                self.logger.info("Calibration ongoing, skipping write")
            else:
                self.logger.info("Data written successfully!")
        except ValidationError as e:
            self.logger.error(f"Failed to validate payload: {e}")
        except Exception as e:
            self.logger.error(f"Failed to decode message: {e}")

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        for reason_code in reason_code_list:
            if reason_code.is_failure:
                self.logger.info(f"Broker rejected you subscription: {reason_code}")
            else:
                self.logger.info(f"Broker granted the following QoS: {reason_code.value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            self.logger.info("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
        else:
            self.logger.info(f"Broker replied with failure: {reason_code_list[0]}")
        self.client.disconnect()

    def on_disconnect(self, client, userdata, reason_code):
        self.logger.info(f"Disconnected from the MQTT broker. Reason code: {reason_code}")
        reconnect_count, reconnect_delay = 0, self.FIRST_RECONNECT_DELAY
        while reconnect_count < self.MAX_RECONNECT_COUNT:
            self.logger.info("Reconnecting in %d seconds...", reconnect_delay)
            time.sleep(reconnect_delay)
            try:
                self.client.reconnect()
                self.logger.info("Reconnected successfully!")
                return
            except Exception as err:
                self.logger.error("%s. Reconnect failed. Retrying...", err)

            reconnect_delay *= self.RECONNECT_RATE
            reconnect_delay = min(reconnect_delay, self.MAX_RECONNECT_DELAY)
            reconnect_count += 1
        self.logger.info("Reconnect failed after %s attempts. Exiting...", reconnect_count)

    def run(self):
        try:
            self.logger.info("Connecting to the MQTT broker...")
            self.client.connect(host=self.broker_url,
                                port=self.broker_port,
                                keepalive=self.keepalive)
            self.client.loop_forever()
        except Exception as e:
            self.logger.info(f"Failed to connect to the MQTT broker: {e}")

    def shutdown(self):
        self.logger.info("Disconnecting from the MQTT broker...")
        self.client.disconnect()
        self.client.loop_stop()
        sys.exit(0)


def main():
    load_dotenv()

    args, _ = parse_arguments()
    config = load_config(args.config)

    influxdb_handler = InfluxDBHandler(url=f"http://{config['mqtt']['broker_url']}:8086",
                                       token=os.getenv('DOCKER_INFLUXDB_INIT_ADMIN_TOKEN'),
                                       org=os.getenv('DOCKER_INFLUXDB_INIT_ORG'),
                                       bucket=os.getenv('DOCKER_INFLUXDB_INIT_BUCKET'))
    sensor_data_writer = SensorDataWriter(influxdb_handler=influxdb_handler)

    mqtt_subscriber = MQTTSubscriber(config=config,
                                     args=args,
                                     sensor_data_writer=sensor_data_writer)

    signal.signal(signal.SIGINT, lambda sig, frame: mqtt_subscriber.shutdown())
    mqtt_subscriber.run()


def parse_arguments():
    parser = argparse.ArgumentParser(description="MQTT subscriber")
    parser.add_argument(
        '--config',
        default="MQTT_subscriber/mqtt_subscriber_config.ini",
        help="Path to the MQTT subscriber configuration file"
    )
    parser.add_argument(
        '--log-level',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help='Set the logging level')
    parser.add_argument(
        "--broker_url",
        type=str,
        required=False,
        default=None,
        help="MQTT broker address")
    parser.add_argument(
        "--broker_port",
        required=False,
        type=int,
        default=None,
        help="MQTT broker port")
    parser.add_argument(
        "--topic",
        required=False,
        type=str,
        default=None,
        help="MQTT topic to subscribe to")
    parser.add_argument(
        "--username",
        required=False,
        type=str,
        default=None,
        help="Username for MQTT broker")
    parser.add_argument(
        "--password",
        required=False,
        type=str,
        default=None,
        help="Password for MQTT broker")
    parser.add_argument(
        "--client_id",
        required=False,
        type=str,
        default=None,
        help="Client ID for MQTT broker")
    parser.add_argument(
        "-q", "--qos",
        required=False,
        type=int,
        default=0,
        choices=[0, 1, 2],
        help="QoS level: affects reliability and network usage",)
    parser.add_argument(
        "--keepalive",
        required=False,
        type=int)
    parser.add_argument(
        "--disable_clean_session",
        action="store_true",
        help="disable 'clean session' (sub + msgs not cleared when client disconnects)")

    # parser.add_argument(
    #     "-P", "--port",
    #     required=False,
    #     type=int,
    #     default=None,
    #     help="Defaults to 8883 for TLS or 1883 for non-TLS")
    # parser.add_argument(
    #     "--insecure",
    #     action="store_true")
    # parser.add_argument(
    #     "-F", "--ca_certs",
    #     required=False,
    #     default=None,
    #     help="CA certificate file for TLS")
    # parser.add_argument(
    #     "--tls-version",
    #     required=False,
    #     default=None,
    #     choices=['tlsv1.2', 'tlsv1.1', 'tlsv1'],
    #     help="TLS protocol version, can be one of tlsv1.2 tlsv1.1 or tlsv1\n")

    # parser.add_argument(
    #     "-D", "--debug",
    #     action="store_true")

    args, unknown = parser.parse_known_args()
    return args, unknown


def load_config(config_file):
    config = configparser.ConfigParser()
    config.read(config_file)
    return config


if __name__ == "__main__":
    main()
