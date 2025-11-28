from os import write
from influxdb_client import InfluxDBClient, Point, WriteOptions
from datetime import datetime, timezone
from schemas import SensorData

class InfluxDBHandler:
    def __init__(self, url: str, token: str, org: str, bucket: str):
        self.url = url
        self.token = token
        self.org = org
        self.bucket = bucket
        self.client = InfluxDBClient(url=self.url, token=self.token, org=self.org)
        self.write_api = self.client.write_api(write_options=WriteOptions(batch_size=1))

    def write_point(self, point: Point):
        self.write_api.write(bucket=self.bucket, org=self.org, record=point)

class SensorDataWriter:
    def __init__(self, influxdb_handler: InfluxDBHandler):
        self.influxdb_handler = influxdb_handler

    def write_data(self, sensor_data: SensorData):
        if int(sensor_data.BME680.data.IAQ) == 0 or \
            int(sensor_data.BME680.data.CO2) == 0 or \
            int(sensor_data.BME680.data.temperature) == 0 or \
            int(sensor_data.BME680.data.pressure) == 0 or \
            int(sensor_data.BME680.data.humidity) == 0:
            return 1 # Do not write the point if any of the fields is - calibration ongoing

        timestamp = datetime.now(tz=timezone.utc)

        microphone_point = Point("microphone") \
            .field("db_SPL_RMS", sensor_data.microphone.db_SPL_RMS) \
            .field("db_SPL_peak", sensor_data.microphone.db_SPL_peak) \
            .time(timestamp)
        self.influxdb_handler.write_point(microphone_point)

        MQ_sensors_point = Point("MQ_sensors") \
            .field("gas_concentration", sensor_data.MQ_sensors.gas_concentration) \
            .field("CO", sensor_data.MQ_sensors.CO) \
            .field("CH4", sensor_data.MQ_sensors.CH4) \
            .time(timestamp)
        self.influxdb_handler.write_point(MQ_sensors_point)

        BME680_data_point = Point("BME680_data") \
            .field("IAQ", sensor_data.BME680.data.IAQ) \
            .field("CO2", sensor_data.BME680.data.CO2) \
            .field("VOC", sensor_data.BME680.data.VOC) \
            .field("temperature", sensor_data.BME680.data.temperature) \
            .field("pressure", sensor_data.BME680.data.pressure) \
            .field("humidity", sensor_data.BME680.data.humidity) \
            .time(timestamp)
        self.influxdb_handler.write_point(BME680_data_point)

        BME680_status_point = Point("BME680_status") \
            .field("IAQ_accuracy", sensor_data.BME680.status.IAQ_accuracy) \
            .field("stabilization_status", sensor_data.BME680.status.stabilization_status) \
            .field("run_in_status", sensor_data.BME680.status.run_in_status) \
            .time(timestamp)
        

        self.influxdb_handler.write_point(BME680_status_point)
        return 0
