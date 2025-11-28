from pydantic import BaseModel

class Microphone(BaseModel):
    db_SPL_RMS: float
    db_SPL_peak: float

class MQSensors(BaseModel):
    gas_concentration: float
    CO: float
    CH4: float

class BME680Data(BaseModel):
    IAQ: float
    CO2: float
    VOC: float
    temperature: float
    pressure: float
    humidity: float

class BME680Status(BaseModel):
    IAQ_accuracy: int
    stabilization_status: int
    run_in_status: int

class BME680(BaseModel):
    data: BME680Data
    status: BME680Status

class SensorData(BaseModel):
    microphone: Microphone
    MQ_sensors: MQSensors
    BME680: BME680