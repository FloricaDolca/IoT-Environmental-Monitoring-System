#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <MQUnifiedsensor.h>
#include <bsec2.h>
/************************ Hardware Related Macros ************************************/
#define Board ("Arduino Nano ESP32")
#define Voltage_Resolution (5)  // Reference Voltage of ADC is 5V
#define ADC_Bit_Resolution (12) // ESP32 ADC Bit resolution is 12bit

#define PIN_MQ135 24
#define PIN_MQ7 23
#define PIN_MQ4 20
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_MICROPHONE 19

#define PANIC_LED LED_BUILTIN
/*********************** Software Related Macros ************************************/
#define BAUD_RATE 115200

#define RatioMQ135CleanAir (3.61)
#define RatioMQ7CleanAir (27.5)
#define RatioMQ4CleanAir (4.4)
/*********************** Data types ************************************/
struct WifiSettings
{
    const char *ssid = " ";
    const char *password = " ";
};

struct MqttBrokerSettings
{
    const char *broker = "192.168.218.123"; // Raspberry Pi  Static IP
    const char *topic = "esp32/sensors/all";
    const char *username = " ";
    const char *password = " ";
    const int port = 1883;
};

struct MQ_Readings
{
    float gas_concentration;
    float CO;
    float CH4;
};

struct BME680_Readings
{
    float IAQ;
    float CO2;
    float VOC;
    float temperature;
    float pressure;
    float humidity;

    int IAQ_accuracy;
    int stabilization_status;
    int run_in_status;
};

struct Microphone_Settings
{
    int bias = 2038;      // DC bias = 2.5V (half of 5V = VCC)
    float SPL_ref = 40.4; // A known SPL level mesaured with a sound level meter
    float ref_avg = 6.89; // Average value reported by the microphone at SPL_ref
    int ref_peak = 38.3;  // Peak value reported by the microphone at SPL_ref
    int ref_rms = 8.97;   // RMS value reported by the microphone at SPL_ref
};

struct Microphone_Readings
{
    float average = 0;
    float db_SPL_peak = 0;
    float db_SPL_avg = 0;
    float db_SPL_RMS = 0;

    int value = 0;
    int n = 0;
    int sum = 0;
    int max = 0;
    float RMS = 0;

    void reset()
    {
        average = 0;
        db_SPL_peak = 0;
        db_SPL_avg = 0;
        db_SPL_RMS = 0;
        value = 0;
        n = 0;
        sum = 0;
        max = 0;
        RMS = 0;
    }
};

/*********************** GLOBALS ***********************************/
unsigned long INTERVAL = 2000; // [ms]
unsigned long last_message = 0;

WifiSettings wifi_settings;
MqttBrokerSettings mqtt_settings;
Microphone_Settings mic_settings;

MQUnifiedsensor MQ135_gas(Board, Voltage_Resolution, ADC_Bit_Resolution, PIN_MQ135, "MQ-135");
MQUnifiedsensor MQ7_CO(Board, Voltage_Resolution, ADC_Bit_Resolution, PIN_MQ7, "MQ-7");
MQUnifiedsensor MQ4_CH4(Board, Voltage_Resolution, ADC_Bit_Resolution, PIN_MQ4, "MQ-4");
Bsec2 BME680_sensor;

MQ_Readings mq_readings;
BME680_Readings bme680_readings;
Microphone_Readings mic_readings;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
/*********************** PROTOTYPES ***********************************/
void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);

void init_MQ_sensors();
void calibrate_MQ_sensors();
void read_MQ_data();

void init_BME680_sensor();
void checkBsecStatus(Bsec2 bsec);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

void read_microphone_sample();
void compute_sound_statistics();

DynamicJsonDocument create_json_message();
/**********************************************************/
void setup()
{
    Serial.begin(BAUD_RATE);
    Wire.begin();
    while (!Serial) // wait for Serial to be available
        delay(10);

    init_MQ_sensors();
    calibrate_MQ_sensors();
    init_BME680_sensor();

    connectToWiFi();
    mqtt_client.setServer(mqtt_settings.broker, mqtt_settings.port);
    mqtt_client.setCallback(mqttCallback);
    connectToMQTTBroker();
    mqtt_client.setBufferSize(1024);
}
/**********************************************************/
void loop()
{
    if (!mqtt_client.connected())
    {
        connectToMQTTBroker();
    }

    if (!BME680_sensor.run())
    {
        checkBsecStatus(BME680_sensor);
    }

    read_microphone_sample();

    unsigned long now = millis();
    if (now - last_message > INTERVAL)
    {
        last_message = now; // Update the last message time

        compute_sound_statistics();
        read_MQ_data();

        Serial.print("MICROPHONE:");
        Serial.printf("\n\t db_SPL_RMS = %f \n\t db_SPL_peak = %f \n", mic_readings.db_SPL_RMS, mic_readings.db_SPL_peak);
        Serial.print("MQ_SENSORS:");
        Serial.printf("\n\t Gas concentration = %f \n\t CO = %f \n\t CH4 = %f \n", mq_readings.gas_concentration, mq_readings.CO, mq_readings.CH4);
        Serial.print("BME680_SENSOR:");
        Serial.printf("\n\t IAQ = %f \n\t CO2 = %f \n\t VOC = %f \n\t Temperature = %f \n\t Pressure = %f \n\t Humidity = %f \n", bme680_readings.IAQ, bme680_readings.CO2, bme680_readings.VOC, bme680_readings.temperature, bme680_readings.pressure, bme680_readings.humidity);
        Serial.printf("\n\t stabilization_status = %d \n\t run_in_status = %d \n", bme680_readings.stabilization_status, bme680_readings.run_in_status);
        Serial.println("---------------------------------------------------");

        DynamicJsonDocument json_doc = create_json_message();
        String payload = "";
        serializeJson(json_doc, payload);
        mqtt_client.publish(mqtt_settings.topic, (char *)payload.c_str());
        serializeJsonPretty(json_doc, Serial);

        mic_readings.reset();
    }
}

// Function definitions
void connectToWiFi()
{
    WiFi.begin(wifi_settings.ssid, wifi_settings.password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to the WiFi network");
}

void connectToMQTTBroker()
{
    while (!mqtt_client.connected())
    {
        String client_id = "esp32-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_settings.username, mqtt_settings.password))
        {
            Serial.println("Connected to MQTT broker");
            mqtt_client.subscribe(mqtt_settings.topic);
            Serial.printf("Subscribed to topic: %s\n", mqtt_settings.topic);
        }
        else
        {
            Serial.printf("Failed to connect to MQTT broker, rc=%d try again in 5 seconds\n", mqtt_client.state());
            delay(5000);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.printf("Message received on topic: %s\n", topic);
    Serial.print("Message:");
    for (unsigned int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void init_MQ_sensors()
{
    Serial.printf("Initializing regressing coefficients for gas sensors"); // Mesaj de test

    Serial.println("\tMQ135: CO2 - Exponential regression");
    MQ135_gas.setRegressionMethod(1);
    MQ135_gas.setA(110.47);
    MQ135_gas.setB(-2.862);

    Serial.println("\tMQ7: CO - Exponential regression");
    MQ7_CO.setRegressionMethod(1);
    MQ7_CO.setA(99.042);
    MQ7_CO.setB(-1.518);

    Serial.println("\tMQ4: CH4 - Exponential regression");
    MQ4_CH4.setRegressionMethod(1);
    MQ4_CH4.setA(1012.7);
    MQ4_CH4.setB(-2.786);

    MQ135_gas.init();
    MQ7_CO.init();
    MQ4_CH4.init();
}

void calibrate_MQ_sensors()
{
    const int N_STEPS = 10;
    Serial.print("Calibrating MQ sensors ... please wait.");
    float calcR0_MQ135 = 0;
    float calcR0_MQ7 = 0;
    float calcR0_MQ4 = 0;

    // Calibrare R0
    for (int i = 1; i <= N_STEPS; i++)
    {
        MQ135_gas.update();
        MQ7_CO.update();
        MQ4_CH4.update();
        calcR0_MQ135 += MQ135_gas.calibrate(RatioMQ135CleanAir);
        calcR0_MQ7 += MQ7_CO.calibrate(RatioMQ7CleanAir);
        calcR0_MQ4 += MQ4_CH4.calibrate(RatioMQ4CleanAir);
        Serial.print(".");
    }
    MQ135_gas.setR0(calcR0_MQ135 / N_STEPS);
    MQ7_CO.setR0(calcR0_MQ7 / N_STEPS);
    MQ4_CH4.setR0(calcR0_MQ4 / N_STEPS);
    Serial.println("  done!.");

    Serial.println("Calibration results:");
    Serial.printf("\tMQ135: R0 = %.2f\n", calcR0_MQ135 / N_STEPS);
    Serial.printf("\tMQ7: R0 = %.2f\n", calcR0_MQ7 / N_STEPS);
    Serial.printf("\tMQ4: R0 = %.2f\n", calcR0_MQ4 / N_STEPS);
    // Check for calibration errors
    if (isinf(calcR0_MQ135) || isinf(calcR0_MQ7) || isinf(calcR0_MQ4))
    {
        Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
        while (1)
            ;
    }

    if (calcR0_MQ135 == 0 || calcR0_MQ7 == 0 || calcR0_MQ4 == 0)
    {
        Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
        while (1)
            ;
    }

    MQ135_gas.serialDebug(true);
    MQ7_CO.serialDebug(true);
    MQ4_CH4.serialDebug(true);
}

void read_MQ_data()
{
    MQ135_gas.update();
    mq_readings.gas_concentration = MQ135_gas.readSensor() * 100;
    MQ7_CO.update();
    mq_readings.CO = MQ7_CO.readSensor();
    MQ4_CH4.update();
    mq_readings.CH4 = MQ4_CH4.readSensor();
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
    {
        return;
    }
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id)
        {
        case BSEC_OUTPUT_IAQ:
            bme680_readings.IAQ = output.signal;
            bme680_readings.IAQ_accuracy = output.accuracy;
            break;
        case BSEC_OUTPUT_CO2_EQUIVALENT:
            bme680_readings.CO2 = output.signal;
            break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            bme680_readings.VOC = output.signal;
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            bme680_readings.temperature = output.signal;
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            bme680_readings.pressure = output.signal;
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            bme680_readings.humidity = output.signal;
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            bme680_readings.stabilization_status = output.signal;
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            bme680_readings.run_in_status = output.signal;
            break;
        default:
            break;
        }
    }
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
    }
    else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
    }
    else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

void init_BME680_sensor()
{
    /* Desired subscription list of BSEC2 outputs */
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY};
    Serial.printf("Initializing BSEC library\n");
    if (!BME680_sensor.begin(BME68X_I2C_ADDR_LOW, Wire))
    {
        checkBsecStatus(BME680_sensor);
    }
    Serial.printf("Subscribing to BSEC outputs\n");
    if (!BME680_sensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP))
    {
        checkBsecStatus(BME680_sensor);
    }
    // Whenever new data is available call the newDataCallback function
    BME680_sensor.attachCallback(newDataCallback);
}

void read_microphone_sample()
{
    mic_readings.value = abs(analogRead(PIN_MICROPHONE) - mic_settings.bias); // Read the microphone
    mic_readings.max = max(mic_readings.max, mic_readings.value);             // Find the peak
    mic_readings.average += mic_readings.value;                               // Accumulate sum of readings
    mic_readings.RMS += ((long)mic_readings.value * mic_readings.value);      // Accumulate sum of squares
    mic_readings.n++;                                                         // Count the number of readings
}

void compute_sound_statistics()
{
    mic_readings.average /= mic_readings.n;                     // Calculate the average
    mic_readings.RMS = sqrt(mic_readings.RMS / mic_readings.n); // Calculate the RMS

    mic_readings.db_SPL_peak = mic_settings.SPL_ref + 20 * log10((float)mic_readings.max / mic_settings.ref_peak);   // Calculate the peak SPL
    mic_readings.db_SPL_avg = mic_settings.SPL_ref + 20 * log10((float)mic_readings.average / mic_settings.ref_avg); // Calculate the average SPL
    mic_readings.db_SPL_RMS = mic_settings.SPL_ref + 20 * log10((float)mic_readings.RMS / mic_settings.ref_rms);     // Calculate the RMS SPL
}

DynamicJsonDocument create_json_message()
{
    DynamicJsonDocument json_doc(1024);
    JsonObject microphone = json_doc["microphone"].to<JsonObject>();

    microphone["db_SPL_RMS"] = mic_readings.db_SPL_RMS;
    microphone["db_SPL_peak"] = mic_readings.db_SPL_peak;

    JsonObject MQ_sensors = json_doc["MQ_sensors"].to<JsonObject>();
    MQ_sensors["gas_concentration"] = mq_readings.gas_concentration;
    MQ_sensors["CO"] = mq_readings.CO;
    MQ_sensors["CH4"] = mq_readings.CH4;

    JsonObject BME680 = json_doc["BME680"].to<JsonObject>();
    JsonObject BME680_data = BME680["data"].to<JsonObject>();
    BME680_data["IAQ"] = bme680_readings.IAQ;
    BME680_data["CO2"] = bme680_readings.CO2;
    BME680_data["VOC"] = bme680_readings.VOC;
    BME680_data["temperature"] = bme680_readings.temperature;
    BME680_data["pressure"] = bme680_readings.pressure;
    BME680_data["humidity"] = bme680_readings.humidity;
    JsonObject BME680_status = BME680["status"].to<JsonObject>();
    BME680_status["IAQ_accuracy"] = bme680_readings.IAQ_accuracy;
    BME680_status["stabilization_status"] = bme680_readings.stabilization_status;
    BME680_status["run_in_status"] = bme680_readings.run_in_status;

    return json_doc;
}