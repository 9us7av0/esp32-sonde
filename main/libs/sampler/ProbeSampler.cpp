#include "ProbeSampler.h"
#include "esp_log.h"
#include "delay.h"
#include "HardwareInterfaces.h"
#include "HardwareImplementations.cpp"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <cmath>

#define psiToPa 6894.76
#define waterDensity 997.0474
#define gravity 9.80665
#define measurementDepthIntervalMeters 1.0
#define tolerance 0.1

#define PRESSURE_SENSOR_INPUT_PIN 36        // pin GPIO36 (ADC0) to pressure sensor
#define TDS_SENSOR_INPUT_PIN      34        // pin GPIO34 (ADC1) to TDS sensor
#define TEMP_SENSOR_INPUT_PIN     18        // pin GPIO18 to DS18B20 sensor's DATA pin
#define TOGGLE_PIN                4         // pin GPIO4 to switch between modes of operation
#define REF_VOLTAGE               5         // Maximum voltage expected at IO pins
#define BASELINE_VOLTAGE          0.5       // measured minimum voltage read from sensors
#define ADC_RESOLUTION            4096.0    // 12 bits of resolution

struct SampleData {
    float temperature;
    float pressure_voltage;
    float pressure;
    float tds_voltage;
    float tds;
    float conductivity;
};
TemperatureSensor* tempSensor;

const float ADC_COMPENSATION_FACTOR = 1;                 // 0dB attenuation
AnalogInput* analogInput;
DateTimeProvider* dateTimeProvider;
const float ADC_COMPENSATION = 1;                 // 0dB attenuation
double depthMeters = 0;
double lastRecordedDepthMeters = 0; //Keep track of last recorded depth
boolean isTestMode = 1;             // 1: test mode, 0: field sampling mode
OneWire oneWire(TEMP_SENSOR_INPUT_PIN);

const char * ProbeSampler::TAG = "ProbeSampler";

std::string twoDecimalString(float value) {
  int whole = (int)value;                       // Extract the whole part
  int decimal = (int)((value - whole) * 100);   // Extract the decimal part
  return std::to_string(whole) + "." + (decimal < 10 ? "0" : "") + std::to_string(decimal);
}

float getTemperatureInCelsius (DallasTemperature t) {
    t.requestTemperatures();                    // Request temperature readings
    return t.getTempCByIndex(0);                // read temperature in °C
}

float getTDS (float tds_input_voltage) {
    float tds = 434.8 * tds_input_voltage;      // assuming 0v = 0ppm, 2.3v = 1000ppm.
     return (tds > 0) ? tds : 0;
}

float getConductivity (float tds_input_voltage, float temperature) {
    /* Conductivity = TDS * conversion factor, where
        0.65 - general purpose
        0.5  - sea water
        0.7  - drinking water
        0.8  - hydroponics */
    float conductivity = getTDS(tds_input_voltage)/0.7; // assuming 0.7 conversion factor
    return (conductivity > 0) ? conductivity : 0;
}

/**
 * @brief Converts the input voltage to pressure.
 * 
 * The formula used is: pressure = coefficientA * voltage + coefficientB
 * Default coefficients assume 0.5V = 0 PSI and 4.5V = 100 PSI.
 * 
 * @param pressure_input_voltage The input voltage from the pressure sensor.
 * @param coefficientA The coefficient for the voltage.
 * @param coefficientB The offset coefficient.
 * @return The calculated pressure.
 */
float getPressure(float pressure_input_voltage, float coefficientA = 25.0, float coefficientB = -12.5) {
    ESP_LOGD("getPressure", "...getting pressure from voltage %f...", pressure_input_voltage);
    float pressure = coefficientA * pressure_input_voltage + coefficientB;
    return (pressure > 0) ? pressure : 0;
}

SampleData readAllSensors() {
    Serial.println("Reading sensors...");
    SampleData data = {0, 0, 0, 0, 0, 0};

    if (!tempSensor || !analogInput) {
        ESP_LOGE("readAllSensors", "Sensors not initialized");
        return data;
    }

    data.temperature = tempSensor->getTemperatureInCelsius();
    data.pressure_voltage = analogInput->getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN);
    data.pressure = getPressure(data.pressure_voltage);
    data.tds_voltage = analogInput->getAnalogInputVoltage(TDS_SENSOR_INPUT_PIN);
    data.tds = getTDS(data.tds_voltage);
    data.conductivity = getConductivity(data.tds_voltage, data.temperature);
    return data;
}

SampleData averageSensorReadings(int numSamples) {
    SampleData accumulatedData = {0, 0, 0, 0, 0, 0};
    if (numSamples <= 0) {
        ESP_LOGE("averageSensorReadings:", "Number of samples must be greater than zero");
        return accumulatedData;
    }

    for (int i = 0; i < numSamples; ++i) {
        SampleData data = readAllSensors();
        accumulatedData.temperature += data.temperature;
        accumulatedData.pressure += data.pressure;
        accumulatedData.pressure_voltage += data.pressure_voltage;
        accumulatedData.tds += data.tds;
        accumulatedData.tds_voltage += data.tds_voltage;
        accumulatedData.conductivity += data.conductivity;
        delayMsec( 10 );                  // delay 10ms between each sample
    }
    accumulatedData.temperature /= numSamples;
    accumulatedData.pressure /= numSamples;
    accumulatedData.pressure_voltage /= numSamples;
    accumulatedData.tds /= numSamples;
    accumulatedData.tds_voltage /= numSamples;
    accumulatedData.conductivity /= numSamples;
    return accumulatedData;
}

std::string writeSampleData (std::string dateTime, float depth, SampleData data, int counter) {

    // writing sample data into string to be sent out via bluetooth
    return "dateTime: " + dateTime + "\nDepth: " + 
    twoDecimalString(depth) + "m\nTemperature: " + 
    twoDecimalString(data.temperature) + "°C\nPressure: " + 
    std::to_string(data.pressure) +  "psi, " + 
    twoDecimalString(data.pressure_voltage) + "v\nTDS: " + 
    std::to_string(data.tds) + "ppm, " + 
    std::to_string(data.tds_voltage) + "v\nConductivity: " + 
    std::to_string(data.conductivity) + "μS/cm\n" +
    std::to_string( counter ) + "\n\n";
}

ProbeSampler::ProbeSampler(const int samples)
    : mSampleCounter(samples), tempSensor(), analogInput() {
    ESP_LOGI(TAG, "Instance created (samples = %d)", samples);
}

ProbeSampler::~ProbeSampler(){ESP_LOGI( TAG, "Instance destroyed" );}

bool ProbeSampler::init() {
    ESP_LOGI(TAG, "Initializing ...");
    delayMsec(1000);
    pinMode(TOGGLE_PIN, INPUT); // initialize toggle pin as input
    isTestMode = digitalRead(TOGGLE_PIN); // if 1: true, if 0: false

    // Initialize tempSensor and analogInput
    tempSensor = new TemperatureSensor();
    analogInput = new AnalogInput();

    if (!tempSensor || !analogInput) {
        ESP_LOGE(TAG, "Failed to initialize sensors");
        return false;
    }

    ESP_LOGI(TAG, "Initializing complete, ready to sample");
    return true;
}

std::string ProbeSampler::getSample() {
    static int counter = 1;

    if( counter > mSampleCounter ) {
        ESP_LOGI( TAG, "getSample() - no more samples" );
        return "";
    } else {
        if (isTestMode) {
            ESP_LOGI(TAG, "Probe in TEST MODE");
            ESP_LOGI( TAG, "getSample retrieved sample #%d ", counter );
            SampleData data = averageSensorReadings(10);
            float depth = (data.pressure * psiToPa) / (waterDensity * gravity);
            return writeSampleData(dateTimeProvider->getDateTime(), depth, data, counter++);
        } else {
            ESP_LOGI(TAG, "Probe in FIELD SAMPLING MODE");
            while(true) {
                double pressurePSI= getPressure(analogInput->getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN));
                ESP_LOGD(TAG, "Pressure: %f psi", pressurePSI);
                depthMeters = (pressurePSI * psiToPa) / (waterDensity * gravity);
                ESP_LOGI(TAG, "Current depth: %f meters, last measure depth: %f meters ", depthMeters, lastRecordedDepthMeters);

                if (depthMeters < lastRecordedDepthMeters && depthMeters < measurementDepthIntervalMeters) {
                    ESP_LOGI(TAG, "Depth: %f meters. Probe went all the way up. Ending sampling...", depthMeters);
                    return ""; 
                } else if (std::abs(depthMeters - lastRecordedDepthMeters) >= measurementDepthIntervalMeters - tolerance) {
                    ESP_LOGI(TAG, "Depth: %f meters. Starting measurements...", depthMeters);
                    lastRecordedDepthMeters = round(depthMeters);
                    return writeSampleData(dateTimeProvider->getDateTime(), depthMeters, averageSensorReadings(10), counter++);
                } 
                delayMsec( 1000 );      //sleep until the next pressure reading
            }
        }       
    }
}