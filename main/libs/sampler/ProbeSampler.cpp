#include "ProbeSampler.h"
#include "esp_log.h"
#include "delay.h"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

const float ADC_COMPENSATION = 1;                 // 0dB attenuation
const double waterDensity = 997.0474;             // kg/m^3
const double gravity = 9.80665;                   // m/s^2
const double psiToPa = 6894.76;                   // 1 psi = 6894.76 Pa
const double measurementDepthIntervalMeters = 1.0;  // 1 meter depth
const double tolerance = 0.1;                     // 10% tolerance
double depthMeters = 0;
double lastRecordedDepthMeters = 0; //Keep track of last recorded depth
boolean testMode = 1; 
OneWire oneWire(TEMP_SENSOR_INPUT_PIN);
DallasTemperature tempSensor(&oneWire);

const char * ProbeSampler::TAG = "ProbeSampler";


std::string getDateTime() {
    // Get the current date and time
    time_t now = time(0);
    tm *ltm = localtime(&now);

    // Store the current date and time in a string variable
    char currentDateTime[20];
    strftime(currentDateTime, sizeof(currentDateTime), "%Y-%m-%d %H:%M:%S", ltm);
    return std::string(currentDateTime);
}

std::string twoDecimalString(float value) {
  int whole = (int)value;                       // Extract the whole part
  int decimal = (int)((value - whole) * 100);   // Extract the decimal part
  return std::to_string(whole) + "." + (decimal < 10 ? "0" : "") + std::to_string(decimal);
}

float getTemperatureInCelsius (DallasTemperature t) {
    t.requestTemperatures();                    // Request temperature readings
    return t.getTempCByIndex(0);                // read temperature in °C
}

float getAnalogInputVoltage (int inputPin) {
    // Calculate the volts per division taking account of the chosen attenuation value.
    float input = analogRead(inputPin);
    return input * REF_VOLTAGE * ADC_COMPENSATION / ADC_RESOLUTION;
}

float getTDS (float tds_input_voltage, float temperature) {
    float tds = 434.8 * tds_input_voltage;                      // assuming 0v = 0ppm, 2.3v = 1000ppm.
     return (tds > 0) ? tds : 0;
}

float getConductivity (float tds_input_voltage, float temperature) {
    /* Conductivity = TDS * conversion factor, where
        0.65 - general purpose
        0.5  - sea water
        0.7  - drinking water
        0.8  - hydroponics */
    float conductivity = getTDS(tds_input_voltage, temperature)/0.7; // assuming 0.7 conversion factor
    return (conductivity > 0) ? conductivity : 0;
}

float getPressure (float pressure_input_voltage) {
        ESP_LOGI( "getPressure", "...getting pressure..." );
        float pressure = 25 * pressure_input_voltage -12.5; // assuming 0.5V = 0 PSI and 4.5V = 100 PSI
        return (pressure > 0) ?  pressure :  0;
}

SampleData readAllSensors() {
    Serial.println("Reading sensors...");
    SampleData data = {0, 0, 0, 0, 0, 0};;

    data.temperature = getTemperatureInCelsius(tempSensor);
    data.pressure_voltage = getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN);
    data.pressure = getPressure(data.pressure_voltage);
    data.tds_voltage = getAnalogInputVoltage(TDS_SENSOR_INPUT_PIN);
    data.tds = getTDS(data.tds_voltage, data.temperature);
    data.conductivity = getConductivity(data.tds_voltage, data.temperature);
    return data;
}

SampleData averageSensorReadings(int numSamples) {
    SampleData accumulatedData = {0, 0, 0, 0, 0, 0};

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

std::string writeSampleDataInTestingMode (std::string dateTime, float depth, SampleData data, int counter) {

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
    Serial.println("attributes: ");
}

ProbeSampler::ProbeSampler( const int samples )
    : mSampleCounter( samples ) {
    ESP_LOGI( TAG, "Instance created (samples = %d)", samples );
}

ProbeSampler::~ProbeSampler(){ESP_LOGI( TAG, "Instance destroyed" );}

bool ProbeSampler::init() {
    ESP_LOGI( TAG, "Initializing ..." );
    tempSensor.begin();                             // initialize temperature sensor
    delayMsec( 1000 );
    pinMode(TOGGLE_PIN, INPUT);                     // initialize toggle pin as input
    testMode = digitalRead(TOGGLE_PIN);             // if 1: true, if 0: false
    ESP_LOGI( TAG, "Initializing complete, ready to sample" );
    return true;
}

std::string ProbeSampler::getSample() {
    static int counter = 1;

    if( counter > mSampleCounter ) {
        ESP_LOGI( TAG, "getSample() - no more samples" );
        return "";
    } else {
        if (testMode) {
            ESP_LOGI(TAG, "Probe in TEST MODE");
            ESP_LOGI( TAG, "getSample retrieved sample #%d ", counter );
            SampleData data = averageSensorReadings(10);
            float depth = (data.pressure * psiToPa) / (waterDensity * gravity);
            return writeSampleDataInTestingMode(getDateTime(), depth, data, counter++);
        } else {
            ESP_LOGI(TAG, "Probe in FIELD SAMPLING MODE");
            bool measuring = false;
            while(true) {
                double pressurePSI = getPressure(getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN));
                ESP_LOGI(TAG, "Pressure: %f psi", pressurePSI);
                depthMeters = (pressurePSI * psiToPa) / (waterDensity * gravity);
                ESP_LOGI(TAG, "Current depth: %f meters, last measure depth: %f meters ", depthMeters, lastRecordedDepthMeters);

                delayMsec( 1000 );      //sleep until the next pressure reading

                if (std::abs(depthMeters - lastRecordedDepthMeters) >= measurementDepthIntervalMeters - tolerance) {
                    if(!measuring) {
                        measuring = true;
                        ESP_LOGI(TAG, "Depth: %f meters. Starting measurements...", depthMeters);
                        std::string sample = writeSampleDataInTestingMode(getDateTime(), depthMeters, averageSensorReadings(10), counter++);
                        lastRecordedDepthMeters = round(depthMeters);
                        measuring = false;
                        return sample;
                    }
                } else {    
                    return "Current Depth: " + std::to_string(depthMeters) + " meters\n";
                }
            }
        }       
    }
}