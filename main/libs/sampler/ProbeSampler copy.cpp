#include "ProbeSampler.h"
#include "HardwareInterfaces.h"
#include <cmath>
#include "HardwareImplementations.cpp"

#define psiToPa 6894.76
#define waterDensity 997.0474
#define gravity 9.80665
#define measurementDepthIntervalMeters 1.0
#define tolerance 0.1

class ProbeSampler {
public:
    ProbeSampler(int samples, ITemperatureSensor* tempSensor, IAnalogInput* analogInput, IDateTimeProvider* dateTimeProvider)
        : mSampleCounter(samples), tempSensor(tempSensor), analogInput(analogInput), dateTimeProvider(dateTimeProvider) {}

    bool init() {
        ESP_LOGI(TAG, "Initializing ...");
        pinMode(TOGGLE_PIN, INPUT);
        isTestMode = digitalRead(TOGGLE_PIN);
        ESP_LOGI(TAG, "Initializing complete, ready to sample");
        return true;
    }

    std::string getSample() {
        static int counter = 1;

        if (counter > mSampleCounter) {
            ESP_LOGI(TAG, "getSample() - no more samples");
            return "";
        } else {
            if (isTestMode) {
                ESP_LOGI(TAG, "Probe in TEST MODE");
                ESP_LOGI(TAG, "getSample retrieved sample #" + std::to_string(counter));
                SampleData data = averageSensorReadings(10);
                float depth = (data.pressure * psiToPa) / (waterDensity * gravity);
                return writeSampleData(dateTimeProvider->getDateTime(), depth, data, counter++);
            } else {
                ESP_LOGI(TAG, "Probe in FIELD SAMPLING MODE");
                while (true) {
                    double pressurePSI = getPressure(analogInput->getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN));
                    ESP_LOGD(TAG, "Pressure: " + std::to_string(pressurePSI) + " psi");
                    depthMeters = (pressurePSI * psiToPa) / (waterDensity * gravity);
                    ESP_LOGI(TAG, "Current depth: " + std::to_string(depthMeters) + " meters, last measure depth: " + std::to_string(lastRecordedDepthMeters) + " meters");

                    if (depthMeters < lastRecordedDepthMeters && depthMeters < measurementDepthIntervalMeters) {
                        ESP_LOGI(TAG, "Depth: " + std::to_string(depthMeters) + " meters. Probe went all the way up. Ending sampling...");
                        return "";
                    } else if (std::abs(depthMeters - lastRecordedDepthMeters) >= measurementDepthIntervalMeters - tolerance) {
                        ESP_LOGI(TAG, "Depth: " + std::to_string(depthMeters) + " meters. Starting measurements...");
                        lastRecordedDepthMeters = round(depthMeters);
                        return writeSampleData(dateTimeProvider->getDateTime(), depthMeters, averageSensorReadings(10), counter++);
                    }
                    delay(1000);
                }
            }
        }
    }

private:
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
            delay(10);
        }
        accumulatedData.temperature /= numSamples;
        accumulatedData.pressure /= numSamples;
        accumulatedData.pressure_voltage /= numSamples;
        accumulatedData.tds /= numSamples;
        accumulatedData.tds_voltage /= numSamples;
        accumulatedData.conductivity /= numSamples;
        return accumulatedData;
    }

    SampleData readAllSensors() {
        SampleData data = {0, 0, 0, 0, 0, 0};
        data.temperature = tempSensor->getTemperatureInCelsius();
        data.pressure_voltage = analogInput->getAnalogInputVoltage(PRESSURE_SENSOR_INPUT_PIN);
        data.pressure = getPressure(data.pressure_voltage);
        data.tds_voltage = analogInput->getAnalogInputVoltage(TDS_SENSOR_INPUT_PIN);
        data.tds = getTDS(data.tds_voltage);
        data.conductivity = getConductivity(data.tds_voltage, data.temperature);
        return data;
    }

    float getPressure(float pressure_input_voltage) {
        ESP_LOGD("getPressure", "...getting pressure from voltage " + std::to_string(pressure_input_voltage) + "...");
        float pressure = 25 * pressure_input_voltage - 12.5;
        return (pressure > 0) ? pressure : 0;
    }

    float getTDS(float tds_input_voltage) {
        float tds = 434.8 * tds_input_voltage;
        return (tds > 0) ? tds : 0;
    }

    float getConductivity(float tds_input_voltage, float temperature) {
        float conductivity = getTDS(tds_input_voltage) / 0.7;
        return (conductivity > 0) ? conductivity : 0;
    }

    std::string writeSampleData(std::string dateTime, float depth, SampleData data, int counter) {
        return "dateTime: " + dateTime + "\nDepth: " +
               std::to_string(depth) + "m\nTemperature: " +
               std::to_string(data.temperature) + "°C\nPressure: " +
               std::to_string(data.pressure) + "psi, " +
               std::to_string(data.pressure_voltage) + "v\nTDS: " +
               std::to_string(data.tds) + "ppm, " +
               std::to_string(data.tds_voltage) + "v\nConductivity: " +
               std::to_string(data.conductivity) + "μS/cm\n" +
               std::to_string(counter) + "\n\n";
    }

    const char* TAG = "ProbeSampler";
    int mSampleCounter;
    ITemperatureSensor* tempSensor;
    IAnalogInput* analogInput;
    IDateTimeProvider* dateTimeProvider;
    double depthMeters = 0;
    double lastRecordedDepthMeters = 0;
    bool isTestMode = true;
};

extern "C" void app_main(void) {
    // Create hardware implementations
    TemperatureSensor tempSensor;
    AnalogInput analogInput;
    DateTimeProvider dateTimeProvider;

    // Create sampler
    ProbeSampler sampler(5, &tempSensor, &analogInput, &dateTimeProvider);

    // Initialize and run the sampler
    sampler.init();
    std::string sample = sampler.getSample();
    ESP_LOGI("app_main", sample);
}