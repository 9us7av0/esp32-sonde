// HardwareImplementations.cpp
#include "HardwareInterfaces.h"
#include "esp_log.h"
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ctime>

#define PRESSURE_SENSOR_INPUT_PIN 36
#define TDS_SENSOR_INPUT_PIN 34
#define TEMP_SENSOR_INPUT_PIN 18
#define REF_VOLTAGE 5
#define ADC_RESOLUTION 4096.0

class TemperatureSensor : public ITemperatureSensor {
public:
    TemperatureSensor() : oneWire(TEMP_SENSOR_INPUT_PIN), tempSensor(&oneWire) {
        tempSensor.begin();
    }

    float getTemperatureInCelsius() override {
        tempSensor.requestTemperatures();
        return tempSensor.getTempCByIndex(0);
    }

private:
    OneWire oneWire;
    DallasTemperature tempSensor;
};

class AnalogInput : public IAnalogInput {
public:
    float getAnalogInputVoltage(int inputPin) override {
        float input = analogRead(inputPin);
        return input * REF_VOLTAGE / ADC_RESOLUTION;
    }
};

class DateTimeProvider : public IDateTimeProvider {
public:
    std::string getDateTime() override {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        char currentDateTime[20];
        strftime(currentDateTime, sizeof(currentDateTime), "%Y-%m-%d %H:%M:%S", ltm);
        return std::string(currentDateTime);
    }
};