// HardwareInterfaces.h
#ifndef HARDWARE_INTERFACES_H
#define HARDWARE_INTERFACES_H

#include <string>

class ITemperatureSensor {
public:
    virtual ~ITemperatureSensor() = default;
    virtual float getTemperatureInCelsius() = 0;
};

class IAnalogInput {
public:
    virtual ~IAnalogInput() = default;
    virtual float getAnalogInputVoltage(int inputPin) = 0;
};

class IDateTimeProvider {
public:
    virtual ~IDateTimeProvider() = default;
    virtual std::string getDateTime() = 0;
};

#endif // HARDWARE_INTERFACES_H