#ifndef PROBESAMPLER_H
#define PROBESAMPLER_H

#include "ISampler.h"
#include "HardwareInterfaces.h"

class ProbeSampler : public ISampler
{
public:
    /// @brief Constructor
    /// @param samples Number of samples to be generated
    /// @param tempSensor Pointer to the temperature sensor interface
    /// @param analogInput Pointer to the analog input interface
    /// @param dateTimeProvider Pointer to the date and time provider interface
    ProbeSampler(const int samples, ITemperatureSensor* tempSensor, IAnalogInput* analogInput, IDateTimeProvider* dateTimeProvider);

    /// @brief Virtual destructor
    virtual ~ProbeSampler();

private:
    // ISampler interface

    /// @brief Initialize sampler, must be called before first call to getSample()
    ///        After this call sampler considered ready to provide samples
    /// @return true if initialization successful, false otherwise
    virtual bool init() override;

    /// @brief  Retrieve next sample from sensors
    /// @return Sample data serialized to string,
    ///         empty string if no more samples available in current cycle (surface reached)
    virtual std::string getSample() override;

    const char* TAG = "ProbeSampler";
    int mSampleCounter;
    ITemperatureSensor* tempSensor;
    IAnalogInput* analogInput;
    IDateTimeProvider* dateTimeProvider;
    double depthMeters = 0;
    double lastRecordedDepthMeters = 0;
    bool isTestMode = true;
};

#endif // PROBESAMPLER_H