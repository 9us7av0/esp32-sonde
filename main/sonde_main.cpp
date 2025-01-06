#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"

#include "delay.h"

// Logger
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

// Arduino framework
#include "Arduino.h"

#include "HardwareImplementations.cpp"
#include "ProbeSampler.h"
#include "CBluetoothPublisherService.h"
#include "CSondeApp.h"

// Tag for logging
static const char * TAG = "sonde_main";

extern "C" void app_main(void) {
    // Create hardware implementations
    TemperatureSensor tempSensor;
    AnalogInput analogInput;
    DateTimeProvider dateTimeProvider;
    // Initialize the Arduino framework
    initArduino();

    // Create sampler and publisher
    ProbeSampler sampler(5, &tempSensor, &analogInput, &dateTimeProvider);
    CBluetoothPublisherService publisher;

    //Create application and run it
    CSondeApp app( sampler, publisher );
    app.run();

    // Restart the device
    for (int i = 10; i >= 0; i--) {
        ESP_LOGW(TAG, "Restarting in %d seconds...", i);
        delayMsec(1000);
    }
    ESP_LOGI(TAG, "Restarting now.");
    fflush(stdout);
    esp_restart();
}
