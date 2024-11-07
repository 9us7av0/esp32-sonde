#ifndef CBLUETOOTHPUBLISHERSERVICE_H
#define CBLUETOOTHPUBLISHERSERVICE_H

#include "IDataPublisherService.h"
#include "BLEServer.h"
#include "BLECharacteristic.h"

#include <memory>

class BLEDescriptor;
class BLE2902;

class CBluetoothPublisherService : public IDataPublisherService
                                 , public BLEServerCallbacks
                                 , public BLECharacteristicCallbacks
{
public:
    CBluetoothPublisherService();
    virtual ~CBluetoothPublisherService();
private:
    // IDataPublisherService interface
    virtual bool publishData( const SampleDataList & data ) override;

    // BLEServerCallbacks interface
    virtual void onConnect(BLEServer* pServer) override;
    virtual void onDisconnect(BLEServer* pServer) override;

    // BLECharacteristicCallbacks interface
    virtual void onWrite(BLECharacteristic *pChar) override;

private:
    /// @brief Initialize BLE service
    /// @return true if service initialized and started successfully, false otherwise
    bool initBLEService();

    static const char * TAG;                ///< Logger tag
    static const char * BLE_DEVICE_NAME;    ///< BLE device name
    static const char * SERVICE_UUID;       ///< UUID for service
    static const char * CHAR_NOTIFY_UUID;   ///< UUID for notification characteristic (used to send data to client)
    static const char * CHAR_RX_UUID;       ///< UUID for Read/Write characteristic (client uses it to initiate data transfer)

    BLEServer * mServer;                        ///< BLE server instance
    BLECharacteristic * mNotifyCharacteristic;  ///< Notify characteristic to send data to client
    BLECharacteristic * mRxCharacteristic;      ///< Read/Write characteristic to initiate data transfer

    std::shared_ptr<BLEDescriptor> mNotifyDescriptor2901; ///< BLE descriptor 2901 for Notify characteristic
    std::shared_ptr<BLE2902> mNotifyDescriptor2902;       ///< BLE descriptor 2902 for Notify characteristic
    std::shared_ptr<BLEDescriptor> mRxDescriptor2901;     ///< BLE descriptor 2901 for Read/Write characteristic
    std::shared_ptr<BLE2902> mRxDescriptor2902;           ///< BLE descriptor 2902 for Read/Write characteristic

    bool mDeviceConnected;                  ///< Flag indicating if device is connected
    bool mNotificationsListenerReady;       ///< Flag indicating if client is ready to receive notifications
};

#endif // CBLUETOOTHPUBLISHERSERVICE_H