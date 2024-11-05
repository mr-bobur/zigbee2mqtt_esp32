
#include "WiFi.h"
#include <OneButton.h>

#include <stdio.h>
#include <stdbool.h>
// #include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <string.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "LittleFS.h"
#include "ArduinoJson.h"
#include <cJSON.h>

#include "app_mqtt.h"
#include "zbhci.h"
#include "app_db.h"
#include "device.h"
#include "web.h"

#define CONFIG_USR_BUTTON_PIN 2
#define CONFIG_BLUE_LIGHT_PIN 3

QueueHandle_t msg_queue;

String staStatus = "n/a";
String staSSID = "";
String staPassword = "";
String apStatus = "n/a";
String apSSID = "";
String mqttServer = "";
uint32_t mqttPort = 0;
String mqttUsername = "";
String mqttPassword = "";

DynamicJsonDocument bridge_json(1024);      // adjust size as needed
DynamicJsonDocument devices_json(4096);     // adjust size as needed
DynamicJsonDocument sub_devices_json(1024); // adjust size as needed
DeviceNode device_node[50];
int device_count = 0;

JsonObject root;
JsonArray devicesArray;

// MQTTClient mqtt;

OneButton btn = OneButton(
    CONFIG_USR_BUTTON_PIN, /** Input pin for the button */
    true,                  /** Button is active LOW */
    false                  /** Enable internal pull-up resistor */
);

String fileaddress = "/device2.json";
void setup()
{
    Serial.begin(115200);

    if (!LittleFS.begin())
    {
        Serial.printf("An Error has occurred while mounting LittleFS\n");
        delay(1000);
        return;
    }

    File f1 = LittleFS.open(fileaddress, FILE_READ);
    deserializeJson(devices_json, f1);
    f1.close();
    root = devices_json.as<JsonObject>();
    devicesArray = root["devices"];
    serializeJsonPretty(devicesArray, Serial);

    if (devicesArray.isNull())
    {
        Serial.println("Aarray null akan");
        root = devices_json.to<JsonObject>();
        devicesArray = root.createNestedArray("devices");
    }

    pinMode(CONFIG_BLUE_LIGHT_PIN, OUTPUT);
    digitalWrite(CONFIG_BLUE_LIGHT_PIN, LOW);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    appLoadDateBase();

    // will be used later!!!!!!!!!!!!!!!!!!!!!
    // webInit();

    // esp_err_t err = nvs_flash_erase();
    // if (err != ESP_OK)
    // {
    //     printf("Error erasing NVS partition: %s\n", esp_err_to_name(err));
    //     return;
    // }

    // err = nvs_flash_init();
    // if (err != ESP_OK)
    // {
    //     printf("Error initializing NVS partition: %s\n", esp_err_to_name(err));
    //     return;
    // }

    // printf("NVS partition erased and reinitialized.\n");

    // appDataBaseInit();

    msg_queue = xQueueCreate(10, sizeof(ts_HciMsg));

    // Power on the zigbee chip:
    pinMode(0, OUTPUT);
    digitalWrite(0, HIGH);

    zbhci_Init(msg_queue);
    delay(100);
    xTaskCreatePinnedToCore(
        zbhciTask,
        "zbhciTask",
        4096,
        NULL,
        5,
        NULL,
        ARDUINO_RUNNING_CORE);

    delay(100);
    zbhci_NetworkStateReq();
    delay(100);
    zbhci_NodesJoinedGetReq(0);
    btn.attachClick(handleClick);
    btn.attachDoubleClick(handleDoubleClick);
}
ts_DstAddr sDstAddr;
int adres1=0;
void loop()
{
    btn.tick();
    delay(10);

    sDstAddr.u16DstAddr = 8831;
    zbhci_ZclOnoffOn(0x02, sDstAddr, 1, 1); 
    // for (int i  = 0; i < 65535; i+=100)
    // {
    //    zbhci_ZclColorMove2Color(0x02, sDstAddr, 1, 1,0,i,100    );
    //    delay(100);
    // } 
    delay(1500);
    zbhci_ZclOnoffOff(0x02, sDstAddr, 1, 1);
    delay(1500);
 
    

}

void zbhciTask(void *pvParameters)
{
    ts_HciMsg sHciMsg;

    while (1)
    {
        bzero(&sHciMsg, sizeof(sHciMsg));
        if (xQueueReceive(msg_queue, &sHciMsg, portMAX_DELAY))
        {
            switch (sHciMsg.u16MsgType)
            {
            case ZBHCI_CMD_BDB_COMMISSION_FORMATION_RSP:
                zbhci_MgmtPermitJoinReq(0xFFFC, 0xFF, 1);
                break;

            case ZBHCI_CMD_NETWORK_STATE_RSP:
                appHandleNetworkStateRsp(&sHciMsg.uPayload.sNetworkStateRspPayloasd);
                break;

            case ZBHCI_CMD_NODES_DEV_ANNCE_IND:
                appHandleDeviceAnnouncementIndication(&sHciMsg.uPayload.sNodesDevAnnceRspPayload);
                break;

            case ZBHCI_CMD_LEAVE_INDICATION:
                appHandleLeaveIndication(&sHciMsg.uPayload.sLeaveIndicationPayload);
                break;

            case ZBHCI_CMD_ZCL_REPORT_MSG_RCV:
                appHandleZCLreportMsgRcv(&sHciMsg.uPayload.sZclReportMsgRcvPayload);
                break;

            case ZBHCI_CMD_ZCL_ATTR_READ_RSP:
                // appHandleZCLReadResponse(&sHciMsg.uPayload.sZclAttrReadRspPayload);
                break;

            default:
                break;
            }
        }
        delay(100);
    }
    vTaskDelete(NULL);
}

void handleClick(void)
{
    // digitalWrite(CONFIG_BLUE_LIGHT_PIN, LOW);
    zbhci_MgmtPermitJoinReq(0xFFFC, 0x00, 1);
}
void handleDoubleClick(void)
{
    // digitalWrite(CONFIG_BLUE_LIGHT_PIN, HIGH);
    zbhci_MgmtPermitJoinReq(0xFFFC, 0xFF, 1);
}
void appHandleNetworkStateRsp(ts_MsgNetworkStateRspPayload *payload)
{
    if (payload->u16NwkAddr != 0x0000)
    {
        zbhci_BdbChannelSet(25);
        delay(100);
        zbhci_BdbCommissionFormation();
    }
    else
    {
        brigeNode.nwkAddr = payload->u16NwkAddr;
        brigeNode.macAddr = payload->u64IeeeAddr;
        brigeNode.panId = payload->u16PanId;
        brigeNode.exPanId = payload->u64extPanId;
        brigeNode.channel = payload->u8Channel;
        appDataBaseRecover();
        zbhci_MgmtPermitJoinReq(0xFFFC, 0xFF, 1);
        digitalWrite(CONFIG_BLUE_LIGHT_PIN, HIGH);
    }
}
void appLoadDateBase(void)
{
    StaticJsonDocument<1024> doc;

    File configfile = LittleFS.open("/db.json", "r");

    DeserializationError error = deserializeJson(doc, configfile);
    if (error)
        Serial.println(F("Failed to read file, using default configuration"));
    configfile.close();

    staSSID = doc["sta"]["ssid"].as<const char *>();
    staPassword = doc["sta"]["pwd"].as<const char *>();
    mqttServer = doc["mqtt"]["server"].as<const char *>();
    mqttPort = doc["mqtt"]["port"].as<uint32_t>();
    mqttUsername = doc["mqtt"]["username"].as<const char *>();
    mqttPassword = doc["mqtt"]["password"].as<const char *>();
}
void appHandleDeviceAnnouncementIndication(ts_MsgNodesDevAnnceRspPayload *payload)
{
    Serial.println("adding new device data");

    DeviceNode *device = NULL;
    device = getEmptyDevice();
    if (!device)
        return;
    device->u16NwkAddr = payload->u16NwkAddr;
    device->u64IeeeAddr = payload->u64IEEEAddr;
    device->u8Type = payload->u8Capability;

    // device_node
    // Create devices array

    // Add data for each device
    bool saved = true;
    Serial.print("Array size:");
    Serial.println(devicesArray.size());
    for (size_t i = 0; i < devicesArray.size(); i++)
    {
        if (uint64_t(devicesArray[i]["u64IeeeAddr"]) == payload->u64IEEEAddr)
        {
            Serial.print("device topildi ardresi:");
            Serial.println(payload->u16NwkAddr);
            saved = false;
            adres1=payload->u16NwkAddr;
            break;
        }
    }
    if (saved)
    {

        JsonObject newDevice = devicesArray.createNestedObject();

        Serial.println(String(payload->u16NwkAddr));
        Serial.println(String(payload->u64IEEEAddr));
        Serial.println(String(payload->u8Capability));

        newDevice["u16NwkAddr"] = String(payload->u16NwkAddr);
        newDevice["u64IeeeAddr"] = String(payload->u64IEEEAddr);
        newDevice["u8Type"] = String(payload->u8Capability);

        serializeJson(devices_json, Serial);
        File file = LittleFS.open(fileaddress, FILE_WRITE);
        serializeJson(devices_json, file);
        file.close();

        uint16_t au16AttrList[1] = {0x0005};
        zbhci_ZclAttrRead(0x02, (ts_DstAddr){payload->u16NwkAddr}, 1, 1, 0, 0x0000, 1, au16AttrList);
    }
}

void appHandleLeaveIndication(ts_MsgLeaveIndicationPayload *payload)
{
    DeviceNode *device = NULL;
    Serial.print("device getjak adresi:");
    Serial.println(payload->u64MacAddr);

    bool saved = true;
    for (size_t i = 0; i < devicesArray.size(); i++)
    {
        if (uint64_t(devicesArray[i]["u64IeeeAddr"]) == payload->u64MacAddr)
        {
            Serial.print(" Getjak device topildi ardresi:");
            Serial.println(payload->u64MacAddr);
            devicesArray.remove(i);

            serializeJson(devices_json, Serial);
            File file = LittleFS.open(fileaddress, FILE_WRITE);
            serializeJson(devices_json, file);
            file.close();
        }
    }

    // device = findDeviceByIeeeaddr(payload->u64MacAddr);
    // if (device == NULL)
    // {
    //     return;
    // }

    // if (!payload->u8Rejoin)
    // {
    //     if (!strncmp((const char *)device->au8ModelId,
    //                  "lumi.sensor_motion.aq2",
    //                  strlen("lumi.sensor_motion.aq2")))
    //     {
    //         rtcgq11lmDelete(device->u64IeeeAddr);
    //     }
    //     else if (!strncmp((const char *)device->au8ModelId,
    //                       "lumi.weather",
    //                       strlen("lumi.weather")))
    //     {
    //         wsdcgq11lmDelete(device->u64IeeeAddr);
    //     }
    //     else if (!strncmp((const char *)device->au8ModelId,
    //                       "LILYGO.Light",
    //                       strlen("LILYGO.Light")))
    //     {
    //         lilygoLightDelete(device->u64IeeeAddr);
    //     }
    //     else if (!strncmp((const char *)device->au8ModelId,
    //                       "LILYGO.Sensor",
    //                       strlen("LILYGO.Sensor")))
    //     {
    //         lilygoSensorDelete(device->u64IeeeAddr);
    //     }
    //     else if (!strncmp((const char *)device->au8ModelId,
    //                       "ESP32C6.Light",
    //                       strlen("ESP32C6.Light")))
    //     {
    //         espressifLightDelete(device->u64IeeeAddr);
    //     }
    //     memset(device, 0, sizeof(DeviceNode));
    //     appDataBaseSave();
    // }
}

void appHandleZCLreportMsgRcv(ts_MsgZclReportMsgRcvPayload *payload)
{
    // payload->

    DeviceNode *device = NULL;

    for (size_t i = 0; i < devicesArray.size(); i++)
    {
        if (uint16_t(devicesArray[i]["u16NwkAddr"]) == payload->u16SrcAddr)
        {
            device = findDeviceByIeeeaddr(devicesArray[i]["u64IeeeAddr"]);
            Serial.print("topilgan device adresi: ");
            Serial.println(payload->u16SrcAddr);
            Serial.print("claster id: ");
            Serial.println(payload->u16ClusterId);
        }
    }
    switch (payload->u16ClusterId)
    {
    case 6:
    {
        Serial.print("Smart Switch:");
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrList[i].u16AttrID == 0x0000)
            {
                Serial.print("State: ");
                Serial.println(bool(payload->asAttrList[i].uAttrData.u8AttrData));
            }
        }
    }
    break;

    // case 0x405: /* Relative Humidity Measurement Cluster */
    // {

    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0000)
    //         {
    //             device->deviceData.wsdcgq11lm.i16Humidity = (int16_t)payload->asAttrList[i].uAttrData.u16AttrData;
    //             if (!strncmp((const char *)device->au8ModelId,
    //                          "lumi.weather",
    //                          strlen("lumi.weather")))
    //             {
    //                 wsdcgq11lmReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.wsdcgq11lm.i16Temperature,
    //                     device->deviceData.wsdcgq11lm.i16Humidity,
    //                     device->deviceData.wsdcgq11lm.i16Pressure);
    //             }
    //             else if (!strncmp((const char *)device->au8ModelId,
    //                               "LILYGO.Sensor",
    //                               strlen("LILYGO.Sensor")))
    //             {
    //                 lilygoSensorReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.sensor.i16Temperature,
    //                     device->deviceData.sensor.i16Humidity);
    //             }
    //         }
    //     }
    // }
    // break;

    // case 0: /* Basic Cluster */
    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0005)
    //         {
    //             memcpy(
    //                 device->au8ModelId,
    //                 payload->asAttrList[i].uAttrData.au8AttrData,
    //                 payload->asAttrList[i].u16DataLen);
    //             if (!strncmp((const char *)payload->asAttrList[i].uAttrData.au8AttrData,
    //                          "lumi.sensor_motion.aq2",
    //                          strlen("lumi.sensor_motion.aq2")))
    //             {
    //                 appDataBaseSave();
    //                 rtcgq11lmAdd(device->u64IeeeAddr);
    //             }
    //             else if (!strncmp((const char *)payload->asAttrList[i].uAttrData.au8AttrData,
    //                               "lumi.weather",
    //                               strlen("lumi.weather")))
    //             {
    //                 appDataBaseSave();
    //                 wsdcgq11lmAdd(device->u64IeeeAddr);
    //             }
    //             else if (!strncmp((const char *)payload->asAttrList[i].uAttrData.au8AttrData,
    //                               "LILYGO.Light",
    //                               strlen("LILYGO.Light")))
    //             {
    //                 appDataBaseSave();
    //                 lilygoLightAdd(device->u64IeeeAddr);
    //             }
    //             else if (!strncmp((const char *)payload->asAttrList[i].uAttrData.au8AttrData,
    //                               "LILYGO.Sensor",
    //                               strlen("LILYGO.Sensor")))
    //             {
    //                 appDataBaseSave();
    //                 lilygoSensorAdd(device->u64IeeeAddr);
    //             }
    //             else if (!strncmp((const char *)payload->asAttrList[i].uAttrData.au8AttrData,
    //                               "ESP32C6.Light",
    //                               strlen("ESP32C6.Light")))
    //             {
    //                 appDataBaseSave();
    //                 espressifLightAdd(device->u64IeeeAddr);
    //                 // The configure method below is needed to make the device reports on/off state changes
    //                 // when the device is controlled manually through the button on it.
    //                 zbhci_BindingReq(
    //                     device->u64IeeeAddr,
    //                     1,
    //                     0x0006,
    //                     0x03,
    //                     (ts_DstAddr){
    //                         .u64DstAddr = brigeNode.macAddr},
    //                     1);
    //             }
    //         }
    //     }
    //     break;

    // case 0x0400: /* Illuminance Measurement Cluster */
    // {
    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0000)
    //         {
    //             device->deviceData.rtcgq11lm.u16Illuminance = payload->asAttrList[i].uAttrData.u16AttrData;
    //             if (!strncmp((const char *)device->au8ModelId,
    //                          "lumi.sensor_motion.aq2",
    //                          strlen("lumi.sensor_motion.aq2")))
    //             {
    //                 rtcgq11lmReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.rtcgq11lm.u8Occupancy,
    //                     device->deviceData.rtcgq11lm.u16Illuminance);
    //             }
    //         }
    //     }
    // }
    // break;

    // case 0x0402: /* Temperature Measurement Cluster */
    // {
    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0000)
    //         {
    //             device->deviceData.wsdcgq11lm.i16Temperature = (int16_t)payload->asAttrList[i].uAttrData.u16AttrData;
    //             if (!strncmp((const char *)device->au8ModelId,
    //                          "lumi.weather",
    //                          strlen("lumi.weather")))
    //             {
    //                 wsdcgq11lmReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.wsdcgq11lm.i16Temperature,
    //                     device->deviceData.wsdcgq11lm.i16Humidity,
    //                     device->deviceData.wsdcgq11lm.i16Pressure);
    //             }
    //             else if (!strncmp((const char *)device->au8ModelId,
    //                               "LILYGO.Sensor",
    //                               strlen("LILYGO.Sensor")))
    //             {
    //                 lilygoSensorReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.sensor.i16Temperature,
    //                     device->deviceData.sensor.i16Humidity);
    //             }
    //         }
    //     }
    // }
    // break;

    // case 0x0403: /* Pressure Measurement Cluster */
    // {
    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0000)
    //         {
    //             device->deviceData.wsdcgq11lm.i16Pressure = (int16_t)payload->asAttrList[i].uAttrData.u16AttrData;
    //             if (!strncmp((const char *)device->au8ModelId,
    //                          "lumi.weather",
    //                          strlen("lumi.weather")))
    //             {
    //                 wsdcgq11lmReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.wsdcgq11lm.i16Temperature,
    //                     device->deviceData.wsdcgq11lm.i16Humidity,
    //                     device->deviceData.wsdcgq11lm.i16Pressure);
    //             }
    //         }
    //     }
    // }
    // break;

    // case 0x0406: /* Occupancy Sensing Cluster */
    // {
    //     for (size_t i = 0; i < payload->u8AttrNum; i++)
    //     {
    //         if (payload->asAttrList[i].u16AttrID == 0x0000)
    //         {
    //             device->deviceData.rtcgq11lm.u8Occupancy = payload->asAttrList[i].uAttrData.u8AttrData;
    //             if (!strncmp((const char *)device->au8ModelId,
    //                          "lumi.sensor_motion.aq2",
    //                          strlen("lumi.sensor_motion.aq2")))
    //             {
    //                 rtcgq11lmReport(
    //                     device->u64IeeeAddr,
    //                     device->deviceData.rtcgq11lm.u8Occupancy,
    //                     device->deviceData.rtcgq11lm.u16Illuminance);
    //             }
    //         }
    //     }
    // }
    // break;

    default:
        break;
    }
}

void appHandleZCLReadResponse(ts_MsgZclAttrReadRspPayload *payload)
{
    DeviceNode *device = NULL;
    device = findDeviceByNwkaddr(payload->u16SrcAddr);
    if (device == NULL)
    {
        return;
    }
    switch (payload->u16ClusterId)
    {
    case 0x0000: /* Basic Cluster */
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0005)
            {
                printf("attr\n");
                memcpy(
                    device->au8ModelId,
                    payload->asAttrReadList[i].uAttrData.au8AttrData,
                    payload->asAttrReadList[i].u16DataLen);
                if (!strncmp((const char *)payload->asAttrReadList[i].uAttrData.au8AttrData,
                             "lumi.sensor_motion.aq2",
                             strlen("lumi.sensor_motion.aq2")))
                {
                    appDataBaseSave();
                    rtcgq11lmAdd(device->u64IeeeAddr);
                }
                else if (!strncmp((const char *)payload->asAttrReadList[i].uAttrData.au8AttrData,
                                  "lumi.weather",
                                  strlen("lumi.weather")))
                {
                    appDataBaseSave();
                    wsdcgq11lmAdd(device->u64IeeeAddr);
                }
                else if (!strncmp((const char *)payload->asAttrReadList[i].uAttrData.au8AttrData,
                                  "LILYGO.Light",
                                  strlen("LILYGO.Light")))
                {
                    appDataBaseSave();
                    lilygoLightAdd(device->u64IeeeAddr);
                }
                else if (!strncmp((const char *)payload->asAttrReadList[i].uAttrData.au8AttrData,
                                  "LILYGO.Sensor",
                                  strlen("LILYGO.Sensor")))
                {
                    appDataBaseSave();
                    lilygoSensorAdd(device->u64IeeeAddr);
                }
                else if (!strncmp((const char *)payload->asAttrReadList[i].uAttrData.au8AttrData,
                                  "ESP32C6.Light",
                                  strlen("ESP32C6.Light")))
                {
                    appDataBaseSave();
                    espressifLightAdd(device->u64IeeeAddr);
                    // The configure method below is needed to make the device reports on/off state changes
                    // when the device is controlled manually through the button on it.
                    zbhci_BindingReq(
                        device->u64IeeeAddr,
                        1,
                        0x0006,
                        0x03,
                        (ts_DstAddr){
                            .u64DstAddr = brigeNode.macAddr},
                        1);
                }
            }
        }
        break;

    case 0x0006:
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.light.u8State = payload->asAttrReadList[i].uAttrData.u8AttrData;
                if (!strncmp((const char *)device->au8ModelId, "LILYGO.Light", strlen("LILYGO.Light")) ||
                    !strncmp((const char *)device->au8ModelId, "ESP32C6.Light", strlen("ESP32C6.Light")))
                {
                    lilygoLightReport(
                        device->u64IeeeAddr,
                        device->deviceData.light.u8State);
                }
            }
        }
    }
    break;

    case 0x0400: /* Illuminance Measurement Cluster */
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.rtcgq11lm.u16Illuminance = payload->asAttrReadList[i].uAttrData.u16AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.sensor_motion.aq2",
                             strlen("lumi.sensor_motion.aq2")))
                {
                    rtcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.rtcgq11lm.u8Occupancy,
                        device->deviceData.rtcgq11lm.u16Illuminance);
                }
            }
        }
    }
    break;

    case 0x0402: /* Temperature Measurement Cluster */
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.wsdcgq11lm.i16Temperature = (int16_t)payload->asAttrReadList[i].uAttrData.u16AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.weather",
                             strlen("lumi.weather")))
                {
                    wsdcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.wsdcgq11lm.i16Temperature,
                        device->deviceData.wsdcgq11lm.i16Humidity,
                        device->deviceData.wsdcgq11lm.i16Pressure);
                }
                else if (!strncmp((const char *)device->au8ModelId,
                                  "LILYGO.Sensor",
                                  strlen("LILYGO.Sensor")))
                {
                    lilygoSensorReport(
                        device->u64IeeeAddr,
                        device->deviceData.sensor.i16Temperature,
                        device->deviceData.sensor.i16Humidity);
                }
            }
        }
    }
    break;

    case 0x0403: /* Pressure Measurement Cluster */
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.wsdcgq11lm.i16Pressure = (int16_t)payload->asAttrReadList[i].uAttrData.u16AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.weather",
                             strlen("lumi.weather")))
                {
                    wsdcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.wsdcgq11lm.i16Temperature,
                        device->deviceData.wsdcgq11lm.i16Humidity,
                        device->deviceData.wsdcgq11lm.i16Pressure);
                }
            }
        }
    }
    break;

    case 0x8104: /* Pressure Measurement Cluster */
    {
        Serial.println("keldi");
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.wsdcgq11lm.i16Pressure = (int16_t)payload->asAttrReadList[i].uAttrData.u16AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.weather",
                             strlen("lumi.weather")))
                {
                    wsdcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.wsdcgq11lm.i16Temperature,
                        device->deviceData.wsdcgq11lm.i16Humidity,
                        device->deviceData.wsdcgq11lm.i16Pressure);
                }
            }
        }
    }
    break;

    case 0x0405: /* Relative Humidity Measurement Cluster */
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.wsdcgq11lm.i16Humidity = (int16_t)payload->asAttrReadList[i].uAttrData.u16AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.weather",
                             strlen("lumi.weather")))
                {
                    wsdcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.wsdcgq11lm.i16Temperature,
                        device->deviceData.wsdcgq11lm.i16Humidity,
                        device->deviceData.wsdcgq11lm.i16Pressure);
                }
                else if (!strncmp((const char *)device->au8ModelId,
                                  "LILYGO.Sensor",
                                  strlen("LILYGO.Sensor")))
                {
                    lilygoSensorReport(
                        device->u64IeeeAddr,
                        device->deviceData.sensor.i16Temperature,
                        device->deviceData.sensor.i16Humidity);
                }
            }
        }
    }
    break;

    case 0x0406: /* Occupancy Sensing Cluster */
    {
        for (size_t i = 0; i < payload->u8AttrNum; i++)
        {
            if (payload->asAttrReadList[i].u16AttrID == 0x0000)
            {
                device->deviceData.rtcgq11lm.u8Occupancy = payload->asAttrReadList[i].uAttrData.u8AttrData;
                if (!strncmp((const char *)device->au8ModelId,
                             "lumi.sensor_motion.aq2",
                             strlen("lumi.sensor_motion.aq2")))
                {
                    rtcgq11lmReport(
                        device->u64IeeeAddr,
                        device->deviceData.rtcgq11lm.u8Occupancy,
                        device->deviceData.rtcgq11lm.u16Illuminance);
                }
            }
        }
    }
    break;

    default:
        break;
    }
}

static void appHandlrMqttPermitJoin(const char *topic, const char *data)
{
    if (!data)
        return;

    cJSON *json = cJSON_Parse(data);
    if (!json)
    {
        Serial.println("json error\n");
        return;
    }

    cJSON *value = cJSON_GetObjectItem(json, "value");
    cJSON *time = cJSON_GetObjectItem(json, "time");
    if (value && time)
    {
        if (value->valueint && time->valueint > 0x00 && time->valueint < 0xFF)
        {
            zbhci_MgmtPermitJoinReq(0xFFFC, time->valueint, 1);
        }
        else if (value->valueint && time->valueint <= 0)
        {
            zbhci_MgmtPermitJoinReq(0xFFFC, 0x00, 1);
        }
        else if (value->valueint && time->valueint >= 0xFF)
        {
            zbhci_MgmtPermitJoinReq(0xFFFC, 0xFF, 1);
        }
    }
    else if (value && !time)
    {
        if (value->valueint)
        {
            zbhci_MgmtPermitJoinReq(0xFFFC, 0xFF, 1);
        }
        else
        {
            zbhci_MgmtPermitJoinReq(0xFFFC, 0x00, 1);
        }
    }

    cJSON_Delete(json);
}
