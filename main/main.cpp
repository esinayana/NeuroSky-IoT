/**
 * @file esp32_bttest.ino
 * ESP32 sketch to connect to Neurosky headset.
 * @date 2018.05.05
 */
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

typedef unsigned char   byte;


// Access token for Ubidots account.
#define TOKEN ""

/* 
 * MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
 * it should be a random and unique ascii string and different from all other devices
 */
#define MQTT_CLIENT_NAME "BOxxG9Mn7VCHVxF"
#define ATTENTION_VAR_LABEL "attention" // Assign the variable label
#define MEDITATION_VAR_LABEL "meditation" // Assign the variable label
#define DEVICE_LABEL "esp32-neuro" // Assign the device label

const char mqttBroker[]  = "things.ubidots.com";

// Wi-Fi connection object.
//WiFiClient wlan;
// Web client used to communicate with IoT cloud service (Ubidots).
//PubSubClient client(wlan);


#if 1
/*
 * Debug message function.
 * Currently uses serial port for output.
 * @param[in]   lvl - (Not used)
 * @param[in]   fmt - printf()-like format string
 * @param[in]   args... - printf() arguments for string formatting.
 */
template <class... Args>
void trace(const char* lvl, const char* fmt, Args... args)
{
    char buf[256];
    sprintf(buf, fmt, args...);

//    ESP_LOGI(lvl, buf);
}
#else
#define trace(_tag, _fmt, ...) //  ESP_LOGI(_tag, _fmt, ##__VA_ARGS__)
#endif

#define GAP_TAG          "GAP"
#define SPP_TAG          "SPP"


typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;


static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static app_gap_cb_t m_dev_info;

QueueHandle_t serialQueue;


static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
        p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static char* uuid2str(esp_bt_uuid_t* uuid, char* str, size_t size)
{
    if (!uuid || !str) {
        return nullptr;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08x", uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
            p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
            p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return nullptr;
    }

    return str;
}

int millis()
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t* rmt_bdname = nullptr;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    char bdname[64] = {};
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(GAP_TAG, "Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            memcpy(bdname, p->val, p->len);
            bdname[p->len] = '\0';
            ESP_LOGI(GAP_TAG, "--Name of Device: %s", bdname);
            break;

        default:
            break;
        }
    }

    app_gap_cb_t* p_dev = &m_dev_info;
#if 0
    /* search for device with MAJOR service class as "rendering" in COD */
    if (p_dev->dev_found && 0 != memcmp(param->disc_res.bda, p_dev->bda, ESP_BD_ADDR_LEN)) {
        return;
    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
        !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE)) {
        return;
    }
#endif  // 0

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;
    for (int i = 0; i < param->disc_res.num_prop; ++i) {
        p = param->disc_res.prop + i;

        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            p_dev->cod = *(uint32_t*)(p->val);
            break;

        case ESP_BT_GAP_DEV_PROP_RSSI:
            p_dev->rssi = *(int8_t*)(p->val);
            break;

        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            uint8_t len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN : (uint8_t)p->len;
            memcpy(p_dev->bdname, (uint8_t*)p->val, len);
            p_dev->bdname[len] = '\0';
            p_dev->bdname_len = len;
            } break;

        case ESP_BT_GAP_DEV_PROP_EIR:
            memcpy(p_dev->eir, (uint8_t *)(p->val), p->len);
            p_dev->eir_len = p->len;
            break;
        }
    }

    if (p_dev->eir && p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);

#if 0
        if (strcmp((char*)p_dev->bdname, "Sichiray") != 0) {
            return;
        }
#endif

        ESP_LOGI(GAP_TAG, "Found a target device, address %s, name %s", bda_str, p_dev->bdname);

        p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
        ESP_LOGI(GAP_TAG, "Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}

void bt_app_gap_init(void)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], m_dev_info.bda);
        }
        break;

    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
//        esp_spp_write(param->srv_open.handle, SPP_DATA_LEN, spp_data);
//        gettimeofday(&time_old, NULL);
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;

    case ESP_SPP_DATA_IND_EVT:
        for (int i = 0; i < param->data_ind.len; ++i) {
//            if (!serialBuffer.pushBack(param->data_ind.data[i])) {
            if (xQueueSend(serialQueue, &param->data_ind.data[i], 10 / portTICK_RATE_MS) != pdTRUE) {
                ESP_LOGW(SPP_TAG, "serial in> [!] buffer overflow! Dropping %d bytes", param->data_ind.len - i);
                break;
            }
        }
        break;

    case ESP_SPP_CONG_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
#endif
        if (param->cong.cong == 0) {
//            esp_spp_write(param->cong.handle, SPP_DATA_LEN, spp_data);
        }
        break;

    case ESP_SPP_WRITE_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);
#else
        gettimeofday(&time_new, NULL);
        data_num += param->write.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        if (param->write.cong == 0) {
  //          esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    }
}


void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    char bda_str[18];
    char uuid_str[37];

    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        update_device_info(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            trace(GAP_TAG, "Device discovery stopped.");
            if (p_dev->dev_found && (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE || p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)) {
                p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
                trace(GAP_TAG, "start SPP discovery...");
                esp_spp_start_discovery(m_dev_info.bda);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            trace(GAP_TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
            p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                trace(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
                for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
                    esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
                    trace(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
                    // trace(GAP_TAG, "--%d", u->len);
                }
            } else {
                trace(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
            }
        }
        break;
    }

    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default:
        trace(GAP_TAG, "event: %d", event);
        break;
    }
    return;
}

void initBluetooth()
{
//    char *dev_name = "ESP_GAP_INQRUIY";
//    esp_bt_dev_set_device_name(dev_name);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE);

    /* register GAP callback function */
    esp_bt_gap_register_callback(bt_app_gap_cb);

    esp_err_t ret = 0;
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        trace(SPP_TAG, "SPP register failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        trace(SPP_TAG, "SPP init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* initialize device information and status */
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    /* start to discover nearby Bluetooth devices */
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
//    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
}


#if 0
void callback(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; ++i) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}
void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");

        // Attemp to connect
        if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
            Serial.println("Connected");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            // Wait 2 seconds before retrying
            delay(2000);
        }
    }
}
#endif // 0

bool btStart()
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return false;
    }

    return true;
}

void setup()
{
    ESP_LOGE(GAP_TAG, "setup() called");

#if 0
    while (!Serial)
        delay(10);     // will pause Zero, Leonardo, etc until serial console opens

    // Set up WLAN connection.
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);
#endif // 0

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!btStart()) {
        ESP_LOGE(GAP_TAG, "Failed to initialize controller");
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(GAP_TAG, "Failed to initialize bluedroid");
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(GAP_TAG, "Failed to enable bluedroid");
        return;
    }

    initBluetooth();

    ESP_LOGE(GAP_TAG, "setup() OK");
}

byte poorQuality = 0;
byte attention = 0;
byte meditation = 0;
bool bigPacket = false;

long lastReceivedPacket = 0;



////////////////////////////////
// Read data from Serial UART //
////////////////////////////////
byte ReadOneByte()
{
    byte val;
//    while (!serialBuffer.popFront(&val));

    while (!xQueueReceive(serialQueue, &val, (portTickType)portMAX_DELAY));

    return val;
}


byte payloadData[64] = {};


void loop()
{
#if 0
    if (!client.connected()) {
        reconnect();
    }
#endif

    /*
     * The code below is taken from MindWave Arduino tutorial.
     * @see http://developer.neurosky.com/docs/doku.php?id=arduino_tutorial
     */

    byte payloadLength = 0;
    byte generatedChecksum = 0;
    byte checksum = 0;

    // Look for sync bytes
    if (ReadOneByte() == 170) {
        if (ReadOneByte() == 170) {

            payloadLength = ReadOneByte();
            if (payloadLength > 169)                      //Payload length can not be greater than 169
                return;

            generatedChecksum = 0;
            for (int i = 0; i < payloadLength; i++) {
                payloadData[i] = ReadOneByte();            //Read payload into memory
                generatedChecksum += payloadData[i];
            }

            checksum = ReadOneByte();                      //Read checksum byte from stream      
            generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum

            if (checksum == generatedChecksum) {
//                trace(0, "got packet len %d", payloadLength);

                poorQuality = 200;
                attention = 0;
                meditation = 0;

                for (int i = 0; i < payloadLength; i++) {    // Parse the payload
                    switch (payloadData[i]) {
                    case 2:
                        i++;
                        poorQuality = payloadData[i];
                        bigPacket = true;
                        break;
                    case 4:
                        i++;
                        attention = payloadData[i];
                        break;
                    case 5:
                        i++;
                        meditation = payloadData[i];
                        break;
                    case 0x80:
                        i = i + 3;
                        break;
                    case 0x83:
                        i = i + 25;
                        break;
                    default:
                        break;
                    } // switch
                } // for loop

#if !DEBUGOUTPUT

                if (bigPacket) {
#if 0
                    if (poorQuality == 0)
                        digitalWrite(LED, HIGH);
                    else
                        digitalWrite(LED, LOW);
#endif
                    ESP_LOGI(GAP_TAG, "PoorQuality: %d, Attention: %d, Meditation: %d, Time since last packet: %ld",
                        poorQuality, attention, meditation, millis() - lastReceivedPacket);

                    char payload[100];
                    char topic[150];

                    sprintf(topic, "/v1.6/devices/%s", DEVICE_LABEL); // Format topic
                    sprintf(payload, "{\"" ATTENTION_VAR_LABEL "\":%u, \"" MEDITATION_VAR_LABEL "\":%u}", attention, meditation);

                    ESP_LOGI(GAP_TAG, "Publishing data to Ubidots Cloud: %s, %s", topic, payload);

//                    client.publish(topic, payload);
                }
#endif        
                bigPacket = false;
            } else {
                // Checksum Error
            }  // end if else for checksum
        } // end if read 0xAA byte
    } // end if read 0xAA byte
}

void loop_task(void* /*arg*/)
{
    while (true) {
        loop();
    }
}


extern "C" void app_main()
{
    setup();
    serialQueue = xQueueCreate(4096, 1);

    TaskHandle_t hTask;
    xTaskCreate(loop_task, "neurosky", 4096, NULL, configMAX_PRIORITIES - 3, &hTask);
}
