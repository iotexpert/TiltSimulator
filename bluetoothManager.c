#include <stdio.h>
#include <stdlib.h>

#include "cybsp.h"

#include "FreeRTOS.h"

#include "bluetoothManager.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "queue.h"
#include "btutil.h"

// Read the queue
#define BTM_QUEUE_RATE 200

/*********************************************************************************
*
* Tilt Database Definition
* 
*********************************************************************************/

#define TILT_IBEACON_HEADER_LEN 25
#define TILT_IBEACON_DATA_LEN 5
#define TILT_IBEACON_LEN (TILT_IBEACON_HEADER_LEN + TILT_IBEACON_DATA_LEN)

typedef struct  {
    char *colorName;    // The color string
    bool enable;
    bool dirty;         // A flag to tell if there has been a change since the adv was last written
    int rate;           // rate in ms
    int tempUpdate;     // The update rate for temperature
    int gravUpdate;     // The update rate for gravity
    uint8_t advData[TILT_IBEACON_LEN];
} tilt_t;

// 02 01 04 = Flags
// 0x1A 0xFF = Manufacturer specific data
// 0x4c 0x00 = Apple
// 0x02 = iBeacon
// 0x15 = remaining data length
#define IBEACON_HEADER 0x02,0x01,0x04,0x1A,0xFF,0x4C,0x00,0x02,0x15

static tilt_t tiltDB [] =
{
    {"Red",    false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x10,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Green" , false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x20,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Black" , false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x30,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Purple", false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x40,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Orange", false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x50,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Blue"  , false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x60,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Yellow", false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x70,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
    {"Pink"  , false, true, 0,0,0, {IBEACON_HEADER,0xA4,0x95,0xBB,0x80,0xC5,0xB1,0x4B,0x44,0xB5,0x12,0x13,0x70,0xF0,0x2D,0x74,0xDE, 0x00,0x00,0x00,0x00,0x00}},
};
#define NUM_TILT (sizeof(tiltDB)/sizeof(tilt_t))

#define IBEACON_TEMP_HI (25)
#define IBEACON_TEMP_LO (26)
#define IBEACON_GRAV_HI (27)
#define IBEACON_GRAV_LO (28)
#define IBEACON_TXPOWER (29)

/*********************************************************************************
* 
* Tilt Data Manager External Interface Queue
*
*********************************************************************************/
typedef enum {
    BTM_CMD_SET_DATA,
    BTM_CMD_PRINT_TABLE,
    BTM_CMD_SET_UPDATE,
    BTM_CMD_ENABLE,
    BTM_CMD_DISABLE,
} btm_cmd_t;

typedef struct {
    btm_cmd_t cmd;
    int num;
    int temperature;
    int gravity;
    int txPower;
} btm_cmdMsg_t;

static QueueHandle_t btm_cmdQueue=0;
static wiced_timer_ext_t btm_processDataTimer;


/*********************************************************************************
* 
* These functions are used to interact with the Bluetooth Advertising Controller
*
*********************************************************************************/
// btm_initialize

void btm_initialize()
{
    static wiced_bt_ble_multi_adv_params_t myParams = {
    .adv_int_min       = BTM_BLE_ADVERT_INTERVAL_MIN,
    .adv_int_max       = 96,
    .adv_type          = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map       = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADVERT_FILTER_ALL_CONNECTION_REQ_ALL_SCAN_REQ,
    .adv_tx_power      = MULTI_ADV_TX_POWER_MAX_INDEX,
    .peer_bd_addr      = {0},
    .peer_addr_type    = BLE_ADDR_PUBLIC,
    .own_bd_addr       = {0},
    .own_addr_type     = BLE_ADDR_PUBLIC,
    };

    for(int i=0;i<NUM_TILT;i++)
    {
        wiced_set_multi_advertisement_data(tiltDB[i].advData,sizeof(tiltDB[i].advData),i+1);
        wiced_set_multi_advertisement_params(i+1,&myParams);
        wiced_start_multi_advertisements( MULTI_ADVERT_STOP, i+1);
    }
}

// btm_setAdvPacket
//
// This function updates what the advertising packets and the state of the BT controller:
// It will loop through the table of iBeacons... then if they are in a slot & they are dirty
// it will move them to the contoller and enable advertisements on that slot

void btm_setAdvPacket()
{
    for(int i=0;i<NUM_TILT;i++)
    {
        if(tiltDB[i].dirty)
        {   
            tiltDB[i].dirty = false;
            wiced_set_multi_advertisement_data(tiltDB[i].advData,sizeof(tiltDB[i].advData),i+1);
        
            if(tiltDB[i].enable)
                wiced_start_multi_advertisements( MULTI_ADVERT_START, i+1 );
            else
                wiced_start_multi_advertisements( MULTI_ADVERT_STOP, i+1 );
        }
    }
}

/*********************************************************************************
* 
* This next set of functions is the API to the database
*
*********************************************************************************/


int btm_getTemperature(int num)
{
    return (uint16_t)tiltDB[num].advData[IBEACON_TEMP_HI] << 8 | tiltDB[num].advData[IBEACON_TEMP_LO];
}

void btm_setTemperature(int num,uint16_t temperature)
{
    if(temperature > 150)
        temperature = 10;
    if(temperature<10)
        temperature = 150;

    int oldtemp = btm_getTemperature(num);

    tiltDB[num].advData[IBEACON_TEMP_HI] = (temperature & 0xFF00) >> 8;    
    tiltDB[num].advData[IBEACON_TEMP_LO] = temperature & 0xFF;
    if(temperature != oldtemp)
        tiltDB[num].dirty = true;    
}

int btm_getGravity(int num)
{
    return (uint16_t)tiltDB[num].advData[IBEACON_GRAV_HI] << 8 | tiltDB[num].advData[IBEACON_GRAV_LO];
}

void btm_setGravity(int num,uint16_t gravity)
{
    // These if's cause the gravity to "wrap around" at 1200 and 900
    if(gravity>1200)
        gravity = 900;
    if(gravity <900)
        gravity = 1200;

    int oldgrav = btm_getGravity(num);
    tiltDB[num].advData[IBEACON_GRAV_HI] = (uint8_t)((gravity & 0xFF00) >> 8);
    tiltDB[num].advData[IBEACON_GRAV_LO] = (uint8_t)(gravity & 0xFF);
    if(oldgrav != gravity)
        tiltDB[num].dirty = true;
}

void btm_setTxPower(int num, int8_t txPower)
{
    tiltDB[num].advData[IBEACON_TXPOWER] = txPower;
    tiltDB[num].dirty = true;
}

void btm_printTable()
{
    printf("\n# Color   E   Rate T  UpT Grav UpG TxP\n");

    for(int i=0;i<NUM_TILT;i++)
    {
        printf("%d %6s  %d %5d %3d %2d %4d %2d %3d\n",i,
            tiltDB[i].colorName,tiltDB[i].enable,
            tiltDB[i].rate*BTM_QUEUE_RATE,
            btm_getTemperature(i),tiltDB[i].tempUpdate,
            btm_getGravity(i),tiltDB[i].gravUpdate,
            tiltDB[i].advData[29]);
    }
}

/* btm_processCmdQueue
*
*  This function is called by a timer every BTM_QUEUE_RATE ms (around 200ms)
*  It processes commands from the GUI
*  It also updates the adverting packets if they are being updated at a rate
*
*/
void btm_processCmdQueue( wiced_timer_callback_arg_t cb_params )
{
    static int count = 0; // This counts the numbers of times the callback has happend

    btm_cmdMsg_t msg;
    while(xQueueReceive(btm_cmdQueue,&msg,0) == pdTRUE)
    {
        switch(msg.cmd)
        {
            case BTM_CMD_SET_DATA:
                btm_setGravity(msg.num,msg.gravity);
                btm_setTemperature(msg.num,msg.temperature);
                btm_setTxPower(msg.num,msg.txPower);
            break;

            case BTM_CMD_PRINT_TABLE:
                btm_printTable();
            break;

            case BTM_CMD_SET_UPDATE:
                tiltDB[msg.num].tempUpdate = msg.temperature;
                tiltDB[msg.num].gravUpdate = msg.gravity;
                tiltDB[msg.num].rate = msg.txPower / BTM_QUEUE_RATE;
            break;

            case BTM_CMD_ENABLE:
                tiltDB[msg.num].enable = true;
                wiced_start_multi_advertisements( MULTI_ADVERT_START, msg.num+1 );
            break;
            case BTM_CMD_DISABLE:
                tiltDB[msg.num].enable = false;
                wiced_start_multi_advertisements( MULTI_ADVERT_STOP, msg.num+1 );
            break;
        }
    }
    count = count + 1;

    // This block of code updates the current values with the update rate
    for(int i=0;i<NUM_TILT;i++)
    {
        // If the slot active
        // and the update rate says that it is time
        if(tiltDB[i].enable && count % tiltDB[i].rate  == 0)
        {
            btm_setTemperature(i,btm_getTemperature(i) + tiltDB[i].tempUpdate);
            btm_setGravity(i,btm_getGravity(i) + tiltDB[i].gravUpdate);
        }
    }
    btm_setAdvPacket();
}

/**************************************************************************************************
* Function Name: app_bt_management_callback()
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            printf("Started BT Stack Succesfully\n");
            btm_initialize();
            btm_cmdQueue = xQueueCreate(10,sizeof(btm_cmdMsg_t));
            wiced_init_timer_ext(&btm_processDataTimer,btm_processCmdQueue,0,WICED_TRUE);
            wiced_start_timer_ext(&btm_processDataTimer,BTM_QUEUE_RATE);
        break;

        case BTM_MULTI_ADVERT_RESP_EVENT: // Do nothing...
        break;

        default:
            printf("Unhandled Bluetooth Management Event: %s\n", btutil_getBTEventName( event));
        break;
    }

    return result;
}

/*********************************************************************************
* 
* These are publicly callable functions to cause actions by the bluetooth manager
* These are called by the GUI
*
*********************************************************************************/

void btm_printTableCmd()
{
    if(btm_cmdQueue == 0)
        return;
    
    btm_cmdMsg_t msg;
    msg.cmd =    BTM_CMD_PRINT_TABLE;
    xQueueSend(btm_cmdQueue,&msg,0);
}

void btm_setDataCmd(int num,int temperature,int gravity,int txPower)
{
    if(btm_cmdQueue == 0)
        return;

    btm_cmdMsg_t msg;
    msg.cmd =    BTM_CMD_SET_DATA;
    msg.num = num;
    msg.temperature = temperature;
    msg.gravity = gravity;
    msg.txPower = txPower;
    xQueueSend(btm_cmdQueue,&msg,0);
}


void btm_updateDataCmd(int num,int rate ,int temperature,int gravity )
{
    if(btm_cmdQueue == 0)
        return;
    btm_cmdMsg_t msg;
    msg.cmd =    BTM_CMD_SET_UPDATE;
    msg.num = num;
    msg.temperature = temperature;
    msg.gravity = gravity;
    msg.txPower = rate;
    xQueueSend(btm_cmdQueue,&msg,0);
}

void btm_updateEnable(int num)
{
    if(btm_cmdQueue == 0 || num<0 || num>=NUM_TILT)
    {
        return;
    }
    btm_cmdMsg_t msg;
    msg.cmd =    BTM_CMD_ENABLE;
    msg.num = num;
    xQueueSend(btm_cmdQueue,&msg,0);
}

void btm_updateDisable(int num)
{
    if(btm_cmdQueue == 0 || num<0 || num>=NUM_TILT)
        return;
    btm_cmdMsg_t msg;
    msg.cmd =    BTM_CMD_DISABLE;
    msg.num = num;
    xQueueSend(btm_cmdQueue,&msg,0);
}
