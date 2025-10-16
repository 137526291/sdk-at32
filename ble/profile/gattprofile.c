/********************************** (C) COPYRIGHT *******************************
 * File Name          : gattprofile.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : 自定义包含五种不同属性的服务，包含可读、可写、通知、可读可写、安全可读
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "gattprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Position of serial_profilechar4 value in attribute array
// #define SIMPLEPROFILE_NOTIFY_CHAR_VALUE_POS    11  use SERIAL_IDX_CHAR_TX_VALUE
#define CHAR_DESCRIPTION 0

/*********************************************************************
 * TYPEDEFS
 */
#define NUS_UUID128_ARR(uuid16)       {0x9E, 0xCA, 0xDC, 0x24, \
                                        0x0E, 0xE5, 0xA9, 0xE0, \
                                        0x93, 0xF3, 0xA3, 0xB5, \
                                        (uuid16 & 0xff),(uuid16&0xff00)>>8, 0x40, 0x6E}
/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0001
const uint8_t serial_profileServUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)};

const uint8_t serial_profileServUUID16[] = NUS_UUID128_ARR(SIMPLEPROFILE_SERV_UUID);
const uint8_t serial_profilechar1UUID[] = NUS_UUID128_ARR(SIMPLEPROFILE_CHAR1RX_UUID);
const uint8_t serial_profilechar2UUID[] = NUS_UUID128_ARR(SIMPLEPROFILE_CHAR2TX_UUID);

// Characteristic 1 UUID: 0x0002
// const uint8_t serial_profilechar1UUID[ATT_BT_UUID_SIZE] = {
//     LO_UINT16(SIMPLEPROFILE_CHAR1RX_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1RX_UUID)};

// // Characteristic 2 UUID: 0xFFF2
// const uint8_t serial_profilechar2UUID[ATT_BT_UUID_SIZE] = {
//     LO_UINT16(SIMPLEPROFILE_CHAR2TX_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2TX_UUID)};

// Characteristic 3 UUID: 0xFFF3
const uint8_t serial_profilechar3UUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID)};

// Characteristic 4 UUID: 0xFFF4
const uint8_t serial_profilechar4UUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)};

// Characteristic 5 UUID: 0xFFF5
const uint8_t serial_profilechar5UUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID)};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static serial_profileCBs_t *serial_profile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
// static const gattAttrType_t serial_profileService = {ATT_BT_UUID_SIZE, serial_profileServUUID};
static const gattAttrType_t serial_profileService16 = {sizeof(serial_profileServUUID16), serial_profileServUUID16};

// Simple Profile Characteristic 1 Properties
static uint8_t serial_profileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 RX Value
static uint8_t serial_profileChar1[SIMPLEPROFILE_CHAR1RX_LEN] = {0};

// Simple Profile Characteristic 1 User Description
static uint8_t serial_profileChar1UserDesp[] = "Characteristic 1 RX\0";
// Simple Profile Characteristic 2 Properties
static uint8_t serial_profileChar2Props = GATT_PROP_NOTIFY;

// Characteristic 2 Value
static uint8_t serial_profileChar2[SIMPLEPROFILE_CHAR2TX_LEN] = {0};
// Simple Profile Characteristic 2 User Description
static uint8_t serial_profileChar2UserDesp[] = "Characteristic 2 TX\0";

// Simple Profile Characteristic 3 Properties
static uint8_t serial_profileChar3Props = GATT_PROP_NOTIFY;

// Characteristic 3 Value
static uint8_t serial_profileChar3[SIMPLEPROFILE_CHAR3_LEN] = {0};

// Simple Profile Characteristic 3 User Description
static uint8_t serial_profileChar3UserDesp[] = "Characteristic 3\0";

// Simple Profile Characteristic 4 Properties
static uint8_t serial_profileChar4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8_t serial_profileChar4[1] = {0};

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t serial_profileChar4Config[1];

// Simple Profile Characteristic 4 User Description
static uint8_t serial_profileChar4UserDesp[] = "Characteristic 4\0";

// Simple Profile Characteristic 5 Properties
static uint8_t serial_profileChar5Props = GATT_PROP_READ;

// Characteristic 5 Value
static uint8_t serial_profileChar5[SIMPLEPROFILE_CHAR5_LEN] = {0};

// Simple Profile Characteristic 5 User Description
static uint8_t serial_profileChar5UserDesp[] = "Characteristic 5\0";

/*********************************************************************
 * Profile Attributes tab - Table Len-Type-Value like
 */

typedef enum {
    SERIAL_IDX_SERVICE = 0,

    SERIAL_IDX_CHAR_RX_DECLARATION,
    SERIAL_IDX_CHAR_RX_VALUE,
    // SERIAL_IDX_CHAR_RX_USER_DESCRIPTION,

    SERIAL_IDX_CHAR_TX_DECLARATION,
    SERIAL_IDX_CHAR_TX_VALUE,
    SERIAL_IDX_CHAR_TX_CFG, //tx have ccc
} profile_attr_index_e;

static gattAttribute_t serial_profileAttrTbl[] = {
    // Simple Profile Service
    {
        {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
        GATT_PERMIT_READ,                       /* permissions */
        0,                                      /* handle */
        (uint8_t *)&serial_profileService16        /* pValue */
    },

    // Characteristic 1 RX Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &serial_profileChar1Props},

    // Characteristic RX Value 1
    {
        {sizeof(serial_profilechar1UUID), serial_profilechar1UUID},
        GATT_PERMIT_READ|GATT_PERMIT_WRITE,
        0,
        serial_profileChar1},
#if CHAR_USER_DESCRIPTION
    // Characteristic 1 RX User Description(optional)
    {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar1UserDesp},
#endif
    // Characteristic 2 TX Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &serial_profileChar2Props},

    // Characteristic Value 2 TX
    {
        {sizeof(serial_profilechar2UUID), serial_profilechar2UUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar2},
    // Characteristic 2 TX configuration
    {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)serial_profileChar4Config},
        
#if CHAR_USER_DESCRIPTION
    // Characteristic 2 TX User Description
    {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar2UserDesp},
#endif
#if 0 //hide 345
    // Characteristic 3 Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &serial_profileChar3Props},

    // Characteristic Value 3
    {
        {ATT_BT_UUID_SIZE, serial_profilechar3UUID},
        GATT_PERMIT_WRITE,
        0,
        serial_profileChar3},

    // Characteristic 3 User Description
    {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar3UserDesp},

    // Characteristic 4 Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &serial_profileChar4Props},

    // Characteristic Value 4
    {
        {ATT_BT_UUID_SIZE, serial_profilechar4UUID},
        0,
        0,
        serial_profileChar4},

    // Characteristic 4 configuration
    {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)serial_profileChar4Config},

    // Characteristic 4 User Description
    {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar4UserDesp},

    // Characteristic 5 Declaration
    {
        {ATT_BT_UUID_SIZE, characterUUID},
        GATT_PERMIT_READ,
        0,
        &serial_profileChar5Props},

    // Characteristic Value 5
    {
        {ATT_BT_UUID_SIZE, serial_profilechar5UUID},
        GATT_PERMIT_AUTHEN_READ,
        0,
        serial_profileChar5},

    // Characteristic 5 User Description
    {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        serial_profileChar5UserDesp},
#endif
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t serial_profile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method);
static bStatus_t serial_profile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

static void serial_profile_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
gattServiceCBs_t serial_profileCBs = {
    serial_profile_ReadAttrCB,  // Read callback function pointer
    serial_profile_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService(uint32_t services)
{
    uint8_t status = SUCCESS;

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, serial_profileChar4Config);

    // Register with Link DB to receive link status change callback
    linkDB_Register(serial_profile_HandleConnStatusCB);

    if(services & SIMPLEPROFILE_SERVICE)
    {
        // Register GATT attribute list and CBs with GATT Server App
        status = GATTServApp_RegisterService(serial_profileAttrTbl,
                                             GATT_NUM_ATTRS(serial_profileAttrTbl),
                                             GATT_MAX_ENCRYPT_KEY_SIZE,
                                             &serial_profileCBs);
    }

    return (status);
}

/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs(serial_profileCBs_t *appCallbacks)
{
    if(appCallbacks)
    {
        serial_profile_AppCBs = appCallbacks;

        return (SUCCESS);
    }
    else
    {
        return (bleAlreadyInRequestedMode);
    }
}

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter(uint8_t param, uint8_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
        case SIMPLEPROFILE_CHAR1:
            if(len == SIMPLEPROFILE_CHAR1RX_LEN)
            {
                tmos_memcpy(serial_profileChar1, value, SIMPLEPROFILE_CHAR1RX_LEN);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case SIMPLEPROFILE_CHAR2:
            if(len == SIMPLEPROFILE_CHAR2TX_LEN)
            {
                tmos_memcpy(serial_profileChar2, value, SIMPLEPROFILE_CHAR2TX_LEN);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case SIMPLEPROFILE_CHAR3:
            if(len == SIMPLEPROFILE_CHAR3_LEN)
            {
                tmos_memcpy(serial_profileChar3, value, SIMPLEPROFILE_CHAR3_LEN);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case SIMPLEPROFILE_CHAR4:
            if(len == SIMPLEPROFILE_CHAR4_LEN)
            {
                tmos_memcpy(serial_profileChar4, value, SIMPLEPROFILE_CHAR4_LEN);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case SIMPLEPROFILE_CHAR5:
            if(len == SIMPLEPROFILE_CHAR5_LEN)
            {
                tmos_memcpy(serial_profileChar5, value, SIMPLEPROFILE_CHAR5_LEN);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }

    return (ret);
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter(uint8_t param, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
        case SIMPLEPROFILE_CHAR1:
            tmos_memcpy(value, serial_profileChar1, SIMPLEPROFILE_CHAR1RX_LEN);
            break;

        case SIMPLEPROFILE_CHAR2:
            tmos_memcpy(value, serial_profileChar2, SIMPLEPROFILE_CHAR2TX_LEN);
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }

    return (ret);
}

/*********************************************************************
 * @fn          serial_profile_Notify
 *
 * @brief       Send a notification containing a heart rate
 *              measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 *
 * @return      Success or Failure
 */
bStatus_t serial_profile_Notify(uint16_t connHandle, attHandleValueNoti_t *pNoti)
{
    uint16_t value = GATTServApp_ReadCharCfg(connHandle, serial_profileChar4Config);

    // If notifications enabled
    if(value & GATT_CLIENT_CFG_NOTIFY)
    {
        // Set the handle
        pNoti->handle = serial_profileAttrTbl[SERIAL_IDX_CHAR_TX_VALUE].handle;

        // Send the notification
        return GATT_Notification(connHandle, pNoti, FALSE);
    }
    return bleIncorrectMode;
}

/*********************************************************************
 * @fn          serial_profile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static bStatus_t serial_profile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method)
{
    bStatus_t status = SUCCESS;


    // Make sure it's not a blob operation (no attributes in the profile are long)
    if(offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if(pAttr->type.len == 2 || pAttr->type.len == 16)
    {
        // 16-bit UUID
        uint16_t uuid = pAttr->type.len == 2 ? \
            BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]) : \
            BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);
        switch(uuid)
        {
            // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
            // gattserverapp handles those reads

            // characteristics 1 and 2 have read permissions
            // characteritisc 3 does not have read permissions; therefore it is not
            //   included here
            // characteristic 4 does not have read permissions, but because it
            //   can be sent as a notification, it is included here
            case SIMPLEPROFILE_CHAR1RX_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR1RX_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR1RX_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR2TX_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR2TX_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR2TX_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR4_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR4_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR4_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            case SIMPLEPROFILE_CHAR5_UUID:
                if(maxLen > SIMPLEPROFILE_CHAR5_LEN)
                {
                    *pLen = SIMPLEPROFILE_CHAR5_LEN;
                }
                else
                {
                    *pLen = maxLen;
                }
                tmos_memcpy(pValue, pAttr->pValue, *pLen);
                break;

            default:
                // Should never get here! (characteristics 3 and 4 do not have read permissions)
                *pLen = 0;
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return (status);
}

/*********************************************************************
 * @fn      serial_profile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t serial_profile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t   notifyApp = 0xFF;

    // If attribute permissions require authorization to write, return error
    if(gattPermitAuthorWrite(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    if(pAttr->type.len >= 2 || pAttr->type.len <= 16)
    {
        // 16-bit UUID
        uint16_t uuid = pAttr->type.len == 2 ? \
            BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]) : \
            BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);

        switch(uuid)
        {
            case SIMPLEPROFILE_CHAR1RX_UUID:
                //Validate the value
                // Make sure it's not a blob oper
                if(offset == 0)
                {
                    if(len > SIMPLEPROFILE_CHAR1RX_LEN)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }
                
                char *data = (char*)pValue;
                //Write the value
                if(status == SUCCESS)
                {
                    tmos_memcpy(pAttr->pValue, pValue, SIMPLEPROFILE_CHAR1RX_LEN);
                    notifyApp = SIMPLEPROFILE_CHAR1;
                }
                PRINT("rx %d %s\r\n", len, data);
                extern void bleSerialTxNotify(uint8_t* data, uint16_t sz);
                bleSerialTxNotify(pValue, len);
                break;
            
            #if 0
            case SIMPLEPROFILE_CHAR3_UUID:
                //Validate the value
                // Make sure it's not a blob oper
                if(offset == 0)
                {
                    if(len > SIMPLEPROFILE_CHAR3_LEN)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                //Write the value
                if(status == SUCCESS)
                {
                    tmos_memcpy(pAttr->pValue, pValue, SIMPLEPROFILE_CHAR3_LEN);
                    notifyApp = SIMPLEPROFILE_CHAR3;
                }
                break;
            #endif

            case GATT_CLIENT_CHAR_CFG_UUID:
                status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                        offset, GATT_CLIENT_CFG_NOTIFY);
                PRINT("GATT char cfg uuid notify\r\n");
                break;

            default:
                // Should never get here! (characteristics 2 and 4 do not have write permissions)
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    // If a charactersitic value changed then callback function to notify application of change
    if((notifyApp != 0xFF) && serial_profile_AppCBs && serial_profile_AppCBs->pfnSimpleProfileChange)
    {
        serial_profile_AppCBs->pfnSimpleProfileChange(notifyApp, pValue, len);
    }

    return (status);
}

/*********************************************************************
 * @fn          serial_profile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void serial_profile_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType)
{
    // Make sure this is not loopback connection
    if(connHandle != LOOPBACK_CONNHANDLE)
    {
        // Reset Client Char Config if connection has dropped
        if((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
           ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) &&
            (!linkDB_Up(connHandle))))
        {
            GATTServApp_InitCharCfg(connHandle, serial_profileChar4Config);
            PRINT("con%d <=> chartxcfg", connHandle);
        }
    }
}

/*********************************************************************
*********************************************************************/
