/**************************************************************************************************
  Filename:       zcl_samplesw.c
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $

  Description:    Zigbee Cluster Library - sample switch application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee On/Off Switch, based on Z-Stack 3.0.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample applicetion.
  
  Application-specific UI peripherals being used:

  - none (LED1 is currently unused by this application).

  Application-specific menu system:

    <TOGGLE LIGHT> Send an On, Off or Toggle command targeting appropriate devices from the binding table.
      Pressing / releasing [OK] will have the following functionality, depending on the value of the 
      zclSampleSw_OnOffSwitchActions attribute:
      - OnOffSwitchActions == 0: pressing [OK] will send ON command, releasing it will send OFF command;
      - OnOffSwitchActions == 1: pressing [OK] will send OFF command, releasing it will send ON command;
      - OnOffSwitchActions == 2: pressing [OK] will send TOGGLE command, releasing it will not send any command.

*********************************************************************/

#if ! defined ZCL_ON_OFF
#error ZCL_ON_OFF must be defined for this project.
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_samplesw.h"
#include "zcl_diagnostic.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_adc.h"

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#include "zcl_ota.h"
#include "hal_ota.h"
#endif

#include "bdb.h"
#include "bdb_interface.h"

#include <stdio.h>
#include "string.h"

/*********************************************************************
 * MACROS
 */

#define APP_TITLE "TI Sample Switch"

#define DATA_PIN P0_6            //定义P0.6口为传感器的输入端CS

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleSw_TaskID;

uint8 zclSampleSwSeqNum;

uint8 zclSampleSw_OnOffSwitchType = ON_OFF_SWITCH_TYPE_MOMENTARY;

uint8 zclSampleSw_OnOffSwitchActions;


#define DATA_H 5
#define DATA_L 6
uint16 test_word;

uint16 temp_VOC;
uint16 temp_O3;
uint16 temp_CHO2;
uint16 temp_CO;
uint16 temp_HUMIDITY;
uint16 temp_TEMPERATURE;
uint16 temp_PM25;
uint16 temp_CO2;
uint16 temp_O2;


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleSw_DstAddr;

// Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleSw_TestEp =
{
  SAMPLESW_ENDPOINT,                  // endpoint
  0,
  &zclSampleSw_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

//static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

devStates_t zclSampleSw_NwkState = DEV_INIT;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#define DEVICE_POLL_RATE                 8000   // Poll rate for end device
#endif

#define SAMPLESW_TOGGLE_TEST_EVT   0x1000
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleSw_HandleKeys( byte shift, byte keys );
static void zclSampleSw_BasicResetCB( void );

static void zclSampleSw_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);


// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_REPORT
static uint8 zclSampleSw_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_REPORT

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
static void zclSampleSw_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg );
#endif

// Uart
#define ZCLSAMPLESW_UART_BUF_LEN        128
static uint8 zclSampleSw_UartBuf[ZCLSAMPLESW_UART_BUF_LEN];
static void zclSampleSw_InitUart(void);
static void zclSampleSw_UartCB(uint8 port, uint8 event);

#ifdef ZDO_COORDINATOR
#else
  static void zclSampleSw_ReportTest(void);
#endif

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * REFERENCED EXTERNALS
 */
extern int16 zdpExternalStateTaskID;

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleSw_CmdCallbacks =
{
  zclSampleSw_BasicResetCB,               // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command

#ifdef ZDO_COORDINATOR
  NULL,
#else
  NULL,    // On/Off cluster commands
#endif
  
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleSw_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleSw_Init( byte task_id )
{
  zclSampleSw_TaskID = task_id;

  // Set destination address to indirect
  zclSampleSw_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleSw_DstAddr.endPoint = 0;
  zclSampleSw_DstAddr.addr.shortAddr = 0;
  
  P0SEL &= ~0x40;                 //设置P0.6为普通IO口
  P0DIR &= ~0x40;                 //P0.6定义为输入口

  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &zclSampleSw_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLESW_ENDPOINT, &zclSampleSw_CmdCallbacks );

  zclSampleSw_ResetAttributesToDefaultValues();
  
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLESW_ENDPOINT, zclSampleSw_NumAttributes, zclSampleSw_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleSw_TaskID );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleSw_TaskID );
  
  bdb_RegisterCommissioningStatusCB( zclSampleSw_ProcessCommissioningStatus );

  // Register for a test endpoint
  afRegister( &sampleSw_TestEp );
  
    // Init Uart
  zclSampleSw_InitUart();
  HalUARTWrite(HAL_UART_PORT_0 ,  "UART0 Init Succeed\r\n" , 20);
  
#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SAMPLESW_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
  // Register for callback events from the ZCL OTA
  zclOTA_Register(zclSampleSw_TaskID);
#endif

  zdpExternalStateTaskID = zclSampleSw_TaskID;
  
#ifdef ZDO_COORDINATOR
  // Init Uart
  zclSampleSw_InitUart();
  
  ZDO_RegisterForZDOMsg ( zclSampleSw_TaskID, Device_annce );
  
  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_FORMATION |
                          BDB_COMMISSIONING_MODE_FINDING_BINDING );
  
  NLME_PermitJoiningRequest(255);
#else
  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING |
                          BDB_COMMISSIONING_MODE_FINDING_BINDING );
  
  osal_start_timerEx(zclSampleSw_TaskID, 
                     SAMPLEAPP_REPORT_EVT, 
                     SAMPLEAPP_REPORT_PERIOD);
#endif
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleSw_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  //Send toggle every 500ms
  if( events & SAMPLESW_TOGGLE_TEST_EVT )
  {
    osal_start_timerEx(zclSampleSw_TaskID,SAMPLESW_TOGGLE_TEST_EVT,500);
    zclGeneral_SendOnOff_CmdToggle( SAMPLESW_ENDPOINT, &zclSampleSw_DstAddr, FALSE, 0 );
    
    // return unprocessed events
    return (events ^ SAMPLESW_TOGGLE_TEST_EVT);
  }
  
  
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleSw_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleSw_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleSw_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          
          break;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
        case ZCL_OTA_CALLBACK_IND:
          zclSampleSw_ProcessOTAMsgs( (zclOTA_CallbackMsg_t*)MSGpkt  );
          break;
#endif
          
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SAMPLEAPP_END_DEVICE_REJOIN_EVT );
  }
#endif
  
#ifdef ZDO_COORDINATOR
#else
  // Rejoin
  if ( events & SAMPLEAPP_REJOIN_EVT )
  {
    bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING |
                      BDB_COMMISSIONING_MODE_FINDING_BINDING );
    
    return ( events ^ SAMPLEAPP_REJOIN_EVT );
  }
  
  // Report
  if ( events & SAMPLEAPP_REPORT_EVT )
  {
      zclSampleSw_ReportTest();
    
      osal_start_timerEx(zclSampleSw_TaskID, 
                     SAMPLEAPP_REPORT_EVT, 
                     SAMPLEAPP_REPORT_PERIOD);
      
      return ( events ^ SAMPLEAPP_REPORT_EVT );
  }
#endif
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleSw_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleSw_HandleKeys( byte shift, byte keys )
{ 
  if(keys & HAL_KEY_SW_6)
  {
    HalLedSet(HAL_LED_2, HAL_LED_MODE_TOGGLE);  
  } 
}
  
/*********************************************************************
 * @fn      zclSampleSw_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleSw_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been processed yet
        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
      }
      else
      {
        #ifdef ZDO_COORDINATOR
        #else
        osal_start_timerEx(zclSampleSw_TaskID, 
                           SAMPLEAPP_REJOIN_EVT, 
                           SAMPLEAPP_REJOIN_PERIOD);
        #endif
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization 
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
      
      //YOUR JOB:
      //We are on a network, what now?
      
    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        osal_start_timerEx(zclSampleSw_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);
      }
    break;
#endif 
  }
}

/*********************************************************************
 * @fn      zclSampleSw_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleSw_BasicResetCB( void )
{
  zclSampleSw_ResetAttributesToDefaultValues();
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleSw_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleSw_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleSw_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleSw_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleSw_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleSw_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleSw_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleSw_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclSampleSw_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleSw_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleSw_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleSw_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleSw_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleSw_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
    if( pInMsg->clusterId == ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG &&
        readRspCmd->attrList[i].attrID == ATTRID_ON_OFF_SWITCH_ACTIONS )
    {
        uint8 val;
        val = *(readRspCmd->attrList[i].data);
        
        HalLcdWriteStringValue("Read:", val, 10, 4);
    }
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleSw_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclSampleSw_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER



#ifdef ZCL_REPORT
/**
 * @fn      zclSampleSw_ProcessInReportCmd
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleSw_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclReportCmd_t *reportCmd;
  uint8 i;
  uint8 resstr[10];     //显示接收数据
  HalLcdWriteString("GetEndDeviceNum", 3); //没数据的时候进行显示
  
  reportCmd = (zclReportCmd_t *)pInMsg->attrCmd;//接收
  for ( i = 0; i < reportCmd->numAttr; i++ )    //看下有几个数据 默认是一个数据
  {                                                     
    if( pInMsg->clusterId == ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG && 
        reportCmd->attrList[i].attrID == ATTRID_ON_OFF_SWITCH_TYPE)     //接收类型
    {
      HalUARTWrite(0, "FSM:", 4);       //提示接收到数据
      uint8 attrDat = *(reportCmd->attrList[i].attrData);       //得到数据
      memset(resstr,0,10);      //清空数组
      sprintf((char *)resstr,(const char*)"%x\r\n",attrDat);    //转为hex格式
      HalUARTWrite(HAL_UART_PORT_0 ,  "RX seqNum:" , 10);
      HalUARTWrite(HAL_UART_PORT_0 ,resstr , osal_strlen((char *)resstr));

      if(attrDat == 0x30)       //判断数据 30开灯 
      {
        HalUARTWrite(HAL_UART_PORT_0 , "LED On",osal_strlen("LED On"));
        HalLcdWriteString( "Led On", HAL_LCD_LINE_3 ); //LCD显示 
        HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF); //低电平开灯
      } 
      if(attrDat == 0x31)       //31关闭灯
      {
        HalUARTWrite(HAL_UART_PORT_0 , "LED Off",osal_strlen("LED Off"));
        HalLcdWriteString( "Led Off", HAL_LCD_LINE_3 ); //LCD显示
        HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);   //高电平关闭灯
      }

      HalUARTWrite(0, "\r\n", 2);         // 回车换行
    }
  }

  return ( TRUE );
}
#endif // ZCL_REPORT




#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
/*********************************************************************
 * @fn      zclSampleSw_ProcessOTAMsgs
 *
 * @brief   Called to process callbacks from the ZCL OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleSw_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg )
{
  uint8 RxOnIdle;

  switch(pMsg->ota_event)
  {
  case ZCL_OTA_START_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Speed up the poll rate
      RxOnIdle = TRUE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate( 2000 );
    }
    break;

  case ZCL_OTA_DL_COMPLETE_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Reset the CRC Shadow and reboot.  The bootloader will see the
      // CRC shadow has been cleared and switch to the new image
      HalOTAInvRC();
      SystemReset();
    }
    else
    {
      // slow the poll rate back down.
      RxOnIdle = FALSE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate(DEVICE_POLL_RATE);
    }
    break;

  default:
    break;
  }
}
#endif // defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)

/****************************************************************************
****************************************************************************/

/**
 * @fn      zclSampleSw_InitUart
 *
 * @brief   init. and open Uart
 */
static void zclSampleSw_InitUart(void)
{
  halUARTCfg_t uartConfig;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 0;
  uartConfig.rx.maxBufSize        = ZCLSAMPLESW_UART_BUF_LEN;
  uartConfig.tx.maxBufSize        = 0;
  uartConfig.idleTimeout          = 6;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = zclSampleSw_UartCB;

  /* Start UART */
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig);
}

/**
 * @fn      zclSampleSw_UartCB
 *CSTX
 * @brief   Uart Callback
uint16 temp_VOC;
uint16 temp_O3;
uint16 temp_CHO2;
uint16 temp_CO;
uint16 temp_HUMIDITY;
uint16 temp_TEMPERATURE;
uint16 temp_PM25;
uint16 temp_CO2;
uint16 temp_O2;

 */
static void zclSampleSw_UartCB(uint8 port, uint8 event)
{
  uint8 rxLen = Hal_UART_RxBufLen(HAL_UART_PORT_0);

  if(rxLen != 0)
  {
    HalUARTRead(HAL_UART_PORT_0  ,  zclSampleSw_UartBuf , rxLen);
    //HalUARTWrite(HAL_UART_PORT_0 ,  zclSampleSw_UartBuf , rxLen);
    zclSampleSw_UartBuf[rxLen+1]=0x0A;
    
    printf("UART Start\n");//打印到debug mode上
    for(int char_count=0;char_count<rxLen;char_count++){
      if(zclSampleSw_UartBuf[char_count]==0xFF&&zclSampleSw_UartBuf[char_count+1]==0x55){
        //printf("find date head\n"); //debug使用
        switch(zclSampleSw_UartBuf[char_count+3]){
        case CMDID_VOC://0x01 直接合併使用
          temp_VOC=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x01\n");
          break;
        case CMDID_O3:
          temp_O3=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x02\n");
          break;
        case CMDID_CHO2:
          temp_CHO2=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x03\n");
          break;
        case CMDID_CO:
          temp_CO=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x04\n");
          break;
        case CMDID_CO2:
          temp_CO2=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x05\n");
          break;
        case CMDID_PM25:
          temp_PM25=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x06\n");
          break;
        case CMDID_TEMPERATURE:
          temp_TEMPERATURE=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x07\n");
          break;
        case CMDID_HUMIDITY:
          temp_HUMIDITY=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x08\n");
          break;
        case CMDID_O2:
          temp_O2=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x09\n");
          break;
        case CMDID_POWER:
          //temp_VOC=(zclSampleSw_UartBuf[char_count+DATA_H]<<8)+zclSampleSw_UartBuf[char_count+DATA_L];
          printf("0x00\n");
          break;
        default:
          printf("CmdID Error\n");
          break;
            
        }
      }
    }

  }
  
}


//下面这个函数是终端发送数据到协调器函数
#ifdef ZDO_COORDINATOR  // CSTX Coordinator!
#else
  static void zclSampleSw_ReportTest(void)
  {
      static uint8 seqNum = 0;  
      uint8 state;      //定义状态 0X31 或者 0X30
      static unsigned char flag = 0;    //状态用于切换
      zclReportCmd_t *reportCmd;
      
      afAddrType_t destAddr;    // 指定目标网络地址进行单播点播传输 16位
      destAddr.addrMode = afAddr16Bit;
      destAddr.endPoint = SAMPLESW_ENDPOINT;
      destAddr.addr.shortAddr = 0x0000;
      
      if(flag == 0x01) //来回切换发送数据
      {
          flag=0;
          state = 0x30;       //LED点亮
          HalUARTWrite(0,"Set Led On\r\n", osal_strlen("Set Led On\r\n"));     //串口显示
          HalLcdWriteString( "Set Led On", HAL_LCD_LINE_3 ); //LCD显示 
      }
      else if(flag == 0x00)
      {
          flag=1;
          state = 0x31;       //LED熄灭
          HalUARTWrite(0,"Set Led Off\r\n", osal_strlen("Set Led Off\r\n"));   //串口显示
          HalLcdWriteString( "Set Led Off", HAL_LCD_LINE_3 ); //LCD显示 
      }
        
      reportCmd = (zclReportCmd_t *)osal_mem_alloc(sizeof(zclReportCmd_t) + 
                                                   sizeof(zclReport_t));        //分配结构体
      if(reportCmd == NULL)
        return;
      
      reportCmd->attrList[0].attrData = (uint8 *)osal_mem_alloc(sizeof(uint8)); //分配数据空间
      if(reportCmd->attrList[0].attrData == NULL)       
        return;
      
      if(seqNum>127)
        seqNum = 0;
      
      reportCmd->numAttr = 10;   //10个数据
      
      reportCmd->attrList[0].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[0].attrData)) = temp_VOC;    //数据

      reportCmd->attrList[1].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[1].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[1].attrData)) = temp_O3;    //数据
      
      reportCmd->attrList[2].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[2].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[2].attrData)) = temp_CHO2;    //数据
            
      reportCmd->attrList[3].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[3].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[3].attrData)) = temp_CO;    //数据
            
      reportCmd->attrList[4].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[4].dataType = ZCL_DATATYPE_UINT16;     //数据类型是
      *((uint8 *)(reportCmd->attrList[4].attrData)) = temp_HUMIDITY;    //数据
            
      reportCmd->attrList[5].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[5].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[5].attrData)) = temp_TEMPERATURE;    //数据
            
      reportCmd->attrList[6].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[6].dataType = ZCL_DATATYPE_UINT16;     //数据类型是UINT8
      *((uint8 *)(reportCmd->attrList[6].attrData)) = temp_PM25;    //数据
            
      reportCmd->attrList[7].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[7].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[7].attrData)) = state;    //数据
       
      reportCmd->attrList[8].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[8].dataType = ZCL_DATATYPE_UINT16;     //数据类型是
      *((uint8 *)(reportCmd->attrList[8].attrData)) = temp_CO2;    //数据
            
      reportCmd->attrList[9].attrID = ATTRID_ON_OFF_SWITCH_TYPE;        //数据 id
      reportCmd->attrList[9].dataType = ZCL_DATATYPE_UINT16;     //数据类型
      *((uint8 *)(reportCmd->attrList[9].attrData)) = temp_O2;    //数据
      
      zcl_SendReportCmd( SAMPLESW_ENDPOINT,                     //发送数据
                         &destAddr,
                         ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG, 
                         reportCmd,
                         ZCL_FRAME_CLIENT_SERVER_DIR, 
                         TRUE, 
                         seqNum++ );

      
      osal_mem_free(reportCmd->attrList[0].attrData);   //释放结构体
      osal_mem_free(reportCmd); //释放数据
  }
#endif