/**
  ******************************************************************************
  * @file    Charger Function_control.c
  * @author  Dynapack ADT, Hsinmo
  * @version V1.0.0
  * @date    3-April-2013
  * @brief   Charger Function control setting
  ******************************************************************************
  * @attention
  *
  * DESCRIPTION....
  *
  * <h2><center>&copy; COPYRIGHT 2013 Dynapack</center></h2>
  ******************************************************************************
  */

//==============================================================================
// Includes
//==============================================================================
#include "inc/hw_memmap.h"

#include "gpio.h"
//#include "adc10_a.h"
#include "ref.h"
//#include "msp_dma.h"
//#include "ucs.h"
#include "MCU_Devices.h"
//==============================================================================
// Global/Extern variables
//==============================================================================
//==============================================================================
// Extern functions
//==============================================================================
//==============================================================================
// Private typedef
//==============================================================================
//==============================================================================
// Private define
//==============================================================================
#define CHARGER_MUX_PORT                GPIO_PORT_P1
#define CHARGER_MUX_A0_PIN              GPIO_PIN3
#define CHARGER_MUX_A1_PIN              GPIO_PIN4

#define CHARGER_ID_LEVEL_PORT           GPIO_PORT_P1
#define CHARGER_ID_LEVEL1_PIN           GPIO_PIN0
#define CHARGER_ID_LEVEL2_PIN           GPIO_PIN1
#define CHARGER_ID_LEVEL3_PIN           GPIO_PIN2

#define CHARGER_VOL_IN_CTR_PORT         GPIO_PORT_P1
#define CHARGER_24VOL_IN_CTR_PIN        GPIO_PIN5
#define CHARGER_36VOL_IN_CTR_PIN        GPIO_PIN6
#define CHARGER_48VOL_IN_CTR_PIN        GPIO_PIN7

//==============================================================================
// Private macro
//==============================================================================
//==============================================================================
// Private Enum
//==============================================================================
//enum CHG_ID_Level{
//    CHG_ID_Level1,
//    CHG_ID_Level2,
//    CHG_ID_Level3
//};

//==============================================================================
// Private variables
//==============================================================================

//==============================================================================
// Private function prototypes
//==============================================================================

//==============================================================================
// Private functions
//==============================================================================

void _Device_Charger_Func_Init(void){

    //set Mux as output pins, and low level
    GPIO_setAsOutputPin(CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN + CHARGER_MUX_A1_PIN);
    GPIO_setOutputLowOnPin(CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN + CHARGER_MUX_A1_PIN);
    //set charger id pin as output pins, and low level
    GPIO_setAsOutputPin(CHARGER_ID_LEVEL_PORT, CHARGER_ID_LEVEL1_PIN + CHARGER_ID_LEVEL2_PIN + CHARGER_ID_LEVEL3_PIN);
    GPIO_setOutputLowOnPin(CHARGER_ID_LEVEL_PORT, CHARGER_ID_LEVEL1_PIN + CHARGER_ID_LEVEL2_PIN + CHARGER_ID_LEVEL3_PIN);
    //set charger voltage ctrl pin as output pins, and low level
    GPIO_setAsOutputPin(CHARGER_VOL_IN_CTR_PORT, CHARGER_24VOL_IN_CTR_PIN + CHARGER_36VOL_IN_CTR_PIN + CHARGER_48VOL_IN_CTR_PIN);
    GPIO_setOutputLowOnPin(CHARGER_VOL_IN_CTR_PORT, CHARGER_24VOL_IN_CTR_PIN + CHARGER_36VOL_IN_CTR_PIN + CHARGER_48VOL_IN_CTR_PIN);
}

void _Device_Set_Chg_ID_Level(CHG_ID_Levels level){

    switch(level){

        case CHG_ID_Level_All_Hi:
            GPIO_setOutputHighOnPin(CHARGER_ID_LEVEL_PORT,CHARGER_ID_LEVEL1_PIN + CHARGER_ID_LEVEL2_PIN + CHARGER_ID_LEVEL3_PIN);
            break;
        case CHG_ID_Level1_Hi:
            GPIO_setOutputHighOnPin(CHARGER_ID_LEVEL_PORT,CHARGER_ID_LEVEL1_PIN);
            break;
        case CHG_ID_Level2_Hi:
            GPIO_setOutputHighOnPin(CHARGER_ID_LEVEL_PORT,CHARGER_ID_LEVEL2_PIN);
            break;
        case CHG_ID_Level3_Hi:
            GPIO_setOutputHighOnPin(CHARGER_ID_LEVEL_PORT,CHARGER_ID_LEVEL3_PIN);
            break;

        case CHG_ID_Level_All_Lo:
        default:
            GPIO_setOutputLowOnPin(CHARGER_ID_LEVEL_PORT,CHARGER_ID_LEVEL1_PIN + CHARGER_ID_LEVEL2_PIN + CHARGER_ID_LEVEL3_PIN);
            break;
    }
}


void _Device_Set_Chger_Mux_Channel(Chger_Mux_Channels channel){

    switch(channel){
        case Chger_Mux_Ch1:
            GPIO_setOutputLowOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN );
            GPIO_setOutputLowOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A1_PIN );
            break;
    case Chger_Mux_Ch2:
            GPIO_setOutputHighOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN );
            GPIO_setOutputLowOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A1_PIN );
            break;
        case Chger_Mux_Ch3:
            GPIO_setOutputLowOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN );
            GPIO_setOutputHighOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A1_PIN );
            break;
        case Chger_Mux_Ch4:
        default:
            GPIO_setOutputHighOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A0_PIN );
            GPIO_setOutputHighOnPin( CHARGER_MUX_PORT, CHARGER_MUX_A1_PIN );
            break;

    }
}

void _Device_Set_Chger_Input_Voltage_Channel(Chger_InputVol_Ch channel){

    switch(channel){

        case Chger_InputVol_Ch_All_Hi:
            GPIO_setOutputHighOnPin(CHARGER_VOL_IN_CTR_PORT, CHARGER_24VOL_IN_CTR_PIN + CHARGER_36VOL_IN_CTR_PIN + CHARGER_48VOL_IN_CTR_PIN);
            break;
        case Chger_24Vol_Ch_Hi:
            GPIO_setOutputHighOnPin(CHARGER_VOL_IN_CTR_PORT,CHARGER_24VOL_IN_CTR_PIN);
            break;
        case Chger_36Vol_Ch_Hi:
            GPIO_setOutputHighOnPin(CHARGER_VOL_IN_CTR_PORT,CHARGER_36VOL_IN_CTR_PIN);
            break;
        case Chger_48Vol_Ch_Hi:
            GPIO_setOutputHighOnPin(CHARGER_VOL_IN_CTR_PORT,CHARGER_48VOL_IN_CTR_PIN);
            break;

        case Chger_InputVol_Ch_All_Lo:
        default:
            GPIO_setOutputLowOnPin(CHARGER_VOL_IN_CTR_PORT, CHARGER_24VOL_IN_CTR_PIN + CHARGER_36VOL_IN_CTR_PIN + CHARGER_48VOL_IN_CTR_PIN);
            break;
    }
}

