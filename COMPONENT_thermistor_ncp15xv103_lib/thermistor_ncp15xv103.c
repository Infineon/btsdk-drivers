/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_thermistor.h"
#include "wiced_platform.h"
#include "wiced_hal_adc.h"

// choose which ref resistance you use
#define THERM_10K

typedef struct
{
    uint16_t                high_pin;           /*  A/D input high pin    */
    uint16_t                low_pin;            /*  A/D input low pin     */
    uint16_t                adc_power_pin;      /*  ADC power pin         */
} thermistor_cfg_t;

thermistor_cfg_t thermistor_cfg =
{
    .high_pin       = ADC_INPUT_P14,
    .low_pin        = ADC_INPUT_P11,
    .adc_power_pin  = WICED_P09,
};

/**********************************************************************************************************
Variables Definitions
**********************************************************************************************************/
#ifdef THERM_100K
#define RES_THERM 100000
uint32_t therm_resistance_table[126] =
{
    4397138, //therm_resistance_table[0] is -40C resistance ohm
    4093139, //therm_resistance_table[1] is -39C resistance ohm
    3811807,
    3551654,
    3311205,
    3088406,
    2883725,
    2691878,
    2514897,
    2349889,
    2196451,
    2055454,
    1924540,
    1802017,
    1687649,
    1581957,
    1482734,
    1391188,
    1305451,
    1225301,
    1150948,
    1081525,
    1016751,
    956000,
    899394,
    846644,
    797227,
    750735,
    707438,
    666907,
    628960,
    593277,
    559868,
    528571,
    499237,
    471627,
    445815,
    421492,
    398640,
    377223,
    357000,
    338014,
    320115,
    303275,
    287415,
    272503,
    258423,
    245152,
    232661,
    220700,
    209568,
    199184,
    189220,
    179898,
    170936,
    162739,
    154630,
    147191,
    140175,
    133380,
    127116,
    121031,
    115405,
    109924,
    104842,
    100000,
    95382,
    91083,
    86863,
    82927,
    79250,
    75719,
    72324,
    69144,
    66080,
    63205,
    60428,
    57819,
    55294,
    52919,
    50685,
    48515,
    46471,
    44547,
    42672,
    40905,
    39241,
    37615,
    36082,
    34584,
    33172,
    31842,
    30590,
    29361,
    28205,
    27070,
    26050,
    25000,
    24060,
    23134,
    22222,
    21368,
    20570,
    19782,
    19048,
    18322,
    17647,
    16980,
    16361,
    15749,
    15183,
    14623,
    14108,
    13597,
    13091,
    12628,
    12169,
    11751,
    11336,
    10962,
    10553,
    10184,
    9854,
    9525,
    9199,
    8875,//therm_resistance_table[125] is 85C resistance ohm
};
#endif

#ifdef THERM_10K
#define RES_THERM 10000
uint32_t therm_resistance_table[126]=
{
    328983,//therm_resistance_table[0] is -40C resistance ohm
    307919,
    288373,
    270136,
    253158,
    237376,
    222722,
    208978,
    196250,
    184232,
    173232,
    162865,
    153124,
    144134,
    135695,
    127787,
    120383,
    113503,
    107063,
    100999,
    95330,
    90000,
    84991,
    80287,
    75893,
    71744,
    67867,
    64241,
    60815,
    57595,
    54567,
    51694,
    49002,
    46459,
    44072,
    41813,
    39691,
    37688,
    35795,
    34012,
    32329,
    30736,
    29234,
    27809,
    26464,
    25192,
    23989,
    22836,
    21761,
    20755,
    19783,
    18871,
    17990,
    17183,
    16400,
    15641,
    14943,
    14283,
    13639,
    13045,
    12464,
    11927,
    11401,
    10913,
    10446,
    10000,
    9573,
    9175,
    8793,
    8425,
    8072,
    7742,
    7423,
    7116,
    6828,
    6558,
    6296,
    6043,
    5805,
    5573,
    5356,
    5145,
    4946,
    4758,
    4576,
    4398,
    4236,
    4072,
    3924,
    3773,
    3636,
    3502,
    3371,
    3248,
    3132,
    3018,
    2911,
    2806,
    2707,
    2610,
    2519,
    2434,
    2350,
    2268,
    2191,
    2115,
    2044,
    1974,
    1905,
    1841,
    1782,
    1723,
    1665,
    1612,
    1559,
    1510,
    1462,
    1415,
    1368,
    1325,
    1282,
    1244,
    1205,
    1168,
    1130,
    1096,//therm_resistance_table[125] is 85C resistance ohm
};
#endif
/**********************************************************************************************************
Function Declarations
**********************************************************************************************************/

/******************************************************************************
* Function Name: thermistor_init
***************************************************************************//**
* init thermistor.
*
* \param None.
*
* \return None
******************************************************************************/
void thermistor_init(void)
{
    wiced_hal_adc_init();
}

/******************************************************************************
* Function Name: thermistor_read
***************************************************************************//**
* The function reads the thermistor temperature.
*
* Return temperature in degrees Celsius * 100
*
******************************************************************************/
int16_t thermistor_read(void)
{
    uint32_t voltage_val_low = 0, voltage_val_high = 0, therm_cal_value = 0;
    int16_t  temp = 8500;
    int i;

    // open ADC
    wiced_hal_gpio_configure_pin(thermistor_cfg.adc_power_pin,  GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    voltage_val_high = wiced_hal_adc_read_voltage(thermistor_cfg.high_pin);
    voltage_val_low  = wiced_hal_adc_read_voltage(thermistor_cfg.low_pin);
    therm_cal_value  = RES_THERM * (voltage_val_high - voltage_val_low) / voltage_val_low;

    WICED_BT_TRACE("thermistor_read high:%d low:%d cal:%d\n", voltage_val_high, voltage_val_low, therm_cal_value);

    //compare test resistor with the table
    if (therm_cal_value >= therm_resistance_table[0])
    {
        //WICED_BT_TRACE("temperature is equal or lower -40.0 C\r\n");
        temp = -4000;
    }
    else
    {
        for (i = 0; i < 125; i++)
        {
            if ((therm_cal_value < therm_resistance_table[i]) && (therm_cal_value >= therm_resistance_table[i + 1]))
            {
                temp = -4000 + 100 * i + (100 * (therm_resistance_table[i] - therm_cal_value) / (therm_resistance_table[i] - therm_resistance_table[i + 1]));
                break;
            }
        }
    }
#if 0
            for (i = 1; i < 126; i++)
        {
            // negative value
            if ((i <= 40) && (therm_cal_value < therm_resistance_table[i - 1]) && (therm_cal_value >= therm_resistance_table[i]))
            {
                *integer_part = 40 - i;
                *decimal_part = 100 * (therm_cal_value - therm_resistance_table[i]) / (therm_resistance_table[i - 1] - therm_resistance_table[i]);

                // 0.0C should be a  positive value
                *is_negative = ((*integer_part != 0) || (*decimal_part != 0));
                break;
            }
            else if ((therm_cal_value <= therm_resistance_table[i - 1]) && (therm_cal_value > therm_resistance_table[i]))
            {
                // positive value
                *integer_part = i - 41;
                *decimal_part = 100 * (therm_resistance_table[i - 1] - therm_cal_value) / (therm_resistance_table[i - 1] - therm_resistance_table[i]);
                break;
            }
        }
    }
#endif
    // power off thermistor
    wiced_hal_gpio_configure_pin(thermistor_cfg.adc_power_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
    return temp;
}
