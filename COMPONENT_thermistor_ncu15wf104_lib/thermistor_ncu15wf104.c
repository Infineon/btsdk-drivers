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

/** @file
 *  thermistor_app.c
 *
 *  @brief
 *
 *  Thermistor library for Murata's ncu15wf104
 *
 */

/*******************************************************************************
 *                               Includes
 *******************************************************************************/
#include "wiced.h"
#include "wiced_hal_adc.h"
#include "wiced_platform.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define BALANCE_RESISTANCE              100000              /* Value of Reference Resistance connected in series with the thermistor */
#define POLL_TIMER_IN_MS                5000               /* Milliseconds timer*/

#ifndef THERMISTOR_aux_0_TRIGGER_OUT

#ifdef CYW20719B1
#define THERMISTOR_PIN  ADC_INPUT_P10                       /*CYW920719Q40EVB-01 has P10 connected to Thermistor*/
#endif

#ifdef CYW20735B1
#define THERMISTOR_PIN  ADC_INPUT_P8                        /*CYW920735Q60EVB-01 has P8 connected to Thermistor*/
#endif

#ifdef CYW20721B1
#define THERMISTOR_PIN  ADC_INPUT_P10                        /*CYW920721EVK-01 has P10 connected to Thermistor*/
#endif

#ifdef CYW20819A1
#define THERMISTOR_PIN  ADC_INPUT_P8                        /*CYW920819EVB-02 has P8 connected to Thermistor*/
#endif

#ifdef CYW20820A1
#define THERMISTOR_PIN  ADC_INPUT_P8                        /*CYW920819EVB-02 has P8 connected to Thermistor*/
#endif

#ifdef CYW20719B2
#define THERMISTOR_PIN  ADC_INPUT_P10                        /*CYW920719B2Q40EVB-01 has P10 connected to Thermistor*/
#endif

#else

#define THERMISTOR_PIN THERMISTOR_aux_0_TRIGGER_OUT

#endif

/******************************************************************************
 *                                Variable/Structure/type Definitions
 ******************************************************************************/
/*
 * This table maps the R-center value to its equivalent temperature value.
 * The temperature is in Degrees Celsius * 100.  Resistance in 0.1 ohms.
 */
typedef struct
{
    int16_t  temp_celsius;
    uint32_t resistance_ohms;
} r_t_look_up_table_t;

const r_t_look_up_table_t r_t_centre[] =
{
    { -4000, 43971193 },
    { -3900, 40928737 },
    { -3800, 38117170 },
    { -3700, 35517485 },
    { -3600, 33112358 },
    { -3500, 30885990 },
    { -3400, 28823960 },
    { -3300, 26913100 },
    { -3200, 25141370 },
    { -3100, 23497780 },
    { -3000, 21972250 },
    { -2900, 20555580 },
    { -2800, 19239320 },
    { -2700, 18015730 },
    { -2600, 16877730 },
    { -2500, 15818810 },
    { -2400, 14831000 },
    { -2300, 13911130 },
    { -2200, 13054130 },
    { -2100, 12255310 },
    { -2000, 11510370 },
    { -1900, 10815350 },
    { -1800, 10166610 },
    { -1700, 9560796 },
    { -1600, 8994806 },
    { -1500, 8465788 },
    { -1400, 7971110 },
    { -1300, 7508341 },
    { -1200, 7075237 },
    { -1100, 6669723 },
    { -1000, 6289882 },
    {  -900, 5933421 },
    {  -800, 5599309 },
    {  -700, 5286016 },
    {  -600, 4992124 },
    {  -500, 4716321 },
    {  -400, 4457716 },
    {  -300, 4214796 },
    {  -200, 3986521 },
    {  -100, 3771927 },
    {   000, 3570117 },
    {   100, 3380058 },
    {   200, 3201216 },
    {   300, 3032866 },
    {   400, 2874335 },
    {   500, 2724995 },
    {   600, 2584264 },
    {   700, 2451598 },
    {   800, 2326491 },
    {   900, 2208471 },
    {  1000, 2097098 },
    {  1100, 1991962 },
    {  1200, 1892681 },
    {  1300, 1798896 },
    {  1400, 1710275 },
    {  1500, 1626506 },
    {  1600, 1547264 },
    {  1700, 1472321 },
    {  1800, 1401420 },
    {  1900, 1334322 },
    {  2000, 1270802 },
    {  2100, 1210658 },
    {  2200, 1153684 },
    {  2300, 1099695 },
    {  2400, 1048521 },
    {  2500, 1000000 },
    {  2600, 953981 },
    {  2700, 910322 },
    {  2800, 868890 },
    {  2900, 829561 },
    {  3000, 792216 },
    {  3100, 756752 },
    {  3200, 723060 },
    {  3300, 691042 },
    {  3400, 660608 },
    {  3500, 631671 },
    {  3600, 604150 },
    {  3700, 577969 },
    {  3800, 553056 },
    {  3900, 529343 },
    {  4000, 506766 },
    {  4100, 485283 },
    {  4200, 464820 },
    {  4300, 445325 },
    {  4400, 426745 },
    {  4500, 409035 },
    {  4600, 392132 },
    {  4700, 376010 },
    {  4800, 360629 },
    {  4900, 345953 },
    {  5000, 331946 },
    {  5100, 318591 },
    {  5200, 305839 },
    {  5300, 293660 },
    {  5400, 282026 },
    {  5500, 270909 },
    {  5600, 260284 },
    {  5700, 250127 },
    {  5800, 240416 },
    {  5900, 231128 },
    {  6000, 222243 },
    {  6100, 213743 },
    {  6200, 205607 },
    {  6300, 197820 },
    {  6400, 190364 },
    {  6500, 183225 },
    {  6600, 176401 },
    {  6700, 169864 },
    {  6800, 163600 },
    {  6900, 157596 },
    {  7000, 151841 },
    {  7100, 146310 },
    {  7200, 141006 },
    {  7300, 135918 },
    {  7400, 131037 },
    {  7500, 126354 },
    {  7600, 121871 },
    {  7700, 117567 },
    {  7800, 113436 },
    {  7900, 109468 },
    {  8000, 105657 },
    {  8100, 101996 },
    {  8200, 98479 },
    {  8300, 95098 },
    {  8400, 91849 },
    {  8500, 88726 },
    {  8600, 85722 },
    {  8700, 82834 },
    {  8800, 80055 },
    {  8900, 77383 },
    {  9000, 74811 },
    {  9100, 72344 },
    {  9200, 69971 },
    {  9300, 67685 },
    {  9400, 65484 },
    {  9500, 63365 },
    {  9600, 61316 },
    {  9700, 59341 },
    {  9800, 57439 },
    {  9900, 55606 },
    { 10000, 53839 },
    { 10100, 52143 },
    { 10200, 50507 },
    { 10300, 48930 },
    { 10400, 47409 },
    { 10500, 45942 },
    { 10600, 44527 },
    { 10700, 43161 },
    { 10800, 41843 },
    { 10900, 40570 },
    { 11000, 39342 },
    { 11100, 38156 },
    { 11200, 37011 },
    { 11300, 35905 },
    { 11400, 34836 },
    { 11500, 33804 },
    { 11600, 32812 },
    { 11700, 31853 },
    { 11800, 30926 },
    { 11900, 30031 },
    { 12000, 29164 },
    { 12100, 28322 },
    { 12200, 27508 },
    { 12300, 26720 },
    { 12400, 25958 },
    { 12500, 25220 }
};

#define TABLE_SIZE sizeof(r_t_centre)/sizeof(r_t_centre[0])


/*******************************************************************
 *                              Function Declarations/Prototypes
 ******************************************************************/
static int32_t convert_resistance_to_temp(uint32_t therm_resist);
static int16_t get_temp_in_celsius(uint32_t vref, uint32_t vadc);


/*******************************************************************
 *                              Function Definitions
 ******************************************************************/
/*
 Function Description:
 @brief    This function initializes thermistor device

 @param    void

 @return    void
 */
void thermistor_init(void)
{
    /*ADC Initialization*/
    wiced_hal_adc_init();
}

/*
 Function name:
 thermistor_get_temperature

 Function Description:
 @brief  This function reads the current temperature value from the thermistor.

 @return temperature in celsius multiplied by 100
 */
int16_t thermistor_read(void)
{
    volatile uint16_t voltage_val_adc_in_mv = 0;
    volatile uint16_t vddio_mv = 0;
    volatile int16_t temp_in_celsius = 0;

    /*
     * Measure the voltage(in milli volts) on the channel being passed as an argument
     * To measure the voltage across the thermistor input channel - ADC_INPUT_P10
     * To measure vref - ADC_INPUT_VDDIO
     */
    /*The wiced_hal_adc automatically powers up ADC block before reading ADC voltage registers and powers down after reading which reduces power consumption. By multiple single shot sampling, it also improves the accuracy of the reading*/
    vddio_mv = wiced_hal_adc_read_voltage(ADC_INPUT_VDDIO); /* Input channel to measure Reference voltage for Voltage divider calculation for Thermistor */
    if (vddio_mv < 1850)
    {                                    /*1.85V is used instead of 1.8V because of +/-3% inaccuracy */
        wiced_hal_adc_set_input_range(ADC_RANGE_0_1P8V);
        voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(THERMISTOR_PIN); /* Input channel to measure DC voltage(temperature)-> GPIO 10 -> J12.1, J14.1 */
    }
    else
    {
        wiced_hal_adc_set_input_range(ADC_RANGE_0_3P6V);
        voltage_val_adc_in_mv = wiced_hal_adc_read_voltage(THERMISTOR_PIN); /* Input channel to measure DC voltage(temperature)-> GPIO 10 -> J12.1, J14.1 */
    }
    temp_in_celsius = get_temp_in_celsius(vddio_mv, voltage_val_adc_in_mv);
    return temp_in_celsius;
}

/*
 Function name:
 get_temp_in_celsius

 Function Description:
 @brief     This function takes in ADC output from VDDIO and voltage divider to calculate the temperature in celsius. The function returns temperature value with 2 decimal points of accuracy as expected by Environmental Sensing Profile.

 @param  vref    voltage in millivolts measured from the VDDIO
 @param  vadc    voltage in millivolts measured from the DC Channel

 @return    temperature in celsius multiplied by 100. This to provide 2 fractional positions of resolution.
 */
static int16_t get_temp_in_celsius(uint32_t vref, uint32_t vadc)
{
    volatile int16_t temp_in_celsius = 0;
    volatile int32_t r_thermistor = 0;
    r_thermistor = ((vref - vadc) * BALANCE_RESISTANCE) / vadc;
    temp_in_celsius = convert_resistance_to_temp(r_thermistor);
    return ((int16_t)temp_in_celsius);
}

/*
 Function name:
 r_t_look_up

 Function Description:
 @brief   Calculate temperature using R Vs T look-up table.
 @details Function finds entries in the table where R1 < R < R2.
 And then calculates temperature as T = T1 + (T2 - T1) / (R2 - R1) * (R - R1)

 @param table            Pointer to R Vs T look-up table
 @param therm_resist     Resistance of thermistor in ohms

 @return Temperature in celsius * 100
*/
int32_t convert_resistance_to_temp(uint32_t therm_resist)
{
    int i;
    int32_t r = therm_resist * 10;  // the table uses ohms * 10
    int32_t r1, r2;
    int32_t t1, t2, t;

    if (r > (int32_t)r_t_centre[0].resistance_ohms)
        return r_t_centre[0].temp_celsius;

    for (i = 1; i < TABLE_SIZE - 1; i++)
    {
        if (r > (int32_t)r_t_centre[i].resistance_ohms)  //find the two points in the table to use
        {
            t1 = r_t_centre[i - 1].temp_celsius;
            t2 = r_t_centre[i].temp_celsius;
            r1 = r_t_centre[i - 1].resistance_ohms;
            r2 = r_t_centre[i].resistance_ohms;
            t = t1 + ((((t2 - t1) * (r - r1)) + ((r2 - r1) / 2)) / (r2 - r1));
            return t;
        }
    }
    return r_t_centre[TABLE_SIZE - 1].temp_celsius;
}
