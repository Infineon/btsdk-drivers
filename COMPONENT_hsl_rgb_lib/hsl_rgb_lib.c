/*
* Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
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

#include "hsl_rgb_lib.h"

/******************************************************************************
* Function Name: hsl_hue_2_rgb
***************************************************************************//**
* Convert HSL Color Hue relative value to RGB Color vector value.
*
* \param p      Value 1 converted from HSL lightness and saturation values, value range: 0 ~ 65535.
* \param q      Value 2 converted from HSL lightness and saturation values, value range: 0 ~ 65535.
* \param tc     Relative HSL Hue value for each RCG Color vector, value range: 0 ~ 65535.
*
* \return None
******************************************************************************/
int hsl_hue_2_rgb(int p, int q, int tc)
{
    tc = (tc < HSL_COLOR_MIN_HUE) ? (tc + HSL_COLOR_MAX_HUE) : tc;
    tc = (tc > HSL_COLOR_MAX_HUE) ? (tc - HSL_COLOR_MAX_HUE) : tc;
    if ((6 * tc) < HSL_COLOR_MAX_HUE)
    {
        return (p + (q - p) * 6 * tc / HSL_COLOR_MAX_HUE);
    }
    else if ((2 * tc) < HSL_COLOR_MAX_HUE)
    {
        return q;
    }
    else if ((3 * tc) < (2 * HSL_COLOR_MAX_HUE))
    {
        return (p + ((q - p) * 4) - ((q - p) * 6 * tc / HSL_COLOR_MAX_HUE));
    }
    return p;
}

/******************************************************************************
* Function Name: hsl_2_rgb
***************************************************************************//**
* Convert HSL Color values to RGB Color values.
*
* \param hue            Input, HSL color hue value, value range: 0 ~ 65535, unit: degree.
* \param saturation     Input, HSL color saturation value, value range: 0 ~ 65535, unit: percentage.
* \param lightness      Input, HSL color lightness, value range: 0 ~ 65535, unit: percentage.
* \param red            Output, RBG color red value, value range: 0 ~255.
* \param green          Output, RBG color green value, value range: 0 ~255.
* \param blue           Output, RBG color blue value, value range: 0 ~255.
*
* \return None
******************************************************************************/
int hsl_2_rgb(int hue, int saturation, int lightness, int *red, int *green, int *blue)
{
    int r, g, b;
    int q, p, tr, tg, tb;

    if (hue > HSL_COLOR_MAX_HUE || saturation > HSL_COLOR_MAX_SATURATION || lightness > HSL_COLOR_MAX_LIGHTNESS ||
        red == NULL_VALUE || green == NULL_VALUE || blue == NULL_VALUE) {
        return -1;
    }

    if (saturation == 0)
    {
        r = lightness * RGB_COLOR_MAX_RED / HSL_COLOR_MAX_LIGHTNESS;
        g = lightness * RGB_COLOR_MAX_GREEN / HSL_COLOR_MAX_LIGHTNESS;
        b = lightness * RGB_COLOR_MAX_BLUE / HSL_COLOR_MAX_LIGHTNESS;
    }
    else
    {
        if (lightness < (HSL_COLOR_MAX_LIGHTNESS / 2))
        {
            q = lightness + (lightness * saturation) / HSL_COLOR_MAX_SATURATION;
        }
        else
        {
            q = (lightness + saturation) - (lightness * saturation) / HSL_COLOR_MAX_SATURATION;
        }
        p = 2 * lightness - q;
        tr = hue + HSL_COLOR_MAX_HUE / 3;
        tg = hue;
        tb = hue - HSL_COLOR_MAX_HUE / 3;
        r = hsl_hue_2_rgb(p, q, tr) * RGB_COLOR_MAX_RED / HSL_COLOR_MAX_SATURATION;
        g = hsl_hue_2_rgb(p, q, tg) * RGB_COLOR_MAX_GREEN / HSL_COLOR_MAX_SATURATION;
        b = hsl_hue_2_rgb(p, q, tb) * RGB_COLOR_MAX_BLUE  / HSL_COLOR_MAX_SATURATION;
    }

    *red = r;
    *green = g;
    *blue = b;
    return 0;
}
