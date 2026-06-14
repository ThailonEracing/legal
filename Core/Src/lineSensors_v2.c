/**
 * @file      lineSensors.c
 * @brief     Implements the line sensor acquisition and interpolation.
 *
 * This file contains the functions required to initialize the line
 * sensors, update their calibration limits, read normalized values,
 * and calculate the interpolated line position.
 *
 * @author    Matheus
 * @date      25-Apr-2026
 * @version   1.0
 */

#include "lineSensors_v2.h"
#include "main.h"

#define LINESENSORS_SENSOR_COUNT 5U
#define LINESENSORS_INITIAL_MAX  1500U
#define LINESENSORS_INITIAL_MIN  1000U

typedef struct
{
  lineSensorsAdcEnum_t xAdc;
  lineSensorsRankEnum_t xRank;
} lineSensorsConfig_t;

typedef struct
{
  uint16_t usBufferAdc1[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc2[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc3[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc4[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc5[LINESENSORS_SENSOR_COUNT];
  uint16_t *pBuffer[LINESENSORS_SENSOR_COUNT];
  uint16_t usMax[LINESENSORS_SENSOR_COUNT];
  uint16_t usMin[LINESENSORS_SENSOR_COUNT];
  float fWeigths[LINESENSORS_SENSOR_COUNT];
} lineSensorsAttributes_t;

static lineSensorsAttributes_t xLineSensors_v2_Attributes;

/**
 * @brief Initializes the line sensors and starts ADC DMA acquisition.
 *
 * Configures the ADC and rank used by each line sensor, starts only the
 * ADC DMA acquisitions that are required, associates each sensor with
 * the correct DMA buffer position, configures the interpolation weights,
 * and resets the calibration values.
 *
 * @param xLeftAdc ADC used by the left sensor.
 * @param xLeftRank Rank used by the left sensor.
 * @param xCenterLeftAdc ADC used by the center-left sensor.
 * @param xCenterLeftRank Rank used by the center-left sensor.
 * @param xCenterAdc ADC used by the center sensor.
 * @param xCenterRank Rank used by the center sensor.
 * @param xCenterRightAdc ADC used by the center-right sensor.
 * @param xCenterRightRank Rank used by the center-right sensor.
 * @param xRightAdc ADC used by the right sensor.
 * @param xRightRank Rank used by the right sensor.
 */
void vLineSensors_v2_Init(lineSensorsAdcEnum_t xLeftAdc,
                      lineSensorsRankEnum_t xLeftRank,
                      lineSensorsAdcEnum_t xCenterLeftAdc,
                      lineSensorsRankEnum_t xCenterLeftRank,
                      lineSensorsAdcEnum_t xCenterAdc,
                      lineSensorsRankEnum_t xCenterRank,
                      lineSensorsAdcEnum_t xCenterRightAdc,
                      lineSensorsRankEnum_t xCenterRightRank,
                      lineSensorsAdcEnum_t xRightAdc,
                      lineSensorsRankEnum_t xRightRank)
{
  extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4, hadc5;

  uint16_t *pAdcBuffer[LINESENSORS_ADC_COUNT];
  pAdcBuffer[LINESENSORS_ADC_1] = xLineSensors_v2_Attributes.usBufferAdc1;
  pAdcBuffer[LINESENSORS_ADC_2] = xLineSensors_v2_Attributes.usBufferAdc2;
  pAdcBuffer[LINESENSORS_ADC_3] = xLineSensors_v2_Attributes.usBufferAdc3;
  pAdcBuffer[LINESENSORS_ADC_4] = xLineSensors_v2_Attributes.usBufferAdc4;
  pAdcBuffer[LINESENSORS_ADC_5] = xLineSensors_v2_Attributes.usBufferAdc5;

  lineSensorsConfig_t xConfig[LINESENSORS_SENSOR_COUNT];
  unsigned char ucIndex;

  xConfig[LINESENSORS_SENSOR_LEFT].xAdc        = xLeftAdc;
  xConfig[LINESENSORS_SENSOR_LEFT].xRank       = xLeftRank;
  xConfig[LINESENSORS_SENSOR_CENTERLEFT].xAdc  = xCenterLeftAdc;
  xConfig[LINESENSORS_SENSOR_CENTERLEFT].xRank = xCenterLeftRank;
  xConfig[LINESENSORS_SENSOR_CENTER].xAdc      = xCenterAdc;
  xConfig[LINESENSORS_SENSOR_CENTER].xRank     = xCenterRank;
  xConfig[LINESENSORS_SENSOR_CENTERRIGHT].xAdc  = xCenterRightAdc;
  xConfig[LINESENSORS_SENSOR_CENTERRIGHT].xRank = xCenterRightRank;
  xConfig[LINESENSORS_SENSOR_RIGHT].xAdc       = xRightAdc;
  xConfig[LINESENSORS_SENSOR_RIGHT].xRank      = xRightRank;

  for (ucIndex = 0U; ucIndex < LINESENSORS_SENSOR_COUNT; ucIndex++)
  {
    xLineSensors_v2_Attributes.pBuffer[ucIndex] =
      &pAdcBuffer[xConfig[ucIndex].xAdc][xConfig[ucIndex].xRank];
  }

   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pAdcBuffer[LINESENSORS_ADC_1], 2U);
   HAL_ADC_Start_DMA(&hadc2, (uint32_t*)pAdcBuffer[LINESENSORS_ADC_2], 1U);
   HAL_ADC_Start_DMA(&hadc3, (uint32_t*)pAdcBuffer[LINESENSORS_ADC_3], 1U);
   HAL_ADC_Start_DMA(&hadc4, (uint32_t*)pAdcBuffer[LINESENSORS_ADC_4], 1U);
   HAL_ADC_Start_DMA(&hadc5, (uint32_t*)pAdcBuffer[LINESENSORS_ADC_5], 1U);

  vLineSensors_v2_SetInterpolationWeigths(-0.5f, -1.0f, 0.0f, 1.0f, 0.5f);
  vLineSensors_v2_ResetCalibration();
}

/**
 * @brief Resets the calibration limits of all line sensors.
 *
 * Initializes the maximum and minimum values used during the
 * calibration process.
 */
void vLineSensors_v2_ResetCalibration(void)
{
  unsigned int uiIndex;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    xLineSensors_v2_Attributes.usMax[uiIndex] = LINESENSORS_INITIAL_MAX;
    xLineSensors_v2_Attributes.usMin[uiIndex] = LINESENSORS_INITIAL_MIN;
  }
}

/**
 * @brief Updates the calibration limits using the current ADC readings.
 *
 * Compares the current sensor readings with the stored calibration
 * limits and updates the maximum and minimum values when necessary.
 */
void vLineSensors_v2_UpdateCalibration(void)
{
  unsigned int uiIndex;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    if (*xLineSensors_v2_Attributes.pBuffer[uiIndex] >
        xLineSensors_v2_Attributes.usMax[uiIndex])
    {
      xLineSensors_v2_Attributes.usMax[uiIndex] =
        *xLineSensors_v2_Attributes.pBuffer[uiIndex];
    }

    if (*xLineSensors_v2_Attributes.pBuffer[uiIndex] <
        xLineSensors_v2_Attributes.usMin[uiIndex])
    {
      xLineSensors_v2_Attributes.usMin[uiIndex] =
        *xLineSensors_v2_Attributes.pBuffer[uiIndex];
    }
  }
}

/**
 * @brief Returns the normalized value of a selected line sensor.
 *
 * @param xSensor Selected sensor identifier.
 * @return Normalized sensor value in the range from 0.0 to 1.0.
 */
float fLineSensors_v2_GetSensorValue(lineSensorsEnum_t xSensor)
{
  float fRange;
  float fValue;

  fRange = (float)(xLineSensors_v2_Attributes.usMax[xSensor] -
    xLineSensors_v2_Attributes.usMin[xSensor]);
  fValue = ((float)(*xLineSensors_v2_Attributes.pBuffer[xSensor]) -
    (float)xLineSensors_v2_Attributes.usMin[xSensor]) / fRange;

  if (1.0f < fValue)
  {
    fValue = 1.0f;
  }

  if (0.0f > fValue)
  {
    fValue = 0.0f;
  }

  return fValue;
}

/**
 * @brief Sets the interpolation weights used by the line sensors.
 *
 * @param fLeftWeigth Weight assigned to the left sensor.
 * @param fCenterLeftWeigth Weight assigned to the center-left sensor.
 * @param fCenterWeigth Weight assigned to the center sensor.
 * @param fCenterRightWeigth Weight assigned to the center-right sensor.
 * @param fRightWeigth Weight assigned to the right sensor.
 */
void vLineSensors_v2_SetInterpolationWeigths(float fLeftWeigth,
  float fCenterLeftWeigth,
  float fCenterWeigth,
  float fCenterRightWeigth,
  float fRightWeigth)
{
  xLineSensors_v2_Attributes.fWeigths[0] = fLeftWeigth;
  xLineSensors_v2_Attributes.fWeigths[1] = fCenterLeftWeigth;
  xLineSensors_v2_Attributes.fWeigths[2] = fCenterWeigth;
  xLineSensors_v2_Attributes.fWeigths[3] = fCenterRightWeigth;
  xLineSensors_v2_Attributes.fWeigths[4] = fRightWeigth;
}

/**
 * @brief Returns the interpolated value calculated from all sensors.
 *
 * @return Interpolated line position based on the configured weights.
 */
float fLineSensors_v2_GetInterpolatedValue(void)
{
  float fNegMax;
  float fPosMax;
  float fValue;
  unsigned int uiIndex;

  fNegMax = 0.0f;
  fPosMax = 0.0f;
  fValue = 0.0f;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    if (0.0f < xLineSensors_v2_Attributes.fWeigths[uiIndex])
    {
      fPosMax += xLineSensors_v2_Attributes.fWeigths[uiIndex];
    }
    else
    {
      fNegMax -= xLineSensors_v2_Attributes.fWeigths[uiIndex];
    }

    fValue += fLineSensors_v2_GetSensorValue((lineSensorsEnum_t)uiIndex) *
      xLineSensors_v2_Attributes.fWeigths[uiIndex];
  }

  return (0.0f < fValue) ? (fValue / fPosMax) : (fValue / fNegMax);
}
