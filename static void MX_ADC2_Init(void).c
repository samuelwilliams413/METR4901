/* ADC2 init function */
static void MX_ADC2_Init(void) {

  /**Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
   */
  sConfig_A.Channel = ADC_CHANNEL_1;
  sConfig_A.Rank = ADC_REGULAR_RANK_1;
  sConfig_A.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_A.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_A.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_A.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_A) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_B.Channel = ADC_CHANNEL_2;
  sConfig_B.Rank = ADC_REGULAR_RANK_1;
  sConfig_B.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_B.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_B.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_B.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_B) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_C.Channel = ADC_CHANNEL_1;
  sConfig_C.Rank = ADC_REGULAR_RANK_1;
  sConfig_C.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_C.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_C.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_C.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_C) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_D.Channel = ADC_CHANNEL_2;
  sConfig_D.Rank = ADC_REGULAR_RANK_1;
  sConfig_D.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_D.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_D.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_D.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_D) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_E.Channel = ADC_CHANNEL_3;
  sConfig_E.Rank = ADC_REGULAR_RANK_1;
  sConfig_E.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_E.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_E.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_E.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_E) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_F.Channel = ADC_CHANNEL_4;
  sConfig_F.Rank = ADC_REGULAR_RANK_1;
  sConfig_F.SingleDiff = ADC_SINGLE_ENDED;
  sConfig_F.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig_F.OffsetNumber = ADC_OFFSET_NONE;
  sConfig_F.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig_F) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}