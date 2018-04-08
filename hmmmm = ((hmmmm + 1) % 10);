

		hmmmm = ((hmmmm + 1) % 10);
		for (i = 0; i < (TXRXBUFFERSIZE - 2); ++i) {
			buffer[i] = 'a' + hmmmm;
		}
		buffer[i++] = '\n';
		buffer[i++] = '\r';
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(trans_delay);

		for (i = 0; i < (TXRXBUFFERSIZE / 4); i++) {
			buffer[i] = RX_buffer[i];
		}
		for (i = 0; i < (TXRXBUFFERSIZE / 4); i++) {
			buffer[i + (TXRXBUFFERSIZE / 4)] = ADC_buffer[i];
		}

		if (HAL_ADC_PollForConversion(&hadc2, 50) == HAL_OK) {
			a = HAL_ADC_GetValue(&hadc2);
			memset(ADC_buffer, 0, TXRXBUFFERSIZE);
			sprintf(ADC_buffer, "Got: |%u|%d\n\r", a, hmmmm++);
			hmmmm++;
			HAL_Delay(trans_delay);
		}

		HAL_Delay(trans_delay);

		HAL_UART_Receive_DMA(&huart1, RX_buffer, len);
		HAL_UART_Receive_DMA(&huart2, RX_buffer, len);
		HAL_Delay(trans_delay);

		//HAL_UART_Transmit_DMA(&huart1, buffer, len);
		//HAL_UART_Transmit_DMA(&huart2, buffer, len);

		HAL_UART_Transmit_DMA(&huart1, ADC_buffer, len);
		HAL_UART_Transmit_DMA(&huart2, ADC_buffer, len);
		HAL_Delay(trans_delay);