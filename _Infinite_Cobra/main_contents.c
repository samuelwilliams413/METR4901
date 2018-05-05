if (LED_ENABLE) {
			if (HAL_GetTick() > (epoch_LED + delay_LED)) {
				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				epoch_LED = HAL_GetTick();
			}
		}

		if (TX_ENABLE) {
			hmmmm = (hmmmm + 1) % 3;
			memset(buffer, ('3' + hmmmm), B_SIZE);
			buffer[B_SIZE - 2] = '\n';
			buffer[B_SIZE - 1] = '\r';
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, buffer, len);
			HAL_UART_Transmit_DMA(&huart2, buffer, len);
		}

		if (RX_ENABLE_PASS) {
			///////////////////////////// RX
			__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);

			memset(RX_buffer1, 0, len);
			HAL_UART_Receive_DMA(&huart1, RX_buffer1, len);
			RX_buffer1[B_SIZE - 2] = '\n';
			RX_buffer1[B_SIZE - 1] = '\r';
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, RX_buffer1, len);
			HAL_UART_Transmit_DMA(&huart2, RX_buffer1, len);
		}

		if (RX_ENABLE_PASS) {
			///////////////////////////// RX
			__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);

			memset(RX_buffer2, 0, len);
			HAL_UART_Receive_DMA(&huart2, RX_buffer2, len);
			RX_buffer2[B_SIZE - 2] = '\n';
			RX_buffer2[B_SIZE - 1] = '\r';
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, RX_buffer2, len);
			HAL_UART_Transmit_DMA(&huart2, RX_buffer2, len);

		}

		if (ADC_ENABLE) {
			Update_ADC_Values();
			memset(ADC_buffer, 0, B_SIZE);
			sprintf(ADC_buffer, "|C|%lu|D|%lu|E|%lu|F|%lu|\n\r",
					(unsigned long) ADC_C_Value, (unsigned long) ADC_D_Value,
					(unsigned long) ADC_E_Value, (unsigned long) ADC_F_Value);
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, ADC_buffer, len);
			HAL_UART_Transmit_DMA(&huart2, ADC_buffer, len);
		}

		if (HX_ENABLE) {
			read_HX711();
			memset(ADC_buffer, 0, B_SIZE);
			sprintf(ADC_buffer, "|A|%lu|B|%lu|\n\r",
					(unsigned long) ADC_A_Value, (unsigned long) ADC_B_Value);
			while (isTransmitting(&huart1, &huart2))
				;
			HAL_UART_Transmit_DMA(&huart1, ADC_buffer, len);
			HAL_UART_Transmit_DMA(&huart2, ADC_buffer, len);
		}