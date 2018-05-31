/**
 * @brief  Checks UART 1 and 2 are busy
 * @param  huart1: UART 1 Handle
 * @param  huart2: UART 2 Handle
 * @retval 1 if either are busy, 0 both are free
 */
int isTransmitting(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2) {
	return ((huart1->gState != HAL_UART_STATE_READY)
			|| (huart2->gState != HAL_UART_STATE_READY)) ? 1 : 0;
}