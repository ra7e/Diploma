
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_hal.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "string.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define GENERATING_POLYNOMIAL 0x1021
	#define HIGHEST_BIT 0x8000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	SPI_HandleTypeDef hspi2;
	I2C_HandleTypeDef hi2c1;
	I2C_HandleTypeDef hi2c2;
  HAL_StatusTypeDef returnValue;
  uint16_t addr, addr1;
  uint8_t receive_data;
  uint16_t addr_array_s[5];
  uint16_t addr_array_r[5];
  int addr_counter = 0;
  uint8_t init_data = 0xFE;
  uint8_t trans_data = 0x01;
  int oled_counter = 1;
  int receive_cnt;
	int num_of_addr = 0;
	int num_of_pin = 0;
	int a = 1;
	uint8_t data_r[28];
	uint16_t * d_pin;
	uint8_t have_new_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void init_pcf();
void next();
void previous();
void ftrans_data(uint8_t *);
uint8_t freceive_data(uint16_t *);
void one_hot_send(uint8_t *);
void Draw();
void one_hot_send_prev(uint8_t *);
void init_pcf_receiver();
void return_spot_of_zero();
void equalaizer();
void PrintChar(int);
int read_data();
void INIT_read_data();
void test_func();
uint16_t calculateCRC (int arrLength, uint8_t *arrayToCalc);
//void write_data(uint16_t, uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init() {
	ssd1306_Init();
}

void loop() {
	HAL_Delay(100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */

	addr_array_s[0] = 0x71;
	addr_array_s[1] = 0x73;
	addr_array_s[2] = 0x75;
	addr_array_s[3] = 0x77;
	addr_array_s[4] = 0x79;

	addr_array_r[0] = 0x71;
	addr_array_r[1] = 0x73;
	addr_array_r[2] = 0x75;
	addr_array_r[3] = 0x77;
	addr_array_r[4] = 0x79;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ssd1306_Init();
  init_pcf();
  init_pcf_receiver();
  INIT_read_data();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 equalaizer();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void test_func()
{
	int num_of_pcf = 0;
	int num_of_pin = 0;
	uint16_t addr = 0;
	uint8_t data = 0;
	uint8_t zero_data = 0xff;

	num_of_pcf = oled_counter / 8;
	num_of_pin = oled_counter % 8;

	switch(num_of_pcf)
	{
		case (0):addr = addr_array_s[0]; break;
		case (1):addr = addr_array_s[1]; break;
		case (2):addr = addr_array_s[2]; break;
		case (3):addr = addr_array_s[3]; break;
		case (4):addr = addr_array_s[4]; break;
	}

	switch(num_of_pin)
	{
		case(0): data = 0xff; break;
		case(1): data = 0xfe; break;
		case(2): data = 0xfd; break;
		case(3): data = 0xfb; break;
		case(4): data = 0xf7; break;
		case(5): data = 0xef; break;
		case(6): data = 0xdf; break;
		case(7): data = 0xbf; break;
		case(8): data = 0x7f; break;
	}

	if(oled_counter > 34)
	{
		oled_counter = 34;
	}
	else if(oled_counter < 0)
	{
		oled_counter = 0;
	}

	for(int i = 0; i <= 4;i++)
	{
		if(addr == addr_array_s[i])
		{
			HAL_I2C_Master_Transmit(&hi2c1, addr, &data, 1, 100);
		}
		else
		{
			HAL_I2C_Master_Transmit(&hi2c1, addr_array_s[i], &zero_data, 1, 100);
		}
	}

}

void equalaizer()
{
	char zero_spot[7]= {0};
	int zero_holder_array [7] = {0};
	int n = 0;

	test_func();

	return_spot_of_zero(zero_holder_array);

	for(int i = 0; i <= 6; i++)
	{
		n += sprintf (&zero_spot[n], "%2i", zero_holder_array[i]);
	}

	ssd1306_SetCursor(1, 5);
	ssd1306_WriteString("Tx-", Font_11x18, White);
	PrintChar(oled_counter);
	ssd1306_SetCursor(1, 40);
	ssd1306_WriteString("Rx-", Font_11x18, White);
	ssd1306_WriteString(zero_spot, Font_11x18, White);
}

void equalaizer_pc()
{
	char zero_spot[3]= {0};
	int zero_holder_array [3] = {0};
	uint16_t quantity_of_elem;
	uint16_t out_pin = 0;
	uint16_t in_pin[3] = {0};
	uint16_t tmp;
	uint8_t id = 0x00;
	int n = 0;
	int k = 0;

	HAL_UART_Receive_IT(&huart2, data_r, 28);
	if (have_new_data){
		read_data();
		test_func();
	}

	return_spot_of_zero(zero_holder_array);

	quantity_of_elem = d_pin[0];

	uint16_t back_array[quantity_of_elem];

	oled_counter = out_pin;

	for(int i = 1; i <= quantity_of_elem; i++)
	{
		tmp = d_pin[i];

		if(i == 1)
		{
			out_pin = tmp;
		}
		if(i > 1)
		{
			in_pin[i - 2] = d_pin[i];
		}
	}

	for(int i = 0; i < quantity_of_elem - 2; i++)
	{
		k += sprintf (&zero_spot[n], "%2i", in_pin[i]);
	}

	for(int i = 0; i <= quantity_of_elem-2; i++)
	{
		if(in_pin[i] == zero_holder_array[i])
		{
			back_array[i+2] = 0xff;
		}
		else
		{
			back_array[i+2] = 0;
		}
	}

	for(int i = 0; i <= 2; i++)
	{
		n += sprintf (&zero_spot[n], "%2i", zero_holder_array[i]);
	}

	back_array[0] = quantity_of_elem;
	back_array[1] = oled_counter;

	write_data(back_array, id);

	ssd1306_SetCursor(1, 5);
	ssd1306_WriteString("Tx-", Font_11x18, White);
	PrintChar(oled_counter + 1);
	ssd1306_SetCursor(1, 40);
	ssd1306_WriteString("Rx-", Font_11x18, White);
	ssd1306_WriteString(zero_spot, Font_11x18, White);
}

void write_data(uint16_t * t_data, uint8_t id)
{
	uint8_t trans_buf[28];

	for (int i = 0; i < 28; i++)
	{
		trans_buf[i] = 0;
	}

	trans_buf[0] = 0x7E;
	trans_buf[1] = 0x7E;

	trans_buf[2] = id;
	trans_buf[3] = t_data[0] - 2;
	for (int i = 0; i < t_data[0] - 1; i++)
	{
		//=QByteArray::fromRawData(reinterpret_cast<const char*>(outbuffer[i]),sizeof(i));
		trans_buf[(i * 2) + 4] = ((t_data[i + 1] >>8) & 0xFF);
		trans_buf[(i * 2) + 5] = (t_data[i + 1] & 0xFF);
	}

	uint16_t CRC_buf = calculateCRC(26, trans_buf);

	trans_buf[26] = ((CRC_buf >>8) & 0xFF);
	trans_buf[27] = (CRC_buf & 0xFF);

	HAL_UART_Transmit(&huart2, trans_buf, 28, 100);
}


void PrintChar(int pin_numb)
{
	char buf[3];
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	sprintf(buf, "%2i", pin_numb);
	ssd1306_WriteString(buf, Font_11x18, White);
	ssd1306_UpdateScreen();
	loop();
}

void init_pcf()
{
	ftrans_data(&init_data);
	uint8_t data = 0xff;
	for(uint16_t addr = 0x72; addr <= 0x78; addr += 0x02)
		HAL_I2C_Master_Transmit(&hi2c1, addr, &data, 1, 100);
}

void ftrans_data(uint8_t *data)
{
	HAL_I2C_Master_Transmit(&hi2c1, addr_array_s[addr_counter], data, 1, 100);
}

uint8_t freceive_data(uint16_t *addr)
{
	uint8_t data;
	HAL_I2C_Master_Receive(&hi2c2, *addr, &data, 1, 100);
	return data;
}


void init_pcf_receiver()

{
	uint8_t data = 0xff;
	for(uint16_t addr = 0x70; addr <= 0x78; addr += 1)
		HAL_I2C_Master_Transmit(&hi2c2, addr, &data, 1, 100);
}



void return_spot_of_zero(int *zero_holder_array)
{
	uint8_t res_data;
	int zero_cnt = -1;

	uint8_t array[8];
	uint8_t full_array[5*8];

	  for(int i = 0; i < 5; i++)
	  {
		  	addr = addr_array_r[i];

		  	res_data = freceive_data(&addr);

	  		array[7] = (res_data & 0x80UL)>>7;
	  		array[6] = (res_data & 0x40UL)>>6;
	  		array[5] = (res_data & 0x20UL)>>5;
	  		array[4] = (res_data & 0x10UL)>>4;
	  		array[3] = (res_data & 0x08UL)>>3;
	  		array[2] = (res_data & 0x04UL)>>2;
	  		array[1] = (res_data & 0x02UL)>>1;
	  		array[0] = res_data & 0x01UL;

	  		for(int k = 0; k < 8; k++)
	  		{
	  			full_array[k+(8*i)] = array[k];
	  		}
	  }

	  for(int i = 0; i <= 34; i++)
	  {
		  if(full_array[i] == 0)
		  {
			  zero_cnt++;
			  if(i == 0){
				  zero_holder_array[zero_cnt] = i;
			  }
			  else
			  {
				  zero_holder_array[zero_cnt] = i + 1;
			  }
		  }
	  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		HAL_UART_Transmit(&huart2, data_r, 28, 100);
		have_new_data = 1;
	}
}

void INIT_read_data()
{

	have_new_data = 0;
}

uint16_t calculateCRC (int arrLength, uint8_t *arrayToCalc)
{

	uint16_t carry, currentWord;
	uint16_t crcReg = 0xffff;

	for (int i = 0; i < arrLength; i++)
	{
		currentWord = arrayToCalc[i] << 8;
		for (int i = 0; i < 8; i++)
		{
			carry = (crcReg ^ currentWord) & HIGHEST_BIT;
			crcReg <<= 1;
			if (carry) crcReg ^= GENERATING_POLYNOMIAL;
			currentWord <<= 1;
		}
	}
	return crcReg;
}

int read_data()
{
	have_new_data = 0;
	uint16_t crc_t = data_r[26] << 8 + data_r[27];

	if(calculateCRC(26, data_r) == crc_t)
		return 0;

	d_pin = malloc(sizeof(uint16_t) * data_r[3] + 2);
	d_pin[0] = data_r[3] + 2;
	uint16_t temp = 0;
	temp = data_r[4] << 8;
	temp = temp + data_r[5];
	d_pin[1] = temp;

	for(int i = 0; i < d_pin[0]; i++)
	{
		temp = 0;
		temp = data_r[i * 2 + 6] << 8;
		temp = temp + data_r[i * 2 + 7];
		d_pin[i + 2] = temp;
	}
	uint16_t test[20];
	for(int i = 0; i < 20; i++)
	{
		test[i] = 0;
	}
	for(int i = 0; i < d_pin[0] + 2; i++)
	{
		test[i] = d_pin[i];
	}
	return 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int next_button = 0;
	//int prev_button = 0;

	if((GPIO_Pin == GPIO_PIN_0))
	{
		if(a == 1){
			oled_counter++;
			a = 0;
		}
		else
		{
			for(int i = 0; i < 150000; i++)
			{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1)
				{
					next_button++;
				}
			a = 1;
			}
		}
	}
	else if((GPIO_Pin == GPIO_PIN_1))
	{
		if(a == 1)
		{
			oled_counter--;
			a = 0;
		}
		else
		{
			for(int i = 0; i < 150000; i++)
			{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1)
				{
					next_button++;
				}
			a = 1;
		}
	}
	}
	else
		__NOP();


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
