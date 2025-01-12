/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_PIN 5
/* USER CODE END PD */

UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


int main(void)
{

  /* USER CODE BEGIN 1 */

	RCC->AHBENR |= (1U<<17);
	uint8_t data[5] = {0};
	uint8_t MSG[55] = {'\0'};
	dht11 *sensor = dht11_init(GPIOA, TEST_PIN, data);
	GPIOA->MODER |= (0b01 << (2*SPEAKER_PIN));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int success = get_data(sensor);
	  if(success != 0) {
		  continue;
	  }
	  int temp = readTemp(sensor);
	  int humidity = readHumidity(sensor);
	  sprintf(MSG, "Current Temp is = %d and current humidity = %d\r\n", temp, humidity);
	  HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	  HAL_Delay(2000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }


  /* USER CODE END 3 */
}
