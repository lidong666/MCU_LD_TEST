/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// === 用户宏定义 ===
#define MOTOR2_ID   0x00    // 电机1 CAN ID
#define MOTOR1_ID   0x01    // 电机2 CAN ID
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --------------------------
// UART 接收相关全局变量
// --------------------------

volatile uint8_t uart_rx_tmp;
// 单字节接收临时缓存，用于 HAL_UART_Receive_IT 接收一个字节时存储临时数据

volatile uint8_t uart_expected_len = 0;
// 当前接收帧的预期长度（字节数），根据命令类型设置
// 例如：查询命令可能 2 字节，控制命令可能 5 字节

volatile uint8_t uart_frame_len = 0;
// 实际接收到的帧长度（字节数），接收完成后用于处理

#define UART_RX_BUF_SIZE 16
// UART 接收缓冲区大小，根据实际命令最大长度设置
// 注意：缓冲区不要太小，否则可能溢出

volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
// UART 接收缓冲区，存储接收到的多字节数据
// 接收到一帧完整数据后用于解析

volatile uint8_t uart_rx_index = 0;
// 当前接收到的字节索引，用于在缓冲区中定位
// 接收完成后需要重置为 0

volatile bool uart_frame_ready = false;
// 标记当前接收的一帧数据是否完整
// 处理完成后需要清除

// CAN 接收相关
volatile uint8_t can_rx_write_index = 0; // CAN接收写索引
volatile uint8_t can_rx_read_index = 0;  // CAN接收读索引
volatile bool can_data_received = false; // 标记是否有新CAN数据
// --------------------------
// CAN 接收消息结构体
// --------------------------
typedef struct {
    uint32_t id;      // CAN 消息 ID（11 位或 29 位扩展 ID）
    uint8_t data[8];  // CAN 消息数据内容，最大 8 字节
    uint8_t len;      // CAN 消息数据长度（实际有效字节数）
    bool valid;       // 消息有效标志，true 表示此条消息有效，false 表示无效或已处理
} CAN_RxMsg;

// --------------------------
// CAN 接收缓冲区
// --------------------------
#define CAN_RX_BUF_SIZE 16      // CAN 接收缓冲区长度，可存储的消息数量

CAN_RxMsg can_rx_buf[CAN_RX_BUF_SIZE];
// 循环/队列形式存放接收到的 CAN 消息
// 消息接收中断或 DMA 将写入此缓冲区
// 用户在主循环或任务中根据 valid 标志读取处理  测试

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f);
int _write(int file, char *ptr, int len);
int can_send_msg(uint32_t id, uint8_t *data, uint8_t len);
void send_gear_cmd_motor1(uint8_t gear, bool forward);
void send_gear_cmd_motor2(uint8_t gear, bool forward);
uint8_t can_receive_msg(uint32_t expected_id, uint8_t* buf);
void parse_and_print_can(const CAN_RxMsg* msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 串口接收回调 */
/* 串口接收回调，支持两个电机独立转速 */
/* 串口接收完成回调函数
 * 每接收到一个字节都会触发一次
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是 USART1 接收
    if (huart->Instance == USART1)
    {
        // --------------------------
        // 保存接收到的字节到缓冲区
        // --------------------------
        if (uart_rx_index < UART_RX_BUF_SIZE - 1) // 防止缓冲区溢出
            uart_rx_buf[uart_rx_index++] = uart_rx_tmp;

        // --------------------------
        // 判断是否接收到一帧数据结束标志
        // 换行 '\n' 或回车 '\r' 表示一帧结束
        // --------------------------
        if (uart_rx_tmp == '\n' || uart_rx_tmp == '\r')
        {
            // --------------------------
            // 去掉前后的空格、制表符和换行符
            // --------------------------
            int start = 0;
            while (start < uart_rx_index && (uart_rx_buf[start] == ' ' || uart_rx_buf[start] == '\t'))
                start++;
            int end = uart_rx_index - 1;
            while (end >= start && (uart_rx_buf[end] == ' ' || uart_rx_buf[end] == '\t' ||
                                    uart_rx_buf[end] == '\r' || uart_rx_buf[end] == '\n'))
                end--;

            // --------------------------
            // 确认有效数据存在
            // --------------------------
            if (end >= start)
            {
                uart_rx_buf[end + 1] = 0;  // 字符串结束符

                // --------------------------
                // 解析两个电机的 RPM
                // 格式示例: "3000 2000" 或 "-3000 2000"
                // --------------------------
                int32_t rpm1_val = 0, rpm2_val = 0;   // 存储解析出的转速
                bool rpm1_reverse = false, rpm2_reverse = false; // 是否反转标志

                char *p = (char*)&uart_rx_buf[start]; // 指向字符串开头

                // -------- 第一个 RPM --------
                if (*p == '-') { rpm1_reverse = true; p++; } // 检查负号
                while (*p >= '0' && *p <= '9')               // 累加数字
                {
                    rpm1_val = rpm1_val * 10 + (*p - '0');
                    p++;
                }
                if (rpm1_reverse) rpm1_val = -rpm1_val;     // 如果负数则取负

                // 跳过空格分隔符
                while (*p == ' ') p++;

                // -------- 第二个 RPM --------
                if (*p == '-') { rpm2_reverse = true; p++; }
                while (*p >= '0' && *p <= '9')
                {
                    rpm2_val = rpm2_val * 10 + (*p - '0');
                    p++;
                }
                if (rpm2_reverse) rpm2_val = -rpm2_val;

                // --------------------------
                // 构造 CAN 数据并发送
                // --------------------------
                uint8_t can_buf[5];
                can_buf[0] = 0x02; // 数据帧标识（固定值）

                // -------- MOTOR1 --------
                can_buf[1] = can_buf[2] = rpm1_val < 0 ? 0xFF : 0x00; // 正反转标志
                //int16_t val1 = (int16_t)((abs(rpm1_val) * 5 + 1) / 2); // 转速换算为 CAN 数据（极对数 2.5）
                int16_t val1 = (int16_t)(abs(rpm1_val) * 5); // 转速换算为 CAN 数据（极对数 5）
                if (rpm1_val < 0) val1 = -val1;                        // 如果反转则取负
                can_buf[3] = (val1 >> 8) & 0xFF;                      // 高字节
                can_buf[4] = val1 & 0xFF;                             // 低字节
                can_send_msg(MOTOR1_ID, can_buf, 5);                 // 发送到 CAN 总线

                // -------- MOTOR2 --------
                can_buf[1] = can_buf[2] = rpm2_val < 0 ? 0xFF : 0x00;
                int16_t val2 = (int16_t)(abs(rpm2_val) * 5);          // 转速换算为 CAN 数据（极对数 5）
                if (rpm2_val < 0) val2 = -val2;
                can_buf[3] = (val2 >> 8) & 0xFF;
                can_buf[4] = val2 & 0xFF;
                can_send_msg(MOTOR2_ID, can_buf, 5);

                // --------------------------
                // 串口打印调试信息
                // --------------------------
                printf("UART -> CAN MOTOR1: %s%d rpm, MOTOR2: %s%d rpm\r\n",
                       rpm1_val < 0 ? "-" : "+", abs(rpm1_val),
                       rpm2_val < 0 ? "-" : "+", abs(rpm2_val));
            }

            // --------------------------
            // 清空接收缓冲区，准备接收下一帧
            // --------------------------
            uart_rx_index = 0;
        }

        // --------------------------
        // 继续接收下一个字节（中断模式）
        // --------------------------
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_tmp, 1);
    }
}


/* CAN 接收回调 - 中断中只做接收和存缓冲 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;  // CAN报文头
    uint8_t rx_data[8];             // CAN报文数据

    // 获取 FIFO0 消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        // 保存到环形缓冲
        CAN_RxMsg* msg = &can_rx_buf[can_rx_write_index];
        msg->id = rx_header.StdId;
        memcpy(msg->data, rx_data, rx_header.DLC);
        msg->len = rx_header.DLC;
        msg->valid = true;

        can_rx_write_index = (can_rx_write_index + 1) % CAN_RX_BUF_SIZE;
        can_data_received = true;  // 标记有新数据
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t last_heartbeat_tick = 0;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // 启动CAN
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_UART_Receive_IT(&huart1, &uart_rx_tmp, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint32_t now = HAL_GetTick();  // 当前系统时间（ms）

	// =======================
	// 周期性心跳包 (500ms)
	// =======================
	if (now - last_heartbeat_tick >= 500)
	{
	  uint8_t hb[1] = {0x00};
	  can_send_msg(MOTOR1_ID, hb, 1);
	  can_send_msg(MOTOR2_ID, hb, 1);
	  last_heartbeat_tick = now;
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);  // 翻转 PE4 灯
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);  // 翻转 PE5 灯
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);  // 翻转 PE6 灯
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转 PC13 灯
	}

	// 判断是否有新的 CAN 数据到达
	if (can_data_received)
	{
	    // 遍历整个环形缓冲区，检查每一个消息
	    for (int i = 0; i < CAN_RX_BUF_SIZE; i++)
	    {
	        // 如果当前位置有有效消息（valid == true）
	        if (can_rx_buf[i].valid)
	        {
	            // 调用解析和打印函数，处理该条 CAN 消息
	            parse_and_print_can(&can_rx_buf[i]);
	            printf("4444444444444444\r\n");

	            // 处理完成后，将消息标记为无效，避免重复处理
	            can_rx_buf[i].valid = false;
	        }
	    }

	    // 所有消息处理完成后，重置 can_data_received 标志
	    // 下次有新数据到达中断才会再次进入处理
	    can_data_received = false;
	}


	HAL_Delay(10);  // 防止循环过快占用CPU

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* CAN2_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
  /* CAN2_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

/* USER CODE BEGIN 4 */
/* 接收CAN数据函数，尝试读取一帧，返回长度 */
uint8_t can_receive_msg(uint32_t expected_id, uint8_t* buf)
{
    if (can_rx_read_index == can_rx_write_index)
        return 0;  // 如果读指针等于写指针，说明缓冲区没有数据，返回0

    CAN_RxMsg* msg = &can_rx_buf[can_rx_read_index];
    // 取当前读指针指向的消息结构体指针

    if (!msg->valid)
        return 0;  // 如果该消息无效（已经读取过），返回0

    if (msg->id == expected_id)
    {
        memcpy(buf, msg->data, msg->len);
        // 把消息数据复制到用户缓冲区

        uint8_t len = msg->len;
        // 记录消息长度

        msg->valid = false;
        // 标记该消息已读，避免重复处理

        can_rx_read_index = (can_rx_read_index + 1) % CAN_RX_BUF_SIZE;
        // 读指针移向下一条消息，环形缓冲区方式

        return len;
        // 返回数据长度，表示成功读取
    }
    else
    {
        // 如果消息ID不匹配，暂不处理
        return 0;
        // 返回0，表示未接收指定ID数据
    }
}

// 发送 CAN 消息函数
// id   : CAN 标准帧 ID
// data : 待发送的数据指针
// len  : 数据长度（字节数）
// 返回值: HAL 状态，0 成功，其他值表示错误
int can_send_msg(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;  // 定义 CAN 发送帧头结构体
    uint32_t TxMailbox;             // 发送邮箱索引（CAN 控制器内部使用）

    // 配置 CAN 发送帧头
    TxHeader.StdId = id;            // 设置标准帧 ID
    TxHeader.ExtId = 0;             // 扩展帧 ID（这里不用）
    TxHeader.RTR   = CAN_RTR_DATA;  // 数据帧（不是远程帧）
    TxHeader.IDE   = CAN_ID_STD;    // 使用标准帧（11 位 ID）
    TxHeader.DLC   = len;           // 数据长度（0~8 字节）
    TxHeader.TransmitGlobalTime = DISABLE; // 不发送全局时间标志

    // 调用 HAL 库函数添加发送消息到 CAN 邮箱
    // &hcan1  : CAN 外设句柄
    // &TxHeader : 发送帧头
    // data      : 数据指针
    // &TxMailbox: 返回使用的邮箱索引
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
}

/* 解析 CAN 数据并打印 */
// 解析并打印 CAN 接收到的电机消息
void parse_and_print_can(const CAN_RxMsg* msg)
{
    // --------------------------
    // 判断是否为 MOTOR1 或 MOTOR2 消息，且数据长度 >= 6
    // --------------------------
    if ((msg->id == MOTOR1_ID || msg->id == MOTOR2_ID) && msg->len >= 6)
    {
        // 判断电机旋转方向：
        // 如果高字节全为 0xFF，表示反向旋转
        bool is_reverse = (msg->data[2] == 0xFF && msg->data[3] == 0xFF);

        // 提取原始 eRPM 值（电机实际编码器单位，16 位）
        uint16_t erpm_raw = ((uint16_t)msg->data[4] << 8) | msg->data[5];

        // 如果是反向，取补码得到负值；否则保持正值
        int16_t erpm = is_reverse ? (int16_t)(~erpm_raw + 1) : (int16_t)erpm_raw;

        // --------------------------
        // 将 eRPM 转换为实际 rpm（整数运算）
        // MOTOR1 极对数 2.5，等价于 erpm / 2.5
        // MOTOR2 极对数 5，等价于 erpm / 5
        // --------------------------
        uint16_t rpm;
        if (msg->id == MOTOR1_ID)
            rpm = (uint16_t)((erpm * 2) / 5);  // 整数运算避免浮点
        else
            rpm = (uint16_t)(erpm / 5);

        // --------------------------
        // 打印电机信息
        // --------------------------
        printf("Motor %s ID=0x%03lX Speed: %s%u rpm\r\n",
               msg->id == MOTOR1_ID ? "1" : "2",  // 电机编号
               (unsigned long)msg->id,            // CAN ID
               is_reverse ? "-" : "+",            // 正负转向符号
               rpm);                              // 实际转速 rpm
    }
    // --------------------------
    // 数据长度不足 6 字节，打印错误信息
    // --------------------------
    else if ((msg->id == MOTOR1_ID || msg->id == MOTOR2_ID) && msg->len < 6)
    {
        printf("Motor %s ID=0x%03lX Data length error\r\n",
               msg->id == MOTOR1_ID ? "1" : "2",  // 电机编号
               (unsigned long)msg->id);           // CAN ID
    }
}


// 重定向 printf 输出到 UART
int fputc(int ch, FILE *f)
{
    // 将单个字符 ch 通过 UART1 发送出去
    // HAL_MAX_DELAY 表示等待直到发送完成，不超时
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch; // 返回发送的字符
}

// 重定向 _write 系统调用，用于 printf 等函数
int _write(int file, char *ptr, int len)
{
    // 将 len 个字节的数据通过 UART1 发送出去
    // ptr 指向要发送的缓冲区
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len; // 返回成功发送的字节数
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
