#ifndef BOARD_H__
#define BOARD_H__

// ==== General ====
#define BOARD_NAME          "Floe v1"
#define APP_NAME            "FLoe"

#define STM32G070xx

#define LSE_FREQ_HZ     32768   // Left it here even if not used

// ST driver system settings
#define STM32_ST_IRQ_PRIORITY               2
#define STM32_ST_USE_TIMER                  15

#ifndef TRUE
#define TRUE            1
#endif

#if 1 // ==== Flash ====
#define FLASH_SZ            0x20000UL // 128k
#define FLASH_PAGE_SZ       2048UL // Constant, see ds
#define SETTINGS_SZ         FLASH_PAGE_SZ
#define BL_SZ_MAX           8192UL // Max sz of bootloader
#define APP_SZ_MAX          (FLASH_SZ - BL_SZ_MAX - SETTINGS_SZ)
#define FLASH_SETTINGS_ADDR (FLASH_BASE + FLASH_SZ - SETTINGS_SZ)
#define APP_START_ADDR      (FLASH_BASE + BL_SZ_MAX)
#endif

#if 1 // ========================== GPIO =======================================
// UART
#define UART_GPIO       GPIOA
#define UART_TX_PIN     9
#define UART_RX_PIN     10

// I2C
#define SCL2_GPIO       GPIOA
#define SCL2_PIN        11
#define SDA2_GPIO       GPIOA
#define SDA2_PIN        12
#define I2C2_AF         AF6

// Counts
#define LED_CNT         7UL // Max
#define LED_CHNL_CNT    (LED_CNT * 3UL)
#define LED_BYTE_CNT    4UL // 32 bits
#define LED_WORD_CNT    2UL // DISP_BYTE_CNT / 2
#define LED_IC_CNT      2UL

// Debug pin
#define DBG_PIN         GPIOB, 0
#define DBG_HI()        PinSetHi(DBG_PIN)
#define DBG_LO()        PinSetLo(DBG_PIN)
#define DBG_TOGGLE()    PinToggle(DBG_PIN)

// LEDs
#define TIMESLOT_TOP    255UL
//#define TIMESLOT_TOP    2UL
#define OUT_EN_PIN      GPIOA, 7, AF4
#define LATCH_TMR       TIM3
#define LATCH_TMR_IN    GPIOB, 4, omPushPull, pudPullDown, AF1
#define LATCH_PIN       GPIOB, 1, omPushPull, pudNone, AF1
#define LED_DATA        GPIOB, 5, omPushPull, pudNone, AF0
#define LED_CLK         GPIOB, 3, omPushPull, pudNone, AF0

// Acc
#define ACG_IRQ_PIN     GPIOC, 13

// Button
#define BTN_PIN         GPIOA, 0
#endif // GPIO

#if 1 // =========================== SPI =======================================
#define LEDS_SPI        SPI1
#endif

#if 1 // =========================== I2C =======================================
#define I2C1_ENABLED        FALSE
#define I2C2_ENABLED        TRUE
#define I2C_BAUDRATE_HZ     400000
#define I2C_USE_DMA         FALSE
#endif

#if 1 // =========================== ADC =======================================
#define ADC_ENABLED     TRUE
#define ADC_CHNL_CNT    2 // Tsns and VRef


#endif

#if 1 // =========================== Timers ====================================
#define LEDS_TIM        TIM3
#define LEDS_TIM_IRQ_VECTOR TIM3_IRQHandler

#define LEDS_DIM_TIM    TIM14
#define LEDS_DIM_CHNL   1

#define TIME_TIMER      TIM7
#define ADC_TIMER       TIM6

#endif


#if 1 // =========================== DMA =======================================
// ==== Uart ====
#define UART_DMA_TX_CHNL    7
#define UART_DMA_TX_REQID   51 // p223 of refman
#define UART_DMA_RX_CHNL    6
#define UART_DMA_RX_REQID   50 // p223 of refman

#define UART_DMA_TX_MODE (  DMA_PRIORITY_LOW | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |       /* Memory pointer increase */ \
                            DMA_DIR_MEM2PER |    /* Direction is memory to peripheral */ \
                            DMA_TCIE         /* Enable Transmission Complete IRQ */)

#define UART_DMA_RX_MODE (  DMA_PRIORITY_MEDIUM | \
                            DMA_MSIZE_8_BIT | \
                            DMA_PSIZE_8_BIT | \
                            DMA_MEM_INC |       /* Memory pointer increase */ \
                            DMA_DIR_PER2MEM |    /* Direction is peripheral to memory */ \
                            DMA_CIRC         /* Circular buffer enable */)

// ==== LEDs ====
#define LEDS_DMA_TX_CHNL    1
#define LEDS_DMA_TX_REQID   17 // SPI1 TX, p223 of refman


// ==== I2C1 ====
#define I2C1_DMA_TX_CHNL    5
#define I2C1_DMA_TX_REQID   11
#define I2C1_DMA_RX_CHNL    4
#define I2C1_DMA_RX_REQID   10

// ==== ADC ====
//#define ADC_DMA_CHNL
//#define ADC_DMA_REQID       5

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN     FALSE
#define UART_TXBUF_SZ       512
#define UART_RXBUF_SZ       256
#define CMD_BUF_SZ          128
#define UART_CHAR_TO_IRQ    '\n'

#define CMD_UART_PARAMS \
    USART1, UART_GPIO, UART_TX_PIN, UART_GPIO, UART_RX_PIN, \
    UART_DMA_TX_CHNL, UART_DMA_RX_CHNL, \
    UART_DMA_TX_MODE, UART_DMA_RX_MODE,\
    UART_DMA_TX_REQID, UART_DMA_RX_REQID, \
     uartclkPCLK

#endif

#endif //BOARD_H__