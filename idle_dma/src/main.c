/**
 **************************************************************************
 * @file     main.c
 * @brief    main program
 **************************************************************************
 *                       Copyright notice & Disclaimer
 *
 * The software Board Support Package (BSP) that is made available to
 * download from Artery official website is the copyrighted work of Artery.
 * Artery authorizes customers to use, copy, and distribute the BSP
 * software and its related documentation for the purpose of design and
 * development in conjunction with Artery microcontrollers. Use of the
 * software is governed by this copyright notice and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
 * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
 * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
 * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
 * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
 *
 **************************************************************************
 */

#include "at32f421_board.h"
#include "at32f421_clock.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>

uint8_t escid; //value 0 or 1
// #setv v0 [v1]. argc=2, v0 for both esc
// argc=3, escv0 = v0, 
void setv_cmd(char argc, char *argv[])
{
    int v,v1;
    if (argc == 2) //v for 0 and 1
    {
        v = strtoul(argv[1], NULL, 10);
        //v[1] = strtoul(argv[2], NULL, 10);
        // set current v
        // esc_setv(v);
        LOGW("setv both %d\r\n", v);
    }

}

void getv_cmd(char argc, char *argv[])
{
}

typedef struct bcmd_s
{
    char *cmd;                           // command name, e.g. "setv", "getv", "start", "stop" ...
    void (*cb)(char argc, char *argv[]); // callback function
} bcmd_t;

const bcmd_t bcmd_tab[] = {
    {"setv", setv_cmd},
    {"getv", getv_cmd},
};

#define COUNTOF(a) (sizeof(a) / sizeof(*(a)))
#define USART2_TX_BUFFER_SIZE (COUNTOF(usart2_tx_buffer) - 1)
#define USART1_TX_BUFFER_SIZE (COUNTOF(usart1_tx_buffer) - 1)
#define USART1_RX_BUFFER_SIZE 64
#define USART2_RX_BUFFER_SIZE 1024 // master u2 pa2 pa15 connect to BLE
#define DMACH_U1TX DMA1_CHANNEL2
#define DMACH_U1RX DMA1_CHANNEL3
#define DMACH_U2TX DMA1_CHANNEL4
#define DMACH_U2RX DMA1_CHANNEL5
//uart2 esc0 -- ble, uart1 esc0 <--> esc1
//for esc0: ble cmd-> uart2, forward to esc1 via uart1
//for esc1: esc cmd-> uart1

uint8_t usart2_tx_buffer[] = "usart transfer by dma interrupt: usart2 -> usart1 using dma";
uint8_t usart1_tx_buffer[] = "usart transfer by dma interrupt: usart1 -> usart2 using dma";

uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];


#define UART_TX_DMA_FUNC_DECLARE(x) \
static inline void dmatx_u##x(char *buf, uint16_t len)           \
    {                                               \
        if (len > USART##x##_TX_BUFFER_SIZE)        \
            len = USART##x##_TX_BUFFER_SIZE;        \
        memcpy(usart##x##_tx_buffer, buf, len);     \
        DMACH_U##x##TX->ctrl_bit.chen = 0;          \
        DMACH_U##x##TX->dtcnt = len;                \
        DMACH_U##x##TX->ctrl_bit.chen = 1;          \
    }

UART_TX_DMA_FUNC_DECLARE(1) //usage: dmatx_u1(buf, len)
UART_TX_DMA_FUNC_DECLARE(2)

#define STRCMD_PREFIX '#'
#define MAX_ARGS 5
#define STRCMD_MAXLEN 20
// we receive cmds like '#setv arg1 arg2' from BLE host, forward to slave esc
void try_parse_cmd(char *buf, uint8_t sz, int forward)
{
    // strtok modifies the original string, so use a copy if the original must be preserved.
    char cmd_copy[STRCMD_MAXLEN];
    char *argv[MAX_ARGS] = {0};
    char argc = 0;
    char *saveptr;
    if (sz < 2 || buf[0] != STRCMD_PREFIX)
        return; // not a cmd
    strncpy(cmd_copy, buf+1, STRCMD_MAXLEN);
    char *token = strtok_r(cmd_copy, " ", &saveptr);
    while (NULL != token && argc < MAX_ARGS)
    {
        argv[argc++] = token;                  // Store the token in the argv array
        token = strtok_r(NULL, " ", &saveptr); // Get the next token
    }

    for (int i = 0; i < COUNTOF(bcmd_tab); i++)
    {
        if (0 == strcmp(bcmd_tab[i].cmd, argv[0]) && bcmd_tab[i].cb)
        {
            LOGI("found cmd %s, argc=%d\r\n", argv[0], argc);
            bcmd_tab[i].cb(argc, argv);
            //uart forward cmd 
            forward ? dmatx_u1(buf, sz) : (void)0;//forward whole str
            break;
        }
    }
}

void u1rx_idle_cb_func(uint8_t *buf, uint16_t len);
void u2rx_idle_cb_func(uint8_t *buf, uint16_t len);
typedef void (*cb_func)(uint8_t *buf, uint16_t len);

void u1rx_idle_cb_func(uint8_t *buf, uint16_t len)
{
    printf("u1idma_cb: %s\r\n", buf);
    try_parse_cmd((char *)buf, len, 0);//esc1
}

void u2rx_idle_cb_func(uint8_t *buf, uint16_t len)
{
    printf("u2cb: [%2d] %s\r\n", len, buf);
    try_parse_cmd((char *)buf, len, 1);//esc0
}

cb_func rxidle_cb_u1 = u1rx_idle_cb_func;
cb_func rxidle_cb_u2 = u2rx_idle_cb_func;

// void u1dmatx(char *buf, uint16_t len)
// {
//     // uint16_t len = strlen(buf);
//     if (len > USART1_TX_BUFFER_SIZE)
//         len = USART1_TX_BUFFER_SIZE;
//     memcpy(usart1_tx_buffer, buf, len);
//     dma_channel_disable(DMACH_U1TX, TRUE);
//     // DMACH_U1TX->ctrl_bit.chen = 0;
//     DMACH_U1TX->dtcnt = len;
//     // DMACH_U1TX->ctrl_bit.chen = 1;
//     dma_channel_enable(DMACH_U1TX, TRUE);
// }

// void u2dmatx(char *buf, uint16_t len)
// {
//     // uint16_t len = strlen(buf);
//     if (len > USART2_TX_BUFFER_SIZE)
//         len = USART2_TX_BUFFER_SIZE;
//     memcpy(usart2_tx_buffer, buf, len);
//     dma_channel_disable(DMACH_U2TX, TRUE);
//     DMACH_U2TX->ctrl_bit.chen = 0;
//     DMACH_U2TX->dtcnt = len;
//     DMACH_U2TX->ctrl_bit.chen = 1;
//     dma_channel_enable(DMACH_U2TX, TRUE);
// }



/**
 * @brief  initialize uart1 in pb6 pb7
 * @param  baudrate: uart baudrate
 * @retval none
 */
void uart1_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;

#if defined(__GNUC__) && !defined(__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    /* enable the uart and gpio clock */
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_0);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE7, GPIO_MUX_0);

    /* dma1 channel2 for usart1 tx configuration */
    dma_reset(DMA1_CHANNEL2);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = USART1_TX_BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)usart1_tx_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL2, &dma_init_struct);

    /* dma1 channel3 for usart1 rx configuration */
    dma_reset(DMA1_CHANNEL3);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = USART1_RX_BUFFER_SIZE; // rxcnt should never exceed this value
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)usart1_rx_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    /* configure uart param */
    usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART1, TRUE);
    usart_receiver_enable(USART1, TRUE);
    usart_interrupt_enable(USART1, USART_IDLE_INT, TRUE);

    usart_dma_transmitter_enable(USART1, TRUE);
    usart_dma_receiver_enable(USART1, TRUE);

    dma_interrupt_enable(DMA1_CHANNEL3, DMA1_FDT3_FLAG, TRUE); // in dma idle mode, FDT should never happen.
    nvic_irq_enable(DMA1_Channel3_2_IRQn, 3, 0);
    nvic_irq_enable(USART1_IRQn, 1, 0);

    dma_channel_enable(DMA1_CHANNEL3, TRUE); /* usart1 rx begin dma receiving */
    usart_enable(USART1, TRUE);
}

/**
 * @brief  initialize uart1 in pb6 pb7
 * @param  baudrate: uart baudrate
 * @retval none
 */
void uart2_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;

#if defined(__GNUC__) && !defined(__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    /* enable the uart and gpio clock */
    crm_periph_clock_enable(CRM_USART2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_2 | GPIO_PINS_15;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio_init_struct);

    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE15, GPIO_MUX_1);

    // /* dma1 channel4 for usart2 tx configuration */
    dma_reset(DMA1_CHANNEL4);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = USART2_TX_BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)usart2_tx_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL4, &dma_init_struct);

    // /* dma1 channel5 for usart2 rx configuration */
    dma_reset(DMA1_CHANNEL5);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = USART2_RX_BUFFER_SIZE;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)usart2_rx_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART2->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA1_CHANNEL5, &dma_init_struct);

    /* configure uart param */
    usart_init(USART2, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(USART2, TRUE);
    usart_receiver_enable(USART2, TRUE);
    usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);

    usart_dma_transmitter_enable(USART2, TRUE);
    usart_dma_receiver_enable(USART2, TRUE);

    dma_interrupt_enable(DMA1_CHANNEL5, DMA1_FDT5_FLAG, TRUE); // in dma idle mode, FDT should never happen.
    nvic_irq_enable(DMA1_Channel5_4_IRQn, 3, 0);
    nvic_irq_enable(USART2_IRQn, 0, 0);
    dma_channel_enable(DMA1_CHANNEL5, TRUE); /* usart2 rx begin dma receiving */
    usart_enable(USART2, TRUE);
}

void u1rxdma_reset(void)
{
    DMA1_CHANNEL3->ctrl_bit.chen = 0;                  // disable dma channel first
    DMA1_CHANNEL3->maddr = (uint32_t)usart1_rx_buffer; // reset memory address
    DMA1_CHANNEL3->dtcnt = USART1_RX_BUFFER_SIZE;      // reset dma counter
    DMA1_CHANNEL3->ctrl_bit.chen = 1;                  // enable dma channel again
}

void u2rxdma_reset(void)
{
    DMA1_CHANNEL5->ctrl_bit.chen = 0;                  // disable dma channel first
    DMA1_CHANNEL5->maddr = (uint32_t)usart2_rx_buffer; // reset memory address
    DMA1_CHANNEL5->dtcnt = USART2_RX_BUFFER_SIZE;      // reset dma counter
    DMA1_CHANNEL5->ctrl_bit.chen = 1;                  // enable dma channel again
}

/**
 * @brief  this function handles usart2 handler.
 * @param  none
 * @retval none
 */
void USART1_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_IDLEF_FLAG))
    {
        // usart_data_receive(USART1);
        usart_flag_clear(USART1, USART_IDLEF_FLAG);
        DMA1_CHANNEL3->ctrl_bit.chen = 0; // disable dma channel first

        uint16_t rxcnt = USART1_RX_BUFFER_SIZE - DMA1_CHANNEL3->dtcnt;
        // callback for received data processing
        rxidle_cb_u1 ? rxidle_cb_u1(usart1_rx_buffer, rxcnt) : (void)0;
        // TODO: enh, use ping pong buffer to avoid data loss
        u1rxdma_reset();
    }
}

void USART2_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART2, USART_IDLEF_FLAG))
    {
        // usart_data_receive(USART2);
        usart_flag_clear(USART2, USART_IDLEF_FLAG);
        DMA1_CHANNEL5->ctrl_bit.chen = 0; // disable dma channel first

        uint16_t rxcnt = USART2_RX_BUFFER_SIZE - DMA1_CHANNEL5->dtcnt;
        // callback for received data processing
        rxidle_cb_u2 ? rxidle_cb_u2(usart2_rx_buffer, rxcnt) : (void)0;
        // TODO: enh, use ping pong buffer to avoid data loss
        u2rxdma_reset();
    }
}

// dma full transfer interrupt handler, should never happen in idle dma mode
void DMA1_Channel3_2_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA1_FDT3_FLAG) != RESET)
    {
        u1rxdma_reset();
        dma_flag_clear(DMA1_FDT3_FLAG);
        printf("FATAL: %s", "dma1 ch3 full transfer aka ovf occur\r\n");
    }
}

void DMA1_Channel5_4_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA1_FDT5_FLAG) != RESET)
    {
        u2rxdma_reset();
        dma_flag_clear(DMA1_FDT5_FLAG);
        printf("FATAL: %s", "dma1 ch5 full transfer aka ovf occur\r\n");
    }
}


int pwm6_adc_init(void);

/**
 * @brief  main function.
 * @param  none
 * @retval none
 */
int main(void)
{
    system_clock_config();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    // dma_configuration();
    uart1_init(115200);
    uart2_init(115200);
    pwm6_adc_init();
    delay_init();
    LOGI("u1 idle dma example fuck at32\r\n");

    /* transmit a data */
    while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
        ;
    usart_data_transmit(USART1, 0x55);

    /* receive a data from the tx pin */
    // while(usart_flag_get(USART1, USART_RDBF_FLAG) == RESET);
    // usart_data_receive(USART1);

    while (1)
    {
        // dmatx_u1("6", 1);
        // while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
        // usart_data_transmit(USART1, 0x55);
        delay_ms(1000);

        // while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
        // usart_data_transmit(USART2, 'a');
    }
}

/**
 * @}
 */

/**
 * @}
 */
