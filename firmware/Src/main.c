/**
 * STM32F103C8T6 + MPU6050 + u-blox ZED-F9P synchronized logger.
 * Wiring: MPU6050 SCL->PB6, SDA->PB7, INT->PA1/TIM2_CH2;
 *         C099-F9P TP->PA0/TIM2_CH1, TX_ZED->PA3/USART2_RX,
 *         RX_ZED->PA2/USART2_TX; debug USB-TTL RX->PA9.
 * USART1 logging uses 460800 8N1 and ZED-F9P UART1 uses 115200 8N1. TIM2 captures
 * PPS and IMU DATA_RDY in the same 1 MHz hardware time domain.
 */
#include <stdbool.h>
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define RCC_CR      REG32(0x40021000u)
#define RCC_CFGR    REG32(0x40021004u)
#define RCC_APB2ENR REG32(0x40021018u)
#define RCC_APB1ENR REG32(0x4002101Cu)
#define FLASH_ACR   REG32(0x40022000u)
#define GPIOA_CRL   REG32(0x40010800u)
#define GPIOA_CRH   REG32(0x40010804u)
#define GPIOA_BSRR  REG32(0x40010810u)
#define GPIOA_BRR   REG32(0x40010814u)
#define GPIOB_CRL   REG32(0x40010C00u)
#define GPIOB_BSRR  REG32(0x40010C10u)
#define GPIOB_BRR   REG32(0x40010C14u)
#define GPIOC_CRH   REG32(0x40011004u)
#define GPIOC_BSRR  REG32(0x40011010u)
#define GPIOC_BRR   REG32(0x40011014u)
#define USART1_SR   REG32(0x40013800u)
#define USART1_DR   REG32(0x40013804u)
#define USART1_BRR  REG32(0x40013808u)
#define USART1_CR1  REG32(0x4001380Cu)
#define USART2_SR   REG32(0x40004400u)
#define USART2_DR   REG32(0x40004404u)
#define USART2_BRR  REG32(0x40004408u)
#define USART2_CR1  REG32(0x4000440Cu)
#define I2C1_CR1    REG32(0x40005400u)
#define I2C1_CR2    REG32(0x40005404u)
#define I2C1_DR     REG32(0x40005410u)
#define I2C1_SR1    REG32(0x40005414u)
#define I2C1_SR2    REG32(0x40005418u)
#define I2C1_CCR    REG32(0x4000541Cu)
#define I2C1_TRISE  REG32(0x40005420u)
#define TIM2_CR1    REG32(0x40000000u)
#define TIM2_DIER   REG32(0x4000000Cu)
#define TIM2_SR     REG32(0x40000010u)
#define TIM2_EGR    REG32(0x40000014u)
#define TIM2_CCMR1  REG32(0x40000018u)
#define TIM2_CCER   REG32(0x40000020u)
#define TIM2_CNT    REG32(0x40000024u)
#define TIM2_PSC    REG32(0x40000028u)
#define TIM2_ARR    REG32(0x4000002Cu)
#define TIM2_CCR1   REG32(0x40000034u)
#define TIM2_CCR2   REG32(0x40000038u)
#define NVIC_ISER0  REG32(0xE000E100u)
#define NVIC_ISER1  REG32(0xE000E104u)
#define SYSTICK_CTRL REG32(0xE000E010u)
#define SYSTICK_LOAD REG32(0xE000E014u)
#define SYSTICK_VAL  REG32(0xE000E018u)

#define USART_TXE (1u << 7)
#define USART_TC  (1u << 6)
#define USART_RXNE (1u << 5)
#define USART_ORE (1u << 3)
#define TIM_UIF   (1u << 0)
#define TIM_CC1IF (1u << 1)
#define TIM_CC2IF (1u << 2)
#define I2C_PE    (1u << 0)
#define I2C_START (1u << 8)
#define I2C_STOP  (1u << 9)
#define I2C_ACK   (1u << 10)
#define I2C_SWRST (1u << 15)
#define I2C_SB    (1u << 0)
#define I2C_ADDR  (1u << 1)
#define I2C_BTF   (1u << 2)
#define I2C_RXNE  (1u << 6)
#define I2C_TXE   (1u << 7)
#define I2C_BERR  (1u << 8)
#define I2C_ARLO  (1u << 9)
#define I2C_AF    (1u << 10)
#define I2C_OVR   (1u << 11)
#define I2C_BUSY  (1u << 1)
#define I2C_ERRORS (I2C_BERR | I2C_ARLO | I2C_AF | I2C_OVR)

#define MPU_SMPLRT_DIV   0x19u
#define MPU_CONFIG       0x1Au
#define MPU_GYRO_CONFIG  0x1Bu
#define MPU_ACCEL_CONFIG 0x1Cu
#define MPU_INT_PIN_CFG  0x37u
#define MPU_INT_ENABLE   0x38u
#define MPU_INT_STATUS   0x3Au
#define MPU_ACCEL_XOUT_H 0x3Bu
#define MPU_USER_CTRL    0x6Au
#define MPU_PWR_MGMT_1   0x6Bu
#define MPU_PWR_MGMT_2   0x6Cu
#define MPU_WHO_AM_I     0x75u
#define I2C_TIMEOUT_MS   5u
#define SAMPLE_RATE_HZ   100u
#define TIMER_TICK_HZ    1000000u
#define GPS_WEEK_US      604800000000ull
#define DEBUG_UART_BAUD  460800u
#define GNSS_UART_BAUD   115200u
#define GNSS_RX_SIZE     1024u
#define RTCM_TX_SIZE     2048u /* Qt limits unacknowledged bytes to 1024. */
#define GNSS_CFG_MAX_PAYLOAD 192u
#define UBX_MAX_PAYLOAD  4096u
#define MAX_SATELLITES   64u
#define MAX_RAWX_MEASUREMENTS 96u
#define RAWX_MEAS_SIZE   32u

typedef struct {
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t cno_dbhz;
    int8_t elev_deg;
    int16_t azim_deg;
    uint8_t used;
} satellite_t;

static volatile uint32_t g_ms;
static uint32_t g_sysclk_hz = 8000000u;
static uint32_t g_pclk1_hz = 8000000u;
static uint32_t g_pclk2_hz = 8000000u;
static uint32_t g_tim2_hz = 8000000u;
static volatile uint32_t g_data_ready;
static volatile uint64_t g_data_ready_us;
static volatile uint32_t g_timer_overflows;
static volatile uint64_t g_pps_capture_us;
static volatile uint32_t g_pps_count;
static volatile uint16_t g_pps_gps_week;
static volatile uint32_t g_pps_gps_tow_ms;
static volatile uint32_t g_pps_time_valid;
static volatile uint16_t g_next_gps_week;
static volatile uint32_t g_next_gps_tow_ms;
static volatile uint32_t g_next_gps_time_valid;
static volatile uint8_t g_gnss_rx[GNSS_RX_SIZE];
static volatile uint32_t g_gnss_rx_time_low[GNSS_RX_SIZE];
static volatile uint16_t g_gnss_rx_head;
static volatile uint16_t g_gnss_rx_tail;
static volatile uint8_t g_rtcm_tx[RTCM_TX_SIZE];
static volatile uint16_t g_rtcm_head, g_rtcm_tail;
static volatile uint32_t g_rtcm_ready, g_rtcm_received, g_rtcm_forwarded;
static volatile uint32_t g_rtcm_dropped, g_rtcm_uart_errors;
static uint32_t g_rtcm_f9p_count, g_rtcm_used, g_rtcm_crc_errors, g_rtcm_last_ms;
static uint16_t g_rtcm_station, g_rtcm_type;
static satellite_t g_satellites[MAX_SATELLITES];
static uint32_t g_sat_itow_ms;
static uint16_t g_sat_gps_week;
static uint8_t g_sat_count;
static uint8_t g_sat_time_valid;
static uint32_t g_sat_generation;
/* Keep the receiver's IEEE-754 fields byte-exact. They are rendered as hex on
 * the logger UART, avoiding slow floating-point formatting on Cortex-M3. */
static uint8_t g_rawx_measurements[MAX_RAWX_MEASUREMENTS][RAWX_MEAS_SIZE];
static uint64_t g_rawx_rcv_tow_bits;
static uint64_t g_rawx_rx_timer_us;
static uint16_t g_rawx_gps_week;
static int8_t g_rawx_leap_s;
static uint8_t g_rawx_rec_stat;
static uint8_t g_rawx_count;
static uint8_t g_rawx_total_count;
static uint32_t g_rawx_generation;
/* Volatile diagnostics can be inspected through ST-Link when no serial port
 * is connected: boot 0=start, 1=collecting, 0xE1=MPU not found. */
volatile uint32_t g_debug_boot_status;
volatile uint32_t g_debug_clock_hz;
volatile uint32_t g_debug_sample_count;
volatile uint32_t g_debug_last_dt_ms;
volatile uint32_t g_debug_last_dt_us;
volatile uint32_t g_debug_i2c_errors;
volatile uint32_t g_debug_probe_mask;
volatile uint32_t g_debug_who_am_i;
volatile uint32_t g_debug_interrupt_count;
volatile uint32_t g_debug_interrupt_overruns;
volatile uint32_t g_debug_pps_count;
volatile uint32_t g_debug_gnss_rx_overruns;
volatile uint32_t g_debug_gnss_messages;
volatile uint32_t g_debug_gnss_checksum_errors;
volatile uint32_t g_debug_gnss_ack_count;
volatile uint32_t g_debug_gnss_nak_count;
volatile uint32_t g_debug_nav_pvt_count;
volatile uint32_t g_debug_tim_tp_count;
volatile uint32_t g_debug_rawx_count;
volatile uint32_t g_debug_rawx_truncated;
volatile uint64_t g_debug_nav_rx_timer_us;
volatile uint32_t g_debug_nav_itow_ms;
volatile uint32_t g_debug_nav_fix_type;
volatile uint32_t g_debug_nav_num_sv;
volatile uint32_t g_debug_nav_flags;
volatile uint32_t g_debug_nav_flags2;
volatile uint32_t g_debug_nav_carr_soln;
volatile int32_t g_debug_nav_lon_e7;
volatile int32_t g_debug_nav_lat_e7;
volatile int32_t g_debug_nav_hmsl_mm;
volatile uint32_t g_debug_nav_hacc_mm;
volatile uint32_t g_debug_nav_vacc_mm;
volatile int32_t g_debug_nav_vel_n_mms;
volatile int32_t g_debug_nav_vel_e_mms;
volatile int32_t g_debug_nav_vel_d_mms;
volatile uint32_t g_debug_nav_gspeed_mms;
volatile uint32_t g_debug_nav_sacc_mms;
volatile uint32_t g_debug_nav_pdop_x100;
volatile int16_t g_debug_last_raw[7];
static uint8_t g_mpu_addr = 0x68u;

/* Clock setup is performed after C runtime initialization in board_init(). */
void SystemInit(void) {}
void SysTick_Handler(void) { ++g_ms; }

static uint64_t timer_capture_time(uint16_t capture, uint32_t status,
                                   uint32_t overflow_before)
{
    uint32_t high = overflow_before;
    /* If update is pending and the captured count is in the lower half, the
     * edge occurred after the wrap even if UIF is handled in the same IRQ. */
    if ((status & TIM_UIF) && capture < 0x8000u) ++high;
    return ((uint64_t)high << 16) | capture;
}

static uint64_t timer_now_unlocked(void)
{
    uint32_t high;
    uint16_t low;
    uint32_t status;

    high = g_timer_overflows;
    low = (uint16_t)TIM2_CNT;
    status = TIM2_SR;

    /* If TIM2 wrapped after reading the software high word but before reading
     * CNT, UIF is pending and a low counter value belongs to the next epoch. */
    if ((status & TIM_UIF) && low < 0x8000u) ++high;
    return ((uint64_t)high << 16) | low;
}

static uint64_t timer_now_us(void)
{
    uint64_t now;
    __asm volatile ("cpsid i" ::: "memory");
    now = timer_now_unlocked();
    __asm volatile ("cpsie i" ::: "memory");
    return now;
}

void TIM2_IRQHandler(void)
{
    uint32_t status = TIM2_SR;
    uint32_t high = g_timer_overflows;

    if (status & TIM_CC1IF) {
        g_pps_capture_us = timer_capture_time((uint16_t)TIM2_CCR1, status, high);
        ++g_pps_count;
        ++g_debug_pps_count;
        g_pps_gps_week = g_next_gps_week;
        g_pps_gps_tow_ms = g_next_gps_tow_ms;
        g_pps_time_valid = g_next_gps_time_valid;
        g_next_gps_time_valid = 0u;
    }
    if (status & TIM_CC2IF) {
        g_data_ready_us = timer_capture_time((uint16_t)TIM2_CCR2, status, high);
        if (g_data_ready) ++g_debug_interrupt_overruns;
        g_data_ready = 1u;
        ++g_debug_interrupt_count;
    }
    if (status & TIM_UIF) g_timer_overflows = high + 1u;
    TIM2_SR = ~(status & (TIM_UIF | TIM_CC1IF | TIM_CC2IF));
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1_SR;
    if (status & (USART_RXNE | USART_ORE | 0x07u)) {
        uint8_t byte = (uint8_t)USART1_DR;
        if (status & (USART_ORE | 0x07u)) ++g_rtcm_uart_errors;
        if (!(status & USART_RXNE)) return;
        ++g_rtcm_received;
        uint16_t next = (uint16_t)((g_rtcm_head + 1u) & (RTCM_TX_SIZE - 1u));
        if (!g_rtcm_ready || next == g_rtcm_tail || (status & 0x07u)) {
            ++g_rtcm_dropped;
            return;
        }
        g_rtcm_tx[g_rtcm_head] = byte;
        g_rtcm_head = next;
        USART2_CR1 |= USART_TXE; /* TXEIE */
    }
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2_SR;
    if (status & (USART_RXNE | USART_ORE)) {
        if (status & USART_ORE) ++g_debug_gnss_rx_overruns;
        uint8_t byte = (uint8_t)USART2_DR;
        uint16_t head = g_gnss_rx_head;
        uint16_t next = (uint16_t)((head + 1u) & (GNSS_RX_SIZE - 1u));
        if (next == g_gnss_rx_tail) ++g_debug_gnss_rx_overruns;
        else {
            g_gnss_rx[head] = byte;
            g_gnss_rx_time_low[head] = (uint32_t)timer_now_unlocked();
            g_gnss_rx_head = next;
        }
    }
    if ((USART2_CR1 & USART_TXE) && (USART2_SR & USART_TXE)) {
        if (g_rtcm_tail != g_rtcm_head) {
            USART2_DR = g_rtcm_tx[g_rtcm_tail];
            g_rtcm_tail = (uint16_t)((g_rtcm_tail + 1u) & (RTCM_TX_SIZE - 1u));
            ++g_rtcm_forwarded;
        } else USART2_CR1 &= ~USART_TXE;
    }
}
static uint32_t millis(void) { return g_ms; }

static void delay_ms(uint32_t delay)
{
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < delay) {}
}

static void board_init(void)
{
    /* Blue Pill uses an 8 MHz HSE. Run the core at 72 MHz, APB1 at 36 MHz,
     * and APB2 at 72 MHz. APB1 timer clocks are doubled back to 72 MHz. */
    RCC_CR |= (1u << 16); /* HSEON */
    uint32_t timeout = 1000000u;
    while ((RCC_CR & (1u << 17)) == 0u && timeout) --timeout;
    if (timeout) {
        FLASH_ACR = (1u << 4) | 2u; /* Prefetch, two flash wait states. */
        RCC_CFGR = (RCC_CFGR & ~((7u << 8) | (0xFu << 18))) |
                   (4u << 8) |      /* APB1 = HCLK / 2 */
                   (1u << 16) |     /* PLL source = HSE */
                   (7u << 18);      /* PLL multiplier = 9 */
        RCC_CR |= (1u << 24); /* PLLON */
        timeout = 1000000u;
        while ((RCC_CR & (1u << 25)) == 0u && timeout) --timeout;
        if (timeout) {
            RCC_CFGR = (RCC_CFGR & ~3u) | 2u; /* System clock = PLL. */
            while ((RCC_CFGR & (3u << 2)) != (2u << 2)) {}
            g_sysclk_hz = 72000000u;
            g_pclk1_hz = 36000000u;
            g_pclk2_hz = 72000000u;
            g_tim2_hz = 72000000u;
        }
    }
    g_debug_clock_hz = g_sysclk_hz;

    SYSTICK_LOAD = g_sysclk_hz / 1000u - 1u;
    SYSTICK_VAL = 0u;
    SYSTICK_CTRL = 7u;

    RCC_APB2ENR |= (1u << 4);
    GPIOC_CRH = (GPIOC_CRH & ~(0xFu << 20)) | (0x2u << 20);
    GPIOC_BSRR = (1u << 13);
}

static void led_set(bool on)
{
    if (on) GPIOC_BRR = (1u << 13);
    else GPIOC_BSRR = (1u << 13);
}

static void uart_init(void)
{
    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 14);
    /* PA9 AF push-pull 10 MHz; PA10 floating input. */
    GPIOA_CRH = (GPIOA_CRH & ~((0xFu << 4) | (0xFu << 8))) |
                (0x9u << 4) | (0x4u << 8);
    USART1_BRR = (g_pclk2_hz + DEBUG_UART_BAUD / 2u) / DEBUG_UART_BAUD;
    USART1_CR1 = (1u << 13) | (1u << 5) | (1u << 3) | (1u << 2);
    NVIC_ISER1 = (1u << (37u - 32u));
}

static void gnss_uart_init(uint32_t baud)
{
    RCC_APB2ENR |= (1u << 2); /* GPIOA */
    RCC_APB1ENR |= (1u << 17); /* USART2 */
    /* PA2 USART2_TX AF push-pull 10 MHz, PA3 USART2_RX floating input. */
    GPIOA_CRL = (GPIOA_CRL & ~((0xFu << 8) | (0xFu << 12))) |
                (0x9u << 8) | (0x4u << 12);
    USART2_CR1 = 0u;
    USART2_BRR = (g_pclk1_hz + baud / 2u) / baud;
    USART2_CR1 = (1u << 13) | (1u << 5) | (1u << 3) | (1u << 2);
    NVIC_ISER1 = (1u << (38u - 32u));
}

static void gnss_uart_putc(uint8_t byte)
{
    while ((USART2_SR & USART_TXE) == 0u) {}
    USART2_DR = byte;
}

static void timer_capture_init(void)
{
    RCC_APB2ENR |= (1u << 2); /* GPIOA */
    RCC_APB1ENR |= (1u << 0); /* TIM2 */

    /* PA0=TIM2_CH1 (F9P TP), PA1=TIM2_CH2 (MPU6050 DATA_RDY).
     * Both sources are active-high push-pull; pull-downs define unplugged pins. */
    GPIOA_CRL = (GPIOA_CRL & ~((0xFu << 0) | (0xFu << 4))) |
                (0x8u << 0) | (0x8u << 4);
    GPIOA_BRR = (1u << 0) | (1u << 1);

    TIM2_CR1 = 0u;
    TIM2_PSC = g_tim2_hz / TIMER_TICK_HZ - 1u;
    TIM2_ARR = 0xFFFFu;
    TIM2_CNT = 0u;
    TIM2_CCMR1 = (1u << 0) | (1u << 8); /* CC1/CC2 mapped to TI1/TI2. */
    TIM2_CCER = (1u << 0) | (1u << 4);  /* Rising-edge captures enabled. */
    TIM2_EGR = 1u;
    TIM2_SR = 0u;
    TIM2_DIER = TIM_UIF | TIM_CC1IF | TIM_CC2IF;
    NVIC_ISER0 = (1u << 28);
    TIM2_CR1 = 1u;
}

static void uart_putc(char c)
{
    while ((USART1_SR & USART_TXE) == 0u) {}
    USART1_DR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_u32(uint32_t value)
{
    char digits[10];
    uint32_t count = 0u;
    do {
        digits[count++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value);
    while (count) uart_putc(digits[--count]);
}

static void uart_u64(uint64_t value)
{
    char digits[20];
    uint32_t count = 0u;
    do {
        digits[count++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value);
    while (count) uart_putc(digits[--count]);
}

static void uart_hex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int32_t shift = 28; shift >= 0; shift -= 4)
        uart_putc(hex[(value >> (uint32_t)shift) & 0x0Fu]);
}

static void uart_hex64(uint64_t value)
{
    uart_hex32((uint32_t)(value >> 32));
    uart_hex32((uint32_t)value);
}

static void uart_i32(int32_t value)
{
    int64_t wide = value;
    if (wide < 0) { uart_putc('-'); wide = -wide; }
    uart_u64((uint64_t)wide);
}

static void uart_i16(int16_t value)
{
    int32_t wide = value;
    if (wide < 0) { uart_putc('-'); wide = -wide; }
    uart_u32((uint32_t)wide);
}

static void uart_flush(void)
{
    while ((USART1_SR & USART_TC) == 0u) {}
}

static void put_le32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static uint16_t get_le16(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t get_le32(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static uint64_t get_le64(const uint8_t *src)
{
    return (uint64_t)get_le32(src) | ((uint64_t)get_le32(src + 4u) << 32);
}

static void gnss_ubx_send(uint8_t msg_class, uint8_t msg_id,
                          const uint8_t *payload, uint16_t length)
{
    uint8_t ck_a = 0u;
    uint8_t ck_b = 0u;
    uint8_t header[4] = {
        msg_class, msg_id, (uint8_t)length, (uint8_t)(length >> 8)
    };

    gnss_uart_putc(0xB5u);
    gnss_uart_putc(0x62u);
    for (uint32_t i = 0u; i < 4u; ++i) {
        ck_a = (uint8_t)(ck_a + header[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
        gnss_uart_putc(header[i]);
    }
    for (uint32_t i = 0u; i < length; ++i) {
        ck_a = (uint8_t)(ck_a + payload[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
        gnss_uart_putc(payload[i]);
    }
    gnss_uart_putc(ck_a);
    gnss_uart_putc(ck_b);
    while ((USART2_SR & USART_TC) == 0u) {}
}

typedef struct {
    uint32_t key;
    uint32_t value;
    uint8_t size;
} gnss_cfg_item_t;

static void gnss_valset(const gnss_cfg_item_t *items, uint32_t count)
{
    uint8_t payload[GNSS_CFG_MAX_PAYLOAD];
    uint16_t length = 4u;
    payload[0] = 0u; /* Message version. */
    payload[1] = 1u; /* RAM layer: reapply safely on every MCU boot. */
    payload[2] = 0u; /* No transaction. */
    payload[3] = 0u;

    for (uint32_t i = 0u; i < count; ++i) {
        if ((uint32_t)length + 4u + items[i].size > sizeof payload) return;
        put_le32(&payload[length], items[i].key);
        length += 4u;
        for (uint32_t byte = 0u; byte < items[i].size; ++byte)
            payload[length++] = (uint8_t)(items[i].value >> (8u * byte));
    }
    gnss_ubx_send(0x06u, 0x8Au, payload, length); /* UBX-CFG-VALSET */
}

static void gnss_send_port_config(void)
{
    static const gnss_cfg_item_t config[] = {
        {0x40520001u, GNSS_UART_BAUD, 4u}, /* CFG-UART1-BAUDRATE */
        {0x10730001u, 1u, 1u}, /* UART1 input UBX */
        {0x10730002u, 0u, 1u}, /* UART1 input NMEA */
        {0x10730004u, 1u, 1u}, /* UART1 input RTCM3 */
        {0x10740001u, 1u, 1u}, /* UART1 output UBX */
        {0x10740002u, 0u, 1u}, /* UART1 output NMEA */
        {0x10740004u, 0u, 1u}, /* UART1 output RTCM3 */
    };
    gnss_valset(config, sizeof config / sizeof config[0]);
}

static void gnss_send_pubx_port_fallback(void)
{
    /* Recovery path for a receiver whose UART accepts NMEA but has UBX input
     * disabled. PUBX,41 first enables UBX-only at 115200; the following
     * UBX-CFG-VALSET then enables RTCM3 input as well. */
    static const char body[] = "PUBX,41,1,0001,0001,115200,0";
    static const char hex[] = "0123456789ABCDEF";
    uint8_t checksum = 0u;
    gnss_uart_putc('$');
    for (uint32_t i = 0u; body[i] != '\0'; ++i) {
        checksum ^= (uint8_t)body[i];
        gnss_uart_putc((uint8_t)body[i]);
    }
    gnss_uart_putc('*');
    gnss_uart_putc((uint8_t)hex[checksum >> 4]);
    gnss_uart_putc((uint8_t)hex[checksum & 0x0Fu]);
    gnss_uart_putc('\r');
    gnss_uart_putc('\n');
    while ((USART2_SR & USART_TC) == 0u) {}
}

static void gnss_send_navigation_config(void)
{
    static const gnss_cfg_item_t config[] = {
        {0x30210001u, 100u, 2u},     /* CFG-RATE-MEAS: 100 ms = 10 Hz */
        {0x30210002u, 1u, 2u},       /* CFG-RATE-NAV: every measurement */
        {0x20210003u, 1u, 1u},       /* CFG-RATE-TIMEREF: GPS */
        {0x20910007u, 10u, 1u},      /* UBX-NAV-PVT UART1: 1 Hz */
        {0x20910016u, 10u, 1u},      /* UBX-NAV-SAT UART1: 1 Hz */
        {0x2091017Eu, 1u, 1u},       /* UBX-TIM-TP UART1: one per 1 Hz pulse */
        {0x20050023u, 0u, 1u},       /* TP definition: period */
        {0x20050030u, 1u, 1u},       /* TP pulse definition: length */
        {0x40050002u, 1000000u, 4u}, /* TP period before lock: 1 s */
        {0x40050003u, 1000000u, 4u}, /* TP period after lock: 1 s */
        {0x40050004u, 100000u, 4u},  /* TP high time before lock: 100 ms */
        {0x40050005u, 100000u, 4u},  /* TP high time after lock: 100 ms */
        {0x10050007u, 1u, 1u},       /* TP1 enable */
        {0x10050008u, 1u, 1u},       /* Synchronize TP1 to GNSS */
        {0x10050009u, 1u, 1u},       /* Use locked TP settings */
        {0x1005000Au, 1u, 1u},       /* Align TP1 to time of week */
        {0x1005000Bu, 1u, 1u},       /* Rising-edge polarity */
        {0x2005000Cu, 1u, 1u},       /* TP1 time grid: GPS */
    };
    gnss_valset(config, sizeof config / sizeof config[0]);
}

static void gnss_send_rawx_config(void)
{
    /* Keep RAWX in its own VALSET transaction. Some F9P firmware revisions
     * reject an unrelated TP key and atomically NAK a mixed transaction. */
    static const gnss_cfg_item_t config[] = {
        {0x209102A5u, 10u, 1u},      /* UBX-RXM-RAWX UART1: 1 Hz */
    };
    gnss_valset(config, sizeof config / sizeof config[0]);
}

static void gnss_configure(void)
{
    /* C099 normally ships at 460800 baud; bare/default F9P UART1 is 38400.
     * Try both, then finish at 115200. A receiver already at 115200 is also
     * handled. The final VALSET is RAM-only to avoid flash wear. */
    static const uint32_t candidates[] = {460800u, 38400u, GNSS_UART_BAUD};
    for (uint32_t i = 0u; i < sizeof candidates / sizeof candidates[0]; ++i) {
        gnss_uart_init(candidates[i]);
        delay_ms(20u);
        gnss_send_port_config();
        gnss_send_pubx_port_fallback();
        delay_ms(100u);
    }
    gnss_uart_init(GNSS_UART_BAUD);
    __asm volatile ("cpsid i" ::: "memory");
    g_gnss_rx_head = 0u;
    g_gnss_rx_tail = 0u;
    __asm volatile ("cpsie i" ::: "memory");
    gnss_send_navigation_config();
    delay_ms(20u);
    gnss_send_rawx_config();
    /* Separate transaction: diagnostic keys must not cause a RAWX config NAK. */
    static const gnss_cfg_item_t rtcm_status[] = {
        {0x20910269u, 1u, 1u}, /* UBX-RXM-RTCM UART1: every input message */
    };
    gnss_valset(rtcm_status, sizeof rtcm_status / sizeof rtcm_status[0]);
    delay_ms(100u);
}

static bool gnss_rx_pop(uint8_t *byte, uint32_t *rx_time_low)
{
    uint16_t tail = g_gnss_rx_tail;
    if (tail == g_gnss_rx_head) return false;
    *byte = g_gnss_rx[tail];
    *rx_time_low = g_gnss_rx_time_low[tail];
    g_gnss_rx_tail = (uint16_t)((tail + 1u) & (GNSS_RX_SIZE - 1u));
    return true;
}

static void gnss_dispatch(uint8_t msg_class, uint8_t msg_id,
                          const uint8_t *payload, uint16_t length,
                          uint64_t rx_timer_us)
{
    ++g_debug_gnss_messages;
    if (msg_class == 0x05u && length == 2u) {
        if (msg_id == 0x01u) ++g_debug_gnss_ack_count;
        else if (msg_id == 0x00u) ++g_debug_gnss_nak_count;
    } else if (msg_class == 0x02u && msg_id == 0x32u && length == 8u) {
        ++g_rtcm_f9p_count;
        g_rtcm_last_ms = millis();
        if (payload[1] & 1u) ++g_rtcm_crc_errors;
        else {
            g_rtcm_station = get_le16(&payload[4]);
            g_rtcm_type = get_le16(&payload[6]);
            if (payload[0] >= 2u && ((payload[1] >> 1) & 3u) == 2u) ++g_rtcm_used;
        }
    } else if (msg_class == 0x01u && msg_id == 0x07u && length >= 92u) {
        uint8_t flags = payload[21];
        g_debug_nav_rx_timer_us = rx_timer_us;
        g_debug_nav_itow_ms = get_le32(&payload[0]);
        g_debug_nav_fix_type = payload[20];
        g_debug_nav_flags = flags;
        g_debug_nav_flags2 = payload[22];
        g_debug_nav_carr_soln = (flags >> 6) & 0x03u;
        g_debug_nav_num_sv = payload[23];
        g_debug_nav_lon_e7 = (int32_t)get_le32(&payload[24]);
        g_debug_nav_lat_e7 = (int32_t)get_le32(&payload[28]);
        g_debug_nav_hmsl_mm = (int32_t)get_le32(&payload[36]);
        g_debug_nav_hacc_mm = get_le32(&payload[40]);
        g_debug_nav_vacc_mm = get_le32(&payload[44]);
        g_debug_nav_vel_n_mms = (int32_t)get_le32(&payload[48]);
        g_debug_nav_vel_e_mms = (int32_t)get_le32(&payload[52]);
        g_debug_nav_vel_d_mms = (int32_t)get_le32(&payload[56]);
        g_debug_nav_gspeed_mms = get_le32(&payload[60]);
        g_debug_nav_sacc_mms = get_le32(&payload[68]);
        g_debug_nav_pdop_x100 = get_le16(&payload[76]);
        ++g_debug_nav_pvt_count;
    } else if (msg_class == 0x01u && msg_id == 0x35u && length >= 8u) {
        uint8_t count = payload[5];
        uint8_t available = (uint8_t)((length - 8u) / 12u);
        if (count > available) count = available;
        if (count > MAX_SATELLITES) count = MAX_SATELLITES;
        for (uint8_t i = 0u; i < count; ++i) {
            uint16_t offset = (uint16_t)(8u + (uint16_t)i * 12u);
            uint32_t flags = get_le32(&payload[offset + 8u]);
            g_satellites[i].gnss_id = payload[offset];
            g_satellites[i].sv_id = payload[offset + 1u];
            g_satellites[i].cno_dbhz = payload[offset + 2u];
            g_satellites[i].elev_deg = (int8_t)payload[offset + 3u];
            g_satellites[i].azim_deg = (int16_t)get_le16(&payload[offset + 4u]);
            g_satellites[i].used = (flags & (1u << 3)) ? 1u : 0u;
        }
        g_sat_itow_ms = get_le32(&payload[0]);
        __asm volatile ("cpsid i" ::: "memory");
        g_sat_gps_week = g_pps_gps_week;
        g_sat_time_valid = (uint8_t)g_pps_time_valid;
        __asm volatile ("cpsie i" ::: "memory");
        g_sat_count = count;
        ++g_sat_generation;
    } else if (msg_class == 0x02u && msg_id == 0x15u && length >= 16u &&
               payload[13] == 1u) {
        uint8_t total = payload[11];
        uint16_t available = (uint16_t)((length - 16u) / RAWX_MEAS_SIZE);
        if ((uint16_t)total > available) total = (uint8_t)available;
        uint8_t stored = total;
        if (stored > MAX_RAWX_MEASUREMENTS) {
            stored = MAX_RAWX_MEASUREMENTS;
            ++g_debug_rawx_truncated;
        }
        g_rawx_rcv_tow_bits = get_le64(&payload[0]);
        g_rawx_gps_week = get_le16(&payload[8]);
        g_rawx_leap_s = (int8_t)payload[10];
        g_rawx_rec_stat = payload[12];
        g_rawx_rx_timer_us = rx_timer_us;
        for (uint8_t i = 0u; i < stored; ++i) {
            uint16_t offset = (uint16_t)(16u + (uint16_t)i * RAWX_MEAS_SIZE);
            for (uint8_t byte = 0u; byte < RAWX_MEAS_SIZE; ++byte)
                g_rawx_measurements[i][byte] = payload[offset + byte];
        }
        g_rawx_total_count = total;
        g_rawx_count = stored;
        ++g_debug_rawx_count;
        ++g_rawx_generation;
    } else if (msg_class == 0x0Du && msg_id == 0x01u && length == 16u) {
        uint32_t tow_ms = get_le32(&payload[0]);
        uint16_t week = get_le16(&payload[12]);
        uint8_t flags = payload[14];
        uint8_t reference = payload[15] & 0x0Fu;
        __asm volatile ("cpsid i" ::: "memory");
        g_next_gps_tow_ms = tow_ms;
        g_next_gps_week = week;
        /* TP is configured on the GPS grid. Reject UTC/unknown references. */
        g_next_gps_time_valid = ((flags & 1u) == 0u && reference == 0u &&
                                  week != 0u) ? 1u : 0u;
        __asm volatile ("cpsie i" ::: "memory");
        ++g_debug_tim_tp_count;
    }
}

static void gnss_process(void)
{
    static uint8_t state;
    static uint8_t msg_class;
    static uint8_t msg_id;
    static uint16_t length;
    static uint16_t index;
    static uint8_t ck_a;
    static uint8_t ck_b;
    static uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t byte;
    uint32_t rx_time_low;
    uint64_t now = timer_now_us();
    uint64_t rx_time_base = now & ~0xFFFFFFFFull;

    while (gnss_rx_pop(&byte, &rx_time_low)) {
        uint64_t byte_rx_us = rx_time_base | rx_time_low;
        /* The receive ring can hold far less than one 32-bit timer period
         * (about 71 minutes), so a future-looking low word is from the
         * immediately preceding period. */
        if (byte_rx_us > now) byte_rx_us -= 0x100000000ull;
        switch (state) {
        case 0u:
            if (byte == 0xB5u) state = 1u;
            break;
        case 1u:
            state = (byte == 0x62u) ? 2u : (byte == 0xB5u ? 1u : 0u);
            break;
        case 2u:
            msg_class = byte; ck_a = byte; ck_b = ck_a; state = 3u; break;
        case 3u:
            msg_id = byte; ck_a += byte; ck_b += ck_a; state = 4u; break;
        case 4u:
            length = byte; ck_a += byte; ck_b += ck_a; state = 5u; break;
        case 5u:
            length |= (uint16_t)byte << 8; ck_a += byte; ck_b += ck_a;
            index = 0u;
            state = length <= UBX_MAX_PAYLOAD ? (length ? 6u : 7u) : 0u;
            break;
        case 6u:
            payload[index++] = byte; ck_a += byte; ck_b += ck_a;
            if (index == length) state = 7u;
            break;
        case 7u:
            if (byte == ck_a) state = 8u;
            else { ++g_debug_gnss_checksum_errors; state = 0u; }
            break;
        default: /* checksum B */
            if (byte == ck_b) {
                gnss_dispatch(msg_class, msg_id, payload, length, byte_rx_us);
            }
            else ++g_debug_gnss_checksum_errors;
            state = 0u;
            break;
        }
    }
}

static void i2c_init(void)
{
    RCC_APB2ENR |= (1u << 0) | (1u << 3);
    RCC_APB1ENR |= (1u << 21);

    /* Recover a slave left mid-transfer when STM32 is reset during logging.
     * First use PB6/PB7 as open-drain GPIO, clock up to 9 remaining bits, and
     * synthesize STOP before handing the pins to I2C1. */
    GPIOB_CRL = (GPIOB_CRL & ~((0xFu << 24) | (0xFu << 28))) |
                (0x5u << 24) | (0x5u << 28);
    GPIOB_BSRR = (1u << 6) | (1u << 7);
    delay_ms(1u);
    for (uint32_t pulse = 0u; pulse < 9u; ++pulse) {
        GPIOB_BRR = (1u << 6);
        delay_ms(1u);
        GPIOB_BSRR = (1u << 6);
        delay_ms(1u);
    }
    GPIOB_BRR = (1u << 7);
    delay_ms(1u);
    GPIOB_BSRR = (1u << 6);
    delay_ms(1u);
    GPIOB_BSRR = (1u << 7);
    delay_ms(1u);

    /* PB6/PB7 AF open-drain 10 MHz; GY-521 supplies pull-ups. */
    GPIOB_CRL = (GPIOB_CRL & ~((0xFu << 24) | (0xFu << 28))) |
                (0xDu << 24) | (0xDu << 28);
    I2C1_CR1 = I2C_SWRST;
    I2C1_CR1 = 0u;
    I2C1_CR2 = g_pclk1_hz / 1000000u;
    I2C1_CCR = g_pclk1_hz / 200000u; /* 100 kHz standard mode. */
    I2C1_TRISE = g_pclk1_hz / 1000000u + 1u;
    I2C1_CR1 = I2C_PE;
}

static void i2c_abort(void)
{
    I2C1_CR1 |= I2C_STOP;
    I2C1_SR1 &= ~I2C_ERRORS;
    I2C1_CR1 |= I2C_SWRST;
    I2C1_CR1 = 0u;
    i2c_init();
}

static bool wait_sr1(uint32_t mask)
{
    uint32_t start = millis();
    while ((I2C1_SR1 & mask) == 0u) {
        if ((I2C1_SR1 & I2C_ERRORS) ||
            (uint32_t)(millis() - start) >= I2C_TIMEOUT_MS) return false;
    }
    return true;
}

static bool wait_idle(void)
{
    uint32_t start = millis();
    while (I2C1_SR2 & I2C_BUSY) {
        if ((uint32_t)(millis() - start) >= I2C_TIMEOUT_MS) return false;
    }
    return true;
}

static void clear_addr(void)
{
    volatile uint32_t dummy = I2C1_SR1;
    dummy = I2C1_SR2;
    (void)dummy;
}

static bool send_address(uint8_t address, bool read)
{
    I2C1_CR1 |= I2C_START;
    if (!wait_sr1(I2C_SB)) return false;
    I2C1_DR = ((uint32_t)address << 1) | (read ? 1u : 0u);
    return wait_sr1(I2C_ADDR);
}

static bool write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    if (!wait_idle() || !send_address(address, false)) goto fail;
    clear_addr();
    if (!wait_sr1(I2C_TXE)) goto fail;
    I2C1_DR = reg;
    if (!wait_sr1(I2C_TXE)) goto fail;
    I2C1_DR = value;
    if (!wait_sr1(I2C_BTF)) goto fail;
    I2C1_CR1 |= I2C_STOP;
    return true;
fail:
    i2c_abort();
    return false;
}

/* Supports one-byte reads and the 14-byte sensor burst read. */
static bool read_regs(uint8_t address, uint8_t reg, uint8_t *data, uint32_t count)
{
    if (count == 0u || count == 2u) return false;
    I2C1_CR1 |= I2C_ACK;
    if (!wait_idle() || !send_address(address, false)) goto fail;
    clear_addr();
    if (!wait_sr1(I2C_TXE)) goto fail;
    I2C1_DR = reg;
    if (!wait_sr1(I2C_BTF)) goto fail;
    /* A STOP between the register-pointer write and the read is accepted by
     * MPU6050 and avoids repeated-START edge cases seen on some Blue Pill and
     * GY-521 clone combinations. */
    I2C1_CR1 |= I2C_STOP;
    if (!wait_idle() || !send_address(address, true)) goto fail;

    if (count == 1u) {
        __asm volatile ("cpsid i" ::: "memory");
        I2C1_CR1 &= ~I2C_ACK;
        clear_addr();
        I2C1_CR1 |= I2C_STOP;
        __asm volatile ("cpsie i" ::: "memory");
        if (!wait_sr1(I2C_RXNE)) goto fail;
        data[0] = (uint8_t)I2C1_DR;
        return true;
    }

    clear_addr();
    while (count > 3u) {
        if (!wait_sr1(I2C_RXNE)) goto fail;
        *data++ = (uint8_t)I2C1_DR;
        --count;
    }
    if (!wait_sr1(I2C_BTF)) goto fail;
    I2C1_CR1 &= ~I2C_ACK;
    *data++ = (uint8_t)I2C1_DR;
    --count;
    I2C1_CR1 |= I2C_STOP;
    *data++ = (uint8_t)I2C1_DR;
    --count;
    if (!wait_sr1(I2C_RXNE)) goto fail;
    *data = (uint8_t)I2C1_DR;
    return true;
fail:
    i2c_abort();
    return false;
}

static bool mpu_read8(uint8_t reg, uint8_t *value)
{
    return read_regs(g_mpu_addr, reg, value, 1u);
}

static bool i2c_probe(uint8_t address)
{
    if (!wait_idle() || !send_address(address, false)) {
        i2c_abort();
        return false;
    }
    clear_addr();
    I2C1_CR1 |= I2C_STOP;
    delay_ms(1u);
    return true;
}

static bool mpu_find(void)
{
    uint8_t who = 0u;
    g_mpu_addr = 0x68u;
    if (i2c_probe(0x68u)) g_debug_probe_mask |= 1u;
    if (mpu_read8(MPU_WHO_AM_I, &who)) {
        g_debug_who_am_i = who;
        if (who == 0x68u) return true;
    }
    g_mpu_addr = 0x69u;
    if (i2c_probe(0x69u)) g_debug_probe_mask |= 2u;
    if (mpu_read8(MPU_WHO_AM_I, &who)) {
        g_debug_who_am_i = who;
        return who == 0x68u;
    }
    return false;
}

static bool mpu_init(void)
{
    if (!mpu_find()) return false;
    if (!write_reg(g_mpu_addr, MPU_PWR_MGMT_1, 0x80u)) return false;
    delay_ms(100u);
    return write_reg(g_mpu_addr, MPU_PWR_MGMT_1, 0x01u) &&
           write_reg(g_mpu_addr, MPU_PWR_MGMT_2, 0x00u) &&
           write_reg(g_mpu_addr, MPU_USER_CTRL, 0x00u) &&
           write_reg(g_mpu_addr, MPU_CONFIG, 0x03u) && /* 42/44 Hz DLPF */
           write_reg(g_mpu_addr, MPU_SMPLRT_DIV, 9u) && /* 100 Hz */
           write_reg(g_mpu_addr, MPU_GYRO_CONFIG, 0x00u) && /* +/-250 dps */
           write_reg(g_mpu_addr, MPU_ACCEL_CONFIG, 0x00u) && /* +/-2 g */
           /* Active-high push-pull, latched until INT_STATUS is read. A
            * latched level is much harder to miss than the short default
            * DATA_RDY pulse during long-duration logging. */
           write_reg(g_mpu_addr, MPU_INT_PIN_CFG, 0x20u) &&
           write_reg(g_mpu_addr, MPU_INT_ENABLE, 0x01u);
}

static int16_t i16be(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static void print_header(void)
{
    uart_puts("# mpu6050_f9p_navigation_protocol_v3\r\n");
    uart_puts("# timer=tim2_1mhz_48bit_extended\r\n");
    uart_puts("# pps=f9p_tp_pa0_tim2_ch1_rising\r\n");
    uart_puts("# trigger=mpu6050_data_ready_pa1_tim2_ch2_rising\r\n");
    uart_puts("# logger_uart=usart1_pa9_460800\r\n");
    uart_puts("# f9p_uart=usart2_pa2_pa3_115200_ubx_rtcm3in\r\n");
    uart_puts("# f9p_output=nav_pvt_1hz_nav_sat_1hz_rxm_rawx_1hz_tim_tp_1hz_gps_grid\r\n");
    uart_puts("# sample_rate_hz=100\r\n");
    uart_puts("# accel_range_g=2\r\n");
    uart_puts("# accel_scale_lsb_per_g=16384\r\n");
    uart_puts("# gyro_range_dps=250\r\n");
    uart_puts("# gyro_scale_lsb_per_dps=131\r\n");
    uart_puts("# temperature_degC=temp_raw/340+36.53\r\n");
    uart_puts("# mpu_i2c_address=0x6");
    uart_putc(g_mpu_addr == 0x68u ? '8' : '9');
    uart_puts("\r\n");
    uart_puts("# IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw\r\n");
    uart_puts("# GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100\r\n");
    uart_puts("# SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used\r\n");
    uart_puts("# RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us\r\n");
    uart_puts("# RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat\r\n");
    uart_puts("# RAWX_END,num_meas\r\n");
    uart_puts("# rawx_float_hex=ieee754_bits_exact\r\n");
}

static uint32_t gps_time_from_local(uint64_t local_us, uint16_t *week,
                                    uint64_t *tow_us)
{
    uint64_t pps_us;
    uint32_t pps_tow_ms;
    uint16_t pps_week;
    uint32_t valid;

    __asm volatile ("cpsid i" ::: "memory");
    pps_us = g_pps_capture_us;
    pps_tow_ms = g_pps_gps_tow_ms;
    pps_week = g_pps_gps_week;
    valid = g_pps_time_valid;
    __asm volatile ("cpsie i" ::: "memory");

    if (!valid) {
        *week = 0u;
        *tow_us = 0u;
        return 0u;
    }

    uint64_t gps_us = (uint64_t)pps_tow_ms * 1000u;
    if (local_us >= pps_us) {
        gps_us += local_us - pps_us;
        while (gps_us >= GPS_WEEK_US) {
            gps_us -= GPS_WEEK_US;
            ++pps_week;
        }
    } else {
        uint64_t before_pps = pps_us - local_us;
        while (before_pps > gps_us) {
            before_pps -= gps_us + 1u;
            gps_us = GPS_WEEK_US - 1u;
            --pps_week;
        }
        gps_us -= before_pps;
    }

    *week = pps_week;
    *tow_us = gps_us;
    return 1u;
}

static void print_sample(uint32_t sample, uint64_t now,
                         const uint8_t data[14])
{
    int16_t values[7] = {
        i16be(data[0], data[1]), i16be(data[2], data[3]),
        i16be(data[4], data[5]), i16be(data[6], data[7]),
        i16be(data[8], data[9]), i16be(data[10], data[11]),
        i16be(data[12], data[13])
    };
    uint16_t gps_week;
    uint64_t gps_tow_us;
    uint32_t time_valid = gps_time_from_local(now, &gps_week, &gps_tow_us);

    uart_puts("IMU,");
    uart_u32(sample); uart_putc(',');
    uart_u32(gps_week); uart_putc(',');
    uart_u64(gps_tow_us); uart_putc(',');
    uart_u32(time_valid); uart_putc(',');
    uart_u64(now);
    for (uint32_t i = 0; i < 7u; ++i) {
        g_debug_last_raw[i] = values[i];
        uart_putc(',');
        uart_i16(values[i]);
    }
    uart_puts("\r\n");
}

static void print_gnss(uint16_t week, uint32_t time_valid)
{
    uart_puts("GNSS,"); uart_u32(week);
    uart_putc(','); uart_u32(g_debug_nav_itow_ms);
    uart_putc(','); uart_u32(time_valid);
    uart_putc(','); uart_u64(g_debug_nav_rx_timer_us);
    uart_putc(','); uart_u32(g_debug_nav_fix_type);
    uart_putc(','); uart_u32(g_debug_nav_num_sv);
    uart_putc(','); uart_u32(g_debug_nav_flags);
    uart_putc(','); uart_u32(g_debug_nav_flags2);
    uart_putc(','); uart_u32(g_debug_nav_carr_soln);
    uart_putc(','); uart_i32(g_debug_nav_lat_e7);
    uart_putc(','); uart_i32(g_debug_nav_lon_e7);
    uart_putc(','); uart_i32(g_debug_nav_hmsl_mm);
    uart_putc(','); uart_u32(g_debug_nav_hacc_mm);
    uart_putc(','); uart_u32(g_debug_nav_vacc_mm);
    uart_putc(','); uart_i32(g_debug_nav_vel_n_mms);
    uart_putc(','); uart_i32(g_debug_nav_vel_e_mms);
    uart_putc(','); uart_i32(g_debug_nav_vel_d_mms);
    uart_putc(','); uart_u32(g_debug_nav_gspeed_mms);
    uart_putc(','); uart_u32(g_debug_nav_sacc_mms);
    uart_putc(','); uart_u32(g_debug_nav_pdop_x100);
    uart_puts("\r\n");
}

static void print_satellite(const satellite_t *sat)
{
    uart_puts("SAT,"); uart_u32(g_sat_gps_week);
    uart_putc(','); uart_u32(g_sat_itow_ms);
    uart_putc(','); uart_u32(g_sat_time_valid);
    uart_putc(','); uart_u32(sat->gnss_id);
    uart_putc(','); uart_u32(sat->sv_id);
    uart_putc(','); uart_u32(sat->cno_dbhz);
    uart_putc(','); uart_i32(sat->elev_deg);
    uart_putc(','); uart_i32(sat->azim_deg);
    uart_putc(','); uart_u32(sat->used);
    uart_puts("\r\n");
}

static void print_sat_end(void)
{
    uart_puts("SAT_END,"); uart_u32(g_sat_gps_week);
    uart_putc(','); uart_u32(g_sat_itow_ms);
    uart_putc(','); uart_u32(g_sat_time_valid);
    uart_putc(','); uart_u32(g_sat_count);
    uart_puts("\r\n");
}

static void print_rawx_header(void)
{
    uart_puts("RAWX,"); uart_u32(g_rawx_gps_week);
    uart_putc(','); uart_hex64(g_rawx_rcv_tow_bits);
    uart_putc(','); uart_i32(g_rawx_leap_s);
    uart_putc(','); uart_u32(g_rawx_rec_stat);
    uart_putc(','); uart_u32(g_rawx_count);
    uart_putc(','); uart_u32(g_rawx_total_count);
    uart_putc(','); uart_u64(g_rawx_rx_timer_us);
    uart_puts("\r\n");
}

static void print_rawx_measurement(const uint8_t *measurement)
{
    uart_puts("RAWX_MEAS,"); uart_u32(measurement[20]);
    uart_putc(','); uart_u32(measurement[21]);
    uart_putc(','); uart_u32(measurement[22]);
    uart_putc(','); uart_u32(measurement[23]);
    uart_putc(','); uart_hex64(get_le64(&measurement[0]));
    uart_putc(','); uart_hex64(get_le64(&measurement[8]));
    uart_putc(','); uart_hex32(get_le32(&measurement[16]));
    uart_putc(','); uart_u32(get_le16(&measurement[24]));
    uart_putc(','); uart_u32(measurement[26]);
    uart_putc(','); uart_u32(measurement[27] & 0x0Fu);
    uart_putc(','); uart_u32(measurement[28] & 0x0Fu);
    uart_putc(','); uart_u32(measurement[29] & 0x0Fu);
    uart_putc(','); uart_u32(measurement[30]);
    uart_puts("\r\n");
}

static void print_rawx_end(void)
{
    uart_puts("RAWX_END,"); uart_u32(g_rawx_count); uart_puts("\r\n");
}

static void print_sync(uint32_t pps_count, uint64_t capture_us,
                       uint16_t week, uint32_t tow_ms, uint32_t time_valid)
{
    uart_puts("# sync,pps="); uart_u32(pps_count);
    uart_puts(",timer_us="); uart_u64(capture_us);
    uart_puts(",gps_week="); uart_u32(week);
    uart_puts(",gps_tow_ms="); uart_u32(tow_ms);
    uart_puts(",time_valid="); uart_u32(time_valid);
    uart_puts(",pvt_itow_ms="); uart_u32(g_debug_nav_itow_ms);
    uart_puts(",fix="); uart_u32(g_debug_nav_fix_type);
    uart_puts(",num_sv="); uart_u32(g_debug_nav_num_sv);
    uart_puts(",lat_e7="); uart_i32(g_debug_nav_lat_e7);
    uart_puts(",lon_e7="); uart_i32(g_debug_nav_lon_e7);
    uart_puts(",hmsl_mm="); uart_i32(g_debug_nav_hmsl_mm);
    uart_puts(",vel_n_mms="); uart_i32(g_debug_nav_vel_n_mms);
    uart_puts(",vel_e_mms="); uart_i32(g_debug_nav_vel_e_mms);
    uart_puts(",vel_d_mms="); uart_i32(g_debug_nav_vel_d_mms);
    uart_puts("\r\n");
}

static void print_rtcm_status(void)
{
    uint32_t received, forwarded, dropped, uart_errors;
    __asm volatile ("cpsid i" ::: "memory");
    received = g_rtcm_received;
    forwarded = g_rtcm_forwarded;
    dropped = g_rtcm_dropped;
    uart_errors = g_rtcm_uart_errors;
    __asm volatile ("cpsie i" ::: "memory");
    uart_puts("#RTCM,"); uart_u32(millis());
    uart_putc(','); uart_u32(g_rtcm_ready);
    uart_putc(','); uart_u32(received);
    uart_putc(','); uart_u32(forwarded);
    uart_putc(','); uart_u32(dropped);
    uart_putc(','); uart_u32(uart_errors);
    uart_putc(','); uart_u32(g_debug_gnss_rx_overruns);
    uart_putc(','); uart_u32(g_rtcm_f9p_count);
    uart_putc(','); uart_u32(g_rtcm_used);
    uart_putc(','); uart_u32(g_rtcm_crc_errors);
    uart_putc(','); uart_u32(g_rtcm_station);
    uart_putc(','); uart_u32(g_rtcm_type);
    uart_putc(','); uart_u32(g_rtcm_f9p_count ? millis() - g_rtcm_last_ms : UINT32_MAX);
    uart_puts("\r\n");
}

int main(void)
{
    uint8_t status = 0u;
    uint8_t frame[14];
    uint32_t sample = 0u;
    uint64_t previous_us = 0u;
    uint32_t errors = 0u;
    uint32_t printed_pps = 0u;
    uint32_t printed_nav_pvt = 0u;
    uint32_t printing_sat_generation = 0u;
    uint8_t printing_sat_index = 0u;
    uint32_t printing_rawx_generation = 0u;
    uint8_t printing_rawx_index = 0u;
    uint8_t printing_rawx_state = 0u;
    uint8_t rawx_config_attempts = 0u;
    uint32_t next_rawx_config_ms = 0u;
    uint32_t next_rtcm_status_ms = 0u;

    board_init();
    uart_init();
    uart_puts("# booting\r\n");
    gnss_configure();
    i2c_init();
    delay_ms(200u);

    if (!mpu_init()) {
        g_debug_boot_status = 0xE1u;
        uart_puts("# ERROR: MPU6050 not found; check 3.3V/GND/PB6/PB7\r\n");
        uart_flush();
        for (;;) led_set(((millis() / 150u) & 1u) != 0u);
    }

    /* INT is configured as active-high and latched until INT_STATUS is read.
     * Start TIM2 first, clear a possibly stale high level, then the following
     * PA1 rising edge is captured in hardware. */
    timer_capture_init();
    /* Bytes received while TIM2 was still stopped have no meaningful local
     * receive timestamp. Discard that startup fragment and resynchronize on
     * the next complete UBX frame. */
    __asm volatile ("cpsid i" ::: "memory");
    g_gnss_rx_tail = g_gnss_rx_head;
    __asm volatile ("cpsie i" ::: "memory");
    (void)mpu_read8(MPU_INT_STATUS, &status);
    __asm volatile ("cpsid i" ::: "memory");
    g_data_ready = 0u;
    __asm volatile ("cpsie i" ::: "memory");

    delay_ms(20u);
    print_header();
    g_debug_boot_status = 1u;

    for (;;) {
        gnss_process();

        /* UART routing and receiver startup time vary across C099 revisions.
         * Retry only until the first valid RAWX frame, then remain silent. */
        if (!g_rtcm_ready && g_debug_rawx_count == 0u && rawx_config_attempts < 5u &&
            (int32_t)(millis() - next_rawx_config_ms) >= 0) {
            gnss_send_rawx_config();
            ++rawx_config_attempts;
            next_rawx_config_ms = millis() + 1000u;
        }
        /* Finish startup retries before opening the bridge. No synchronous
         * UBX writes may interleave with RTCM bytes after this point. */
        if (g_debug_rawx_count != 0u || rawx_config_attempts >= 5u) g_rtcm_ready = 1u;
        if ((int32_t)(millis() - next_rtcm_status_ms) >= 0) {
            print_rtcm_status();
            next_rtcm_status_ms = millis() + 100u;
        }

        if (g_debug_nav_pvt_count != printed_nav_pvt) {
            uint16_t week;
            uint32_t valid;
            __asm volatile ("cpsid i" ::: "memory");
            week = g_pps_gps_week;
            valid = g_pps_time_valid;
            __asm volatile ("cpsie i" ::: "memory");
            print_gnss(week, valid);
            printed_nav_pvt = g_debug_nav_pvt_count;
        }

        if (g_pps_count != printed_pps) {
            uint32_t count;
            uint64_t capture;
            uint16_t week;
            uint32_t tow;
            uint32_t valid;
            __asm volatile ("cpsid i" ::: "memory");
            count = g_pps_count;
            capture = g_pps_capture_us;
            week = g_pps_gps_week;
            tow = g_pps_gps_tow_ms;
            valid = g_pps_time_valid;
            __asm volatile ("cpsie i" ::: "memory");
            print_sync(count, capture, week, tow, valid);
            printed_pps = count;
        }

        /* TIM2_CH2 has already captured the edge; slower I2C and debug UART
         * work stays outside the interrupt handler. */
        if (!g_data_ready) continue;
        __asm volatile ("cpsid i" ::: "memory");
        uint64_t now = g_data_ready_us;
        g_data_ready = 0u;
        __asm volatile ("cpsie i" ::: "memory");

        /* Acknowledge/clear MPU6050 DATA_RDY status so the next data-ready
         * event can generate a fresh rising edge. This is not polling:
         * TIM2_CH2 has already captured the DATA_RDY event. */
        if (!mpu_read8(MPU_INT_STATUS, &status)) {
            ++g_debug_i2c_errors;
            ++errors;
            continue;
        }
        if (!read_regs(g_mpu_addr, MPU_ACCEL_XOUT_H, frame, 14u)) {
            ++g_debug_i2c_errors;
            if (++errors >= 5u) {
                uart_puts("# ERROR: I2C retry\r\n");
                errors = 0u;
                delay_ms(20u);
                (void)mpu_init();
            }
            continue;
        }

        errors = 0u;
        uint64_t dt = previous_us ? now - previous_us : 0u;
        print_sample(sample++, now, frame);

        /* Spread the once-per-second sky-view burst over IMU epochs so serial
         * output never stalls acquisition for tens of milliseconds. */
        if (g_sat_generation != printing_sat_generation) {
            printing_sat_generation = g_sat_generation;
            printing_sat_index = 0u;
        }
        if (printing_sat_generation != 0u) {
            if (printing_sat_index < g_sat_count) {
                print_satellite(&g_satellites[printing_sat_index++]);
            } else if (printing_sat_index == g_sat_count) {
                print_sat_end();
                ++printing_sat_index;
            }
        }
        /* RAWX can contain many dual-frequency measurements. Emit at most one
         * RAWX record per 100 Hz IMU epoch so acquisition and UART2 parsing
         * remain responsive. */
        if (g_rawx_generation != printing_rawx_generation) {
            printing_rawx_generation = g_rawx_generation;
            printing_rawx_index = 0u;
            printing_rawx_state = 1u;
        }
        if (printing_rawx_state == 1u) {
            print_rawx_header();
            printing_rawx_state = 2u;
        } else if (printing_rawx_state == 2u) {
            if (printing_rawx_index < g_rawx_count) {
                print_rawx_measurement(g_rawx_measurements[printing_rawx_index++]);
            } else {
                print_rawx_end();
                printing_rawx_state = 0u;
            }
        }
        g_debug_sample_count = sample;
        g_debug_last_dt_us = (uint32_t)dt;
        g_debug_last_dt_ms = (uint32_t)(dt / 1000u);
        previous_us = now;
        led_set((sample % SAMPLE_RATE_HZ) < 4u);
    }
}
