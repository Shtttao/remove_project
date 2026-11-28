#include "board.h"
#include "stm32h5xx.h"

/* stm32h5xx.h 通过 CMSIS 把硬件寄存器映射为 C 结构体。例如 TIM2->CNT
   由定时器硬件递增。TIM2 即使在 CPU 轮询 SPI 的时候也会继续计时。 */
#define HSI_HZ 64000000u
#define UART_BAUD 115200u
/* 接收溢出、模式故障、TI 帧错误或发送欠载，任何一个都使本次事务失败。 */
#define SPI_FAILURE_FLAGS (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_TIFRE | SPI_SR_UDR)

uint32_t board_micros(void)
{
    return TIM2->CNT;
}

static void delay_us(uint32_t us)
{
    /* 无符号减法按 2^32 取模。即便计数从 0xffffffff 跨到 0，也能求经过的微秒数。 */
    uint32_t start = board_micros();
    while ((uint32_t)(board_micros() - start) < us) {
    }
}

void board_delay_ms(void *context, uint32_t ms)
{
    (void)context;
    while (ms > 0u) {
        /* 每次至多换算 1000 ms，避免长延时换算为微秒时乘法溢出。 */
        uint32_t chunk = ms > 1000u ? 1000u : ms;
        /* 读取 CNT 时可能已接近下一次跳变，多等一格才能保证至少经过指定时长。 */
        delay_us(chunk * 1000u + 1u);
        ms -= chunk;
    }
}

static void set_pin(GPIO_TypeDef *port, unsigned int pin, uint32_t mode, uint32_t af, uint32_t pull)
{
    /* 本板所选引脚的端口内编号均小于 8。MODER、PUPDR、OSPEEDR 每脚占两位，
       AFR[0] 每脚占四位，MODIFY_REG 只改掩码覆盖的位，保留邻脚配置。 */
    uint32_t shift = 2u * pin;
    port->OTYPER &= ~(1u << pin);
    /* 推挽输出。pull 为 0、1、2 时分别表示无上下拉、上拉、下拉。 */
    MODIFY_REG(port->PUPDR, 3u << shift, pull << shift);
    MODIFY_REG(port->OSPEEDR, 3u << shift, 1u << shift);
    MODIFY_REG(port->AFR[0], 15u << (4u * pin), af << (4u * pin));
    /* 先准备复用编号，再接通引脚。mode=1 为普通输出，mode=2 接到复用外设。 */
    MODIFY_REG(port->MODER, 3u << shift, mode << shift);
}

void board_status_led(bool on)
{
    /* X Pulse 的 PC0 接状态灯阴极。低电平吸收电流点亮，高电平熄灭。 */
    GPIOC->BSRR = on ? GPIO_BSRR_BR0 : GPIO_BSRR_BS0;
}

static void reset_spi(void)
{
    /* 拉起再释放 SPI1 的复位位，可清空上次失败事务的 FIFO 和状态。
       两次读回让总线上的复位操作依次完成，然后才配置 SPI 寄存器。 */
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    (void)RCC->APB2RSTR;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    (void)RCC->APB2RSTR;
    /* 软件管理片选时，SSI 给 SPI 主机的内部 NSS 输入提供高电平。
       传感器真正看到的片选来自 PA4，由 spi_transfer 操作。 */
    SPI1->CR1 = SPI_CR1_SSI;
    /* DSIZE=7 编码 8 位数据，FTHLV=0 表示 FIFO 每凑齐 1 个数据即可处理。
       MBR=5 编码 64 分频，正常 64 MHz 内核时钟得到 1 MHz 的 SCK。 */
    MODIFY_REG(SPI1->CFG1, SPI_CFG1_DSIZE | SPI_CFG1_FTHLV | SPI_CFG1_MBR,
               (7u << SPI_CFG1_DSIZE_Pos) | (5u << SPI_CFG1_MBR_Pos));
    /* MASTER 选择主机，SSM 选择软件片选，AFCNTR 让空闲时 SPI 继续控制引脚。
       CPOL、CPHA 均为 0，即时钟空闲为低、上升沿采样。数据默认高位先发。 */
    SPI1->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR;
}

void board_init(void)
{
    /* 启动文件已调用 ST 的 SystemInit，系统使用片内 HSI，各总线不分频，
       HSI 本身仍保留原分频。VOS3 下先设置并读回 Flash 的 3 个等待周期，
       再清零 HSIDIV 切换到 64 MHz，保证升频后仍能正确取指。 */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS);
    if ((FLASH->ACR & FLASH_ACR_LATENCY) == FLASH_ACR_LATENCY_3WS) {
        RCC->CR &= ~RCC_CR_HSIDIV;
    }
    /* 允许外设内核使用 HSI，按 HSIDIV 的实际读数换算时钟。若 Flash 等待周期
       未通过上面的读回检查，此处仍会按保留下来的分频计算 TIM2 和 USART2。 */
    RCC->CR |= RCC_CR_HSIKERON;
    SystemCoreClock = HSI_HZ >> ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos);

    /* SPI1 内核时钟默认取 PLL1Q。本工程未启用 PLL，先以 HSI 驱动 CLKP，
       再把 SPI1SEL 设为二进制 100 选中 CLKP。USART2SEL=0 选择 APB1 时钟，
       当前 APB1 与内核同频。外设时钟和分频值共同决定通信速率，下面的 EN 位则打开总线时钟门控。 */
    RCC->CCIPR5 &= ~RCC_CCIPR5_CKERPSEL;
    MODIFY_REG(RCC->CCIPR3, RCC_CCIPR3_SPI1SEL, RCC_CCIPR3_SPI1SEL_2);
    RCC->CCIPR1 &= ~RCC_CCIPR1_USART2SEL;
    /* 打开 GPIOA/B/C、TIM2、USART2、SPI1 的总线时钟。读回 ENR 给门控生效留时间，
       再访问对应外设，避免刚开时钟时的首次寄存器写入丢失。 */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN | RCC_APB1LENR_USART2EN;
    (void)RCC->APB1LENR;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    /* 先停表。定时器按 PSC+1 分频，64 MHz 时 PSC=63，得到每格 1 µs。
       ARR 设到 32 位上限。UG 令预分频值立即生效，再清除更新标志并启动。 */
    TIM2->CR1 = 0u;
    TIM2->PSC = SystemCoreClock / 1000000u - 1u;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;
    TIM2->CR1 = TIM_CR1_CEN;

    /* BSRR 的 BS4 只将 PA4 置高，BR4 只将它拉低。先预置高电平再切输出，
       避免配置其余引脚时误选中低有效的 ICM-42688-P。 */
    GPIOA->BSRR = GPIO_BSRR_BS4;
    set_pin(GPIOA, 4u, 1u, 0u, 1u);
    /* 同样先写输出电平，再把 PC0 切换到输出模式，避免初始化时状态灯闪一下。 */
    board_status_led(false);
    set_pin(GPIOC, 0u, 1u, 0u, 0u);
    reset_spi();
    /* X Pulse 外部 SPI1 接口走 PB3/PB4/PB5，避开板载 QSPI 占用的 PA6/PA7。 */
    set_pin(GPIOB, 3u, 2u, 5u, 2u);  /* SCK，模式 0 空闲时为低。 */
    set_pin(GPIOB, 4u, 2u, 5u, 1u);  /* MISO，弱上拉使开路读为 0xff。 */
    set_pin(GPIOB, 5u, 2u, 5u, 0u);  /* MOSI，主机发送寄存器地址和数据。 */

    /* 关闭串口后配置 8 数据位、无校验、1 停止位，保留 16 倍过采样。
       PRESC=0 表示时钟不分频。BRR 为时钟频率除以目标波特率并四舍五入。 */
    USART2->CR1 = 0u;
    USART2->CR2 = 0u;
    USART2->CR3 = 0u;
    USART2->PRESC = 0u;
    USART2->BRR = (SystemCoreClock + UART_BAUD / 2u) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
    set_pin(GPIOA, 2u, 2u, 7u, 1u);  /* AF7，USART2_TX，空闲为高，115200 8-N-1。 */
}

static bool spi_wait(uint32_t flag, uint32_t start, uint32_t timeout_us)
{
    /* 所有字节和 EOT 检查共用整次事务的 start。先查错误，再取就绪标志，
       即使同时出现数据和错误，也不会把损坏的事务当作成功。 */
    do {
        uint32_t status = SPI1->SR;
        if (status & SPI_FAILURE_FLAGS) {
            return false;
        }
        if (status & flag) {
            return true;
        }
    } while ((uint32_t)(board_micros() - start) < timeout_us);
    return false;
}

static bool spi_transfer(uint8_t address, uint8_t value, uint8_t *rx, size_t len)
{
    /* len 仅为有效数据长度，硬件 TSIZE 还需计入首个地址字节。
       1 MHz 时每字节在线上占 8 µs。超时另留固定和逐字节的软件轮询余量。 */
    size_t frames = len + 1u;
    uint32_t start = board_micros();
    uint32_t timeout_us = 1000u + 32u * (uint32_t)frames;

    /* 先装入整次传输帧数，SPE 使能外设，拉低 CS 等待建立时间，再写 CSTART。 */
    SPI1->CR2 = (uint32_t)frames;
    SPI1->CR1 |= SPI_CR1_SPE;
    GPIOA->BSRR = GPIO_BSRR_BR4;
    delay_us(2u);
    SPI1->CR1 |= SPI_CR1_CSTART;
    for (size_t i = 0u; i < frames; ++i) {
        if (!spi_wait(SPI_SR_TXP, start, timeout_us)) {
            goto failed;
        }
        /* TXP 表示发送 FIFO 有空位。用 8 位访问使每次写 TXDR 仅提交一个字节。
           第一个字节发地址，后续发写入值，读操作传入的 value 为 0。 */
        *(volatile uint8_t *)&SPI1->TXDR = i == 0u ? address : value;
        if (!spi_wait(SPI_SR_RXP, start, timeout_us)) {
            goto failed;
        }
        /* SPI 全双工，每发一个字节就同时收到一个。RXP 表示现在可读，
           地址阶段收到的字节丢弃。写操作也要读空 RXDR，避免接收溢出。 */
        uint8_t received = *(volatile uint8_t *)&SPI1->RXDR;
        if (i > 0u && rx != NULL) {
            rx[i - 1u] = received;
        }
    }
    if (!spi_wait(SPI_SR_EOT, start, timeout_us)) {
        goto failed;
    }
    /* ST 勘误 ES0565 指出，EOT 可能先于最后一个 SCK 下降沿。
       再保持 CS 和 SPE 两个计数刻度，随后释放片选，写 1 清除 EOT、TXTF 标志。 */
    delay_us(2u);
    GPIOA->BSRR = GPIO_BSRR_BS4;
    SPI1->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    return true;

failed:
    /* 立即释放 CS 使传感器退出当前事务，再复位 SPI，供下次事务从头开始。 */
    GPIOA->BSRR = GPIO_BSRR_BS4;
    reset_spi();
    return false;
}

bool board_read(void *context, uint8_t reg, uint8_t *data, size_t len)
{
    (void)context;
    /* 地址只占低 7 位，最高位 1 表示读。TSIZE 有 16 位，必须给地址留一格。 */
    if (reg > 0x7fu || data == NULL || len == 0u || len >= UINT16_MAX) {
        return false;
    }
    return spi_transfer((uint8_t)(reg | 0x80u), 0u, data, len);
}

bool board_write(void *context, uint8_t reg, uint8_t value)
{
    (void)context;
    /* 写命令最高位保持为 0，只跟一个数据字节。无需返回同时收到的数据。 */
    return reg <= 0x7fu && spi_transfer(reg, value, NULL, 1u);
}

static bool uart_wait(uint32_t flag)
{
    /* 每次等一个串口条件最多 2 ms。115200 8-N-1 发一个字节约需 87 µs。 */
    uint32_t start = board_micros();
    while ((USART2->ISR & flag) != flag) {
        if ((uint32_t)(board_micros() - start) >= 2000u) {
            return false;
        }
    }
    return true;
}

bool board_uart_tx(const uint8_t *data, size_t len)
{
    /* TEACK 确认发送器已启用，TXE_TXFNF 确认能写下一个字节。
       最后等 TC，涵盖最后一字节移出移位寄存器、停止位发送完毕的时间。 */
    if (data == NULL || len == 0u || !uart_wait(USART_ISR_TEACK)) {
        return false;
    }
    for (size_t i = 0u; i < len; ++i) {
        if (!uart_wait(USART_ISR_TXE_TXFNF)) {
            return false;
        }
        USART2->TDR = data[i];
    }
    return uart_wait(USART_ISR_TC);
}
