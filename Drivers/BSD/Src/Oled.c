//
// Created by JiangyaF on 2026/10/11.
//
// SSD1306 OLED 驱动实现（128x64，I2C）
//
// 设计要点：
//
//   1. 本地显存 + 整屏刷新。
//      直接写屏的话，画一屏要几千次 I2C 事务；本驱动在 RAM 里改显存，
//      最后 OLED_Refresh() 用 16 次事务推完（8 页 x (1 命令 + 1 数据)）。
//
//   2. 页寻址模式下，一页内的 128 列地址会自动回卷到 0。
//      所以刷新时不需要每字节重设列地址——但为了兼容性，每页仍然显式设一次。
//
//   3. 所有坐标变换（翻转）集中在 OLED_Tx/Ty 两个内联函数里，
//      绘制函数一律只认"逻辑坐标"，翻转功能就不会漏改某处。
//
//   4. 坐标越界一律静默忽略，不做断言。
//      因为显示是"辅助功能"，不该因为一个坐标算错就把整车搞崩。
//

#include "Oled.h"
#include "OledFont.h"
#include <string.h>

/* I2C 句柄定义在 main.c 中 */
extern I2C_HandleTypeDef hi2c2;

/* ------------------------------------------------------------------------- */
/* DMA 完成通知                                                                */
/* ------------------------------------------------------------------------- */

#if (OLED_ASYNC_MODE)
/*
 * 用 FreeRTOS 计数信号量做传输完成通知。
 *
 * 为什么用信号量而不是轮询 hi2c2.State：
 *   轮询是忙等，CPU 白白空转。用信号量可以让任务进入阻塞态，
 *   期间调度器把 CPU 让给控制任务和蓝牙任务，DMA 传完后再唤醒。
 *   对 100kHz I2C 传 129 字节约 12ms，这段时间足够控制任务跑半圈。
 *
 * 用【计数】信号量而不是二值：万一回调比等待先发生（极端时序），
 * 计数不会丢，下一次 Take 能立刻拿到。
 */
#include "semphr.h"
/* 信号量只在启用信号量等待时才需要 */
#if (OLED_USE_RTOS_SEM != 0U)
static SemaphoreHandle_t s_dma_done_sem = NULL;
#endif
#endif /* OLED_ASYNC_MODE */

/*
 * DMA 诊断计数器（不管是否启用 DMA 都定义，便于统一读取）
 *   s_async_ok      : 成功启动并等到完成的传输次数
 *   s_async_timeout : 等待超时的次数（>0 说明中断/回调链有问题）
 *   s_async_busy    : HAL 返回 BUSY 的次数（说明上一笔没等干净）
 */
static volatile uint32_t s_async_ok      = 0U;
static volatile uint32_t s_async_timeout = 0U;
static volatile uint32_t s_async_busy    = 0U;

/*
 * DMA 是否可用。
 *
 * ⚠️ 这是本驱动最重要的一个标志，原因是 FreeRTOS 的硬性约束：
 *    调度器启动之前不能调用会阻塞的同步 API（xSemaphoreTake 等），
 *    因为此时 pxCurrentTCB 还没建立，任务阻塞/唤醒机制无法工作，
 *    调用会让整个系统卡死（表现为：OLED 停在旧画面、其它任务全不运行）。
 *
 * 而 OLED_Init() 恰恰是在 vTaskStartScheduler() 之前调用的。
 * 所以：
 *   - OLED_Init() 阶段：s_async_ready = 0 -> 走轮询，安全
 *   - 调度器启动后，任务里调用 OLED_EnableAsync() -> s_async_ready = 1
 *     -> 走 I2C 中断（OLED_USE_IT=1）或 DMA（OLED_USE_DMA=1）
 */
static volatile uint8_t s_async_ready = 0U;

/* ------------------------------------------------------------------------- */
/* 显存                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 显存布局：[页][列]，共 8 页 x 128 列 = 1024 字节。
 * 页 p 的 bit0 是第 p*8 行的像素（上方），bit7 是第 p*8+7 行（下方）。
 */
static uint8_t s_gram[OLED_PAGE_COUNT][OLED_WIDTH];

/* 当前是否已做过 180 度翻转 */
static uint8_t s_flipped = 0U;

/* 是否处于反色显示 */
static uint8_t s_inverted = 0U;

/* ------------------------------------------------------------------------- */
/* 坐标变换                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t OLED_Tx(uint8_t x)
{
    return (s_flipped != 0U) ? (uint8_t)(OLED_WIDTH - 1U - x) : x;
}

static uint8_t OLED_Ty(uint8_t y)
{
    return (s_flipped != 0U) ? (uint8_t)(OLED_HEIGHT - 1U - y) : y;
}

/* ------------------------------------------------------------------------- */
/* 底层 I2C                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * ⚠️ DMA 发送缓冲区必须放在【静态区】，不能放栈上。
 *
 * 原因：HAL_I2C_Master_Transmit_DMA() 是异步的，它只是把缓冲区地址
 * 交给 DMA 控制器就立刻返回。如果缓冲区是函数内的局部数组，函数返回后
 * 这块栈内存会被后续调用覆盖，DMA 就会把垃圾数据发到屏幕上
 * （表现为花屏、随机点）。
 *
 * 静态缓冲区只多占 (32 + 129) = 161 字节 .bss，换来的确定性很值。
 *
 * 线程安全：本驱动只在 OledTask 一个任务里调用，且每次发送前都会
 * 等待上一笔完成，所以单份缓冲区不会冲突。
 */
static uint8_t s_cmd_buf[32];               /* 命令缓冲（控制字节 + 最多 31 字节命令）*/
static uint8_t s_data_buf[OLED_WIDTH + 1U]; /* 数据缓冲（控制字节 + 128 字节显存）  */

/*
 * 等待 I2C 回到就绪状态（上一笔 DMA 传完）。
 *
 * 为什么必须等：HAL 的 I2C 同一时刻只能有一笔传输，DMA 还没结束时
 * 再调 HAL_I2C_Master_Transmit_DMA() 会直接返回 HAL_BUSY，那一页数据
 * 就被静默丢弃了——现象是屏幕有若干行不更新。
 *
 * 用 HAL_GetTick() 轮询而不是 vTaskDelay：保持驱动与 FreeRTOS 解耦，
 * 这样同一份代码在裸机和 RTOS 下都能用。
 * 在 RTOS 下这段是忙等，但正常情况只需等几毫秒（DMA 传完即返回）。
 */
#if (OLED_ASYNC_MODE)
/*
 * 等待一笔异步传输完成（I2C 中断或 DMA）。
 * 返回 1 = 等到了完成信号；0 = 超时。
 */
static uint8_t OLED_WaitTxComplete(void)
{
    uint8_t got = 0U;

#if (OLED_USE_RTOS_SEM != 0U)
    if (s_dma_done_sem != NULL)
    {
        got = (xSemaphoreTake(s_dma_done_sem,
                              pdMS_TO_TICKS(OLED_DMA_TIMEOUT_MS)) == pdTRUE) ? 1U : 0U;
    }
#else
    {
        uint32_t start = HAL_GetTick();

        while (hi2c2.State != HAL_I2C_STATE_READY)
        {
            if ((HAL_GetTick() - start) >= OLED_DMA_TIMEOUT_MS)
            {
                break;
            }
        }
        got = (hi2c2.State == HAL_I2C_STATE_READY) ? 1U : 0U;
    }
#endif

    if (got == 0U)
    {
        s_async_timeout++;
    }

    return got;
}
#endif /* OLED_ASYNC_MODE */

/*
 * 统一的底层发送入口。
 *
 * 所有发送都经过这里，好处是 DMA/轮询的差异只在这一处，
 * 上层（命令/数据/刷新）不需要关心用的是哪种方式。
 */
static uint8_t OLED_I2C_Send(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return 0U;
    }

#if (OLED_ASYNC_MODE)
    /*
     * 只有在调度器启动后才允许走异步传输（见 s_async_ready 的说明）。
     * 初始化阶段走下面的轮询分支。
     */
    if (s_async_ready != 0U)
    {
        HAL_StatusTypeDef st;

        /*
         * 只有上一笔还没结束时才需要等。
         *
         * 不能无条件等待：第一次异步发送时并没有"上一笔"，
         * 无条件等会白等 50ms 超时，还让 timeout 计数凭空 +1，
         * 干扰诊断（看上去像有问题，其实没有）。
         */
        if (hi2c2.State != HAL_I2C_STATE_READY)
        {
            (void)OLED_WaitTxComplete();
        }

#if (OLED_USE_DMA != 0U)
        /* DMA 方式：硬件搬运数据，CPU 完全不参与 */
        st = HAL_I2C_Master_Transmit_DMA(&hi2c2,
                                         (uint16_t)(OLED_I2C_ADDR_7BIT << 1),
                                         (uint8_t *)buf,
                                         len);
#else
        /*
         * 中断方式：CPU 在 I2C 中断里逐字节搬运。
         *
         * 用中断的好处（相比轮询）：发起后立即返回，任务进入阻塞态，
         * 发送期间的 CPU 时间让给控制任务和蓝牙任务。
         *
         * 前提：I2C2_EV_IRQn 必须在 NVIC 里使能且优先级数值 >= 5。
         *      CubeMX 里对应 "I2C2 event interrupt"。
         */
        st = HAL_I2C_Master_Transmit_IT(&hi2c2,
                                        (uint16_t)(OLED_I2C_ADDR_7BIT << 1),
                                        (uint8_t *)buf,
                                        len);
#endif

        if (st == HAL_BUSY)
        {
            s_async_busy++;
            return 0U;
        }

        if (st != HAL_OK)
        {
            return 0U;      /* 启动失败 */
        }

        /*
         * 必须等这一笔真的传完才返回。
         * 因为缓冲区是静态复用的，不等就返回的话下一次调用会覆写
         * s_data_buf，而传输还没结束。
         */
        if (OLED_WaitTxComplete() != 0U)
        {
            s_async_ok++;
            return 1U;
        }

        return 0U;          /* 超时 */
    }
#endif

    /* 轮询发送：初始化阶段、或异步模式全部关闭时走这里 */
    return (HAL_I2C_Master_Transmit(&hi2c2,
                                    (uint16_t)(OLED_I2C_ADDR_7BIT << 1),
                                    (uint8_t *)buf, len,
                                    OLED_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/*
 * I2C 主机发送完成回调（由 HAL 在 STOP 之后调用）。
 *
 * 注意：HAL 在调用本回调之前已经把 hi2c->State 置为 READY，
 * 所以这里是"真正传完"的时刻，可以安全地开始下一笔。
 */
/*
 * 由 main.c 的 HAL_I2C_MasterTxCpltCallback() 调用。
 *
 * 为什么不在这里直接定义 HAL 回调：
 *   当 USE_HAL_I2C_REGISTER_CALLBACKS=0 时，HAL 库内部有一个 __weak 的
 *   HAL_I2C_MasterTxCpltCallback。链接器解析 HAL 自己的调用时，会先用
 *   库里的弱符号满足，导致写在 Oled.c 里的强定义被 --gc-sections 丢弃。
 *   而 main.c 的 .o 是强制参与链接的，在它里面定义强符号才能真正覆盖。
 *
 *   ⚠️ 不要试图改 USE_HAL_I2C_REGISTER_CALLBACKS：那个宏在
 *      stm32f1xx_hal_conf.h 里，CubeMX 重新生成代码时会把它改回 0。
 *
 * 本函数在中断上下文运行，只做 xSemaphoreGiveFromISR。
 */
void OLED_NotifyTxComplete(void)
{
#if (OLED_ASYNC_MODE) && (OLED_USE_RTOS_SEM != 0U)
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_dma_done_sem != NULL)
    {
        (void)xSemaphoreGiveFromISR(s_dma_done_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
#endif
}

/* 发送若干【命令】字节 */
static uint8_t OLED_WriteCmd(const uint8_t *cmds, uint16_t len)
{
    if ((cmds == NULL) || (len == 0U) || (len > (sizeof(s_cmd_buf) - 1U)))
    {
        return 0U;
    }

    s_cmd_buf[0] = 0x00;      /* 控制字节：后面是命令 */
    memcpy(&s_cmd_buf[1], cmds, len);

    return OLED_I2C_Send(s_cmd_buf, (uint16_t)(len + 1U));
}

/* 发送单条【命令】 */
static uint8_t OLED_Cmd1(uint8_t cmd)
{
    return OLED_WriteCmd(&cmd, 1U);
}

/* 发送两字节命令（SSD1306 里很多命令需要一个参数） */
static uint8_t OLED_Cmd2(uint8_t cmd, uint8_t arg)
{
    uint8_t tmp[2];

    tmp[0] = cmd;
    tmp[1] = arg;

    return OLED_WriteCmd(tmp, 2U);
}

/*
 * 发送一页【数据】（128 字节）。
 * 控制字节 0x40 之后跟 128 字节显存，一次事务推完。
 */
static uint8_t OLED_WritePageData(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U) || (len > OLED_WIDTH))
    {
        return 0U;
    }

    s_data_buf[0] = 0x40;     /* 控制字节：后面是数据 */
    memcpy(&s_data_buf[1], data, len);

    return OLED_I2C_Send(s_data_buf, (uint16_t)(len + 1U));
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

uint8_t OLED_Init(void)
{
#if (OLED_ASYNC_MODE) && (OLED_USE_RTOS_SEM != 0U)
    /* 创建 DMA 完成信号量。只需要创建一次，重复初始化时复用 */
    if (s_dma_done_sem == NULL)
    {
        s_dma_done_sem = xSemaphoreCreateCounting(4U, 0U);
        if (s_dma_done_sem == NULL)
        {
            return 0U;      /* 堆不够，无法安全使用 DMA */
        }
    }
#endif

    /* SSD1306 上电后内部电荷泵需要稳定时间，手册要求约 100ms。
       这里给 100ms，避免刚上电就发命令被忽略。 */
    HAL_Delay(100U);

    /* 依次发送初始化命令。命令值来自厂商例程的 OLED_Init()，逐条核对过。 */
    if (OLED_Cmd1(0xAE) == 0U) { return 0U; }    /* 关显示（配置期间保持熄灭） */
    OLED_Cmd1(0x00);                             /* 列低 4 位地址 = 0            */
    OLED_Cmd1(0x10);                             /* 列高 4 位地址 = 0            */
    OLED_Cmd1(0x40);                             /* 显示起始行 = 0               */
    OLED_Cmd2(0x81, 0xCF);                       /* 对比度 = 0xCF                */
    OLED_Cmd1(0xA1);                             /* 段重映射：左右正常            */
    OLED_Cmd1(0xC8);                             /* 扫描方向：上下正常            */
    OLED_Cmd1(0xA6);                             /* 正常显示（非反色）            */
    OLED_Cmd2(0xA8, 0x3F);                       /* 多路复用比 = 1/64             */
    OLED_Cmd2(0xD3, 0x00);                       /* 显示偏移 = 0                  */
    OLED_Cmd2(0xD5, 0x80);                       /* 时钟分频 / 振荡频率           */
    OLED_Cmd2(0xD9, 0xF1);                       /* 预充电周期                    */
    OLED_Cmd2(0xDA, 0x12);                       /* COM 引脚硬件配置              */
    OLED_Cmd2(0xDB, 0x30);                       /* VCOMH 取消选择电平            */
    OLED_Cmd2(0x20, 0x02);                       /* 页寻址模式（0x02 = PAGE）     */
    OLED_Cmd2(0x8D, 0x14);                       /* 使能内部电荷泵                */

    OLED_Clear();

    if (OLED_Refresh() == 0U)
    {
        return 0U;      /* 清屏失败说明 I2C 不通，直接返回，不必再开显示 */
    }

    if (OLED_Cmd1(0xAF) == 0U) { return 0U; }    /* 开显示 */
    OLED_Cmd1(0xA4);                             /* 显示内容跟随显存（非全亮）    */

    s_flipped  = 0U;
    s_inverted = 0U;

    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 显示控制                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t OLED_DisplayOn(void)
{
    return OLED_Cmd1(0xAF);
}

uint8_t OLED_DisplayOff(void)
{
    return OLED_Cmd1(0xAE);
}

uint8_t OLED_SetContrast(uint8_t contrast)
{
    return OLED_Cmd2(0x81, contrast);
}

uint8_t OLED_SetInvert(uint8_t invert)
{
    s_inverted = (invert != 0U) ? 1U : 0U;
    return OLED_Cmd1((s_inverted != 0U) ? 0xA7 : 0xA6);
}

uint8_t OLED_SetFlip(uint8_t flip)
{
    s_flipped = (flip != 0U) ? 1U : 0U;

    /*
     * 翻转有两个层面，必须同时处理：
     *   1. 硬件层面：段重映射(0xA0/0xA1) 和 扫描方向(0xC0/0xC8)
     *   2. 软件层面：OLED_Tx/OLED_Ty 里的坐标镜像
     *
     * 只改硬件的话，显存里的内容方向对了但绘制坐标没变，结果还是反的；
     * 只改软件的话，绘制坐标对了但屏的扫描方向没变，也是反的。
     */
    if (s_flipped != 0U)
    {
        OLED_Cmd1(0xA0);
        OLED_Cmd1(0xC0);
    }
    else
    {
        OLED_Cmd1(0xA1);
        OLED_Cmd1(0xC8);
    }

    /* 方向变了，把已有内容重新推一遍 */
    return OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 显存操作                                                                    */
/* ------------------------------------------------------------------------- */

void OLED_Clear(void)
{
    memset(s_gram, 0x00, sizeof(s_gram));
}

void OLED_EnableAsync(void)
{
#if (OLED_ASYNC_MODE)
    s_async_ready = 1U;
#endif
}

void OLED_GetAsyncStats(uint32_t *ok, uint32_t *timeout, uint32_t *busy)
{
    if (ok != NULL)      { *ok      = s_async_ok; }
    if (timeout != NULL) { *timeout = s_async_timeout; }
    if (busy != NULL)    { *busy    = s_async_busy; }
}

uint8_t OLED_Refresh(void)
{
    uint8_t page;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        /*
         * 页寻址模式下必须先设页地址和列地址，再连续写 128 字节。
         * 列地址会自动递增并在 127 处回卷，所以 128 字节正好覆盖一整页。
         */
        if (OLED_Cmd1((uint8_t)(0xB0U + page)) == 0U) { return 0U; }  /* 页地址 */
        if (OLED_Cmd1(0x00) == 0U) { return 0U; }                     /* 列低 4 位 */
        if (OLED_Cmd1(0x10) == 0U) { return 0U; }                     /* 列高 4 位 */

        if (OLED_WritePageData(s_gram[page], OLED_WIDTH) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
    uint8_t px;
    uint8_t py;
    uint8_t page;
    uint8_t bit;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;     /* 越界静默忽略 */
    }

    px   = OLED_Tx(x);
    py   = OLED_Ty(y);
    page = (uint8_t)(py / 8U);
    bit  = (uint8_t)(py % 8U);

    if (color != 0U)
    {
        s_gram[page][px] |= (uint8_t)(1U << bit);
    }
    else
    {
        s_gram[page][px] &= (uint8_t)(~(1U << bit));
    }
}

/* ------------------------------------------------------------------------- */
/* 图形                                                                        */
/* ------------------------------------------------------------------------- */

void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < w; i++)
    {
        OLED_DrawPoint((uint8_t)(x + i), y, color);
    }
}

void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < h; i++)
    {
        OLED_DrawPoint(x, (uint8_t)(y + i), color);
    }
}

void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t fill, uint8_t color)
{
    uint8_t i;

    if ((w == 0U) || (h == 0U))
    {
        return;
    }

    if (fill != 0U)
    {
        for (i = 0U; i < h; i++)
        {
            OLED_DrawHLine(x, (uint8_t)(y + i), w, color);
        }
        return;
    }

    OLED_DrawHLine(x, y, w, color);                          /* 上边 */
    OLED_DrawHLine(x, (uint8_t)(y + h - 1U), w, color);      /* 下边 */
    OLED_DrawVLine(x, y, h, color);                          /* 左边 */
    OLED_DrawVLine((uint8_t)(x + w - 1U), y, h, color);      /* 右边 */
}

void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    /*
     * Bresenham 直线算法：只用整数加减和比较，不用浮点、不用除法。
     * 在 M3 上比"按斜率插值"快得多，也不会因为浮点误差画歪。
     */
    int16_t dx  = (int16_t)x1 - (int16_t)x0;
    int16_t dy  = (int16_t)y1 - (int16_t)y0;
    int16_t sx  = (dx >= 0) ? 1 : -1;
    int16_t sy  = (dy >= 0) ? 1 : -1;
    int16_t err;
    int16_t x = (int16_t)x0;
    int16_t y = (int16_t)y0;

    if (dx < 0) { dx = -dx; }
    if (dy < 0) { dy = -dy; }

    err = (int16_t)(dx - dy);

    for (;;)
    {
        OLED_DrawPoint((uint8_t)x, (uint8_t)y, color);

        if ((x == (int16_t)x1) && (y == (int16_t)y1))
        {
            break;
        }

        {
            int16_t e2 = (int16_t)(err * 2);

            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 文字                                                                        */
/* ------------------------------------------------------------------------- */

uint8_t OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color)
{
    uint8_t idx;
    uint8_t col;
    uint8_t width;

    /* 不可打印字符统一显示成空格，避免越界访问字体表 */
    if ((ch < 0x20) || (ch > 0x7E))
    {
        ch = ' ';
    }
    idx = (uint8_t)(ch - 0x20);

    if (size == OLED_FONT_8X16)
    {
        width = 8U;

        /*
         * 8x16 字体：每字符 16 字节。
         * 前 8 字节是上半页（对应 y ~ y+7），后 8 字节是下半页（y+8 ~ y+15）。
         * 字体的点阵本来就是"每列一字节、bit0 在上"，和显存的页内位序一致，
         * 所以可以直接按位或进去，不需要做位翻转。
         */
        for (col = 0U; col < 6U; col++)
        {
            uint8_t upper = oled_font_8x16[idx][col];
            uint8_t lower = oled_font_8x16[idx][col + 8U];
            uint8_t bit;

            for (bit = 0U; bit < 8U; bit++)
            {
                uint8_t on_up = (uint8_t)((upper >> bit) & 0x01U);
                uint8_t on_lo = (uint8_t)((lower >> bit) & 0x01U);

                /* 颜色为黑时反过来写，这样黑字可以"抠"在实心背景上 */
                OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + bit),
                               (color != 0U) ? on_up : (uint8_t)(on_up ^ 0x01U));
                OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + 8U + bit),
                               (color != 0U) ? on_lo : (uint8_t)(on_lo ^ 0x01U));
            }
        }
        return (uint8_t)(x + width);
    }

    /* 默认按 6x8 处理 */
    width = 6U;
    for (col = 0U; col < 6U; col++)
    {
        uint8_t data = oled_font_6x8[idx][col];
        uint8_t bit;

        for (bit = 0U; bit < 8U; bit++)
        {
            uint8_t on = (uint8_t)((data >> bit) & 0x01U);

            OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + bit),
                           (color != 0U) ? on : (uint8_t)(on ^ 0x01U));
        }
    }

    return (uint8_t)(x + width);
}

uint8_t OLED_ShowString(uint8_t x, uint8_t y, const char *text, uint8_t size, uint8_t color)
{
    uint8_t cx = x;
    uint8_t cy = y;
    uint8_t width;
    uint8_t height;

    if (text == NULL)
    {
        return y;
    }

    width  = (size == OLED_FONT_8X16) ? 8U : 6U;
    height = (size == OLED_FONT_8X16) ? 16U : 8U;

    while (*text != '\0')
    {
        /* 自动换行：右边放不下就回到最左并下移一行 */
        if ((uint16_t)cx + width > OLED_WIDTH)
        {
            cx = 0U;
            cy = (uint8_t)(cy + height);
        }

        /* 底部放不下就停止，避免越界写 */
        if ((uint16_t)cy + height > OLED_HEIGHT)
        {
            break;
        }

        cx = OLED_ShowChar(cx, cy, *text, size, color);
        text++;
    }

    return cy;
}

/* ------------------------------------------------------------------------- */
/* 整车状态面板                                                                */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

//
// Created by JiangyaF on 2026/10/11.
//
