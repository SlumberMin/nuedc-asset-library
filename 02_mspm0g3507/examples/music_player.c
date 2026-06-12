/**
 * @file music_player.c
 * @brief MSPM0G3507 蜂鸣器音乐播放器
 *
 * 功能：无源蜂鸣器播放旋律 + 内置旋律库 + 节拍控制 + 按键选曲
 * 应用：电子琴、音乐盒、电赛演示项目
 *
 * 硬件连接：
 *   无源蜂鸣器: PWM信号=PA24 (TIMER0 CH0)
 *   按键1: PB0 (上一曲)
 *   按键2: PB1 (下一曲)
 *   按键3: PB2 (播放/暂停)
 *   LED: PB15 (节拍指示灯)
 *
 * @author 电赛资产库
 * @date 2026
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>

/* ========== 音符频率定义 ========== */

/* 音符频率表 (Hz) - C3到B5
 * 使用定时器PWM产生对应频率的方波驱动蜂鸣器
 */
#define NOTE_SILENT     0
#define NOTE_C3    131
#define NOTE_D3    147
#define NOTE_E3    165
#define NOTE_F3    175
#define NOTE_G3    196
#define NOTE_A3    220
#define NOTE_B3    247
#define NOTE_C4    262   /* 中央C */
#define NOTE_CS4   277
#define NOTE_D4    294
#define NOTE_DS4   311
#define NOTE_E4    330
#define NOTE_F4    349
#define NOTE_FS4   370
#define NOTE_G4    392
#define NOTE_GS4   415
#define NOTE_A4    440   /* 标准音A */
#define NOTE_AS4   466
#define NOTE_B4    494
#define NOTE_C5    523
#define NOTE_CS5   554
#define NOTE_D5    587
#define NOTE_DS5   622
#define NOTE_E5    659
#define NOTE_F5    698
#define NOTE_FS5   740
#define NOTE_G5    784
#define NOTE_GS5   831
#define NOTE_A5    880
#define NOTE_AS5   932
#define NOTE_B5    988
#define NOTE_C6    1047

/* 音符时值 (基于BPM) */
#define WHOLE       4     /* 全音符 = 4拍 */
#define HALF        2     /* 二分音符 = 2拍 */
#define QUARTER     1     /* 四分音符 = 1拍 */
#define EIGHTH      0     /* 八分音符 = 半拍 (用0标记, 实际计算) */
#define DOTTED_Q    5     /* 附点四分 (用5标记, 实际=1.5拍) */

/* 音符结构 */
typedef struct {
    uint16_t freq;    /* 频率(Hz), 0=休止符 */
    uint8_t  beats;   /* 时值标记 */
} Note_t;

/* 旋律结构 */
typedef struct {
    const char *name;     /* 曲名 */
    const Note_t *notes;  /* 音符数组 */
    uint16_t     count;   /* 音符数量 */
    uint16_t     bpm;     /* 速度 */
} Melody_t;

/* ========== 内置旋律库 ========== */

/* 小星星 - Twinkle Twinkle Little Star */
static const Note_t melody_star[] = {
    {NOTE_C4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_A4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_G4, HALF},
    {NOTE_F4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_D4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_C4, HALF},
    {NOTE_G4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_F4, QUARTER},
    {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, HALF},
    {NOTE_G4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_F4, QUARTER},
    {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, HALF},
    {NOTE_C4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_A4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_G4, HALF},
    {NOTE_F4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_D4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_C4, HALF},
};

/* 生日快乐 */
static const Note_t melody_birthday[] = {
    {NOTE_C4, EIGHTH}, {NOTE_C4, EIGHTH}, {NOTE_D4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, HALF},
    {NOTE_C4, EIGHTH}, {NOTE_C4, EIGHTH}, {NOTE_D4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_F4, HALF},
    {NOTE_C4, EIGHTH}, {NOTE_C4, EIGHTH}, {NOTE_C5, QUARTER},
    {NOTE_A4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, QUARTER},
    {NOTE_AS4, EIGHTH}, {NOTE_AS4, EIGHTH}, {NOTE_A4, QUARTER},
    {NOTE_F4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_F4, HALF},
};

/* 欢乐颂 - Ode to Joy */
static const Note_t melody_ode[] = {
    {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_G4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_E4, DOTTED_Q}, {NOTE_D4, EIGHTH}, {NOTE_D4, HALF},
    {NOTE_E4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_G4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_D4, DOTTED_Q}, {NOTE_C4, EIGHTH}, {NOTE_C4, HALF},
};

/* 茉莉花 */
static const Note_t melody_jasmine[] = {
    {NOTE_E4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_C5, QUARTER},
    {NOTE_B4, EIGHTH}, {NOTE_A4, EIGHTH}, {NOTE_G4, QUARTER}, {NOTE_A4, HALF},
    {NOTE_G4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_G4, EIGHTH}, {NOTE_E4, EIGHTH}, {NOTE_D4, QUARTER}, {NOTE_C4, HALF},
    {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_D4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_G4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_G4, HALF},
};

/* 卡农 (简化版) */
static const Note_t melody_canon[] = {
    {NOTE_E5, QUARTER}, {NOTE_D5, QUARTER}, {NOTE_C5, QUARTER}, {NOTE_B4, QUARTER},
    {NOTE_A4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_B4, QUARTER},
    {NOTE_C5, QUARTER}, {NOTE_B4, QUARTER}, {NOTE_A4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_G4, QUARTER},
    {NOTE_A4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_F4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_D4, QUARTER}, {NOTE_C4, QUARTER}, {NOTE_D4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_E4, QUARTER},
    {NOTE_C4, QUARTER}, {NOTE_E4, QUARTER}, {NOTE_G4, QUARTER}, {NOTE_C5, QUARTER},
};

/* 旋律列表 */
static const Melody_t g_melodies[] = {
    { "Star",     melody_star,     sizeof(melody_star)/sizeof(Note_t),     120 },
    { "Birthday", melody_birthday, sizeof(melody_birthday)/sizeof(Note_t), 100 },
    { "Ode Joy",  melody_ode,      sizeof(melody_ode)/sizeof(Note_t),      108 },
    { "Jasmine",  melody_jasmine,  sizeof(melody_jasmine)/sizeof(Note_t),  80  },
    { "Canon",    melody_canon,    sizeof(melody_canon)/sizeof(Note_t),    72  },
};
#define MELODY_COUNT (sizeof(g_melodies)/sizeof(Melody_t))

/* ========== 播放器状态 ========== */
typedef enum {
    PLAYER_STOP,
    PLAYER_PLAY,
    PLAYER_PAUSE
} PlayerState_t;

static volatile PlayerState_t g_state = PLAYER_STOP;
static uint8_t  g_curMelody = 0;
static uint16_t g_curNote = 0;
static volatile bool g_nextNote = false;
static volatile uint32_t g_beatTimer = 0;

/* ========== 蜂鸣器PWM控制 ========== */

/* 系统时钟和定时器配置 */
#define SYS_CLK_HZ     32000000UL
#define PWM_TIMER_LOAD  65535   /* 定时器重装值 */

/**
 * @brief 设置蜂鸣器发声频率
 * @param freq 频率(Hz), 0=静音
 */
static void buzzer_set_freq(uint16_t freq) {
    if (freq == 0) {
        /* 静音: PWM输出0 */
        DL_TimerG_setCaptureCompareValue(TIMER0, 0, DL_TIMER_CC_0_INDEX);
        return;
    }

    /* 计算定时器重装值: LOAD = CLK / freq - 1 */
    uint32_t load = SYS_CLK_HZ / freq - 1;
    if (load > PWM_TIMER_LOAD) load = PWM_TIMER_LOAD;

    DL_TimerG_setLoadValue(TIMER0, load);
    /* 50%占空比: compare = load / 2 */
    DL_TimerG_setCaptureCompareValue(TIMER0, load / 2, DL_TIMER_CC_0_INDEX);
}

/**
 * @brief 蜂鸣器静音
 */
static void buzzer_mute(void) {
    buzzer_set_freq(0);
}

/* ========== 按键处理 ========== */

#define BTN_PREV_PORT   GPIOB
#define BTN_PREV_PIN    DL_GPIO_PIN_0
#define BTN_NEXT_PORT   GPIOB
#define BTN_NEXT_PIN    DL_GPIO_PIN_1
#define BTN_PLAY_PORT   GPIOB
#define BTN_NEXT_PIN    DL_GPIO_PIN_2

/* 简单消抖读取 */
static bool btn_read(GPIO_Regs *port, uint32_t pin) {
    if (!(DL_GPIO_readPins(port, pin))) {  /* 低电平有效 */
        for (volatile uint32_t i = 0; i < 50000; i++);  /* 消抖 */
        if (!(DL_GPIO_readPins(port, pin))) {
            while (!(DL_GPIO_readPins(port, pin)));  /* 等待松开 */
            return true;
        }
    }
    return false;
}

/* ========== 定时器中断: 节拍控制 ========== */

/* 使用TIMG0作为节拍定时器 */
void TIMG0_IRQHandler(void) {
    if (DL_TimerG_getPendingInterrupt(TIMG0) == DL_TIMER_IIDX_ZERO) {
        g_beatTimer++;
        if (g_beatTimer >= 10) {  /* 每10ms检查一次 */
            g_beatTimer = 0;
            g_nextNote = true;
        }
    }
}

/* ========== LED节拍指示 ========== */
#define LED_PORT    GPIOB
#define LED_PIN     DL_GPIO_PIN_15

static void led_flash(void) {
    DL_GPIO_setPins(LED_PORT, LED_PIN);
    for (volatile uint32_t i = 0; i < 50000; i++);
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

/* ========== 播放器控制 ========== */

/**
 * @brief 获取音符时值(毫秒)
 * @param beats 时值标记
 * @param bpm 速度
 * @return 持续时间(ms)
 */
static uint32_t get_note_duration(uint8_t beats, uint16_t bpm) {
    /* 四分音符时值 = 60000 / bpm 毫秒 */
    uint32_t quarter_ms = 60000UL / bpm;

    switch (beats) {
        case WHOLE:   return quarter_ms * 4;
        case HALF:    return quarter_ms * 2;
        case QUARTER: return quarter_ms;
        case EIGHTH:  return quarter_ms / 2;
        case DOTTED_Q: return quarter_ms * 3 / 2;
        default:      return quarter_ms;
    }
}

/**
 * @brief 播放当前音符
 */
static void play_current_note(void) {
    const Melody_t *m = &g_melodies[g_curMelody];
    if (g_curNote >= m->count) {
        /* 旋律播放完毕, 循环 */
        g_curNote = 0;
    }

    const Note_t *note = &m->notes[g_curNote];
    uint32_t duration = get_note_duration(note->beats, m->bpm);

    if (note->freq > 0) {
        buzzer_set_freq(note->freq);
        led_flash();
    } else {
        buzzer_mute();
    }

    /* 延时(音符持续时间的85%, 留15%作为间隔) */
    uint32_t play_time = duration * 85 / 100;
    uint32_t gap_time = duration - play_time;

    /* 等待音符播放完成 */
    for (volatile uint32_t i = 0; i < play_time * 1000; i++);

    /* 音符间短暂停顿 */
    buzzer_mute();
    for (volatile uint32_t i = 0; i < gap_time * 1000; i++);

    g_curNote++;
}

/* ========== 主函数 ========== */
int main(void) {
    /* 初始化系统 */
    SYSCFG_DL_init();

    /* 启动PWM定时器 (蜂鸣器) */
    DL_TimerG_startCounter(TIMER0);

    /* 启动节拍定时器 */
    NVIC_EnableIRQ(TIMG0_IRQn);
    DL_TimerG_startCounter(TIMG0);

    /* 初始静音 */
    buzzer_mute();

    /* 开机提示音: 短促的Do-Mi-Sol */
    buzzer_set_freq(NOTE_C5);
    for (volatile uint32_t i = 0; i < 2000000; i++);
    buzzer_set_freq(NOTE_E5);
    for (volatile uint32_t i = 0; i < 2000000; i++);
    buzzer_set_freq(NOTE_G5);
    for (volatile uint32_t i = 0; i < 3000000; i++);
    buzzer_mute();

    /* 主循环 */
    while (1) {
        /* 按键处理 */
        if (btn_read(BTN_PREV_PORT, BTN_PREV_PIN)) {
            /* 上一曲 */
            if (g_curMelody > 0)
                g_curMelody--;
            else
                g_curMelody = MELODY_COUNT - 1;
            g_curNote = 0;
            buzzer_mute();
        }

        if (btn_read(BTN_NEXT_PORT, BTN_NEXT_PIN)) {
            /* 下一曲 */
            g_curMelody = (g_curMelody + 1) % MELODY_COUNT;
            g_curNote = 0;
            buzzer_mute();
        }

        if (btn_read(BTN_PLAY_PORT, DL_GPIO_PIN_2)) {
            /* 播放/暂停切换 */
            if (g_state == PLAYER_PLAY) {
                g_state = PLAYER_PAUSE;
                buzzer_mute();
            } else {
                g_state = PLAYER_PLAY;
            }
        }

        /* 播放逻辑 */
        if (g_state == PLAYER_PLAY) {
            play_current_note();
        }
    }
}
