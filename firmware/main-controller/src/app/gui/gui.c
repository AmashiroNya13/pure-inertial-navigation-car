/**
 * @file gui.c
 * @brief 应用层按键模式选择与指示灯界面实现。
 */

#include "../../../inc/app/gui/gui.h"

typedef struct
{
    boolean is_pressed;          /**< 按键当前按下标志。 */
    boolean long_press_reported; /**< 长按事件已上报标志。 */
    uint64 press_start_tick;     /**< 按键按下起始时刻，单位：sysTick 计数。 */
} gui_key_runtime_t;

typedef struct
{
    gui_mode_t active_mode;                /**< 当前已确认运行模式。 */
    gui_mode_t selected_mode;              /**< 当前待确认候选模式。 */
    gui_key_runtime_t select_key_runtime;  /**< 模式选择按键运行状态。 */
    gui_key_runtime_t confirm_key_runtime; /**< 模式确认按键运行状态。 */
    sysTick_delay_t mode_led_delay;        /**< 模式指示灯非阻塞延时状态。 */
    sysTick_delay_t status_led_delay;      /**< 状态指示灯非阻塞延时状态。 */
    uint64 last_scheduler_tick;            /**< 上一次 GUI 调度时刻，单位：sysTick 计数。 */
    uint32 mode_led_pending_blinks;        /**< 模式指示灯剩余闪烁次数，单位：次。 */
    boolean mode_led_is_on;                /**< 模式指示灯当前点亮标志。 */
    boolean status_led_is_on;              /**< 状态指示灯当前点亮标志。 */
} gui_runtime_t;

static gui_runtime_t gui_runtime;

/**
 * @brief 获取模式对应的指示灯闪烁次数。
 * @param[in] mode 当前候选模式。
 * @return 指示灯闪烁次数，单位：次。
 */
static uint32 gui_mode_blink_cnt_get(gui_mode_t mode)
{
    return (uint32)mode + 1u;
}

/**
 * @brief 获取 GUI 使用的当前 sysTick 计数。
 * @param[in] void 无参数。
 * @return 当前 sysTick 计数。
 */
static uint64 gui_tick_get(void)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();

    return sysTick_getTick(gui_cfg->sysTick_id);
}

/**
 * @brief 将毫秒时间转换为 GUI 使用的 sysTick 计数。
 * @param[in] milliseconds 时间长度，单位：毫秒。
 * @return sysTick 计数。
 */
static uint32 gui_ticks_from_ms_get(uint32 milliseconds)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();

    return sysTick_getTicksFromMilliseconds(gui_cfg->sysTick_id, milliseconds);
}

/**
 * @brief 将候选模式切换到下一个模式。
 * @param[in] void 无参数。
 * @return void
 */
static void gui_select_next_mode(void)
{
    if ((uint32)gui_runtime.selected_mode >= ((uint32)GUI_MODE_CNT - 1u))
    {
        gui_runtime.selected_mode = GUI_MODE_LEARNING;
        return;
    }

    gui_runtime.selected_mode = (gui_mode_t)((uint32)gui_runtime.selected_mode + 1u);
}

/**
 * @brief 更新候选模式指示灯闪烁节奏。
 * @param[in] void 无参数。
 * @return void
 */
static void gui_mode_led_pattern_update(void)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();

    if (sysTick_delay_isElapsed(gui_cfg->sysTick_id, &gui_runtime.mode_led_delay) == FALSE)
    {
        return;
    }

    if (gui_runtime.mode_led_is_on != FALSE)
    {
        gui_runtime.mode_led_is_on = FALSE;
        device_led_set_state(gui_cfg->mode_led_id, DEVICE_LED_DARK);

        if (gui_runtime.mode_led_pending_blinks > 0u)
        {
            sysTick_delay_nonBlockingStartMilliseconds(gui_cfg->sysTick_id,
                                                       &gui_runtime.mode_led_delay,
                                                       gui_cfg->mode_led_off_ms);
        }
        else
        {
            sysTick_delay_nonBlockingStartMilliseconds(gui_cfg->sysTick_id,
                                                       &gui_runtime.mode_led_delay,
                                                       gui_cfg->mode_led_group_gap_ms);
        }

        return;
    }

    if (gui_runtime.mode_led_pending_blinks == 0u)
    {
        gui_runtime.mode_led_pending_blinks = gui_mode_blink_cnt_get(gui_runtime.selected_mode);
    }

    gui_runtime.mode_led_pending_blinks--;
    gui_runtime.mode_led_is_on = TRUE;
    device_led_set_state(gui_cfg->mode_led_id, DEVICE_LED_LIGHT);
    sysTick_delay_nonBlockingStartMilliseconds(gui_cfg->sysTick_id,
                                               &gui_runtime.mode_led_delay,
                                               gui_cfg->mode_led_on_ms);
}

/**
 * @brief 更新模式确认状态指示灯。
 * @param[in] void 无参数。
 * @return void
 */
static void gui_status_led_update(void)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();

    if (gui_runtime.selected_mode == gui_runtime.active_mode)
    {
        gui_runtime.status_led_is_on = TRUE;
        device_led_set_state(gui_cfg->status_led_id, DEVICE_LED_LIGHT);
        sysTick_delay_reset(&gui_runtime.status_led_delay);
        return;
    }

    if (sysTick_delay_isElapsed(gui_cfg->sysTick_id, &gui_runtime.status_led_delay) == FALSE)
    {
        return;
    }

    gui_runtime.status_led_is_on = (gui_runtime.status_led_is_on == FALSE) ? TRUE : FALSE;
    device_led_set_state(gui_cfg->status_led_id,
                         (gui_runtime.status_led_is_on != FALSE) ? DEVICE_LED_LIGHT : DEVICE_LED_DARK);
    sysTick_delay_nonBlockingStartMilliseconds(gui_cfg->sysTick_id,
                                               &gui_runtime.status_led_delay,
                                               gui_cfg->status_led_pending_blink_ms);
}

/**
 * @brief 更新单个 GUI 按键的短按和长按事件。
 * @param[in] key_id 设备层按键编号。
 * @param[in] key_runtime 按键运行状态指针。
 * @param[in] allow_short_press TRUE 表示允许短按事件。
 * @param[in] allow_long_press TRUE 表示允许长按事件。
 * @return void
 */
static void gui_key_update(device_key_id_t key_id,
                           gui_key_runtime_t* key_runtime,
                           boolean allow_short_press,
                           boolean allow_long_press)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();
    boolean is_pressed = (device_key_getState(key_id) == DEVICE_KEY_ONPRESS) ? TRUE : FALSE;
    uint64 current_tick = gui_tick_get();
    uint32 short_press_min_tick = gui_ticks_from_ms_get(gui_cfg->short_press_min_ms);
    uint32 long_press_tick = gui_ticks_from_ms_get(gui_cfg->long_press_ms);

    if ((is_pressed != FALSE) && (key_runtime->is_pressed == FALSE))
    {
        key_runtime->is_pressed = TRUE;
        key_runtime->long_press_reported = FALSE;
        key_runtime->press_start_tick = current_tick;
        return;
    }

    if (is_pressed != FALSE)
    {
        if ((allow_long_press != FALSE)
            && (key_runtime->long_press_reported == FALSE)
            && ((current_tick - key_runtime->press_start_tick) >= long_press_tick))
        {
            key_runtime->long_press_reported = TRUE;
            gui_runtime.active_mode = gui_runtime.selected_mode;
        }

        return;
    }

    if (key_runtime->is_pressed != FALSE)
    {
        uint64 press_duration_tick = current_tick - key_runtime->press_start_tick;

        if ((allow_short_press != FALSE)
            && (key_runtime->long_press_reported == FALSE)
            && (press_duration_tick >= short_press_min_tick))
        {
            gui_select_next_mode();
        }
    }

    key_runtime->is_pressed = FALSE;
    key_runtime->long_press_reported = FALSE;
    key_runtime->press_start_tick = 0u;
}

/**
 * @brief 执行一次按键模式扫描与指示灯调度。
 * @param[in] void 无参数。
 * @return void
 */
void gui_scheduler_callback(void)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();
    uint64 current_tick = gui_tick_get();
    uint32 scheduler_period_tick = gui_ticks_from_ms_get(gui_cfg->scheduler_period_ms);

    if ((current_tick - gui_runtime.last_scheduler_tick) < scheduler_period_tick)
    {
        return;
    }

    gui_runtime.last_scheduler_tick = current_tick;
    gui_key_update(gui_cfg->select_key_id, &gui_runtime.select_key_runtime, TRUE, FALSE);
    gui_key_update(gui_cfg->confirm_key_id, &gui_runtime.confirm_key_runtime, FALSE, TRUE);
    gui_mode_led_pattern_update();
    gui_status_led_update();
}

/**
 * @brief 初始化应用层按键模式选择与指示灯界面。
 * @param[in] void 无参数。
 * @return void
 */
void gui_init_all(void)
{
    gui_cfg_t* gui_cfg = gui_cfg_get();

    gui_runtime.active_mode = gui_cfg->default_mode;
    gui_runtime.selected_mode = gui_cfg->default_mode;
    gui_runtime.select_key_runtime.is_pressed = FALSE;
    gui_runtime.select_key_runtime.long_press_reported = FALSE;
    gui_runtime.select_key_runtime.press_start_tick = 0u;
    gui_runtime.confirm_key_runtime.is_pressed = FALSE;
    gui_runtime.confirm_key_runtime.long_press_reported = FALSE;
    gui_runtime.confirm_key_runtime.press_start_tick = 0u;
    gui_runtime.last_scheduler_tick = gui_tick_get();
    gui_runtime.mode_led_pending_blinks = 0u;
    gui_runtime.mode_led_is_on = FALSE;
    gui_runtime.status_led_is_on = TRUE;

    sysTick_delay_reset(&gui_runtime.mode_led_delay);
    sysTick_delay_reset(&gui_runtime.status_led_delay);

    device_led_set_state(gui_cfg->mode_led_id, DEVICE_LED_DARK);
    device_led_set_state(gui_cfg->status_led_id, DEVICE_LED_LIGHT);
}

/**
 * @brief 获取当前已确认的运行模式。
 * @param[in] void 无参数。
 * @return 当前已确认的运行模式。
 */
gui_mode_t gui_mode_get(void)
{
    return gui_runtime.active_mode;
}

/**
 * @brief 获取当前待确认的候选模式。
 * @param[in] void 无参数。
 * @return 当前待确认的候选模式。
 */
gui_mode_t gui_mode_selected_get(void)
{
    return gui_runtime.selected_mode;
}
