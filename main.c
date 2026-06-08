#include <LPC17xx.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <Board_LED.h>
#include <Board_GLCD.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Thu vien can cho ham atoi()

// External declaration for GLCD font
extern GLCD_FONT GLCD_Font_16x24;

// Constants
#define DEBOUNCE_TIME_MS 150
#define MAX_ADC_VALUE 4095
#define MIN_DURATION_MS 100
#define MAX_DURATION_MS 5000

// Pin Definitions
#define JOYSTICK_UP_PIN     (1 << 23)  // P1.23
#define JOYSTICK_DOWN_PIN   (1 << 25)  // P1.25
#define JOYSTICK_CENTER_PIN (1 << 20)  // P1.20
#define JOYSTICK_LEFT_PIN   (1 << 24)  // P1.24
#define JOYSTICK_RIGHT_PIN  (1 << 26)  // P1.26

// Colors
#define COLOR_WHITE  0xFFFFFF
#define COLOR_BLACK  0x000000
#define COLOR_BLUE   0x0000FF
#define COLOR_RED    0xFF0000
#define COLOR_GRAY   0xC0C0C0

// Joystick States
#define JOYSTICK_UP     0x01
#define JOYSTICK_DOWN   0x02
#define JOYSTICK_CENTER 0x04
#define JOYSTICK_LEFT   0x08
#define JOYSTICK_RIGHT  0x10

// Data Structures
typedef enum {
    ACTUATOR_HEATER,
    ACTUATOR_SPRINKLER,
    ACTUATOR_LIGHT,
    ACTUATOR_COUNT
} ActuatorType;

typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL,
    MODE_TIMER
} SystemMode;

typedef struct {
    const char *name;
    volatile int *threshold;
    volatile int *duration;
    uint32_t pin;
    LPC_GPIO_TypeDef *port;
} ActuatorConfig;

typedef struct {
    volatile int hours;
    volatile int minutes;
    volatile int seconds;
} TimeData;

// Global Variables
static SemaphoreHandle_t adc_mutex;
static SemaphoreHandle_t glcd_mutex;
static SemaphoreHandle_t uart_mutex_id; // Mutex moi de bao ve UART
static SemaphoreHandle_t actuator_sems[ACTUATOR_COUNT];

static volatile SystemMode current_mode = MODE_AUTO; // M?c d?nh là AUTO
static volatile int adc_values[ACTUATOR_COUNT];
static volatile TimeData current_time = {0, 0, 0};
static volatile TimeData timers[ACTUATOR_COUNT] = {
    {0, 0, 30}, // Heater default
    {0, 0, 30}, // Sprinkler default
    {0, 0, 30}  // Light default
};
static volatile int timer_triggered[ACTUATOR_COUNT] = {0, 0, 0};
static volatile int selected_menu = 0;
static volatile int selected_field = 0;
static volatile int selected_actuator = 0;

// Actuator Durations (cho Timer Mode)
static volatile int heater_duration = 500;
static volatile int sprinkler_duration = 600;
static volatile int light_duration = 700;

// Actuator Thresholds
static volatile int threshold_values[ACTUATOR_COUNT] = {1600, 2000, 3000};

// Actuator Configurations
static ActuatorConfig actuators[ACTUATOR_COUNT] = {
    {"Heater",    &threshold_values[0], &heater_duration,    (1 << 29), LPC_GPIO1},
    {"Sprinkle",  &threshold_values[1], &sprinkler_duration, (1 << 31), LPC_GPIO1},
    {"Light",     &threshold_values[2], &light_duration,     (1 << 2),  LPC_GPIO2}
};

// Function Prototypes
void init_hardware(void);
void init_adc(void);
void init_gpio(void);
void init_uart(void);
void init_timer(void);
void send_uart_string(const char *str);
int read_adc_channel(uint8_t channel);
void toggle_actuator(ActuatorType type);
void sensor_thread(void *arg);
void uart_thread(void *arg);
void monitor_thread(void *arg);
void control_thread(void *arg);
void uart_receive_thread(void *arg);
void menu_thread(void *arg);
void timer_monitor_thread(void *arg);
void display_menu(int prev_menu);
void show_sensors(void);
void control_actuators(void);
void adjust_threshold(ActuatorType type);
void adjust_durations(void);
void control_timers(void);
void update_time(void);
uint32_t read_joystick(void);

// Hardware Initialization
void init_hardware(void) {
    init_adc();
    init_gpio();
    init_uart();
    init_timer();
}

void init_adc(void) {
    LPC_PINCON->PINSEL1 |= (1 << 14) | (1 << 16) | (1 << 18);
    LPC_SC->PCONP |= (1 << 12); 
    LPC_ADC->ADCR = (7 << 0) | (4 << 8) | (1 << 21); 
}

void init_gpio(void) {
    LPC_GPIO1->FIODIR |= (1 << 29) | (1 << 31) | (1 << 28); 
    LPC_GPIO2->FIODIR |= (1 << 2);
    LPC_PINCON->PINSEL3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); 
    LPC_PINCON->PINMODE3 &= ~((3 << 14) | (3 << 18) | (3 << 8) | (3 << 16) | (3 << 20)); 
    LPC_GPIO1->FIODIR &= ~(JOYSTICK_UP_PIN | JOYSTICK_DOWN_PIN | JOYSTICK_CENTER_PIN | 
                          JOYSTICK_LEFT_PIN | JOYSTICK_RIGHT_PIN); 
}

void init_uart(void) {
    uint32_t Fdiv;
    uint32_t pclkdiv, pclk;

    LPC_SC->PCONP |= (1 << 3);                  // B?t ngu?n cho UART0
    LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6); // C?u hình chân P0.2 là TXD0, P0.3 là RXD0

    // Ð?c b? chia clock ngo?i vi d? tính ra xung nh?p (PCLK) c?p cho UART0
    pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
    switch (pclkdiv) {
        case 0x00: pclk = SystemCoreClock / 4; break;
        case 0x01: pclk = SystemCoreClock;     break;
        case 0x02: pclk = SystemCoreClock / 2; break;
        case 0x03: pclk = SystemCoreClock / 8; break;
    }

    LPC_UART0->LCR = 0x83; // 8 bit data, 1 stop bit, Enable DLAB (m? ch?t d? ghi b? chia)
    
    // T? d?ng tính toán thông s? chia Baudrate d? d?t m?c 9600
    Fdiv = (pclk / 16) / 9600; 
    LPC_UART0->DLM = Fdiv / 256;
    LPC_UART0->DLL = Fdiv % 256;
    
    LPC_UART0->LCR = 0x03; // Khóa ch?t DLAB (Hoàn t?t c?u hình)
		LPC_UART0->FCR = 0x07;
}

void init_timer(void) {
    LPC_SC->PCONP |= (1 << 2); 
    LPC_TIM1->TCR = 0x02; 
    LPC_TIM1->PR = 0; 
    LPC_TIM1->MR0 = SystemCoreClock - 1; // 1 Hz
    LPC_TIM1->MCR = 0x03; 
    NVIC_EnableIRQ(TIMER1_IRQn);
    LPC_TIM1->TCR = 0x01; 
}

void TIMER1_IRQHandler(void) {
    if (LPC_TIM1->IR & 0x01) {
        LPC_TIM1->IR = 0x01; 
        update_time();
    }
}

void update_time(void) {
    current_time.seconds++;
    if (current_time.seconds >= 60) {
        current_time.seconds = 0;
        current_time.minutes++;
        if (current_time.minutes >= 60) {
            current_time.minutes = 0;
            current_time.hours++;
            if (current_time.hours >= 24) {
                current_time.hours = 0;
            }
        }
    }
}

void send_uart_string(const char *str) {
    // Dung Mutex de khong bi chong cheo ky tu khi 2 luong cung goi truyen UART
    xSemaphoreTake(uart_mutex_id, portMAX_DELAY);
    while (*str) {
        while (!(LPC_UART0->LSR & (1 << 5))); 
        LPC_UART0->THR = *str++;
    }
    xSemaphoreGive(uart_mutex_id);
}

int read_adc_channel(uint8_t channel) {
    LPC_ADC->ADCR &= ~(0x7 << 0); 
    LPC_ADC->ADCR |= (1 << channel); 
    LPC_ADC->ADCR |= (1 << 24); 
    while (!(LPC_ADC->ADGDR & (1U << 31))); 
    return (LPC_ADC->ADGDR >> 4) & 0xFFF; 
}

uint32_t read_joystick(void) {
    uint32_t state = 0;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_UP_PIN))     state |= JOYSTICK_UP;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_DOWN_PIN))   state |= JOYSTICK_DOWN;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_CENTER_PIN)) state |= JOYSTICK_CENTER;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_LEFT_PIN))   state |= JOYSTICK_LEFT;
    if (!(LPC_GPIO1->FIOPIN & JOYSTICK_RIGHT_PIN))  state |= JOYSTICK_RIGHT;
    return state;
}

void toggle_actuator(ActuatorType type) {
    ActuatorConfig *act = &actuators[type];
    if (act->port->FIOPIN & act->pin) {
        act->port->FIOCLR = act->pin; 
    } else {
        act->port->FIOSET = act->pin; 
    }
}

// ================= RTOS THREADS =================

void sensor_thread(void *arg) {
    static int warning_sent[ACTUATOR_COUNT] = {0}; // Tranh spam canh bao lien tuc

    while (1) {
        xSemaphoreTake(adc_mutex, portMAX_DELAY);
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            adc_values[i] = read_adc_channel(i);
            
            // CANH BAO KHI VUOT NGUONG O TAT CA CAC MODE
            if (adc_values[i] > *actuators[i].threshold) {
                if (!warning_sent[i]) {
                    char warn_buf[64];
                    snprintf(warn_buf, sizeof(warn_buf), "! WARN: %s ADC (%d) > Muc Nguong (%d)\n", 
                             actuators[i].name, adc_values[i], *actuators[i].threshold);
                    send_uart_string(warn_buf);
                    warning_sent[i] = 1;
                }
            } else {
                warning_sent[i] = 0; // Reset lai co khi ve muc an toan
            }
        }
        xSemaphoreGive(adc_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void uart_thread(void *arg) {
    char buffer[128];
    const char *mode_str[] = {"AUTO", "MANUAL", "TIMER"};
    
    while (1) {
        xSemaphoreTake(adc_mutex, portMAX_DELAY);
        snprintf(buffer, sizeof(buffer), "MODE:%s | TEMP:%d | MOIST:%d | LIGHT:%d\n",
                 mode_str[current_mode], adc_values[0], adc_values[1], adc_values[2]);
        xSemaphoreGive(adc_mutex);
        
        send_uart_string(buffer);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void timer_monitor_thread(void *arg) {
    while (1) {
        if (current_mode == MODE_TIMER) {
            for (int i = 0; i < ACTUATOR_COUNT; i++) {
                if (current_time.hours == timers[i].hours &&
                    current_time.minutes == timers[i].minutes &&
                    current_time.seconds == timers[i].seconds) {
                    
                    if (!timer_triggered[i]) { 
                        timer_triggered[i] = 1;
                        xSemaphoreGive(actuator_sems[i]); 
                    }
                } else {
                    timer_triggered[i] = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void monitor_thread(void *arg) {
    ActuatorType type = *(ActuatorType *)arg;
    while (1) {
        if (current_mode == MODE_AUTO) {
            xSemaphoreTake(adc_mutex, portMAX_DELAY);
            int current_val = adc_values[type];
            int current_thresh = *actuators[type].threshold;
            xSemaphoreGive(adc_mutex);

            if (current_val > current_thresh) {
                actuators[type].port->FIOSET = actuators[type].pin; // ON
            } else {
                actuators[type].port->FIOCLR = actuators[type].pin; // OFF
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void control_thread(void *arg) {
    ActuatorType type = *(ActuatorType *)arg;
    while (1) {
        xSemaphoreTake(actuator_sems[type], portMAX_DELAY);
        if (current_mode == MODE_TIMER) {
            actuators[type].port->FIOSET = actuators[type].pin;
            vTaskDelay(pdMS_TO_TICKS(*actuators[type].duration));
            actuators[type].port->FIOCLR = actuators[type].pin;
        } 
    }
}

void uart_receive_thread(void *arg) {
    char buffer[64];
    int idx = 0;
    while (1) {
        // S?A L?I ? ÐÂY: Dùng while thay vì if d? d?c h?t toàn b? chu?i g?i t?i
        while (LPC_UART0->LSR & 0x01) {
            char c = LPC_UART0->RBR;
            // Ch? nh?n du?c ký t? k?t thúc dòng (Enter)
            if (c == '\n' || c == '\r' || idx >= 63) {
                if (idx > 0) {
                    buffer[idx] = '\0';
                    idx = 0; // Reset index d? nh?n l?nh ti?p theo
                    
                    // --- DOI CHE DO ---
                    if (strncmp(buffer, "MODE:", 5) == 0) {
                        SystemMode new_mode = current_mode;
                        if (strstr(buffer, "AUTO"))        new_mode = MODE_AUTO;
                        else if (strstr(buffer, "MANUAL")) new_mode = MODE_MANUAL;
                        else if (strstr(buffer, "TIMER"))  new_mode = MODE_TIMER;
                        
                        if (new_mode != current_mode) {
                            current_mode = new_mode;
                            // T?t thi?t b? cho an toàn khi chuy?n mode
                            for(int i = 0; i < ACTUATOR_COUNT; i++) {
                                actuators[i].port->FIOCLR = actuators[i].pin;
                            }
                            if (selected_menu == 0) display_menu(-1);
                            send_uart_string("-> Da doi CHE DO thanh cong\n");
                        } else {
                            send_uart_string("-> He thong dang o san che do nay\n");
                        }
                    } 
                    // --- CHE DO MANUAL: DIEU KHIEN THU CONG ---
                    else if (strncmp(buffer, "CMD:", 4) == 0) {
                        if (current_mode == MODE_MANUAL) {
                            for (int i = 0; i < ACTUATOR_COUNT; i++) {
                                char on_cmd[16], off_cmd[16];
                                snprintf(on_cmd, sizeof(on_cmd), "CMD:%s:ON", actuators[i].name);
                                snprintf(off_cmd, sizeof(off_cmd), "CMD:%s:OFF", actuators[i].name);
                                if (strstr(buffer, on_cmd)) {
                                    actuators[i].port->FIOSET = actuators[i].pin;
                                    send_uart_string("-> OK: Da BAT thiet bi\n");
                                }
                                if (strstr(buffer, off_cmd)) {
                                    actuators[i].port->FIOCLR = actuators[i].pin;
                                    send_uart_string("-> OK: Da TAT thiet bi\n");
                                }
                            }
                        } else {
                            send_uart_string("-> LOI: Lenh CMD chi hoat dong trong MODE MANUAL!\n");
                        }
                    }
                    // --- CHE DO TIMER: SET THOI GIAN ---
                    else if (strncmp(buffer, "TIMER:", 6) == 0) {
                        if (current_mode == MODE_TIMER) {
                            char *ptr = buffer + 6;
                            char *act_str = strtok(ptr, ":");
                            char *h_str = strtok(NULL, ":");
                            char *m_str = strtok(NULL, ":");
                            char *s_str = strtok(NULL, ":");
                            
                            if (act_str && h_str && m_str && s_str) {
                                int h = atoi(h_str);
                                int m = atoi(m_str);
                                int s = atoi(s_str);
                                int found = 0;
                                
                                for (int i = 0; i < ACTUATOR_COUNT; i++) {
                                    if (strcmp(actuators[i].name, act_str) == 0) {
                                        timers[i].hours = (h >= 0 && h <= 23) ? h : 0;
                                        timers[i].minutes = (m >= 0 && m <= 59) ? m : 0;
                                        timers[i].seconds = (s >= 0 && s <= 59) ? s : 0;
                                        
                                        char ack[64];
                                        snprintf(ack, sizeof(ack), "-> OK: Da dat Timer %s luc %02d:%02d:%02d\n", 
                                                 act_str, timers[i].hours, timers[i].minutes, timers[i].seconds);
                                        send_uart_string(ack);
                                        found = 1;
                                        break;
                                    }
                                }
                                if (!found) send_uart_string("-> LOI: Sai ten Actuator (Heater/Sprinkle/Light)!\n");
                            } else {
                                send_uart_string("-> LOI: Sai cu phap. Mau dung: TIMER:Heater:14:30:00\n");
                            }
                        } else {
                            send_uart_string("-> LOI: Lenh TIMER chi hoat dong trong MODE TIMER!\n");
                        }
                    }
                }
            } else {
                buffer[idx++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ================= DISPLAY & MENUS =================

void display_menu(int prev_menu) {
    const char *mode_strings[] = {"[ AUTO ]  ", "[ MANUAL ]", "[ TIMER ] "};
    char menu_items[8][30];
    
    snprintf(menu_items[0], 30, "1. Mode: %s", mode_strings[current_mode]);
    strcpy(menu_items[1], "2. Xem Thong So (ADC)");
    strcpy(menu_items[2], "3. DK Thu Cong (Manual)");
    strcpy(menu_items[3], "4. Cai Dat Thoi Gian");
    strcpy(menu_items[4], "5. Nguong: Heater");
    strcpy(menu_items[5], "6. Nguong: Sprinkle");
    strcpy(menu_items[6], "7. Nguong: Light");
    strcpy(menu_items[7], "8. Thoi luong sang (Timer)");

    static int first_call = 1;

    xSemaphoreTake(glcd_mutex, portMAX_DELAY);
    if (first_call || prev_menu == -1) {
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        
        GLCD_SetForegroundColor(COLOR_WHITE);
        GLCD_SetBackgroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "   GREENHOUSE MENU   ");
        
        for (int i = 0; i < 8; i++) {
            char display_text[35];
            snprintf(display_text, sizeof(display_text), i == selected_menu ? "> %s" : "  %s", menu_items[i]);
            GLCD_SetBackgroundColor(i == selected_menu ? COLOR_GRAY : COLOR_WHITE);
            GLCD_SetForegroundColor(COLOR_BLACK);
            GLCD_DrawString(0, (i + 1) * 24, display_text);
        }
        first_call = 0;
    } else if (prev_menu != selected_menu) {
        char display_text[35];
        
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_SetForegroundColor(COLOR_BLACK);
        GLCD_DrawString(0, (prev_menu + 1) * 24, "                    ");
        snprintf(display_text, sizeof(display_text), "  %s", menu_items[prev_menu]);
        GLCD_DrawString(0, (prev_menu + 1) * 24, display_text);
        
        GLCD_SetBackgroundColor(COLOR_GRAY);
        GLCD_DrawString(0, (selected_menu + 1) * 24, "                    ");
        snprintf(display_text, sizeof(display_text), "> %s", menu_items[selected_menu]);
        GLCD_DrawString(0, (selected_menu + 1) * 24, display_text);
    }
    xSemaphoreGive(glcd_mutex);
}

void show_sensors(void) {
    char buffer[32];
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    xSemaphoreTake(glcd_mutex, portMAX_DELAY);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    xSemaphoreGive(glcd_mutex);

    while (1) {
        xSemaphoreTake(glcd_mutex, portMAX_DELAY);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "     SENSOR DATA     ");

        GLCD_SetForegroundColor(COLOR_BLACK);
        snprintf(buffer, sizeof(buffer), " Thoi gian: %02d:%02d:%02d", current_time.hours, current_time.minutes, current_time.seconds);
        GLCD_DrawString(0, 2 * 24, buffer);

        xSemaphoreTake(adc_mutex, portMAX_DELAY);
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            snprintf(buffer, sizeof(buffer), " %-9s: %04d/4095", actuators[i].name, adc_values[i]);
            GLCD_DrawString(0, (4 + i) * 24, buffer);
        }
        xSemaphoreGive(adc_mutex);
        
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 8 * 24, "  An CENTER de thoat");
        xSemaphoreGive(glcd_mutex);

        uint32_t joystick = read_joystick();
        uint32_t current_time_ms = xTaskGetTickCount();
        if (current_time_ms - last_action >= DEBOUNCE_TIME_MS && 
            (joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
            last_action = current_time_ms;
            break;
        }
        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    display_menu(-1);
}

void control_actuators(void) {
    static int first_call = 1;
    int selected = 0;
    int prev_selected = -1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    if (first_call) {
        xSemaphoreTake(glcd_mutex, portMAX_DELAY);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "   MANUAL CONTROL    ");
        
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            char display_text[35];
            GLCD_SetBackgroundColor(i == selected ? COLOR_GRAY : COLOR_WHITE);
            GLCD_SetForegroundColor(COLOR_BLACK);
            GLCD_DrawString(0, (i + 2) * 24, "                    ");
            snprintf(display_text, sizeof(display_text), i == selected ? "> %-9s: [%s]" : "  %-9s:  %s ",
                     actuators[i].name, 
                     actuators[i].port->FIOPIN & actuators[i].pin ? " ON" : "OFF");
            GLCD_DrawString(0, (i + 2) * 24, display_text);
        }
        
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 7 * 24, " LEN/XUONG de chon");
        GLCD_DrawString(0, 8 * 24, " TRAI/PHAI de bat/tat");
        xSemaphoreGive(glcd_mutex);
        first_call = 0;
    }

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time_ms = xTaskGetTickCount();
        int update_display = 0;

        if (current_time_ms - last_action >= DEBOUNCE_TIME_MS) {
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                prev_selected = selected;
                selected = (selected > 0) ? selected - 1 : 0;
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                prev_selected = selected;
                selected = (selected < ACTUATOR_COUNT - 1) ? selected + 1 : ACTUATOR_COUNT - 1;
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & (JOYSTICK_LEFT | JOYSTICK_RIGHT)) && 
                !(prev_joystick & (JOYSTICK_LEFT | JOYSTICK_RIGHT))) {
                toggle_actuator(selected);
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time_ms;
                break;
            }

            if (update_display) {
                xSemaphoreTake(glcd_mutex, portMAX_DELAY);
                char display_text[35];
                if (prev_selected != -1) {
                    GLCD_SetBackgroundColor(COLOR_WHITE);
                    GLCD_SetForegroundColor(COLOR_BLACK);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, "                    ");
                    snprintf(display_text, sizeof(display_text), "  %-9s:  %s ",
                             actuators[prev_selected].name,
                             actuators[prev_selected].port->FIOPIN & actuators[prev_selected].pin ? " ON" : "OFF");
                    GLCD_DrawString(0, (prev_selected + 2) * 24, display_text);
                }
                GLCD_SetBackgroundColor(COLOR_GRAY);
                GLCD_DrawString(0, (selected + 2) * 24, "                    ");
                snprintf(display_text, sizeof(display_text), "> %-9s: [%s]",
                         actuators[selected].name,
                         actuators[selected].port->FIOPIN & actuators[selected].pin ? " ON" : "OFF");
                GLCD_DrawString(0, (selected + 2) * 24, display_text);
                xSemaphoreGive(glcd_mutex);
            }
        }
        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    display_menu(-1);
}

void adjust_threshold(ActuatorType type) {
    char buffer[30];
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;
    int current_adc_display = -1;

    xSemaphoreTake(glcd_mutex, portMAX_DELAY);
    GLCD_SetBackgroundColor(COLOR_WHITE);
    GLCD_ClearScreen();
    
    GLCD_SetForegroundColor(COLOR_BLUE);
    snprintf(buffer, sizeof(buffer), " SET NGUONG: %s", actuators[type].name);
    GLCD_DrawString(0, 0, buffer);
    
    GLCD_SetForegroundColor(COLOR_RED);
    GLCD_DrawString(0, 7 * 24, " L/R: +/- 100");
    GLCD_DrawString(0, 8 * 24, " U/D: +/- 10  C: Thoat");
    
    GLCD_SetForegroundColor(COLOR_BLACK);
    snprintf(buffer, sizeof(buffer), " Muc Nguong: [%d]  ", *actuators[type].threshold);
    GLCD_DrawString(0, 4 * 24, buffer);
    xSemaphoreGive(glcd_mutex);

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time_ms = xTaskGetTickCount();

        xSemaphoreTake(adc_mutex, portMAX_DELAY);
        int live_adc = adc_values[type];
        xSemaphoreGive(adc_mutex);
        
        if (live_adc != current_adc_display) {
            xSemaphoreTake(glcd_mutex, portMAX_DELAY);
            GLCD_SetBackgroundColor(COLOR_WHITE);
            GLCD_SetForegroundColor(COLOR_BLACK);
            snprintf(buffer, sizeof(buffer), " ADC Hien tai: %04d", live_adc);
            GLCD_DrawString(0, 2 * 24, buffer);
            xSemaphoreGive(glcd_mutex);
            current_adc_display = live_adc;
        }

        if (current_time_ms - last_action >= DEBOUNCE_TIME_MS) {
            int changed = 0;
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                *actuators[type].threshold += 10; changed = 1;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                *actuators[type].threshold -= 10; changed = 1;
            }
            if ((joystick & JOYSTICK_RIGHT) && !(prev_joystick & JOYSTICK_RIGHT)) {
                *actuators[type].threshold += 100; changed = 1;
            }
            if ((joystick & JOYSTICK_LEFT) && !(prev_joystick & JOYSTICK_LEFT)) {
                *actuators[type].threshold -= 100; changed = 1;
            }
            
            if (changed) {
                if (*actuators[type].threshold > MAX_ADC_VALUE) *actuators[type].threshold = MAX_ADC_VALUE;
                if (*actuators[type].threshold < 0) *actuators[type].threshold = 0;
                
                xSemaphoreTake(glcd_mutex, portMAX_DELAY);
                GLCD_SetBackgroundColor(COLOR_WHITE);
                GLCD_SetForegroundColor(COLOR_BLACK);
                snprintf(buffer, sizeof(buffer), " Muc Nguong: [%d]  ", *actuators[type].threshold);
                GLCD_DrawString(0, 4 * 24, buffer);
                xSemaphoreGive(glcd_mutex);
                last_action = current_time_ms;
            }

            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                last_action = current_time_ms;
                break;
            }
        }
        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    display_menu(-1);
}

void adjust_durations(void) {
    static int first_call = 1;
    int selected = 0;
    int prev_selected = -1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;

    if (first_call) {
        xSemaphoreTake(glcd_mutex, portMAX_DELAY);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, " CHINH THOI GIAN BAT ");
        
        for (int i = 0; i < ACTUATOR_COUNT; i++) {
            char display_text[35];
            GLCD_SetBackgroundColor(i == selected ? COLOR_GRAY : COLOR_WHITE);
            GLCD_SetForegroundColor(COLOR_BLACK);
            GLCD_DrawString(0, (i + 2) * 24, "                    ");
            snprintf(display_text, sizeof(display_text), i == selected ? "> %-9s: [%04d] ms" : "  %-9s:  %04d ms",
                     actuators[i].name, *actuators[i].duration);
            GLCD_DrawString(0, (i + 2) * 24, display_text);
        }
        
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 7 * 24, " U/D: Chon  L/R: Chinh");
        GLCD_DrawString(0, 8 * 24, " CENTER: Thoat");
        xSemaphoreGive(glcd_mutex);
        first_call = 0;
    }

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time_ms = xTaskGetTickCount();

        if (current_time_ms - last_action >= DEBOUNCE_TIME_MS) {
            int update_display = 0;
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                prev_selected = selected;
                selected = (selected > 0) ? selected - 1 : 0;
                update_display = 1;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                prev_selected = selected;
                selected = (selected < ACTUATOR_COUNT - 1) ? selected + 1 : ACTUATOR_COUNT - 1;
                update_display = 1;
            }
            if ((joystick & JOYSTICK_LEFT) && !(prev_joystick & JOYSTICK_LEFT)) {
                *actuators[selected].duration -= 100;
                if (*actuators[selected].duration < MIN_DURATION_MS) *actuators[selected].duration = MIN_DURATION_MS;
                update_display = 1;
            }
            if ((joystick & JOYSTICK_RIGHT) && !(prev_joystick & JOYSTICK_RIGHT)) {
                *actuators[selected].duration += 100;
                if (*actuators[selected].duration > MAX_DURATION_MS) *actuators[selected].duration = MAX_DURATION_MS;
                update_display = 1;
            }
            
            if (update_display) last_action = current_time_ms;

            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time_ms;
                break;
            }

            if (update_display) {
                xSemaphoreTake(glcd_mutex, portMAX_DELAY);
                char display_text[35];
                if (prev_selected != -1) {
                    GLCD_SetBackgroundColor(COLOR_WHITE);
                    GLCD_SetForegroundColor(COLOR_BLACK);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, "                    ");
                    snprintf(display_text, sizeof(display_text), "  %-9s:  %04d ms",
                             actuators[prev_selected].name, *actuators[prev_selected].duration);
                    GLCD_DrawString(0, (prev_selected + 2) * 24, display_text);
                }
                GLCD_SetBackgroundColor(COLOR_GRAY);
                GLCD_SetForegroundColor(COLOR_BLACK);
                GLCD_DrawString(0, (selected + 2) * 24, "                    ");
                snprintf(display_text, sizeof(display_text), "> %-9s: [%04d] ms",
                         actuators[selected].name, *actuators[selected].duration);
                GLCD_DrawString(0, (selected + 2) * 24, display_text);
                xSemaphoreGive(glcd_mutex);
            }
        }
        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    display_menu(-1);
}

void control_timers(void) {
    static int first_call = 1;
    uint32_t prev_joystick = 0;
    uint32_t last_action = 0;
    char buffer[32];
    int prev_selected_field = -1;
    int prev_selected_actuator = -1;
    int update_display = 0;

    selected_actuator = 0;
    selected_field = 0; 

    if (first_call) {
        xSemaphoreTake(glcd_mutex, portMAX_DELAY);
        GLCD_SetBackgroundColor(COLOR_WHITE);
        GLCD_ClearScreen();
        
        GLCD_SetForegroundColor(COLOR_BLUE);
        GLCD_DrawString(0, 0, "   CAI DAT HEN GIO   ");
        
        GLCD_SetForegroundColor(COLOR_RED);
        GLCD_DrawString(0, 7 * 24, " U/D: Chinh Thong so");
        GLCD_DrawString(0, 8 * 24, " L/R: Chuyen  C: Xong");
        xSemaphoreGive(glcd_mutex);
        first_call = 0;
        update_display = 1;
    }

    while (1) {
        uint32_t joystick = read_joystick();
        uint32_t current_time_ms = xTaskGetTickCount();

        if (current_time_ms - last_action >= DEBOUNCE_TIME_MS) {
            if ((joystick & JOYSTICK_UP) && !(prev_joystick & JOYSTICK_UP)) {
                if (selected_field == 0) timers[selected_actuator].hours = (timers[selected_actuator].hours < 23) ? timers[selected_actuator].hours + 1 : 0;
                else if (selected_field == 1) timers[selected_actuator].minutes = (timers[selected_actuator].minutes < 59) ? timers[selected_actuator].minutes + 1 : 0;
                else if (selected_field == 2) timers[selected_actuator].seconds = (timers[selected_actuator].seconds < 59) ? timers[selected_actuator].seconds + 1 : 0;
                else if (selected_field == -1) {
                    prev_selected_actuator = selected_actuator;
                    selected_actuator = (selected_actuator > 0) ? selected_actuator - 1 : 0;
                }
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_DOWN) && !(prev_joystick & JOYSTICK_DOWN)) {
                if (selected_field == 0) timers[selected_actuator].hours = (timers[selected_actuator].hours > 0) ? timers[selected_actuator].hours - 1 : 23;
                else if (selected_field == 1) timers[selected_actuator].minutes = (timers[selected_actuator].minutes > 0) ? timers[selected_actuator].minutes - 1 : 59;
                else if (selected_field == 2) timers[selected_actuator].seconds = (timers[selected_actuator].seconds > 0) ? timers[selected_actuator].seconds - 1 : 59;
                else if (selected_field == -1) {
                    prev_selected_actuator = selected_actuator;
                    selected_actuator = (selected_actuator < ACTUATOR_COUNT - 1) ? selected_actuator + 1 : ACTUATOR_COUNT - 1;
                }
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_LEFT) && !(prev_joystick & JOYSTICK_LEFT)) {
                prev_selected_field = selected_field;
                selected_field = (selected_field > -1) ? selected_field - 1 : -1;
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_RIGHT) && !(prev_joystick & JOYSTICK_RIGHT)) {
                prev_selected_field = selected_field;
                selected_field = (selected_field < 2) ? selected_field + 1 : 2;
                update_display = 1;
                last_action = current_time_ms;
            }
            if ((joystick & JOYSTICK_CENTER) && !(prev_joystick & JOYSTICK_CENTER)) {
                first_call = 1;
                last_action = current_time_ms;
                break;
            }
        }

        if (update_display || prev_selected_actuator != selected_actuator || prev_selected_field != selected_field) {
            xSemaphoreTake(glcd_mutex, portMAX_DELAY);
            GLCD_SetBackgroundColor(COLOR_WHITE);
            for (int i = 0; i < ACTUATOR_COUNT; i++) {
                GLCD_SetForegroundColor(COLOR_BLACK);
                
                char t_str[20];
                if (i == selected_actuator) {
                    GLCD_SetBackgroundColor(COLOR_GRAY);
                    if (selected_field == 0) snprintf(t_str, 20, "[%02d]:%02d:%02d", timers[i].hours, timers[i].minutes, timers[i].seconds);
                    else if (selected_field == 1) snprintf(t_str, 20, "%02d:[%02d]:%02d", timers[i].hours, timers[i].minutes, timers[i].seconds);
                    else if (selected_field == 2) snprintf(t_str, 20, "%02d:%02d:[%02d]", timers[i].hours, timers[i].minutes, timers[i].seconds);
                    else snprintf(t_str, 20, "%02d:%02d:%02d", timers[i].hours, timers[i].minutes, timers[i].seconds);
                } else {
                    GLCD_SetBackgroundColor(COLOR_WHITE);
                    snprintf(t_str, 20, " %02d:%02d:%02d ", timers[i].hours, timers[i].minutes, timers[i].seconds);
                }

                snprintf(buffer, sizeof(buffer), "%s%-9s: %s",
                         i == selected_actuator ? "> " : "  ",
                         actuators[i].name, t_str);
                
                GLCD_DrawString(0, (i + 2) * 24, "                    ");
                GLCD_DrawString(0, (i + 2) * 24, buffer);
            }
            xSemaphoreGive(glcd_mutex);
            update_display = 0;
            prev_selected_actuator = selected_actuator;
            prev_selected_field = selected_field;
        }

        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    display_menu(-1);
}

void menu_thread(void *arg) {
    init_gpio();
    GLCD_Initialize();
    GLCD_SetFont(&GLCD_Font_16x24);
    display_menu(-1);

    int prev_menu = selected_menu;
    uint32_t prev_joystick = 0;

    while (1) {
        uint32_t joystick = read_joystick();
        int updated = 0;

        if (joystick && (joystick != prev_joystick)) {
            if (joystick & JOYSTICK_UP) {
                if (selected_menu > 0) {
                    selected_menu--;
                    updated = 1;
                }
            } else if (joystick & JOYSTICK_DOWN) {
                if (selected_menu < 7) {
                    selected_menu++;
                    updated = 1;
                }
            } 
            else if (selected_menu == 0 && (joystick & (JOYSTICK_LEFT | JOYSTICK_RIGHT))) {
                if (joystick & JOYSTICK_LEFT) {
                    current_mode = (current_mode == 0) ? 2 : current_mode - 1;
                }
                if (joystick & JOYSTICK_RIGHT) {
                    current_mode = (current_mode == 2) ? 0 : current_mode + 1;
                }
                
                for(int i = 0; i < ACTUATOR_COUNT; i++) {
                    actuators[i].port->FIOCLR = actuators[i].pin;
                }
                
                display_menu(-1); 
                vTaskDelay(pdMS_TO_TICKS(200));     
            }
            else if (joystick & JOYSTICK_CENTER) {
                switch (selected_menu) {
                    case 0: break; 
                    case 1: show_sensors(); break;
                    case 2: control_actuators(); break;
                    case 3: control_timers(); break;
                    case 4: adjust_threshold(ACTUATOR_HEATER); break;
                    case 5: adjust_threshold(ACTUATOR_SPRINKLER); break;
                    case 6: adjust_threshold(ACTUATOR_LIGHT); break;
                    case 7: adjust_durations(); break;
                }
            }
        }

        if (updated || selected_menu != prev_menu) {
            display_menu(prev_menu);
            prev_menu = selected_menu;
        }
        
        prev_joystick = joystick;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Main Function
int main(void) {
    SystemCoreClockUpdate();
    init_hardware();

    adc_mutex = xSemaphoreCreateMutex();
    glcd_mutex = xSemaphoreCreateMutex();
    uart_mutex_id = xSemaphoreCreateMutex(); // Khoi tao Mutex cho UART
    
    actuator_sems[ACTUATOR_HEATER] = xSemaphoreCreateBinary();
    actuator_sems[ACTUATOR_SPRINKLER] = xSemaphoreCreateBinary();
    actuator_sems[ACTUATOR_LIGHT] = xSemaphoreCreateBinary();
    xSemaphoreGive(actuator_sems[ACTUATOR_HEATER]);

    xTaskCreate(sensor_thread, "Sensor", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(uart_thread, "UARTTx", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(uart_receive_thread, "UARTRx", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(menu_thread, "Menu", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(timer_monitor_thread, "TimerMon", 256, NULL, tskIDLE_PRIORITY + 1, NULL); 
    static ActuatorType types[ACTUATOR_COUNT] = {ACTUATOR_HEATER, ACTUATOR_SPRINKLER, ACTUATOR_LIGHT};
    for (int i = 0; i < ACTUATOR_COUNT; i++) {
        xTaskCreate(monitor_thread, "Monitor", 256, &types[i], tskIDLE_PRIORITY + 1, NULL);
        xTaskCreate(control_thread, "Control", 256, &types[i], tskIDLE_PRIORITY + 1, NULL);
    }

    vTaskStartScheduler();
    while (1);
}