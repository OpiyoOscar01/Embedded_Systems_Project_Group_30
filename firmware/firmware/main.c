/******************************************************************************
 * ECOBIN SMART WASTE MANAGEMENT SYSTEM - COMPLETE FIRMWARE
 * 
 * @file    ecobin_firmware.c
 * @brief   Production firmware for EcoBin smart waste monitoring system
 * @target  ATmega328P @ 16MHz
 * @author  EcoBin Development Team
 * @version 2.0
 * @date    2025
 * 
 * @description
 * Firmware implementing state-machine based waste bin 
 * monitoring with ultrasonic fill level sensing, fire detection, LCD 
 * interface, and PIN-protected maintenance system.
 * 
 * Features:
 *   - Multi-threshold fill level monitoring (80%, 95%)
 *   - Priority-based fire detection with confirmation
 *   - LCD 16×2 real-time status display
 *   - Keypad-based maintenance interface (4×3 matrix)
 *   - Non-volatile event logging (EEPROM)
 *   - Sensor calibration with median filtering
 *   - Configurable alert patterns
 * 
 * Hardware Configuration:
 *   - ATmega328P microcontroller @ 16MHz
 *   - HC-SR04 ultrasonic sensor
 *   - LCD 16×2 (HD44780, 4-bit mode)
 *   - Flame sensor module (active LOW)
 *   - 4×3 keypad matrix
 *   - Active buzzer
 *   - Status LED
 * 
 * Compliance:
 *   - MISRA-C guidelines (subset)
 *   - IEC 61508 safety considerations
 *   - Embedded C coding standards
 * 
 * SDG Alignment:
 *   - SDG 11: Sustainable Cities and Communities
 *   - SDG 6: Clean Water and Sanitation
 * 
 * License: Educational/Research Use
 ******************************************************************************/

// ============================================================================
// HEADER INCLUDES
// ============================================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


// ============================================================================
// HARDWARE CONFIGURATION - PORT/PIN DEFINITIONS
// ============================================================================

// Ultrasonic Sensor HC-SR04
#define US_TRIG_PORT        PORTD
#define US_TRIG_DDR         DDRD
#define US_TRIG_BIT         PD2

#define US_ECHO_PIN_REG     PIND
#define US_ECHO_DDR         DDRD
#define US_ECHO_PORT        PORTD
#define US_ECHO_BIT         PD3

// LCD 16x2 (4-bit mode, HD44780 compatible)
#define LCD_RS_PORT         PORTB
#define LCD_RS_DDR          DDRB
#define LCD_RS_BIT          PB0

#define LCD_EN_PORT         PORTB
#define LCD_EN_DDR          DDRB
#define LCD_EN_BIT          PB1

#define LCD_D4_PORT         PORTB
#define LCD_D4_DDR          DDRB
#define LCD_D4_BIT          PB2

#define LCD_D5_PORT         PORTB
#define LCD_D5_DDR          DDRB
#define LCD_D5_BIT          PB3

#define LCD_D6_PORT         PORTB
#define LCD_D6_DDR          DDRB
#define LCD_D6_BIT          PB4

#define LCD_D7_PORT         PORTB
#define LCD_D7_DDR          DDRB
#define LCD_D7_BIT          PB5

// Flame Sensor (Active LOW)
#define FLAME_PIN_REG       PIND
#define FLAME_DDR           DDRD
#define FLAME_PORT          PORTD
#define FLAME_BIT           PD4

// ============================================================================
// KEYPAD 4×3 (4 Rows, 3 Columns) - UPDATED CONFIGURATION
// ============================================================================

// Row Pins (Outputs - scan lines)
#define KEY_ROW_A_PORT      PORTC
#define KEY_ROW_A_DDR       DDRC
#define KEY_ROW_A_BIT       PC0

#define KEY_ROW_B_PORT      PORTC
#define KEY_ROW_B_DDR       DDRC
#define KEY_ROW_B_BIT       PC1

#define KEY_ROW_C_PORT      PORTC
#define KEY_ROW_C_DDR       DDRC
#define KEY_ROW_C_BIT       PC2

#define KEY_ROW_D_PORT      PORTC
#define KEY_ROW_D_DDR       DDRC
#define KEY_ROW_D_BIT       PC3

// Column Pins (Inputs with pullups)
#define KEY_COL_1_PIN_REG   PINC
#define KEY_COL_1_DDR       DDRC
#define KEY_COL_1_PORT      PORTC
#define KEY_COL_1_BIT       PC4

#define KEY_COL_2_PIN_REG   PINC
#define KEY_COL_2_DDR       DDRC
#define KEY_COL_2_PORT      PORTC
#define KEY_COL_2_BIT       PC5

#define KEY_COL_3_PIN_REG   PIND
#define KEY_COL_3_DDR       DDRD
#define KEY_COL_3_PORT      PORTD
#define KEY_COL_3_BIT       PD5


// Actuators
#define BUZZER_PORT         PORTD
#define BUZZER_DDR          DDRD
#define BUZZER_BIT          PD6

#define LED_PORT            PORTD
#define LED_DDR             DDRD
#define LED_BIT             PD7


// ============================================================================
// SYSTEM CONSTANTS AND CONFIGURATION
// ============================================================================

// Clock Configuration
#ifndef F_CPU
#define F_CPU               16000000UL  // 16 MHz system clock
#endif

// Sensor Parameters
#define BIN_HEIGHT_CM       100         // Total bin height (cm)
#define SENSOR_OFFSET_CM    5           // Sensor mounting offset (cm)
#define US_TIMEOUT_US       30000       // Ultrasonic timeout (30ms)
#define US_SAMPLE_COUNT     5           // Samples for median filter
#define US_SAMPLE_DELAY_MS  50          // Delay between samples

// Threshold Configuration
#define PRIMARY_THRESHOLD   80          // Primary alert at 80%
#define CRITICAL_THRESHOLD  95          // Critical alert at 95%
#define THRESHOLD_HYSTERESIS 3          // Hysteresis (3%)

// Timing Parameters (milliseconds)
#define FIRE_CHECK_INTERVAL     500     // Fire sensor check period
#define FILL_CHECK_INTERVAL     2000    // Fill measurement period
#define LCD_REFRESH_INTERVAL    2000    // Display update period
#define DEBOUNCE_DELAY          50      // Keypad debounce time

// Security
#define MAINTENANCE_PIN     "1234"      // Default PIN
#define PIN_LENGTH          4           // PIN digit count
#define PIN_TIMEOUT_MS      5000        // PIN entry timeout

// Data Logging (FIXED: Reduced to prevent RAM overflow)
#define MAX_LOG_ENTRIES     20          // RAM log buffer size (160 bytes)
#define EEPROM_LOG_ENTRIES  50          // Persistent EEPROM logs

// EEPROM Addresses
#define EEPROM_ADDR_CALIBRATION 0x00    // Calibration offset (1 byte)
#define EEPROM_ADDR_LOG_COUNT   0x04    // Log count (2 bytes)
#define EEPROM_ADDR_LOG_START   0x10    // Log entries start

// Display Configuration
#define LCD_ROWS            2
#define LCD_COLS            16
#define BIN_ID              "001"


// ============================================================================
// TYPE DEFINITIONS AND ENUMERATIONS
// ============================================================================

/**
 * @brief System state machine states
 */
typedef enum {
    STATE_INIT,                 // System initialization
    STATE_NORMAL,               // Normal operation (fill < 80%)
    STATE_PRIMARY_ALERT,        // Primary alert (80% ? fill < 95%)
    STATE_CRITICAL_ALERT,       // Critical alert (fill ? 95%)
    STATE_FIRE_ALERT,           // Fire detected
    STATE_MAINTENANCE           // Maintenance mode
} SystemState_t;

/**
 * @brief Event types for logging
 */
typedef enum {
    EVENT_SYSTEM_BOOT,          // System powered on
    EVENT_THRESHOLD_PRIMARY,    // Primary threshold reached
    EVENT_THRESHOLD_CRITICAL,   // Critical threshold reached
    EVENT_FIRE_DETECTED,        // Flame sensor triggered
    EVENT_FIRE_CLEARED,         // Fire condition cleared
    EVENT_RESET_PERFORMED,      // Manual reset executed
    EVENT_CALIBRATION_START,    // Calibration initiated
    EVENT_CALIBRATION_DONE,     // Calibration completed
    EVENT_SENSOR_ERROR,         // Sensor malfunction
    EVENT_MAINTENANCE_ENTER,    // Entered maintenance mode
    EVENT_MAINTENANCE_EXIT      // Exited maintenance mode
} EventType_t;

/**
 * @brief LED blinking patterns
 */
typedef enum {
    LED_OFF,                    // LED disabled
    LED_SOLID,                  // Solid on
    LED_SLOW_BLINK,             // 1 Hz (1000ms period)
    LED_FAST_BLINK,             // 2 Hz (500ms period)
    LED_RAPID_BLINK             // 5 Hz (200ms period)
} LEDPattern_t;

/**
 * @brief Buzzer sound patterns
 */
typedef enum {
    BUZZER_OFF,                 // Silent
    BUZZER_BEEP,                // Single short beep
    BUZZER_INTERMITTENT,        // 200ms ON, 1800ms OFF
    BUZZER_CONTINUOUS,          // Solid tone
    BUZZER_ALARM                // 100ms ON, 100ms OFF
} BuzzerPattern_t;

/**
 * @brief Event log entry structure
 */
typedef struct {
    uint32_t timestamp;         // Seconds since boot
    EventType_t event_type;     // Event type identifier
    uint8_t fill_level;         // Fill percentage at event time
    uint16_t distance_cm;       // Raw distance reading
} LogEntry_t;

/**
 * @brief Main system data structure
 */
typedef struct {
    SystemState_t current_state;    // Active state
    SystemState_t previous_state;   // Previous state
    uint8_t fill_percentage;        // Current fill (0-100%)
    uint16_t distance_cm;           // Current distance (cm)
    bool fire_detected;             // Fire sensor status
    bool fire_confirmed;            // Fire confirmation flag
    bool authenticated;             // Maintenance access
    uint16_t log_count;             // Total logged events
    int8_t calibration_offset;      // Sensor calibration offset
    uint32_t uptime_seconds;        // System uptime (seconds)
    LogEntry_t log_buffer[MAX_LOG_ENTRIES]; // Event log buffer
} SystemData_t;

/**
 * @brief Timing control structure
 */
typedef struct {
    uint32_t last_fire_check;       // Last fire check timestamp
    uint32_t last_fill_check;       // Last fill check timestamp
    uint32_t last_lcd_update;       // Last LCD update timestamp
    uint32_t last_led_toggle;       // Last LED toggle timestamp
    uint32_t last_buzzer_toggle;    // Last buzzer toggle timestamp
    uint32_t state_entry_time;      // State entry timestamp
} TimingControl_t;

/**
 * @brief Sensor data structure
 */
typedef struct {
    uint16_t raw_samples[US_SAMPLE_COUNT]; // Raw sensor samples
    uint16_t filtered_distance;     // Median filtered value
    uint8_t valid_sample_count;     // Valid readings count
    uint8_t error_count;            // Consecutive error counter
} SensorData_t;


// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System data structures
static SystemData_t g_system;
static TimingControl_t g_timing;
static SensorData_t g_sensor;

// Actuator control
static LEDPattern_t g_led_pattern = LED_OFF;
static BuzzerPattern_t g_buzzer_pattern = BUZZER_OFF;
static bool g_buzzer_state = false;

// Millisecond counter (updated by Timer0 ISR)
static volatile uint32_t g_millis_counter = 0;


// ============================================================================
// FUNCTION PROTOTYPES - HARDWARE ABSTRACTION LAYER
// ============================================================================

// GPIO Functions
static void HAL_GPIO_Init(void);
static inline void HAL_GPIO_Write(volatile uint8_t *port, uint8_t bit, bool state);
static inline bool HAL_GPIO_Read(volatile uint8_t *pin_reg, uint8_t bit);
static inline void HAL_GPIO_Toggle(volatile uint8_t *port, uint8_t bit);

// Timer Functions
static void HAL_Timer_Init(void);
static uint32_t HAL_Millis(void);
static void HAL_Delay_ms(uint16_t ms);
static inline void HAL_Delay_us(uint16_t us);

// EEPROM Functions
static void HAL_EEPROM_Write_Byte(uint16_t address, uint8_t data);
static uint8_t HAL_EEPROM_Read_Byte(uint16_t address);
static void HAL_EEPROM_Write_Block(uint16_t address, const void *data, uint16_t length);
static void HAL_EEPROM_Read_Block(uint16_t address, void *buffer, uint16_t length);


// ============================================================================
// FUNCTION PROTOTYPES - DEVICE DRIVERS
// ============================================================================

// LCD Driver
static void LCD_Send_Nibble(uint8_t nibble, bool is_data);
static void LCD_Send_Byte(uint8_t byte, bool is_data);
static void LCD_Command(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_Init(void);
static void LCD_Clear(void);
static void LCD_Set_Cursor(uint8_t row, uint8_t col);
static void LCD_Print(const char *text);
static void LCD_Print_Number(uint16_t number);

// Ultrasonic Sensor Driver
static void US_Trigger_Pulse(void);
static uint32_t US_Measure_Echo_Pulse(void);
static uint16_t US_Calculate_Distance(uint32_t pulse_width_us);
static uint16_t US_Measure_Distance_Single(void);

// Flame Sensor Driver
static bool Flame_Sensor_Read(void);
static bool Flame_Sensor_Confirm(void);

// Keypad Driver
static char Keypad_Scan_Raw(void);
static char Keypad_Read_Debounced(void);

// Actuator Drivers
static void LED_Set_Pattern(LEDPattern_t pattern);
static void LED_Update(void);
static void Buzzer_Set_Pattern(BuzzerPattern_t pattern);
static void Buzzer_Update(void);


// ============================================================================
// FUNCTION PROTOTYPES - APPLICATION LAYER
// ============================================================================

// Sensor Processing
static void Measure_Fill_Level(void);
static void Sort_Array(uint16_t *array, uint8_t size);
static void Calibrate_Sensor(void);
static void Check_Fire_Sensor(void);

// User Interface
static void Update_LCD_Display(void);
static void Display_Fire_Alert(void);
static void Display_Welcome_Screen(void);
static void Display_Maintenance_Menu(void);
static bool Prompt_PIN_Entry(void);
static void Handle_Maintenance_Menu(void);
static void Perform_System_Reset(void);
static void Display_Event_Logs(void);
static const char* Get_Event_Text(EventType_t event);

// Event Logging
static void Log_Event(EventType_t event, uint8_t fill_level);
static void Save_Event_To_EEPROM(const LogEntry_t *entry);
static void Load_Event_Log_From_EEPROM(void);

// State Machine
static void Transition_To_State(SystemState_t new_state);
static void Check_State_Transitions(void);
static void Exit_Maintenance_Mode(void);

// System Functions
static void System_Init(void);
static void Main_Loop(void);

// Utility Functions
static void Integer_To_String(uint16_t num, char *buffer, uint8_t base);
static bool String_Compare(const char *str1, const char *str2, uint8_t length);


// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

/**
 * @brief Timer0 overflow interrupt - 1ms system tick
 * 
 * Configured for 1ms overflow:
 *   - Timer0: 8-bit, Normal mode
 *   - Prescaler: 64 (16MHz / 64 = 250kHz = 4µs per tick)
 *   - Overflow: 256 ticks = 1.024ms ? 1ms
 *   - Preload: 6 (256 - 250 = 6) for exactly 1ms
 */
ISR(TIMER0_OVF_vect) {
    TCNT0 = 6;  // Reload for 1ms interval
    g_millis_counter++;
    
    // Update uptime seconds counter
    if ((g_millis_counter % 1000) == 0) {
        g_system.uptime_seconds++;
    }
}


// ============================================================================
// HARDWARE ABSTRACTION LAYER IMPLEMENTATION
// ============================================================================

/**
 * @brief Initialize all GPIO ports and pins
 */
static void HAL_GPIO_Init(void) {
    // Ultrasonic Sensor
    US_TRIG_DDR |= (1 << US_TRIG_BIT);      // TRIG as output
    US_ECHO_DDR &= ~(1 << US_ECHO_BIT);     // ECHO as input
    US_TRIG_PORT &= ~(1 << US_TRIG_BIT);    // TRIG initial LOW
    
    // LCD Pins (all outputs, initial LOW)
    LCD_RS_DDR |= (1 << LCD_RS_BIT);
    LCD_EN_DDR |= (1 << LCD_EN_BIT);
    LCD_D4_DDR |= (1 << LCD_D4_BIT);
    LCD_D5_DDR |= (1 << LCD_D5_BIT);
    LCD_D6_DDR |= (1 << LCD_D6_BIT);
    LCD_D7_DDR |= (1 << LCD_D7_BIT);
    
    LCD_RS_PORT &= ~(1 << LCD_RS_BIT);
    LCD_EN_PORT &= ~(1 << LCD_EN_BIT);
    LCD_D4_PORT &= ~(1 << LCD_D4_BIT);
    LCD_D5_PORT &= ~(1 << LCD_D5_BIT);
    LCD_D6_PORT &= ~(1 << LCD_D6_BIT);
    LCD_D7_PORT &= ~(1 << LCD_D7_BIT);
    
    // Flame Sensor (input with pullup)
    FLAME_DDR &= ~(1 << FLAME_BIT);
    FLAME_PORT |= (1 << FLAME_BIT);         // Enable pullup
    
    // ========================================================================
    // Keypad 4×3 (4 Rows as Outputs, 3 Columns as Inputs with Pullups)
    // ========================================================================
    
    // Configure Rows as Outputs (initial HIGH - inactive)
    KEY_ROW_A_DDR |= (1 << KEY_ROW_A_BIT);
    KEY_ROW_B_DDR |= (1 << KEY_ROW_B_BIT);
    KEY_ROW_C_DDR |= (1 << KEY_ROW_C_BIT);
    KEY_ROW_D_DDR |= (1 << KEY_ROW_D_BIT);
    
    KEY_ROW_A_PORT |= (1 << KEY_ROW_A_BIT);  // HIGH (inactive)
    KEY_ROW_B_PORT |= (1 << KEY_ROW_B_BIT);  // HIGH (inactive)
    KEY_ROW_C_PORT |= (1 << KEY_ROW_C_BIT);  // HIGH (inactive)
    KEY_ROW_D_PORT |= (1 << KEY_ROW_D_BIT);  // HIGH (inactive)
    
    // Configure Columns as Inputs with Pullups
    KEY_COL_1_DDR &= ~(1 << KEY_COL_1_BIT);
    KEY_COL_2_DDR &= ~(1 << KEY_COL_2_BIT);
    KEY_COL_3_DDR &= ~(1 << KEY_COL_3_BIT);
    
    KEY_COL_1_PORT |= (1 << KEY_COL_1_BIT);  // Enable pullup
    KEY_COL_2_PORT |= (1 << KEY_COL_2_BIT);  // Enable pullup
    KEY_COL_3_PORT |= (1 << KEY_COL_3_BIT);  // Enable pullup
    
    // Buzzer (output, initial LOW)
    BUZZER_DDR |= (1 << BUZZER_BIT);
    BUZZER_PORT &= ~(1 << BUZZER_BIT);
    
    // LED (output, initial LOW)
    LED_DDR |= (1 << LED_BIT);
    LED_PORT &= ~(1 << LED_BIT);
}

/**
 * @brief Write digital state to GPIO pin
 */
static inline void HAL_GPIO_Write(volatile uint8_t *port, uint8_t bit, bool state) {
    if (state) {
        *port |= (1 << bit);
    } else {
        *port &= ~(1 << bit);
    }
}

/**
 * @brief Read digital state from GPIO pin
 */
static inline bool HAL_GPIO_Read(volatile uint8_t *pin_reg, uint8_t bit) {
    return (*pin_reg & (1 << bit)) != 0;
}

/**
 * @brief Toggle GPIO pin state
 */
static inline void HAL_GPIO_Toggle(volatile uint8_t *port, uint8_t bit) {
    *port ^= (1 << bit);
}

/**
 * @brief Initialize Timer0 for 1ms system tick
 */
static void HAL_Timer_Init(void) {
    // Set Timer0 to Normal mode
    TCCR0A = 0x00;
    
    // Set prescaler to 64 (CS01 | CS00)
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // Preload timer for 1ms overflow
    TCNT0 = 6;
    
    // Enable Timer0 overflow interrupt
    TIMSK0 = (1 << TOIE0);
}

/**
 * @brief Get milliseconds since system boot (thread-safe)
 */
static uint32_t HAL_Millis(void) {
    uint32_t ms;
    cli();  // Disable interrupts
    ms = g_millis_counter;
    sei();  // Enable interrupts
    return ms;
}

/**
 * @brief Blocking delay in milliseconds
 */
static void HAL_Delay_ms(uint16_t ms) {
    uint32_t start = HAL_Millis();
    while ((HAL_Millis() - start) < ms) {
        // Busy wait
    }
}

/**
 * @brief Blocking delay in microseconds
 */
static inline void HAL_Delay_us(uint16_t us) {
    while (us--) {
        _delay_us(1);
    }
}

/**
 * @brief Write single byte to EEPROM
 */
static void HAL_EEPROM_Write_Byte(uint16_t address, uint8_t data) {
    while (EECR & (1 << EEPE));  // Wait for previous write to complete
    EEAR = address;
    EEDR = data;
    EECR |= (1 << EEMPE);        // Master write enable
    EECR |= (1 << EEPE);         // Write enable
}

/**
 * @brief Read single byte from EEPROM
 */
static uint8_t HAL_EEPROM_Read_Byte(uint16_t address) {
    while (EECR & (1 << EEPE));  // Wait for previous write to complete
    EEAR = address;
    EECR |= (1 << EERE);         // Read enable
    return EEDR;
}

/**
 * @brief Write block of data to EEPROM
 */
static void HAL_EEPROM_Write_Block(uint16_t address, const void *data, uint16_t length) {
    const uint8_t *src = (const uint8_t *)data;
    for (uint16_t i = 0; i < length; i++) {
        HAL_EEPROM_Write_Byte(address + i, src[i]);
    }
}

/**
 * @brief Read block of data from EEPROM
 */
static void HAL_EEPROM_Read_Block(uint16_t address, void *buffer, uint16_t length) {
    uint8_t *dest = (uint8_t *)buffer;
    for (uint16_t i = 0; i < length; i++) {
        dest[i] = HAL_EEPROM_Read_Byte(address + i);
    }
}


// ============================================================================
// LCD DRIVER IMPLEMENTATION (HD44780, 4-bit mode)
// ============================================================================

/**
 * @brief Send 4-bit nibble to LCD
 */
static void LCD_Send_Nibble(uint8_t nibble, bool is_data) {
    // Set RS pin (0=command, 1=data)
    HAL_GPIO_Write(&LCD_RS_PORT, LCD_RS_BIT, is_data);
    
    // Set data pins D7-D4
    HAL_GPIO_Write(&LCD_D4_PORT, LCD_D4_BIT, (nibble & 0x01) != 0);
    HAL_GPIO_Write(&LCD_D5_PORT, LCD_D5_BIT, (nibble & 0x02) != 0);
    HAL_GPIO_Write(&LCD_D6_PORT, LCD_D6_BIT, (nibble & 0x04) != 0);
    HAL_GPIO_Write(&LCD_D7_PORT, LCD_D7_BIT, (nibble & 0x08) != 0);
    
    // Generate enable pulse
    HAL_GPIO_Write(&LCD_EN_PORT, LCD_EN_BIT, true);
    _delay_us(1);
    HAL_GPIO_Write(&LCD_EN_PORT, LCD_EN_BIT, false);
    _delay_us(100);
}

/**
 * @brief Send 8-bit byte to LCD in 4-bit mode
 */
static void LCD_Send_Byte(uint8_t byte, bool is_data) {
    LCD_Send_Nibble((byte >> 4) & 0x0F, is_data);  // Upper nibble
    LCD_Send_Nibble(byte & 0x0F, is_data);         // Lower nibble
}

/**
 * @brief Send command to LCD
 */
static void LCD_Command(uint8_t cmd) {
    LCD_Send_Byte(cmd, false);
    _delay_ms(2);
}

/**
 * @brief Send data character to LCD
 */
static void LCD_Data(uint8_t data) {
    LCD_Send_Byte(data, true);
}

/**
 * @brief Initialize LCD in 4-bit mode
 */
static void LCD_Init(void) {
    _delay_ms(50);  // Wait for LCD power-up
    
    // Initialization sequence for 4-bit mode
    LCD_Send_Nibble(0x03, false);
    _delay_ms(5);
    LCD_Send_Nibble(0x03, false);
    _delay_us(150);
    LCD_Send_Nibble(0x03, false);
    _delay_us(150);
    LCD_Send_Nibble(0x02, false);  // Switch to 4-bit mode
    _delay_us(150);
    
    // Function set: 4-bit, 2 lines, 5×8 font
    LCD_Command(0x28);
    // Display control: Display ON, Cursor OFF, Blink OFF
    LCD_Command(0x0C);
    // Entry mode: Increment cursor, No display shift
    LCD_Command(0x06);
    // Clear display
    LCD_Command(0x01);
    _delay_ms(2);
}

/**
 * @brief Clear LCD display
 */
static void LCD_Clear(void) {
    LCD_Command(0x01);
    _delay_ms(2);
}

/**
 * @brief Set cursor position (row: 0-1, col: 0-15)
 */
static void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0;
    address += col;
    LCD_Command(address);
}

/**
 * @brief Print string to LCD
 */
static void LCD_Print(const char *text) {
    while (*text) {
        LCD_Data(*text++);
    }
}

/**
 * @brief Print unsigned integer to LCD
 */
static void LCD_Print_Number(uint16_t number) {
    char buffer[6];
    Integer_To_String(number, buffer, 10);
    LCD_Print(buffer);
}


// ============================================================================
// ULTRASONIC SENSOR DRIVER IMPLEMENTATION
// ============================================================================

/**
 * @brief Send 10µs trigger pulse to HC-SR04
 */
static void US_Trigger_Pulse(void) {
    HAL_GPIO_Write(&US_TRIG_PORT, US_TRIG_BIT, false);
    _delay_us(2);
    HAL_GPIO_Write(&US_TRIG_PORT, US_TRIG_BIT, true);
    _delay_us(10);
    HAL_GPIO_Write(&US_TRIG_PORT, US_TRIG_BIT, false);
}

/**
 * @brief Measure echo pulse width in microseconds (FIXED)
 * @return Pulse width in µs, or 0 on timeout
 */
static uint32_t US_Measure_Echo_Pulse(void) {
    uint32_t timeout_counter = 0;
    
    // Wait for echo pin to go HIGH (start of pulse)
    while (!HAL_GPIO_Read(&US_ECHO_PIN_REG, US_ECHO_BIT)) {
        _delay_us(1);
        if (++timeout_counter > US_TIMEOUT_US) {
            return 0;  // Timeout
        }
    }
    
    // Measure pulse width in microseconds
    uint32_t pulse_width = 0;
    timeout_counter = 0;
    
    while (HAL_GPIO_Read(&US_ECHO_PIN_REG, US_ECHO_BIT)) {
        _delay_us(1);
        pulse_width++;
        if (++timeout_counter > US_TIMEOUT_US) {
            return 0;  // Timeout
        }
    }
    
    return pulse_width;
}

/**
 * @brief Convert echo pulse width to distance in cm
 * @return Distance in cm, or 0xFFFF on error
 */
static uint16_t US_Calculate_Distance(uint32_t pulse_width_us) {
    if (pulse_width_us == 0 || pulse_width_us > US_TIMEOUT_US) {
        return 0xFFFF;  // Invalid measurement
    }
    
    // Distance (cm) = pulse_width (µs) / 58
    uint16_t distance_cm = pulse_width_us / 58;
    
    // Apply calibration offset
    distance_cm += g_system.calibration_offset;
    
    // Validate range (2cm to 400cm for HC-SR04)
    if (distance_cm < 2 || distance_cm > 400) {
        return 0xFFFF;  // Out of range
    }
    
    return distance_cm;
}

/**
 * @brief Perform single distance measurement
 * @return Distance in cm, or 0xFFFF on error
 */
static uint16_t US_Measure_Distance_Single(void) {
    US_Trigger_Pulse();
    uint32_t pulse_width = US_Measure_Echo_Pulse();
    uint16_t distance = US_Calculate_Distance(pulse_width);
    return distance;
}


// ============================================================================
// FLAME SENSOR DRIVER IMPLEMENTATION
// ============================================================================

/**
 * @brief Read flame sensor digital output
 * @return true if flame detected, false otherwise
 * @note Sensor outputs LOW when flame is detected (active LOW)
 */
static bool Flame_Sensor_Read(void) {
    return !HAL_GPIO_Read(&FLAME_PIN_REG, FLAME_BIT);
}

/**
 * @brief Confirm flame detection with multiple readings
 * @return true if flame confirmed, false otherwise
 */
static bool Flame_Sensor_Confirm(void) {
    const uint8_t CONFIRMATION_SAMPLES = 3;
    const uint16_t CONFIRMATION_DELAY_MS = 100;
    
    uint8_t confirmed_count = 0;
    
    for (uint8_t i = 0; i < CONFIRMATION_SAMPLES; i++) {
        if (Flame_Sensor_Read()) {
            confirmed_count++;
        }
        HAL_Delay_ms(CONFIRMATION_DELAY_MS);
    }
    
    // Require majority consensus (2 out of 3)
    return (confirmed_count >= 2);
}


// ============================================================================
// KEYPAD DRIVER IMPLEMENTATION (4×3 Matrix)
// ============================================================================

/**
 * @brief Scan 4×3 keypad matrix
 * @return Character of pressed key, or '\0' if no key pressed
 * 
 * Key Layout:
 *   1  2  3
 *   4  5  6
 *   7  8  9
 *   *  0  #
 */
static char Keypad_Scan_Raw(void) {
    // Key mapping table [row][col]
    const char key_map[4][3] = {
        {'1', '2', '3'},  // Row A
        {'4', '5', '6'},  // Row B
        {'7', '8', '9'},  // Row C
        {'*', '0', '#'}   // Row D
    };
    
    // Array of row ports and bits for easy iteration
    volatile uint8_t *row_ports[4] = {
        &KEY_ROW_A_PORT,
        &KEY_ROW_B_PORT,
        &KEY_ROW_C_PORT,
        &KEY_ROW_D_PORT
    };
    
    const uint8_t row_bits[4] = {
        KEY_ROW_A_BIT,
        KEY_ROW_B_BIT,
        KEY_ROW_C_BIT,
        KEY_ROW_D_BIT
    };
    
    // Scan each row
    for (uint8_t row = 0; row < 4; row++) {
        // Set all rows HIGH (inactive)
        KEY_ROW_A_PORT |= (1 << KEY_ROW_A_BIT);
        KEY_ROW_B_PORT |= (1 << KEY_ROW_B_BIT);
        KEY_ROW_C_PORT |= (1 << KEY_ROW_C_BIT);
        KEY_ROW_D_PORT |= (1 << KEY_ROW_D_BIT);
        
        // Set current row LOW (active)
        *(row_ports[row]) &= ~(1 << row_bits[row]);
        _delay_us(10);  // Allow signal to settle
        
        // Check each column
        // Column 1
        if (!(KEY_COL_1_PIN_REG & (1 << KEY_COL_1_BIT))) {
            // Set row back to HIGH before returning
            *(row_ports[row]) |= (1 << row_bits[row]);
            return key_map[row][0];
        }
        
        // Column 2
        if (!(KEY_COL_2_PIN_REG & (1 << KEY_COL_2_BIT))) {
            *(row_ports[row]) |= (1 << row_bits[row]);
            return key_map[row][1];
        }
        
        // Column 3
        if (!(KEY_COL_3_PIN_REG & (1 << KEY_COL_3_BIT))) {
            *(row_ports[row]) |= (1 << row_bits[row]);
            return key_map[row][2];
        }
    }
    
    // No key pressed - set all rows HIGH
    KEY_ROW_A_PORT |= (1 << KEY_ROW_A_BIT);
    KEY_ROW_B_PORT |= (1 << KEY_ROW_B_BIT);
    KEY_ROW_C_PORT |= (1 << KEY_ROW_C_BIT);
    KEY_ROW_D_PORT |= (1 << KEY_ROW_D_BIT);
    
    return '\0';  // No key pressed
}

/**
 * @brief Read keypad with software debouncing
 * @return '1'-'9', '*', '0', '#', or '\0' if no stable key press
 */
static char Keypad_Read_Debounced(void) {
    static char last_key = '\0';
    static uint32_t last_change_time = 0;
    
    char current_key = Keypad_Scan_Raw();
    uint32_t current_time = HAL_Millis();
    
    // Detect key state change
    if (current_key != last_key) {
        last_key = current_key;
        last_change_time = current_time;
        return '\0';  // Key state unstable
    }
    
    // Check if key state is stable for debounce period
    if ((current_time - last_change_time) >= DEBOUNCE_DELAY) {
        if (current_key != '\0') {
            last_key = '\0';  // Reset for next key press
            return current_key;
        }
    }
    
    return '\0';
}


// ============================================================================
// ACTUATOR DRIVER IMPLEMENTATION
// ============================================================================

/**
 * @brief Set LED blinking pattern
 */
static void LED_Set_Pattern(LEDPattern_t pattern) {
    g_led_pattern = pattern;
    
    if (pattern == LED_OFF) {
        HAL_GPIO_Write(&LED_PORT, LED_BIT, false);
    } else if (pattern == LED_SOLID) {
        HAL_GPIO_Write(&LED_PORT, LED_BIT, true);
    }
    
    g_timing.last_led_toggle = HAL_Millis();
}

/**
 * @brief Update LED state based on current pattern
 */
static void LED_Update(void) {
    uint32_t current_time = HAL_Millis();
    uint16_t interval_ms = 0;
    
    switch (g_led_pattern) {
        case LED_SLOW_BLINK:
            interval_ms = 1000;
            break;
        case LED_FAST_BLINK:
            interval_ms = 500;
            break;
        case LED_RAPID_BLINK:
            interval_ms = 200;
            break;
        default:
            return;  // No update needed
    }
    
    if ((current_time - g_timing.last_led_toggle) >= interval_ms) {
        HAL_GPIO_Toggle(&LED_PORT, LED_BIT);
        g_timing.last_led_toggle = current_time;
    }
}

/**
 * @brief Set buzzer sound pattern
 */
static void Buzzer_Set_Pattern(BuzzerPattern_t pattern) {
    g_buzzer_pattern = pattern;
    
    if (pattern == BUZZER_OFF) {
        HAL_GPIO_Write(&BUZZER_PORT, BUZZER_BIT, false);
        g_buzzer_state = false;
    } else if (pattern == BUZZER_CONTINUOUS) {
        HAL_GPIO_Write(&BUZZER_PORT, BUZZER_BIT, true);
        g_buzzer_state = true;
    } else if (pattern == BUZZER_BEEP) {
        HAL_GPIO_Write(&BUZZER_PORT, BUZZER_BIT, true);
        HAL_Delay_ms(100);
        HAL_GPIO_Write(&BUZZER_PORT, BUZZER_BIT, false);
        g_buzzer_pattern = BUZZER_OFF;
    }
    
    g_timing.last_buzzer_toggle = HAL_Millis();
}

/**
 * @brief Update buzzer state based on current pattern
 */
static void Buzzer_Update(void) {
    uint32_t current_time = HAL_Millis();
    uint16_t interval_ms = 0;
    
    switch (g_buzzer_pattern) {
        case BUZZER_INTERMITTENT:
            interval_ms = g_buzzer_state ? 200 : 1800;
            break;
        case BUZZER_ALARM:
            interval_ms = 100;
            break;
        default:
            return;  // No update needed
    }
    
    if ((current_time - g_timing.last_buzzer_toggle) >= interval_ms) {
        g_buzzer_state = !g_buzzer_state;
        HAL_GPIO_Write(&BUZZER_PORT, BUZZER_BIT, g_buzzer_state);
        g_timing.last_buzzer_toggle = current_time;
    }
}


// ============================================================================
// SENSOR PROCESSING IMPLEMENTATION
// ============================================================================

/**
 * @brief Measure bin fill level with multi-sample median filtering
 */
static void Measure_Fill_Level(void) {
    // Collect multiple samples
    g_sensor.valid_sample_count = 0;
    
    for (uint8_t i = 0; i < US_SAMPLE_COUNT; i++) {
        uint16_t distance = US_Measure_Distance_Single();
        
        if (distance != 0xFFFF) {
            g_sensor.raw_samples[i] = distance;
            g_sensor.valid_sample_count++;
        } else {
            g_sensor.raw_samples[i] = 0xFFFF;
        }
        
        HAL_Delay_ms(US_SAMPLE_DELAY_MS);
    }
    
    // Check if sufficient valid samples
    if (g_sensor.valid_sample_count < (US_SAMPLE_COUNT / 2)) {
        // Sensor error - insufficient valid readings
        g_sensor.error_count++;
        
        if (g_sensor.error_count >= 3) {
            Log_Event(EVENT_SENSOR_ERROR, g_system.fill_percentage);
            g_sensor.error_count = 0;
        }
        
        return;  // Keep previous values
    }
    
    g_sensor.error_count = 0;  // Reset error counter
    
    // Apply median filter
    Sort_Array(g_sensor.raw_samples, US_SAMPLE_COUNT);
    uint16_t median_distance = g_sensor.raw_samples[US_SAMPLE_COUNT / 2];
    
    // Update system data
    g_system.distance_cm = median_distance;
    
    // Calculate fill percentage
    uint8_t empty_distance_cm = BIN_HEIGHT_CM - SENSOR_OFFSET_CM;
    int16_t fill_distance_cm = empty_distance_cm - median_distance;
    
    if (fill_distance_cm < 0) {
        fill_distance_cm = 0;
    }
    
    uint8_t fill_percentage = (fill_distance_cm * 100) / empty_distance_cm;
    
    // Clamp to valid range (0-100%)
    if (fill_percentage > 100) {
        fill_percentage = 100;
    }
    
    g_system.fill_percentage = fill_percentage;
}

/**
 * @brief Sort array in ascending order (Bubble Sort)
 */
static void Sort_Array(uint16_t *array, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                uint16_t temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief Calibrate ultrasonic sensor with empty bin reference
 */
static void Calibrate_Sensor(void) {
    const uint8_t CALIBRATION_SAMPLES = 10;
    const uint16_t CALIBRATION_DELAY_MS = 300;
    
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("Calibrating...");
    LCD_Set_Cursor(1, 0);
    LCD_Print("Empty bin now!");
    
    HAL_Delay_ms(3000);
    
    // Take multiple reference measurements
    uint32_t total_distance = 0;
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint16_t distance = US_Measure_Distance_Single();
        
        if (distance != 0xFFFF) {
            total_distance += distance;
            valid_count++;
        }
        
        HAL_Delay_ms(CALIBRATION_DELAY_MS);
        
        // Display progress
        LCD_Set_Cursor(1, 0);
        LCD_Print("Sample: ");
        LCD_Print_Number(i + 1);
        LCD_Print("/");
        LCD_Print_Number(CALIBRATION_SAMPLES);
    }
    
    // Check if calibration successful
    if (valid_count >= (CALIBRATION_SAMPLES / 2)) {
        // Calculate calibration offset
        uint16_t measured_average = total_distance / valid_count;
        uint8_t expected_empty_distance = BIN_HEIGHT_CM - SENSOR_OFFSET_CM;
        
        g_system.calibration_offset = expected_empty_distance - measured_average;
        
        // Save to EEPROM
        HAL_EEPROM_Write_Byte(EEPROM_ADDR_CALIBRATION, (uint8_t)g_system.calibration_offset);
        
        // Log calibration event
        Log_Event(EVENT_CALIBRATION_DONE, 0);
        
        // Display success
        LCD_Clear();
        LCD_Set_Cursor(0, 0);
        LCD_Print("Calibration OK");
        LCD_Set_Cursor(1, 0);
        LCD_Print("Offset: ");
        LCD_Print_Number((uint16_t)g_system.calibration_offset);
        LCD_Print(" cm");
        HAL_Delay_ms(2000);
    } else {
        // Calibration failed
        LCD_Clear();
        LCD_Set_Cursor(0, 0);
        LCD_Print("Calib Failed!");
        LCD_Set_Cursor(1, 0);
        LCD_Print("Check sensor");
        HAL_Delay_ms(3000);
    }
}

/**
 * @brief Check flame sensor and update fire detection status
 */
static void Check_Fire_Sensor(void) {
    if (Flame_Sensor_Read()) {
        // Flame detected - confirm before triggering alert
        if (!g_system.fire_confirmed) {
            // Perform multi-sample confirmation
            if (Flame_Sensor_Confirm()) {
                g_system.fire_detected = true;
                g_system.fire_confirmed = true;
                Log_Event(EVENT_FIRE_DETECTED, g_system.fill_percentage);
            }
        } else {
            // Fire already confirmed - maintain alert state
            g_system.fire_detected = true;
        }
    } else {
        // No flame detected
        if (g_system.fire_confirmed) {
            // Fire was previously detected and now cleared
            Log_Event(EVENT_FIRE_CLEARED, g_system.fill_percentage);
        }
        
        g_system.fire_detected = false;
        g_system.fire_confirmed = false;
    }
}


// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

/**
 * @brief Convert unsigned integer to string (FIXED: Added bounds checking)
 * @param num Number to convert
 * @param buffer Output buffer
 * @param base Number base (10 for decimal)
 */
static void Integer_To_String(uint16_t num, char *buffer, uint8_t base) {
    char temp[16];
    uint8_t i = 0;
    
    // Handle zero case
    if (num == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }
    
    // Convert number to string (reversed) with bounds checking
    while (num != 0 && i < 15) {
        uint8_t remainder = num % base;
        temp[i++] = (remainder < 10) ? (remainder + '0') : (remainder - 10 + 'A');
        num /= base;
    }
    
    // Reverse string into buffer
    uint8_t j = 0;
    while (i > 0 && j < 15) {
        buffer[j++] = temp[--i];
    }
    buffer[j] = '\0';
}

/**
 * @brief Compare two strings for equality (FIXED: Custom implementation)
 * @param str1 First string
 * @param str2 Second string
 * @param length Number of characters to compare
 * @return true if strings match, false otherwise
 */
static bool String_Compare(const char *str1, const char *str2, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        if (str1[i] != str2[i]) {
            return false;
        }
    }
    return true;
}


// ============================================================================
// USER INTERFACE IMPLEMENTATION
// ============================================================================

/**
 * @brief Update LCD with current system status
 */
static void Update_LCD_Display(void) {
    LCD_Clear();
    
    // Row 1: Bin ID and Fill Level
    LCD_Set_Cursor(0, 0);
    LCD_Print("Bin:#");
    LCD_Print(BIN_ID);
    LCD_Print(" F:");
    LCD_Print_Number(g_system.fill_percentage);
    LCD_Print("%");
    
    // Row 2: System Status
    LCD_Set_Cursor(1, 0);
    LCD_Print("Stat:");
    
    switch (g_system.current_state) {
        case STATE_NORMAL:
            LCD_Print("Normal");
            break;
        case STATE_PRIMARY_ALERT:
            LCD_Print("Alert-1");
            break;
        case STATE_CRITICAL_ALERT:
            LCD_Print("CRITICAL");
            break;
        case STATE_FIRE_ALERT:
            LCD_Print("FIRE!!!");
            break;
        case STATE_MAINTENANCE:
            LCD_Print("Maint.");
            break;
        default:
            LCD_Print("Unknown");
            break;
    }
    
    g_timing.last_lcd_update = HAL_Millis();
}

/**
 * @brief Display emergency fire alert screen
 */
static void Display_Fire_Alert(void) {
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("!!! FIRE !!!");
    LCD_Set_Cursor(1, 0);
    LCD_Print("EVACUATE NOW");
}

/**
 * @brief Display system welcome screen on boot
 */
static void Display_Welcome_Screen(void) {
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("EcoBin v2.0");
    LCD_Set_Cursor(1, 0);
    LCD_Print("Initializing...");
    HAL_Delay_ms(2000);
}

/**
 * @brief Display maintenance mode menu options
 */
static void Display_Maintenance_Menu(void) {
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("1:Reset 2:Cal");
    LCD_Set_Cursor(1, 0);
    LCD_Print("3:Logs  4:Exit");
}

/**
 * @brief Prompt user to enter maintenance PIN (FIXED: Using String_Compare)
 * @return true if authenticated, false otherwise
 */
static bool Prompt_PIN_Entry(void) {
    char input_pin[PIN_LENGTH + 1] = {0};
    uint8_t index = 0;
    uint32_t timeout_start = HAL_Millis();
    
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("Enter PIN:");
    LCD_Set_Cursor(1, 0);
    
    // Collect PIN digits
    while (index < PIN_LENGTH) {
        // Check for timeout
        if ((HAL_Millis() - timeout_start) > PIN_TIMEOUT_MS) {
            LCD_Clear();
            LCD_Set_Cursor(0, 0);
            LCD_Print("Timeout");
            HAL_Delay_ms(1000);
            return false;
        }
        
        // Read keypad
        char key = Keypad_Read_Debounced();
        
        if (key != '\0') {
            input_pin[index++] = key;
            
            // Display asterisk for security
            LCD_Data('*');
            
            // Audio feedback
            Buzzer_Set_Pattern(BUZZER_BEEP);
            
            timeout_start = HAL_Millis();  // Reset timeout
        }
        
        HAL_Delay_ms(10);
    }
    
    input_pin[PIN_LENGTH] = '\0';
    
    // Verify PIN using custom string comparison
    if (String_Compare(input_pin, MAINTENANCE_PIN, PIN_LENGTH)) {
        LCD_Clear();
        LCD_Set_Cursor(0, 0);
        LCD_Print("Access Granted");
        HAL_Delay_ms(1000);
        return true;
    } else {
        LCD_Clear();
        LCD_Set_Cursor(0, 0);
        LCD_Print("Access Denied");
        LCD_Set_Cursor(1, 0);
        LCD_Print("Invalid PIN");
        HAL_Delay_ms(2000);
        return false;
    }
}

/**
 * @brief Process maintenance menu key presses
 */
static void Handle_Maintenance_Menu(void) {
    char key = Keypad_Read_Debounced();
    
    switch (key) {
        case '1':
            Perform_System_Reset();
            break;
        
        case '2':
            Calibrate_Sensor();
            Display_Maintenance_Menu();
            break;
        
        case '3':
            Display_Event_Logs();
            Display_Maintenance_Menu();
            break;
        
        case '4':
            Exit_Maintenance_Mode();
            break;
    }
}

/**
 * @brief Reset system with two-step confirmation
 */
static void Perform_System_Reset(void) {
    // First confirmation
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("Reset System?");
    LCD_Set_Cursor(1, 0);
    LCD_Print("1=Yes 2=No");
    
    uint32_t timeout_start = HAL_Millis();
    
    while ((HAL_Millis() - timeout_start) < PIN_TIMEOUT_MS) {
        char key = Keypad_Read_Debounced();
        
        if (key == '1') {
            // Second confirmation
            LCD_Clear();
            LCD_Set_Cursor(0, 0);
            LCD_Print("Confirm Reset");
            LCD_Set_Cursor(1, 0);
            LCD_Print("Press 1 again");
            
            HAL_Delay_ms(1000);
            
            // Wait for final confirmation
            uint32_t confirm_timeout = HAL_Millis();
            while ((HAL_Millis() - confirm_timeout) < 3000) {
                char confirm_key = Keypad_Read_Debounced();
                
                if (confirm_key == '1') {
                    // Perform reset
                    Log_Event(EVENT_RESET_PERFORMED, g_system.fill_percentage);
                    
                    g_system.fire_detected = false;
                    g_system.fire_confirmed = false;
                    
                    LCD_Clear();
                    LCD_Set_Cursor(0, 0);
                    LCD_Print("System Reset");
                    LCD_Set_Cursor(1, 0);
                    LCD_Print("Complete");
                    HAL_Delay_ms(2000);
                    
                    // Return to normal operation
                    Transition_To_State(STATE_NORMAL);
                    return;
                }
                
                HAL_Delay_ms(10);
            }
            
            // Confirmation timeout
            LCD_Clear();
            LCD_Set_Cursor(0, 0);
            LCD_Print("Reset Cancelled");
            HAL_Delay_ms(1000);
            Display_Maintenance_Menu();
            return;
            
        } else if (key == '2') {
            Display_Maintenance_Menu();
            return;
        }
        
        HAL_Delay_ms(10);
    }
    
    // Timeout - cancel reset
    Display_Maintenance_Menu();
}

/**
 * @brief Display stored event logs on LCD
 */
static void Display_Event_Logs(void) {
    if (g_system.log_count == 0) {
        LCD_Clear();
        LCD_Set_Cursor(0, 0);
        LCD_Print("No logs stored");
        HAL_Delay_ms(2000);
        return;
    }
    
    // Determine number of logs to display
    uint8_t display_count = (g_system.log_count < 5) ? g_system.log_count : 5;
    uint16_t start_index = g_system.log_count - display_count;
    
    for (uint16_t i = start_index; i < g_system.log_count; i++) {
        LogEntry_t *log_entry = &g_system.log_buffer[i];
        
        LCD_Clear();
        
        // Row 1: Event type
        LCD_Set_Cursor(0, 0);
        const char *event_text = Get_Event_Text(log_entry->event_type);
        LCD_Print(event_text);
        
        // Row 2: Timestamp and fill level
        LCD_Set_Cursor(1, 0);
        LCD_Print("T:");
        LCD_Print_Number((uint16_t)log_entry->timestamp);
        LCD_Print("s F:");
        LCD_Print_Number(log_entry->fill_level);
        LCD_Print("%");
        
        HAL_Delay_ms(3000);
        
        // Allow early exit with key '4'
        if (Keypad_Read_Debounced() == '4') {
            break;
        }
    }
}

/**
 * @brief Convert event type to display string
 */
static const char* Get_Event_Text(EventType_t event) {
    switch (event) {
        case EVENT_SYSTEM_BOOT:         return "System Boot";
        case EVENT_THRESHOLD_PRIMARY:   return "Primary Alert";
        case EVENT_THRESHOLD_CRITICAL:  return "Critical Alert";
        case EVENT_FIRE_DETECTED:       return "Fire Detected";
        case EVENT_FIRE_CLEARED:        return "Fire Cleared";
        case EVENT_RESET_PERFORMED:     return "System Reset";
        case EVENT_CALIBRATION_START:   return "Calib Start";
        case EVENT_CALIBRATION_DONE:    return "Calib Done";
        case EVENT_SENSOR_ERROR:        return "Sensor Error";
        case EVENT_MAINTENANCE_ENTER:   return "Maint Enter";
        case EVENT_MAINTENANCE_EXIT:    return "Maint Exit";
        default:                        return "Unknown Event";
    }
}


// ============================================================================
// EVENT LOGGING IMPLEMENTATION
// ============================================================================

/**
 * @brief Log system event to RAM buffer and EEPROM
 */
static void Log_Event(EventType_t event, uint8_t fill_level) {
    // Check if RAM buffer is full
    if (g_system.log_count >= MAX_LOG_ENTRIES) {
        // Shift all entries (circular buffer)
        for (uint16_t i = 0; i < MAX_LOG_ENTRIES - 1; i++) {
            g_system.log_buffer[i] = g_system.log_buffer[i + 1];
        }
        g_system.log_count = MAX_LOG_ENTRIES - 1;
    }
    
    // Create new log entry
    LogEntry_t new_entry;
    new_entry.timestamp = g_system.uptime_seconds;
    new_entry.event_type = event;
    new_entry.fill_level = fill_level;
    new_entry.distance_cm = g_system.distance_cm;
    
    // Add to RAM buffer
    g_system.log_buffer[g_system.log_count] = new_entry;
    g_system.log_count++;
    
    // Save to EEPROM (last N entries only)
    Save_Event_To_EEPROM(&new_entry);
}

/**
 * @brief Save event log entry to EEPROM for persistence
 */
static void Save_Event_To_EEPROM(const LogEntry_t *entry) {
    // Update log count in EEPROM
    HAL_EEPROM_Write_Block(EEPROM_ADDR_LOG_COUNT, 
                           &g_system.log_count, 
                           sizeof(g_system.log_count));
    
    // Save entry if within EEPROM capacity
    if (g_system.log_count <= EEPROM_LOG_ENTRIES) {
        uint16_t entry_index = g_system.log_count - 1;
        uint16_t address = EEPROM_ADDR_LOG_START + (entry_index * sizeof(LogEntry_t));
        HAL_EEPROM_Write_Block(address, entry, sizeof(LogEntry_t));
    }
}

/**
 * @brief Load event logs from EEPROM to RAM on system boot
 */
static void Load_Event_Log_From_EEPROM(void) {
    // Read log count
    HAL_EEPROM_Read_Block(EEPROM_ADDR_LOG_COUNT, 
                          &g_system.log_count, 
                          sizeof(g_system.log_count));
    
    // Validate log count
    if (g_system.log_count > MAX_LOG_ENTRIES) {
        g_system.log_count = 0;  // Invalid data - reset
        return;
    }
    
    // Load log entries
    uint16_t load_count = (g_system.log_count < EEPROM_LOG_ENTRIES) 
                          ? g_system.log_count 
                          : EEPROM_LOG_ENTRIES;
    
    for (uint16_t i = 0; i < load_count; i++) {
        uint16_t address = EEPROM_ADDR_LOG_START + (i * sizeof(LogEntry_t));
        HAL_EEPROM_Read_Block(address, 
                              &g_system.log_buffer[i], 
                              sizeof(LogEntry_t));
    }
}


// ============================================================================
// STATE MACHINE IMPLEMENTATION
// ============================================================================

/**
 * @brief Perform state transition with cleanup and initialization
 */
static void Transition_To_State(SystemState_t new_state) {
    // Exit actions for current state
    switch (g_system.current_state) {
        case STATE_FIRE_ALERT:
            // Clear fire alert actuators
            LED_Set_Pattern(LED_OFF);
            Buzzer_Set_Pattern(BUZZER_OFF);
            break;
        
        case STATE_MAINTENANCE:
            // Log maintenance exit
            Log_Event(EVENT_MAINTENANCE_EXIT, g_system.fill_percentage);
            g_system.authenticated = false;
            break;
        
        default:
            break;
    }
    
    // Update state
    g_system.previous_state = g_system.current_state;
    g_system.current_state = new_state;
    g_timing.state_entry_time = HAL_Millis();
    
    // Entry actions for new state
    switch (new_state) {
        case STATE_NORMAL:
            LED_Set_Pattern(LED_OFF);
            Buzzer_Set_Pattern(BUZZER_OFF);
            break;
        
        case STATE_PRIMARY_ALERT:
            Log_Event(EVENT_THRESHOLD_PRIMARY, g_system.fill_percentage);
            LED_Set_Pattern(LED_SLOW_BLINK);
            Buzzer_Set_Pattern(BUZZER_INTERMITTENT);
            break;
        
        case STATE_CRITICAL_ALERT:
            Log_Event(EVENT_THRESHOLD_CRITICAL, g_system.fill_percentage);
            LED_Set_Pattern(LED_FAST_BLINK);
            Buzzer_Set_Pattern(BUZZER_CONTINUOUS);
            break;
        
        case STATE_FIRE_ALERT:
            // Fire entry logged by fire detection function
            LED_Set_Pattern(LED_RAPID_BLINK);
            Buzzer_Set_Pattern(BUZZER_ALARM);
            Display_Fire_Alert();
            break;
        
        case STATE_MAINTENANCE:
            Log_Event(EVENT_MAINTENANCE_ENTER, g_system.fill_percentage);
            LED_Set_Pattern(LED_OFF);
            Buzzer_Set_Pattern(BUZZER_OFF);
            Display_Maintenance_Menu();
            break;
        
        default:
            break;
    }
    
    Update_LCD_Display();
}

/**
 * @brief Check conditions for state transitions based on fill level
 */
static void Check_State_Transitions(void) {
    uint8_t fill = g_system.fill_percentage;
    
    switch (g_system.current_state) {
        case STATE_NORMAL:
            if (fill >= CRITICAL_THRESHOLD) {
                Transition_To_State(STATE_CRITICAL_ALERT);
            } else if (fill >= PRIMARY_THRESHOLD) {
                Transition_To_State(STATE_PRIMARY_ALERT);
            }
            break;
        
        case STATE_PRIMARY_ALERT:
            if (fill >= CRITICAL_THRESHOLD) {
                Transition_To_State(STATE_CRITICAL_ALERT);
            } else if (fill < (PRIMARY_THRESHOLD - THRESHOLD_HYSTERESIS)) {
                Transition_To_State(STATE_NORMAL);
            }
            break;
        
        case STATE_CRITICAL_ALERT:
            // Critical state is locked - requires maintenance reset
            // No automatic transition to lower states
            break;
        
        default:
            break;
    }
}

/**
 * @brief Exit maintenance mode and return to appropriate operational state
 */
static void Exit_Maintenance_Mode(void) {
    g_system.authenticated = false;
    
    // Determine appropriate return state based on fill level
    if (g_system.fire_detected) {
        Transition_To_State(STATE_FIRE_ALERT);
    } else if (g_system.fill_percentage >= CRITICAL_THRESHOLD) {
        Transition_To_State(STATE_CRITICAL_ALERT);
    } else if (g_system.fill_percentage >= PRIMARY_THRESHOLD) {
        Transition_To_State(STATE_PRIMARY_ALERT);
    } else {
        Transition_To_State(STATE_NORMAL);
    }
}


// ============================================================================
// SYSTEM INITIALIZATION
// ============================================================================

/**
 * @brief Initialize all system components and peripherals
 */
static void System_Init(void) {
    // Hardware Initialization
    HAL_GPIO_Init();
    HAL_Timer_Init();
    LCD_Init();
    
    // Enable global interrupts
    sei();
    
    // Initialize system data structure
    g_system.current_state = STATE_INIT;
    g_system.previous_state = STATE_INIT;
    g_system.fill_percentage = 0;
    g_system.distance_cm = 0;
    g_system.fire_detected = false;
    g_system.fire_confirmed = false;
    g_system.authenticated = false;
    g_system.log_count = 0;
    g_system.uptime_seconds = 0;
    
    // Initialize sensor data
    g_sensor.valid_sample_count = 0;
    g_sensor.error_count = 0;
    
    // Initialize timing
    g_timing.last_fire_check = 0;
    g_timing.last_fill_check = 0;
    g_timing.last_lcd_update = 0;
    g_timing.last_led_toggle = 0;
    g_timing.last_buzzer_toggle = 0;
    g_timing.state_entry_time = 0;
    
    // Initialize actuator patterns
    g_led_pattern = LED_OFF;
    g_buzzer_pattern = BUZZER_OFF;
    g_buzzer_state = false;
    
    // Load calibration from EEPROM
    g_system.calibration_offset = (int8_t)HAL_EEPROM_Read_Byte(EEPROM_ADDR_CALIBRATION);
    
    // Validate calibration offset (should be in reasonable range)
    if (g_system.calibration_offset < -50 || g_system.calibration_offset > 50) {
        g_system.calibration_offset = 0;  // Invalid - use default
    }
    
    // Load event logs from EEPROM
    Load_Event_Log_From_EEPROM();
    
    // Log system boot event
    Log_Event(EVENT_SYSTEM_BOOT, 0);
    
    // Display welcome screen
    Display_Welcome_Screen();
    
    // Perform initial calibration prompt (optional)
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("Calibrate Now?");
    LCD_Set_Cursor(1, 0);
    LCD_Print("1=Yes  Any=Skip");
    
    uint32_t timeout_start = HAL_Millis();
    while ((HAL_Millis() - timeout_start) < 5000) {
        char key = Keypad_Read_Debounced();
        if (key == '1') {
            Calibrate_Sensor();
            break;
        } else if (key != '\0') {
            break;
        }
        HAL_Delay_ms(10);
    }
    
    // Initial system state
    LCD_Clear();
    LCD_Set_Cursor(0, 0);
    LCD_Print("System Ready");
    HAL_Delay_ms(1000);
    
    Transition_To_State(STATE_NORMAL);
}


// ============================================================================
// MAIN LOOP
// ============================================================================

/**
 * @brief Main program loop - executes continuously
 */
static void Main_Loop(void) {
    while (1) {
        uint32_t current_time = HAL_Millis();
        
        // ====================================================================
        // PRIORITY 1: FIRE DETECTION (HIGHEST PRIORITY - SAFETY CRITICAL)
        // ====================================================================
        
        if ((current_time - g_timing.last_fire_check) >= FIRE_CHECK_INTERVAL) {
            g_timing.last_fire_check = current_time;
            Check_Fire_Sensor();
            
            if (g_system.fire_detected) {
                if (g_system.current_state != STATE_FIRE_ALERT) {
                    Transition_To_State(STATE_FIRE_ALERT);
                }
            } else {
                if (g_system.current_state == STATE_FIRE_ALERT) {
                    // Fire cleared - return to previous state
                    Exit_Maintenance_Mode();  // Reuses state selection logic
                }
            }
        }
        
        // ====================================================================
        // PRIORITY 2: KEYPAD INPUT PROCESSING
        // ====================================================================
        
        char key = Keypad_Read_Debounced();
        
        if (key != '\0') {
            switch (g_system.current_state) {
                case STATE_NORMAL:
                case STATE_PRIMARY_ALERT:
                    if (key == '1') {
                        if (Prompt_PIN_Entry()) {
                            Transition_To_State(STATE_MAINTENANCE);
                        }
                    }
                    break;
                
                case STATE_CRITICAL_ALERT:
                case STATE_FIRE_ALERT:
                    // Only maintenance access allowed in locked states
                    if (key == '1') {
                        if (Prompt_PIN_Entry()) {
                            Transition_To_State(STATE_MAINTENANCE);
                        }
                    }
                    break;
                
                case STATE_MAINTENANCE:
                    Handle_Maintenance_Menu();
                    break;
                
                default:
                    break;
            }
        }
        
        // ====================================================================
        // PRIORITY 3: STATE-SPECIFIC OPERATIONS
        // ====================================================================
        
        switch (g_system.current_state) {
            case STATE_NORMAL:
            case STATE_PRIMARY_ALERT:
                // Periodic fill level measurement
                if ((current_time - g_timing.last_fill_check) >= FILL_CHECK_INTERVAL) {
                    g_timing.last_fill_check = current_time;
                    Measure_Fill_Level();
                    Check_State_Transitions();
                    Update_LCD_Display();
                }
                break;
            
            case STATE_CRITICAL_ALERT:
                // Continue monitoring but locked state
                if ((current_time - g_timing.last_fill_check) >= FILL_CHECK_INTERVAL) {
                    g_timing.last_fill_check = current_time;
                    Measure_Fill_Level();
                    Update_LCD_Display();
                }
                break;
            
            case STATE_FIRE_ALERT:
                // Fire alert display (continuous)
                // Handled by state entry and actuator updates
                break;
            
            case STATE_MAINTENANCE:
                // Maintenance menu handled by keypad processing
                break;
            
            default:
                break;
        }
        
        // ====================================================================
        // PRIORITY 4: ACTUATOR UPDATES (LED AND BUZZER PATTERNS)
        // ====================================================================
        
        LED_Update();
        Buzzer_Update();
        
        // ====================================================================
        // PRIORITY 5: POWER MANAGEMENT (Optional - Low Power Mode)
        // ====================================================================
        
        // Enter low power mode during normal operation with low fill
        // Uncomment and implement for battery operation
        /*
        if (g_system.current_state == STATE_NORMAL && 
            g_system.fill_percentage < 50) {
            // Implement sleep mode here
            // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            // sleep_enable();
            // sleep_cpu();
            // sleep_disable();
        }
        */
        
        // Small delay to prevent CPU spinning
        HAL_Delay_ms(10);
    }
}


// ============================================================================
// PROGRAM ENTRY POINT
// ============================================================================

/**
 * @brief Main function - program entry point
 * @return Never returns
 */
int main(void) {
    // Initialize system
    System_Init();
    
    // Enter main loop (never returns)
    Main_Loop();
    
    return 0;
}


// ============================================================================
// END OF FIRMWARE - ECOBIN SMART WASTE MANAGEMENT SYSTEM
// ============================================================================
