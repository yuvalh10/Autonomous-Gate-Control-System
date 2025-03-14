
/* Library includes */
#include "em_device.h"     // Device-specific definitions
#include "em_chip.h"       // Chip initialization and configuration
#include "em_cmu.h"        // Clock Management Unit functions
#include "em_gpio.h"       // GPIO configuration and control
#include "em_timer.h"      // Timer configuration and control
#include "em_emu.h"        // Energy Management Unit for power modes
#include "em_eusart.h"     // UART communication functions
#include "sl_segmentlcd.h" // Segmented LCD control functions
#include "gpiointerrupt.h" // GPIO interrupt handling
#include "stdbool.h"       // Boolean type and logic
#include "sl_udelay.h"     // Microsecond delay utility

/*******************************************************************************
 * System Configuration Constants
 ******************************************************************************/
/* PWM Configuration */
#define PWM_FREQ            1000    // PWM frequency in Hz
#define INITIAL_DUTY_CYCLE  60      // Initial PWM duty cycle (%)
#define TIMER1_MS           15      // 15ms delay for debouncing

/* GPIO Port and Pin Definitions */
#define BUTTONS_PORT        gpioPortB
#define SERVO_PORT          gpioPortD
#define UART_PORT           gpioPortD
#define BUZZER_PORT         gpioPortD
#define LED_PORT            gpioPortD
#define SERVO_PIN           11      // Pin for signal output for the servo motor
#define PERSON_SHOW         1       // Pin for persons display
#define CLOSE_BUTTON_PIN    6       // Pin for closing the gate
#define UART_RX_PIN         7       // UART receive pin

/* System Constants */
#define MAX_CAPACITY       2        // Maximum number of persons allowed
#define TRANSITION_DELAY   1000000 // Delay for state transitions (in microseconds)
#define DISPLAY_DELAY      20000000 // Delay for displaying messages (in microseconds)
#define UART_BAUD_RATE     9600     // UART communication speed

/* Servo positions */
#define SERVO_PERIOD   780000 // Timer top value
// 20ms period (50Hz) for 39MHz clock (0.02*39M)

// For 1ms pulse (0 degrees) = 39M * 0.77ms ≈ 30000 (Pulse Width = x/Clock Frequency)
#define SERVO_CLOSED   30000    // 0.77ms pulse for 0 degrees

// For 1.6ms pulse (90 degrees) = 39M * 1.6ms) ≈ 62000 (Pulse Width = x/Clock Frequency)
#define SERVO_OPEN     62000    // 1.6ms pulse for 90 degrees

/* Bluetooth Command Codes */
#define ENTER_GATE         'F'     // Command to increment person count
#define EXIT_GATE          'B'     // Command to decrement person count

/* Gate States */
#define STATE_CLOSED        6       // Gate fully closed state
#define STATE_OPEN          1       // Gate fully open state
#define NO_BUTTON_PRESSED   0xFF    // Indicates no button is currently pressed

/*LED & Buzzer Pins*/
#define RED 9
#define YELLOW 10
#define GREEN 12
#define BUZZER 8
#define ON 1
#define OFF 0

/*******************************************************************************
 * Static Variables
 ******************************************************************************/
static volatile int state = STATE_CLOSED;
static volatile int PersonsInside = 0;
static volatile bool Emergency = false;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static void GPIOCallback(uint8_t pin);
void displayTransitionState(const char* stateMessage);
int checkButtonState(void);
void GateHandling(void);
void MaxMinCapacity(char* s);
void CLOSED_LCD(void);
void OPEN_LCD(void);
void LCD_CONFIGURATIONS(void);
void BUTTONS_CONFIGURATIONS(void);
void BLUETOOTH_INIT(void);
void Servo_Init(void);
void openGate(void);
void closeGate(void);
void BuzzerOn(void);
void BuzzerOff(void);
void LED_Handling(int red, int yellow, int green);

/*******************************************************************************
 * Callback Functions
 ******************************************************************************/
static void GPIOCallback(uint8_t pin) {
    // Static variable to track display mode for Button 1
    // true = show person count, false = show gate state
    static bool showNumber = true;
    // Ensure Gecko symbol is always visible on LCD
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1);

    // Handle Open Button (Button 1) press
    if (pin == PERSON_SHOW) {
        if (showNumber) {
            // Display Mode 1: Show current number of people inside
            sl_segment_lcd_number(PersonsInside);
            sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1);
        } else {
            // Display Mode 2: Show current gate state
            if (state == STATE_OPEN) {
                OPEN_LCD();    // Display "OPEN" if gate is open
            } else if (state == STATE_CLOSED) {
                CLOSED_LCD();  // Display "CLOSED" if gate is closed
            }
        }
        // Toggle between display modes for next button press
        showNumber = !showNumber;
    }
    // Handle Close Button press
    else if (pin == CLOSE_BUTTON_PIN) {
        // Reset person counter when gate operation is initiated
        PersonsInside = 0;
        Emergency = true; // we are at emergency situation
        if (state == STATE_CLOSED) {
            // If gate is closed, initiate opening sequence
            state = STATE_OPEN;
            BuzzerOn(); // turns on the buzzer
            displayTransitionState("OPENING");  // Show transition animation
            openGate();
            OPEN_LCD();                         // Update display to show "OPEN"
        } else if (state == STATE_OPEN) {
            // If gate is open, wait for close button press
            while (state == STATE_OPEN) {
                state = checkButtonState();    // Poll for button state change
            }
            // Initiate closing sequence
            state = STATE_CLOSED;
            BuzzerOff(); // turns off the buzzer
            closeGate();
            displayTransitionState("CLOSING"); // Show transition animation
            CLOSED_LCD();                     // Update display to show "CLOSED"
            Emergency = false; // emergency situation has ended
        }
    }
}

/*******************************************************************************
 * Display Functions
 ******************************************************************************/
void displayTransitionState(const char* stateMessage) {
    LED_Handling(OFF,ON,OFF);                     // Turns on the yellow LED
    sl_segment_lcd_write(stateMessage);           // Write the transition message to LCD
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1); // Ensure Gecko symbol remains visible
    sl_udelay_wait(TRANSITION_DELAY);             // Hold the message for defined delay period
}

int checkButtonState(void) {
    // Check if person show button is pressed (logic low indicates press)
    if (GPIO_PinInGet(BUTTONS_PORT, PERSON_SHOW) == 0) {
        return STATE_OPEN;
    }
    // Check if close button is pressed (logic low indicates press)
    if (GPIO_PinInGet(BUTTONS_PORT, CLOSE_BUTTON_PIN) == 0) {
        return STATE_CLOSED;

    }
    // No button is currently pressed
    return NO_BUTTON_PRESSED;
}

void GateHandling(void) {
    // Begin opening sequence
    displayTransitionState("OPENING");   // Show opening transition message
    openGate();
    OPEN_LCD();                          // Display gate open state
    sl_udelay_wait(TRANSITION_DELAY);    // Wait for specified delay

    // Begin closing sequence
    displayTransitionState("CLOSING");   // Show closing transition message
    closeGate();
    CLOSED_LCD();                        // Display gate closed state
}

/*******************************************************************************
 * PWM Servo Configuration
 ******************************************************************************/
void Servo_Init(void) {
    // Enable clocks
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_TIMER0, true);

    // Configure servo pin as push-pull output
    GPIO_PinModeSet(SERVO_PORT, SERVO_PIN, gpioModePushPull, 0);

    // Configure Timer for PWM
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale1;
    timerInit.enable = false;

    TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
    timerCCInit.mode = timerCCModePWM;

    // Initialize timer
    TIMER_Init(TIMER0, &timerInit);
    TIMER_InitCC(TIMER0, 0, &timerCCInit);

    // Set PWM period (50Hz)
    TIMER_TopSet(TIMER0, SERVO_PERIOD);

    // Configure PWM routing to GPIO pin
    GPIO->TIMERROUTE[0].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO->TIMERROUTE[0].CC0ROUTE = (SERVO_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                                   | (SERVO_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

    // Set initial position (closed)
    TIMER_CompareSet(TIMER0, 0, SERVO_CLOSED);

    // Enable timer
    TIMER_Enable(TIMER0, true);
}

/*******************************************************************************
 * LCD Functions
 ******************************************************************************/
void MaxMinCapacity(char* s) {
    sl_segment_lcd_write(s);                     // Write the capacity message
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1); // Display Gecko symbol
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_PAD0, 1);  // Enable PAD0 indicator
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_PAD1, 1);  // Enable PAD1 indicator
    sl_udelay_wait(DISPLAY_DELAY);              // Hold message for defined period
    CLOSED_LCD();                               // Return to closed state display
}

void CLOSED_LCD(void) {
    LED_Handling(ON,OFF,OFF); // turns on the red LED
    sl_segment_lcd_write("CLOSED");              // Display "CLOSED" text
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1); // Show Gecko symbol
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_PAD0, 1);  // Enable PAD0 for locked indication
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_PAD1, 1);  // Enable PAD1 for locked indication
}

void OPEN_LCD(void) {
    LED_Handling(OFF,OFF,ON); // turns on the green LED
    sl_segment_lcd_write("OPEN");                  // Display "OPEN" text
    sl_segment_lcd_symbol(SL_LCD_SYMBOL_GECKO, 1); // Show Gecko symbol only
}

/*******************************************************************************
 * Buzzer and LED Functions
 ******************************************************************************/
void BuzzerOn(void) // Turns on the buzzer
{
  GPIO_PinModeSet(BUZZER_PORT, BUZZER, gpioModePushPull, 1);
}

void BuzzerOff(void) // Turns off the buzzer
{
  GPIO_PinModeSet(BUZZER_PORT, BUZZER, gpioModePushPull, 0);
}

void LED_Handling(int red, int yellow, int green) // Turns on specific LED
{
  GPIO_PinModeSet(LED_PORT, RED, gpioModePushPull, red);
  GPIO_PinModeSet(LED_PORT, YELLOW, gpioModePushPull, yellow);
  GPIO_PinModeSet(LED_PORT, GREEN, gpioModePushPull, green);
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void LCD_CONFIGURATIONS(void) {
    sl_segment_lcd_init(false);  // Initialize LCD in normal power mode
    CLOSED_LCD();               // Set initial display state to closed
}

void BUTTONS_CONFIGURATIONS(void) {
    // Enable required peripheral clocks
    CMU_ClockEnable(cmuClock_GPIO, true);  // Enable GPIO clock
    CMU_ClockEnable(cmuClock_LCD, true);   // Enable LCD clock

    // Configure button pins as inputs with pull-up resistors
    // Pull-up means pin reads high when button is not pressed
    GPIO_PinModeSet(BUTTONS_PORT, PERSON_SHOW, gpioModeInputPull, 1);      // Person show button
    GPIO_PinModeSet(BUTTONS_PORT, CLOSE_BUTTON_PIN, gpioModeInputPull, 1); // Close button

    // Initialize GPIO interrupt handling
    GPIOINT_Init();  // Initialize GPIO interrupt module

    // Configure external interrupts for both buttons
    // Parameters: port, pin, interrupt number, rising edge, falling edge, enable
    GPIO_ExtIntConfig(BUTTONS_PORT, PERSON_SHOW, PERSON_SHOW, false, true, true);           // Person show button
    GPIO_ExtIntConfig(BUTTONS_PORT, CLOSE_BUTTON_PIN, CLOSE_BUTTON_PIN, false, true, true); // Close button

    // Register callback function for button interrupts
    GPIOINT_CallbackRegister(PERSON_SHOW, GPIOCallback);      // Register person show button callback
    GPIOINT_CallbackRegister(CLOSE_BUTTON_PIN, GPIOCallback); // Register close button callback
}

void BLUETOOTH_INIT(void) {
    // Enable required peripheral clocks
    CMU_ClockEnable(cmuClock_GPIO, true);     // Enable GPIO clock
    CMU_ClockEnable(cmuClock_EUSART1, true);  // Enable UART clock

    // Configure UART receive pin
    GPIO_PinModeSet(UART_PORT, UART_RX_PIN, gpioModeInput, 0);

    // Initialize UART with default high-frequency configuration
    EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
    init.baudrate = UART_BAUD_RATE;       // Set communication speed
    init.oversampling = eusartOVS4;       // 4x oversampling for better reliability

    // Configure UART pin routing
    GPIO->EUSARTROUTE[1].RXROUTE = (UART_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
                                   | (UART_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
    GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN;  // Enable receive pin routing

    // Initialize UART with configured parameters
    EUSART_UartInitHf(EUSART1, &init);

    // Configure and enable UART interrupts
    NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);   // Clear any pending interrupts
    NVIC_EnableIRQ(EUSART1_RX_IRQn);         // Enable UART receive interrupts
    EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL); // Enable UART receive interrupt flag
}

/*******************************************************************************
 * Interrupt Handlers
 ******************************************************************************/
void EUSART1_RX_IRQHandler(void) {

    // Read the received data from Bluetooth
    uint8_t receivedData = EUSART_Rx(EUSART1);


    if (!Emergency) // Available only if not in emergency situation
      {
        switch(receivedData) {     // Handle received data based on its value

            case ENTER_GATE: // Command to enter the gate
                // Check if the current number of persons inside is less than the maximum capacity
                if(PersonsInside < MAX_CAPACITY) {
                    PersonsInside++; // Increment the count of persons inside
                    GateHandling(); // Handle gate operations
                } else {
                    MaxMinCapacity("FULL-UP"); // Notify that the gate is full
                }
                break;

            case EXIT_GATE: // Command to exit the gate
                // Check if there are persons inside to exit
                if(PersonsInside > 0) {
                    PersonsInside--; // Decrement the count of persons inside
                    GateHandling(); // Handle gate operations
                } else {
                    MaxMinCapacity("EMPTY"); // Notify that there are no persons inside
                }
                break;
        }
      }
    // Clear the interrupt flag for receive events
    EUSART_IntClear(EUSART1, EUSART_IF_RXFL);
}

/*******************************************************************************
 * Controlling Servo Motor Functions
 ******************************************************************************/
void openGate(void) {
  // Gradually set the servo to the open position (90 degrees)

  // Incrementally adjust the servo pulse width from half-open to fully open
  for(int i = SERVO_OPEN/2; i <= SERVO_OPEN; i++)
    {
      TIMER_CompareSet(TIMER0, 0, i); // Update PWM duty cycle for the servo
      sl_udelay_wait(5);              // Small delay for smoother movement
    }
  sl_udelay_wait(200000);  // Delay for making gate remains fully open
}

void closeGate(void) {
    // Set servo to closed position (0 degrees)
    TIMER_CompareSet(TIMER0, 0, SERVO_CLOSED);
}

/*******************************************************************************
 * Application Functions
 ******************************************************************************/
void app_init(void) {

    // Configure the LCD display for the application
    LCD_CONFIGURATIONS();

    // Configure button inputs
    BUTTONS_CONFIGURATIONS();

    // Initialize Bluetooth module
    BLUETOOTH_INIT();

    // Initialize Servo motor
    Servo_Init();
}

void app_process_action(void) {}
