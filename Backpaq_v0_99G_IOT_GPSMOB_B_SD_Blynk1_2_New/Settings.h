
/*
 * Board configuration (see examples below).
 */

#if defined(USE_WROVER_BOARD)

  #define BOARD_BUTTON_PIN            15
  #define BOARD_BUTTON_ACTIVE_LOW     true

  #define BOARD_LED_PIN_R             0
  #define BOARD_LED_PIN_G             2
  #define BOARD_LED_PIN_B             4
  #define BOARD_LED_INVERSE           false
  #define BOARD_LED_BRIGHTNESS        128
  #define BOARD_LED_BRIGHTNESS        64

#elif defined(USE_TTGO_T7)

  // This board does not have a built-in button
  // Connect a button to gpio0 <> GND
  #define BOARD_BUTTON_PIN            0
  #define BOARD_BUTTON_ACTIVE_LOW     true

  #define BOARD_LED_PIN               19
  #define BOARD_LED_INVERSE           false
  #define BOARD_LED_BRIGHTNESS        64

#else

  #warning "Custom board configuration is used"
 // Config for AdaFruit Feather Huzzah32

  //#define BOARD_BUTTON_PIN           0                     // Pin where user button is attached
  #define BOARD_BUTTON_PIN            33                     // Pin where user button is attached
  #define BOARD_BUTTON_ACTIVE_LOW     true                  // true if button is "active-low"

 // #define BOARD_LED_PIN             27                     // Set LED pin - if you have a single-color LED attached
 // #define BOARD_LED_PIN_R           27                    // Set R,G,B pins - if your LED is PWM RGB
 // #define BOARD_LED_PIN_G           13
 // #define BOARD_LED_PIN_B           21
  #define BOARD_LED_PIN_WS2812        27                     // Set if your LED is WS2812 RGB
  #define BOARD_LED_INVERSE           false                 // true if LED is common anode, false if common cathode
  #define BOARD_LED_BRIGHTNESS        32                    // 0..255 brightness control (64 is default)

#endif


/*
 * Advanced options
 */

#define BUTTON_HOLD_TIME_INDICATION   3000
#define BUTTON_HOLD_TIME_ACTION       10000
#define BUTTON_PRESS_TIME_ACTION      50

#define BOARD_PWM_MAX                 1023

#define BOARD_LEDC_CHANNEL_1     1
#define BOARD_LEDC_CHANNEL_2     2
#define BOARD_LEDC_CHANNEL_3     3
#define BOARD_LEDC_TIMER_BITS    10
#define BOARD_LEDC_BASE_FREQ     12000

#if !defined(CONFIG_DEVICE_PREFIX)
#define CONFIG_DEVICE_PREFIX          "Blynk"
#endif
#define CONFIG_AP_URL                 "blynk.setup"
#define CONFIG_DEFAULT_SERVER         "blynk.cloud"
#define CONFIG_DEFAULT_PORT           443

#define WIFI_CLOUD_MAX_RETRIES        500
#define WIFI_NET_CONNECT_TIMEOUT      50000
#define WIFI_CLOUD_CONNECT_TIMEOUT    50000
#define WIFI_AP_IP                    IPAddress(192, 168, 4, 1)
#define WIFI_AP_Subnet                IPAddress(255, 255, 255, 0)
//#define WIFI_CAPTIVE_PORTAL_ENABLE


//#define USE_TICKER
//#define USE_TIMER_ONE
//#define USE_TIMER_THREE
//#define USE_TIMER_FIVE
#define USE_PTHREAD

//#define BLYNK_NO_DEFAULT_BANNER

#if defined(APP_DEBUG)
  #define DEBUG_PRINT(...)  BLYNK_LOG1(__VA_ARGS__)
  #define DEBUG_PRINTF(...) BLYNK_LOG(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
#endif
