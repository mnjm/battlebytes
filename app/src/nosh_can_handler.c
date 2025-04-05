
/* Platform dependent includes */
#include "asdk_platform.h"
/* ASDK includes */
#include "asdk_error.h"
/* Application specific includes */
#include <string.h>
#include "app_can.h"
#include "asdk_app.h"
/* Debug Print includes */
#include "debug_print.h"

// Define specific CAN IDs used in the application.
#define CAN_ST1_ID 0x300
#define CAN_ST2_ID 0x301
#define CAN_RX1_ID 0x305
#define CAN_RX2_ID 0x306

#define BRAKE_LAMP_ON 0x02


#define TAIL_LAMP_ON 0x01


#define INDICATOR_LEFT_ON 0x01
#define INDICATOR_RIGHT_ON 0x02
#define INDICATOR_LEFT_OFF 0x03
#define INDICATOR_RIGHT_OFF 0x04
#define INDICATOR_BOTH_OFF 0x00
#define INDICATOR_BLINK_INTERVAL_MS 300
#define INDICATOR_ON_TIME_MS 300

#define THROTTLE_FRWD_SPEED 0x05
#define THROTTLE_BCWD_SPEED 0x03
#define INDICATOR_BLINK_INTERVAL_MS 300
#define INDICATOR_ON_DURATION_MS 300
#define INDICATOR_OFF_DURATION_MS 300

uint8_t nosh_rx2_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint32_t last_indicator_toggle_time = 0;
static bool indicator_state = false; // current on/off state for blinking

// Buffers to store received data for specific CAN IDs.
uint8_t _300_buff[8] = {0};
uint8_t _301_buff[8] = {0};

typedef struct
{
    bool horn_button;         // Byte 0: 1 bit, pressed or not
    uint8_t brakes_input;     // Byte 1: 1 byte, brake status
    uint8_t indicator_switch; // Byte 2: 1 byte, indicator status
    int8_t throttle;          // Byte 3: 1 byte, throttle value
    bool sidestand_status;    // Byte 4: 1 bit, sidestand engaged or not
    bool start_button;        // Byte 5: 1 bit, start button pressed or not
} VehicleStatus1;

// Global structure object
VehicleStatus1 st1_obj;

// Function to parse the buffer and populate the structure
void parse_st1(uint8_t *buffer)
{
    // Parsing horn button (Byte 0)
    st1_obj.horn_button = (buffer[0] & 0x01) != 0;
    if (st1_obj.horn_button)
    {
        printf("Horn pressed\n");
    }

    // Parsing brakes input (Byte 1)
    st1_obj.brakes_input = buffer[1];
    if (st1_obj.brakes_input > 0)
    {
        printf("Brakes input: %d\n", st1_obj.brakes_input);
    }

    // Parsing indicator switch (Byte 2)
    st1_obj.indicator_switch = buffer[2];
    if (st1_obj.indicator_switch > 0)
    {
        printf("Indicator switch: %d\n", st1_obj.indicator_switch);
    }

    // Parsing throttle (Byte 3)
    st1_obj.throttle = (int8_t)buffer[3]; // Assuming byte is signed
    if (st1_obj.throttle > 0)
    {
        printf("Throttle: %d\n", st1_obj.throttle);
    }

    // Parsing sidestand status (Byte 4)
    st1_obj.sidestand_status = (buffer[4] & 0x01) != 0;
    if (st1_obj.sidestand_status)
    {
        printf("Sidestand engaged\n");
    }

    // Parsing start button (Byte 5)
    st1_obj.start_button = (buffer[5] & 0x01) != 0;
    if (st1_obj.start_button)
    {
        printf("Start button pressed\n");
    }
}

typedef struct
{
    uint8_t riding_mode;   // Byte 0: 0 -> Neutral, 1 -> Forward, etc.
    uint8_t vehicle_speed; // Byte 1: Vehicle speed in km/h
    int16_t roll;          // Byte 2-3: Roll (little endian)
    int16_t pitch;         // Byte 4-5: Pitch (little endian)
    int16_t yaw;           // Byte 6-7: Yaw (little endian)
} VehicleStatus2;

VehicleStatus2 st2_obj;

// Function to parse the buffer and populate the structure
void parse_st2(uint8_t *buffer)
{
    // Parse Riding Mode (Byte 0)
    st2_obj.riding_mode = buffer[0];

    // Parse Vehicle Speed (Byte 1)
    st2_obj.vehicle_speed = buffer[1];

    // Parse Roll (Bytes 2-3)
    st2_obj.roll = (int16_t)(buffer[2] | (buffer[3] << 8)); // Little Endian

    // Parse Pitch (Bytes 4-5)
    st2_obj.pitch = (int16_t)(buffer[4] | (buffer[5] << 8)); // Little Endian

    // Parse Yaw (Bytes 6-7)
    st2_obj.yaw = (int16_t)(buffer[6] | (buffer[7] << 8)); // Little Endian
}

// Function to print out the parsed vehicle status
void print_st2(const VehicleStatus2 *status)
{
    DEBUG_PRINTF("Riding Mode: %d\n", st2_obj.riding_mode);
    DEBUG_PRINTF("Vehicle Speed: %d km/h\n", st2_obj.vehicle_speed);
    DEBUG_PRINTF("Roll: %d degrees\n", st2_obj.roll);
    DEBUG_PRINTF("Pitch: %d degrees\n", st2_obj.pitch);
    DEBUG_PRINTF("Yaw: %d degrees\n", st2_obj.yaw);
}

/**
 * @brief Prints the contents of a buffer in hexadecimal format for debugging.
 *
 * @param buff Pointer to the buffer to print.
 */
void print_buffer(uint8_t *buff)
{
    for (int i = 0; i < 8; i++)
    {
        DEBUG_PRINTF("%02X ", buff[i]); // %02X formats as 2-digit uppercase hex
    }
    DEBUG_PRINTF("\r\n"); // Add a newline for better readability
}

/**
 * @brief Handles received CAN messages based on their CAN ID.
 *
 * @param can_id The CAN ID of the received message.
 * @param msg Pointer to the received message data.
 * @param len The length of the received message data.
 */
void nosh_can_handler(uint32_t can_id, uint8_t *msg, uint8_t len)
{
    if (can_id == CAN_ST1_ID)
    {
        if (len <= sizeof(_300_buff))
        {
            memcpy(_300_buff, msg, len);
            if (memcmp(_300_buff, msg, len) == 0)
            {
                //  DEBUG_PRINTF("Memcpy success for ID 0x%03X to _300_buff\r\n", can_id);
            }
            else
            {
                DEBUG_PRINTF("Memcpy failed for ID 0x%03X to _300_buff\r\n", can_id);
            }
            // DEBUG_PRINTF("Debug State1 (ID: 0x%03X) - ", can_id);
            parse_st1(_300_buff);
            // print_buffer(_300_buff);
        }
        else
        {
            DEBUG_PRINTF("Error: Received data length (%d) exceeds buffer size for ID 0x%03X\r\n", len, can_id);
        }
    }
    else if (can_id == CAN_ST2_ID)
    {
        if (len <= sizeof(_301_buff))
        {
            memcpy(_301_buff, msg, len);
            if (memcmp(_301_buff, msg, len) == 0)
            {
                // DEBUG_PRINTF("Memcpy success for ID 0x%03X to _301_buff\r\n", can_id);
            }
            else
            {
                DEBUG_PRINTF("Memcpy failed for ID 0x%03X to _301_buff\r\n", can_id);
            }
            // DEBUG_PRINTF("Debug State2 (ID: 0x%03X) - ", can_id);
            // print_buffer(_301_buff);
            parse_st2(_301_buff);
            // print_st2();
        }
        else
        {
            DEBUG_PRINTF("Error: Received data length (%d) exceeds buffer size for ID 0x%03X\r\n", len, can_id);
        }
    }
    else
    {
        DEBUG_PRINTF("Received CAN message with unhandled ID: 0x%03X, Length: %d\r\n", can_id, len);
    }
}

void horn_horn(void)
{
    if (st1_obj.horn_button)
    {
        nosh_rx2_msg[0] = 0x01; // Command type: Set Horn
        nosh_rx2_msg[1] = 0x01; // Enable horn
        app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);

        // Optional: Add delay if needed, but better to handle timing in main loop
        // ASDK_DELAY(100);
    }
    else
    {
        nosh_rx2_msg[0] = 0x01; // Command type: Set Horn
        nosh_rx2_msg[1] = 0x00; // Disable horn
        app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
    }
}

void control_brake_lamp(void)
{
    nosh_rx2_msg[0] = 0x03; // Command type: Set Tail/Brake Lamp

    if (st1_obj.brakes_input > 0)
    {
        // Any brake pressed - activate brake lamp
        nosh_rx2_msg[1] = TAIL_LAMP_ON;
    }
    else
    {
        // No brakes pressed - only tail lamp
        nosh_rx2_msg[1] = BRAKE_LAMP_ON ;
    }
    app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
}



// void control_indicators(uint32_t current_time) {
//     static uint8_t last_indicator_status = 0;

//     // Check if indicator switch status changed or it's time to toggle
//     if (st1_obj.indicator_switch != last_indicator_status ||
//         (current_time - last_indicator_toggle_time) >= INDICATOR_BLINK_INTERVAL_MS) {

//         // Handle center button press (turn off indicators)
//         if (st1_obj.indicator_switch == 3) {
//             uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF};  // Turn off left
//             app_can_send(CAN_RX1_ID, data, 2);

//             data[1] = INDICATOR_RIGHT_OFF;  // Turn off right
//             app_can_send(CAN_RX1_ID, data, 2);

//             last_indicator_status = st1_obj.indicator_switch;
//             indicator_state = false;  // Reset indicator state
//             return;
//         }

//         // Handle left or right indicator
//         if (st1_obj.indicator_switch == 1 || st1_obj.indicator_switch == 2) {
//             // Toggle indicator state
//             indicator_state = !indicator_state;

//             uint8_t data[2] = {0x04, 0};  // Command type: Set Indicator

//             if (st1_obj.indicator_switch == 1) {  // Left indicator
//                 data[1] = indicator_state ? INDICATOR_LEFT_ON : INDICATOR_LEFT_OFF;
//             } else {  // Right indicator
//                 data[1] = indicator_state ? INDICATOR_RIGHT_ON : INDICATOR_RIGHT_OFF;
//             }

//             app_can_send(CAN_RX1_ID, data, 2);
//             last_indicator_toggle_time = current_time;
//         } else {
//             // No indicators active (status 0) - ensure both are off
//             uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF};  // Turn off left
//             app_can_send(CAN_RX1_ID, data, 2);

//             data[1] = INDICATOR_RIGHT_OFF;  // Turn off right
//             app_can_send(CAN_RX1_ID, data, 2);

//             indicator_state = false;  // Reset indicator state
//         }

//         last_indicator_status = st1_obj.indicator_switch;
//     }
// }

// void control_indicators(uint32_t current_time)
// {
//     static uint8_t last_indicator_status = 0;
//     static uint32_t blink_start_time = 0;
//     static bool is_blinking = false;
//     // Check if indicator switch status changed
//     if (st1_obj.indicator_switch != last_indicator_status)
//     {
//         // Reset blinking state when switch changes
//         is_blinking = false;
//         blink_start_time = current_time;
//         // Handle center button press (turn off indicators)
//         if (st1_obj.indicator_switch == 3)
//         {
//             uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF}; // Turn off left
//             app_can_send(CAN_RX1_ID, data, 2);
//             data[1] = INDICATOR_RIGHT_OFF; // Turn off right
//             app_can_send(CAN_RX1_ID, data, 2);
//             last_indicator_status = st1_obj.indicator_switch;
//             return;
//         }
//         last_indicator_status = st1_obj.indicator_switch;
//     }
//     // Handle left or right indicator blinking
//     if (st1_obj.indicator_switch == 1 || st1_obj.indicator_switch == 2)
//     {
//         uint32_t elapsed_time = current_time - blink_start_time;
//         // Calculate blink cycle
//         uint32_t cycle_time = elapsed_time % INDICATOR_BLINK_INTERVAL_MS;
//         // Determine if indicator should be on or off
//         bool should_be_on = (cycle_time < INDICATOR_ON_TIME_MS);
//         // Only send message if state changes
//         if (should_be_on != is_blinking)
//         {
//             uint8_t data[2] = {0x04, 0}; // Command type: Set Indicator
//             if (st1_obj.indicator_switch == 1)
//             { // Left indicator
//                 data[1] = should_be_on ? INDICATOR_LEFT_ON : INDICATOR_LEFT_OFF;
//             }
//             else
//             { // Right indicator
//                 data[1] = should_be_on ? INDICATOR_RIGHT_ON : INDICATOR_RIGHT_OFF;
//             }
//             app_can_send(CAN_RX1_ID, data, 2);
//             is_blinking = should_be_on;
//         }
//     }
//     else if (is_blinking)
//     {
//         // No indicators active (status 0) - ensure both are off
//         uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF}; // Turn off left
//         app_can_send(CAN_RX1_ID, data, 2);
//         data[1] = INDICATOR_RIGHT_OFF; // Turn off right
//         app_can_send(CAN_RX1_ID, data, 2);
//         is_blinking = false;
//     }
// }

// bool throttle_mode = false;  // false = backward

// void throttle_control(void) {
//     // nosh_rx2_msg[0] = 0x02; // Command type: Set Throttle
//     // nosh_rx2_msg[1] = st1_obj.throttle;
//     app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
//     if (st1_obj.start_button) {
//         throttle_mode = ! throttle_mode;
//         DEBUG_PRINTF("Throttle Control Forward %d\n", throttle_mode);
//     }

//     if (st1_obj.throttle > 0) {
//         if (throttle_mode) {
//             nosh_rx2_msg[0] = 0x01;
//             nosh_rx2_msg[2] = THROTTLE_FRWD_SPEED;
//             app_can_send(CAN_RX2_ID, nosh_rx2_msg, 8);
//         } else {
//             nosh_rx2_msg[0] = 0x02;
//             nosh_rx2_msg[2] = THROTTLE_BCWD_SPEED;
//             app_can_send(CAN_RX2_ID, nosh_rx2_msg, 8);
//         }
//     }
// }

bool throttle_mode = false; // false = backward, true = forward
bool old_start_mem = false; // Memory for the previous state of the start button
void throttle_control(void)
{
    // Check if the start button state has changed from not pressed to pressed
    if (st1_obj.start_button && !old_start_mem)
    {
        throttle_mode = !throttle_mode; // Toggle throttle mode
        DEBUG_PRINTF("Throttle Control Mode: %s\n", throttle_mode ? "Forward" : "Backward");
    }
    // Update the old start button memory
    old_start_mem = st1_obj.start_button;
    // Prepare the message
    // Check if throttle is applied
    if (st1_obj.throttle > 0)
    {
        if (throttle_mode)
        {
            // Forward mode
            nosh_rx2_msg[0] = 0x01;                // Command for forward throttle
            nosh_rx2_msg[2] = THROTTLE_FRWD_SPEED; // Set forward speed
            DEBUG_PRINTF("Throttle Forward Speed: %d kmph\n", THROTTLE_FRWD_SPEED);
        }
        else
        {
            // Backward mode
            nosh_rx2_msg[0] = 0x02;                // Command for backward throttle
            nosh_rx2_msg[2] = THROTTLE_BCWD_SPEED; // Set backward speed
            DEBUG_PRINTF("Throttle Backward Speed: %d kmph\n", THROTTLE_BCWD_SPEED);
        }
        // Send the throttle command
        app_can_send(CAN_RX2_ID, nosh_rx2_msg, 8);
    }
    else
    {
        // If throttle is zero, stop the vehicle
        nosh_rx2_msg[0] = 0x00; // Command to stop
        nosh_rx2_msg[2] = 0;    // Speed is zero
        app_can_send(CAN_RX2_ID, nosh_rx2_msg, 8);
        DEBUG_PRINTF("Throttle Speed: 0 kmph (Stopped)\n");
    }
}

void side_stand_control(void)
{
    bool is_engaged = (_300_buff[4] & 0x01) != 0; // Check bit 0 of byte 4
    DEBUG_PRINTF("Side stand status: %d\n", is_engaged);
    if (is_engaged)
    {
        DEBUG_PRINTF("Side stand engaged - Vehicle immobilized\n");
        // Send immobilization command (0x05, 0x01)
        uint8_t data[1] = {0x00};
        app_can_send(CAN_RX2_ID, data, 0);
    }
    else
    {
        DEBUG_PRINTF("Side stand disengaged - Vehicle operational\n");
        // // Send operational command (0x05, 0x00)
        // uint8_t data[2] = {0x00, 0x00};
        // app_can_send(CAN_RX1_ID, data, 2);
    }
}

void start_button_control(void)
{
    nosh_rx2_msg[0] = 0x06;
    nosh_rx2_msg[1] = st1_obj.start_button;
    app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
}

// void control_indicators(uint32_t current_time) {
//     static uint8_t last_indicator_status = 0;
//     static uint32_t blink_start_time = 0;
//     static bool is_blinking = false;
//     // Check if indicator switch status changed
//     if (st1_obj.indicator_switch != last_indicator_status) {
//         // Reset blinking state when switch changes
//         is_blinking = false;
//         blink_start_time = current_time;
//         // Handle center button press (turn off indicators)
//         if (st1_obj.indicator_switch == 3) {
//             uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF};  // Turn off left
//             app_can_send(CAN_RX1_ID, data, 2);
//             data[1] = INDICATOR_RIGHT_OFF;  // Turn off right
//             app_can_send(CAN_RX1_ID, data, 2);
//             last_indicator_status = st1_obj.indicator_switch;
//             return;
//         }
//         last_indicator_status = st1_obj.indicator_switch;
//     }
//     // Handle left or right indicator blinking
//     if (st1_obj.indicator_switch == 1 || st1_obj.indicator_switch == 2) {
//         uint32_t elapsed_time = current_time - blink_start_time;
//         // Calculate blink cycle
//         uint32_t cycle_time = elapsed_time % INDICATOR_BLINK_INTERVAL_MS;
//         // Determine if indicator should be on or off
//         bool should_be_on = (cycle_time < INDICATOR_ON_TIME_MS);
//         // Only send message if state changes
//         if (should_be_on != is_blinking) {
//             uint8_t data[2] = {0x04, 0};  // Command type: Set Indicator
//             if (st1_obj.indicator_switch == 1) {  // Left indicator
//                 data[1] = should_be_on ? INDICATOR_LEFT_ON : INDICATOR_LEFT_OFF;
//             } else {  // Right indicator
//                 data[1] = should_be_on ? INDICATOR_RIGHT_ON : INDICATOR_RIGHT_OFF;
//             }
//             app_can_send(CAN_RX1_ID, data, 2);
//             is_blinking = should_be_on;
//         }
//     } else if (is_blinking) {
//         // No indicators active (status 0) - ensure both are off
//         uint8_t data[2] = {0x04, INDICATOR_LEFT_OFF};  // Turn off left
//         app_can_send(CAN_RX1_ID, data, 2);
//         data[1] = INDICATOR_RIGHT_OFF;  // Turn off right
//         app_can_send(CAN_RX1_ID, data, 2);
//         is_blinking = false;
//     }
// }

void control_indicators(uint32_t current_time_ms)
{
    static uint8_t active_indicator = 0; // 0=off, 1=left, 2=right
    static bool indicator_state = false; // current on/off state
    static uint32_t last_toggle_time = 0;
    // Check for button presses (edge detection)
    static uint8_t last_button_state = 0;
    uint8_t current_button_state = st1_obj.indicator_switch;
    // Detect rising edge (button press)
    if (current_button_state != last_button_state && current_button_state > 0)
    {
        // Center button cancels all indicators
        if (current_button_state == 3)
        {
            active_indicator = 0;
            indicator_state = false;
            // Immediately turn off both indicators
            nosh_rx2_msg[0] = 0x04; // Command type: Set Indicator
            nosh_rx2_msg[1] = INDICATOR_LEFT_OFF;
            app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
            nosh_rx2_msg[1] = INDICATOR_RIGHT_OFF;
            app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
        }
        // Left button press - activate left indicator
        else if (current_button_state == 1)
        {
            active_indicator = 1;
            indicator_state = true;
            nosh_rx2_msg[0] = 0x04; // Command type: Set Indicator
            nosh_rx2_msg[1] = INDICATOR_BOTH_OFF;
            app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
            last_toggle_time = current_time_ms;
        }
        // Right button press - activate right indicator
        else if (current_button_state == 2)
        {
            active_indicator = 2;
            indicator_state = true;
            nosh_rx2_msg[0] = 0x04; // Command type: Set Indicator
            nosh_rx2_msg[1] = INDICATOR_BOTH_OFF;
            app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
            last_toggle_time = current_time_ms;
        }
    }
    last_button_state = current_button_state;
    // Handle blinking for active indicator
    if (active_indicator > 0 &&
        (current_time_ms - last_toggle_time >=
         (indicator_state ? INDICATOR_ON_DURATION_MS : INDICATOR_OFF_DURATION_MS)))
    {
        indicator_state = !indicator_state;
        last_toggle_time = current_time_ms;
        nosh_rx2_msg[0] = 0x04; // Command type: Set Indicator
        if (active_indicator == 1)
        {
            nosh_rx2_msg[1] = indicator_state ? INDICATOR_LEFT_ON : INDICATOR_LEFT_OFF;
        }
        else
        { // active_indicator == 2
            nosh_rx2_msg[1] = indicator_state ? INDICATOR_RIGHT_ON : INDICATOR_RIGHT_OFF;
        }
        app_can_send(CAN_RX1_ID, nosh_rx2_msg, 8);
#ifdef DEBUG
        DEBUG_PRINTF("Indicator %s: %s at %lums\n",
                     active_indicator == 1 ? "LEFT" : "RIGHT",
                     indicator_state ? "ON" : "OFF",
                     current_time_ms);
#endif
    }
}