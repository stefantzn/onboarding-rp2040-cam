#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/gpio.h"
#include "cam.h"
#include "ImageData.h"
#include "LCD_1in14_V2.h" 
#include "GUI_Paint.h"
#include "hardware/clocks.h"

uint8_t image_buf[324*324]; // Camera buffer, 8 bit color
uint16_t displayBuf[240*135]; // Display buffer, in a weird RGB565 format

#define FLAG_VALUE 1234 // This flag value is going to be sent across cores!
uint8_t imageReady = 0; // Only if the camera buffer has a new image, we display it!

int main() {

    stdio_init_all(); // Initialzie stdio
    vreg_set_voltage(VREG_VOLTAGE_1_10); // Set voltage to 1.1V
    set_sys_clock_khz(250000, true);  // Set system clock to 250MHz

    // 20 loops x 100 ms = 20s
    int loops = 20;

    // Wait for USB to connect
    while (!tud_cdc_connected()) { 

        sleep_ms(100);

        if (--loops == 0) { // If USB doesn't connect in 2 seconds, break

            break;
        }
    }

    printf("tud_cdc_connected(%d)\n", tud_cdc_connected() ? 1 : 0);
    multicore_launch_core1(core1_entry);
    uint32_t ack = multicore_fifo_pop_blocking();

    // Wait for acknowledgment from core 1
    if (ack != FLAG_VALUE) {

        printf("Error: core1_entry() returned %d\n", ack); // Core 0 failed to acknowledge core 1
    }
    else {
        
        // Notify core 1 that core 0 is ready
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("core1_entry() returned %d\n", ack); // Success
    }

    while (true) {

        if (imageReady == 1) { // Image is ready to be displayed

            // Display the image
            LCD_1IN14_V2_Display(displayBuf);
            imageReady = 0;
        }

        sleep_ms(1);
    }
    
    tight_loop_contents(); // Placeholder to find tight loops more easily
}

void core1_entry() {

    DEV_Module_Init(); // Initialize all the screens and peripherals
    LCD_1IN14_V2_Init(HORIZONTAL); // Initialize the LCD screen in the horizontal direction
    LCD_1IN14_V2_Clear(BLACK);  // Clear the screen to black

    UDOUBLE Imagesize = LCD_1IN14_V2_HEIGHT * LCD_1IN14_V2_WIDTH; // Image size
    UWORD *BlackImage; // Black image

    if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) { // Allocate memory for the black image

        printf("Failed to apply for black memory...\r\n"); // Error
        exit(0);
    }

    // Notify core 0 that core 1 is ready
    multicore_fifo_push_blocking(FLAG_VALUE);
    
    // Wait for acknowledgment from core 0
    uint32_t ack = multicore_fifo_pop_blocking();

    if (ack != FLAG_VALUE) { // If core 0 fails to acknowledge core 1

        printf("Error: core 0 failed to acknowledge core 1!\n"); // Error
    }
    else {

        printf("Successfully received acknowledgment from core 0!\n"); // Success
    }

    // Logo drawing portion
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN14_V2.WIDTH, LCD_1IN14_V2.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_SetRotate(ROTATE_0);
    Paint_DrawImage(realityLabsLogo, 0, 0, 240, 135);
    LCD_1IN14_V2_Display(BlackImage);

    // Camera initialization
    struct cam_config config;
    cam_config_struct(&config);
    cam_init(&config);

    while (true) {  // Infinite loop to capture images

        // Capture an image
        cam_capture_frame(&config);
        uint16_t index = 0;

        // Convert the image to a format that can be displayed, loop through the image buffer pixel by pixel
        for (int y = 134; y > 0; y--) {
            for (int x = 0; x < 240; x++) {
                

                uint16_t c = image_buf[(y)*324+(x)]; // Index to the current pixel

                /*
                * Red: Mask with 0xF8 then shift left 8 bits   (5 bits)
                * Green: Mask with 0xFC then shift left 3 bits (6 bits)
                * Blue: Mask with 0xF8 then shift right 3 bits (5 bits)
                * 
                * Combine the RGB values (using or operation) to form a 16 bit color
                */
                uint16_t imageRGB = (((c & 0xF8) << 8) | ((c & 0xFC) << 3) | ((c & 0xF8) >> 3));
                
                displayBuf[index++] = (imageRGB >> 8) | (imageRGB << 8);
            }
        }

        imageReady = 1;
    }
}

