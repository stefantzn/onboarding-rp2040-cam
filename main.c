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

uint8_t image_buf[324*324]; //camera buffer, 8 bit color
uint16_t displayBuf[240*135]; //display buffer, in a weird RGB565 format

#define FLAG_VALUE 1234 // this flag value is going to be sent across cores!
uint8_t imageReady = 0; //only if the camera buffer has a new image, we display it!

int main() {

    stdio_init_all(); // initialzie stdio
    vreg_set_voltage(VREG_VOLTAGE_1_10); // set voltage to 1.1V
    set_sys_clock_khz(250000, true);  // set system clock to 250MHz

    int loops = 20;

    while (!tud_cdc_connected()) { // wait for USB to connect

        sleep_ms(100);

        if (--loops == 0) { // if USB doesn't connect in 2 seconds, break

            break;
        }
    }

    printf("tud_cdc_connected(%d)\n", tud_cdc_connected() ? 1 : 0);
    multicore_launch_core1(core1_entry);
    uint32_t ack = multicore_fifo_pop_blocking();

    if (ack != FLAG_VALUE) {

        printf("Error: core1_entry() returned %d\n", ack); // Core 0 failed to acknowledge core 1
    }
    else {

        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("core1_entry() returned %d\n", ack); // Success
    }

    while (true) {

        if (imageReady == 1) { // image is ready to be displayed

            LCD_1IN14_V2_Display(displayBuf);
            imageReady = 0;
        }

        sleep_ms(1);
    }
    
    tight_loop_contents(); // placeholder to find tight loops more easily
}

void core1_entry() {

    DEV_Module_Init(); // initialize all the screens and peripherals
    LCD_1IN14_V2_Init(HORIZONTAL); // initialize the LCD screen in the horizontal direction
    LCD_1IN14_V2_Clear(BLACK);  // clear the screen to black

    UDOUBLE Imagesize = LCD_1IN14_V2_HEIGHT * LCD_1IN14_V2_WIDTH; // image size
    UWORD *BlackImage; // black image

    if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) { // allocate memory for the black image

        printf("Failed to apply for black memory...\r\n"); // error
        exit(0);
    }

    // Notify core 0 that core 1 is ready
    multicore_fifo_push_blocking(FLAG_VALUE);
    
    // Wait for acknowledgment from core 0
    uint32_t ack = multicore_fifo_pop_blocking();

    if (ack != FLAG_VALUE) { // if core 0 fails to acknowledge core 1

        printf("Error: core 0 failed to acknowledge core 1!\n");
    }
    else {

        printf("Successfully received acknowledgment from core 0!\n");
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

    while (true) {  // infinite loop to capture images

        // Capture an image
        cam_capture_frame(&config);
        uint16_t index = 0;

        // Convert the image to a format that can be displayed, loop through the image buffer pixel by pixel
        for (int y = 134; y > 0; y--) {
            for (int x = 0; x < 240; x++) {
                

                uint16_t c = image_buf[(y)*324+(x)]; //index to the current pixel

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

