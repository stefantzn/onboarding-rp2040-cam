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

uint8_t image_buf[324*324];
uint16_t displayBuf[240*135];

#define FLAG_VALUE 1234
uint8_t imageReady = 0;

void core1_entry() {
    
    // LCD Init
    DEV_Module_Init();
    LCD_1IN14_V2_Init(HORIZONTAL);
    LCD_1IN14_V2_Clear(BLACK);
    UDOUBLE Imagesize = LCD_1IN14_V2_HEIGHT * LCD_1IN14_V2_WIDTH;
    UWORD *BlackImage;
    if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL)
    {
        printf("Failed to allocate uwu ...\r\n");
        exit(0);
    }

    // Notify core 0 that core 1 is ready
    multicore_fifo_push_blocking(FLAG_VALUE);

    // Wait for acknowledgment from core 0
    uint32_t ack  = multicore_fifo_pop_blocking();
    if (ack != FLAG_VALUE)
        printf("Error: Core 1 failed to receive acknowledgment from core 0!\n");
    else
        printf("Success: Core 1 Received acknowledgment from core 0!\n");


    
    // Create a new image cache and draw on the image
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN14_V2.WIDTH, LCD_1IN14_V2.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_SetRotate(ROTATE_0);
    Paint_DrawImage(realityLabsLogo, 0, 0, 240, 135);
    LCD_1IN14_V2_Display(BlackImage);

    // CAM Init
    struct cam_config config;
    cam_config_struct(&config);
    cam_init(&config);

    // processing image
    while (true) {
        cam_capture_frame(&config);

        uint16_t index = 0;
        for (int y = 134; y > 0; y--) {
            for (int x = 0; x < 240; x++) {
                uint16_t c = image_buf[(y)*324+(x)];
                uint16_t imageRGB = (((c & 0xF8) << 8) | ((c & 0xFC) << 3) | ((c & 0xF8) >> 3));
                displayBuf[index++] = (imageRGB >> 8) | (imageRGB << 8) ; //omfg im gonna im gonna AAAAAAAA FUCK
            }
        }
        imageReady = 1;
    }
}

int main() {
    stdio_init_all();
    vreg_set_voltage(VREG_VOLTAGE_1_10);
    set_sys_clock_khz(250000, true);

    int loops = 20;
    while (!tud_cdc_connected()) {
        sleep_ms(100);
        if (--loops == 0)
            break;
    }
    printf("tud_cdc_connected(%d)\n", tud_cdc_connected() ? 1 : 0);

    multicore_launch_core1(core1_entry);

    // Wait for acknowledgment from core 1
    uint32_t ack = multicore_fifo_pop_blocking();
    if (ack != FLAG_VALUE)
        printf("Error: Core 0 failed to receive acknowledgment from core 1!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("Success: Core 0 Received acknowledgment from core 1!\n");
    }

    // Show Image
    while (true) {
        if (imageReady == 1) {
            LCD_1IN14_V2_Display(displayBuf);
            // Reset the imageReady flag after displaying the image
            imageReady = 0;
        }
        sleep_ms(1);
    }
    tight_loop_contents();
}