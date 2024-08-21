#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "uart.h"

#define DRIVE_1A  GPIO_NUM_32 //green 
#define DRIVE_2A GPIO_NUM_33 //grey
#define DRIVE_3A  GPIO_NUM_25 //blue 
#define DRIVE_4A GPIO_NUM_26 //brown
 
#define STEP_DELAY 50


/* Full step drive sequence */ 
int step_sequence[4][4] = {
    {1, 0, 0, 0}, // Step 1: 1A High, 2A Low, 3A High, 4A Low
    {0, 0, 1, 0}, // Step 2: 1A Low, 2A High, 3A High, 4A Low
    {0, 1, 0, 0}, // Step 3: 1A Low, 2A High, 3A Low, 4A High
    {0, 0, 0, 1}  // Step 4: 1A High, 2A Low, 3A Low, 4A High
};

SemaphoreHandle_t xMutex;
char input;

void init_gpo(void);
int get_steps(void);
void clear_input_buffer(void);
void drive_left(void *pvParameters);
void drive_right(void *pvParameters);

/* drive a motor */
/* step angle is 1.8 deg */
void app_main(void)
{
    init_stdin_stdout();
    init_gpo();


    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL){
        printf("Failed to create mutex\n");
        return;
    }
    int steps = 0;
    /* create tasks to run on a single core */
    while (1){

        input = getchar();
        clear_input_buffer();
        switch (input) {

            case 'L':
            
                steps = get_steps();
                xTaskCreatePinnedToCore(drive_left, "DriveLeft", 1024*2, &steps, 1, NULL, 0);

                break;

            case 'R':

                steps = get_steps();
                xTaskCreatePinnedToCore(drive_right, "DriveRight", 1024*2, &steps, 1, NULL, 0);

                break;

            default:
                break;
        }
    }
}


void init_gpo(){

    gpio_set_direction(DRIVE_1A, GPIO_MODE_OUTPUT);
    gpio_set_direction(DRIVE_2A, GPIO_MODE_OUTPUT);
    gpio_set_direction(DRIVE_3A, GPIO_MODE_OUTPUT);
    gpio_set_direction(DRIVE_4A, GPIO_MODE_OUTPUT);

}


int get_steps(){

    int s;
    printf("\nEnter steps: ");
    if(scanf("%d", &s) != 1){
        fprintf(stderr, "\nInteger steps only.");
        return 1;
    }
    printf("\nUhh, %i steps.", s);
    return s;
}

void clear_input_buffer() {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

/* use a mutex to ensure only one direction is driven at a time. */

void drive_left(void *pvParameters){

    int steps = *(int*)pvParameters;
    int step;
    printf("\nDriving left for %i steps.", steps);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE){
        for (int i =0; i < steps; i++){

            step = i%4;
            gpio_set_level(DRIVE_1A, step_sequence[step][0]);
            gpio_set_level(DRIVE_2A, step_sequence[step][1]);
            gpio_set_level(DRIVE_3A, step_sequence[step][2]);
            gpio_set_level(DRIVE_4A, step_sequence[step][3]);

            // Delay between steps
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        }
    } xSemaphoreGive(xMutex);
 
}

void drive_right(void *pvParameters){

    int steps = *(int*)pvParameters;
    int step;
    printf("\nDriving right for %i steps.", steps);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE){
        for (int i = steps; i > 0; i--){

            step = i%4;
            gpio_set_level(DRIVE_1A, step_sequence[step][0]);
            gpio_set_level(DRIVE_2A, step_sequence[step][1]);
            gpio_set_level(DRIVE_3A, step_sequence[step][2]);
            gpio_set_level(DRIVE_4A, step_sequence[step][3]);

            // Delay between steps
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
        }
    } xSemaphoreGive(xMutex);

}