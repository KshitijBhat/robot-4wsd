#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "quadrature.pio.h"


#define byte uint8_t
#define numChars 32
char receivedChars[numChars];

#define ENA 15
#define IN1 14
#define IN2 13
#define QUADRATURE1_A_PIN 16
#define QUADRATURE1_B_PIN 17

#define ENB 9
#define IN3 8
#define IN4 7
#define QUADRATURE2_A_PIN 18
#define QUADRATURE2_B_PIN 19

long encoder_count_s = 0;  
long encoder_count_d = 0;  
PIO pio00 = pio0;
PIO pio11 = pio1;
uint sm1 = 0;
uint sm2 = 1;


void counter()
{   
    // counter
    pio_sm_exec_wait_blocking(pio00, sm1, pio_encode_in(pio_x, 32));
    encoder_count_s = pio_sm_get_blocking(pio00, sm1);

    pio_sm_exec_wait_blocking(pio11, sm2, pio_encode_in(pio_x, 32));
    encoder_count_d = pio_sm_get_blocking(pio11, sm2);
}

void command_steering_motor(int speed)
{
    int pwm_level;
    if(speed>255){
        speed = 255;
    }
    else if(speed<-255){
        speed = -255;
    }
    if(speed>0){
        gpio_put(IN1, 0);  //spin forward
        gpio_put(IN2, 1);
        pwm_level = (int)abs(speed);
    }
    else if(speed<0){
        gpio_put(IN1, 1);  //spin forward
        gpio_put(IN2, 0);
        pwm_level = (int)abs(speed);
    }
    else{
        gpio_put(IN1, 0);  //stop
        gpio_put(IN2, 0);
        pwm_level = 0;
    }
    
    uint slice_num = pwm_gpio_to_slice_num(ENA);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_level);
    pwm_set_enabled(slice_num, true);
}

void command_driving_motor(int speed)
{
    int pwm_level;
    if(speed>255){
        speed = 255;
    }
    else if(speed<-255){
        speed = -255;
    }
    if(speed>0){
        gpio_put(IN3, 0);  //spin forward
        gpio_put(IN4, 1);
        pwm_level = (int)abs(speed);
    }
    else if(speed<0){
        gpio_put(IN3, 1);  //spin forward
        gpio_put(IN4, 0);
        pwm_level = (int)abs(speed);
    }
    else{
        gpio_put(IN3, 0);  //stop
        gpio_put(IN4, 0);
        pwm_level = 0;
    }
    
    uint slice_num = pwm_gpio_to_slice_num(ENB);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_level);
    pwm_set_enabled(slice_num, true);
}


void getInput(int *steerCommand, int *driveCommand)
{
    char rc;
    rc = getchar();
    char endMarker = '.';
    static byte ndx = 0;
    if (rc == '\r')
    {
        counter();
        printf("%d\t%d\n",encoder_count_s, encoder_count_d);
    }
    else if (rc == endMarker) 
    {
        ndx = 0;
        char delim[] = " ";
        char *ptr = strtok(receivedChars, delim);
        *steerCommand = atoi(ptr);
        ptr = strtok(NULL, delim);
        *driveCommand = atoi(ptr);
    }
    else
    {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
}

int main(){
    //Initialise I/O
    stdio_init_all(); 

    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_function(ENA, GPIO_FUNC_PWM);

    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);
    gpio_set_function(ENB, GPIO_FUNC_PWM);

    PIO pio00 = pio0;
    PIO pio11 = pio1;
    uint offset0 = pio_add_program(pio00, &quadrature_program);
    uint offset1 = pio_add_program(pio11, &quadrature_program);
    // uint sm1 = pio_claim_unused_sm(pio, true);
    // uint sm2 = pio_claim_unused_sm(pio, true);
    quadrature_program_init(pio00, sm1, offset0, QUADRATURE1_A_PIN, QUADRATURE1_B_PIN);
    quadrature_program_init(pio11, sm2, offset1, QUADRATURE2_A_PIN, QUADRATURE2_B_PIN);


    int steerCommand = 0;
    int driveCommand = 0;
    //Main Loop 
    while(1){

        //Get User Input
        getInput(&steerCommand, &driveCommand);

        command_steering_motor(steerCommand);
        command_driving_motor(driveCommand);        
    }
}