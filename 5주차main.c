#include "stm32f10x.h"

#define MEM_OFFSET(x, y) (volatile unsigned int *) (x + y);

unsigned int BASE_RCC = 0x40021000;
unsigned int BASE_PORT_A = 0x40010800;
unsigned int BASE_PORT_B = 0x40010C00;
unsigned int BASE_PORT_C = 0x40011000;
unsigned int BASE_PORT_D = 0x40011400;
unsigned int OFFSET_APB2_CLOCK = 0x00000018;
unsigned int OFFSET_PORT_CONF_LOW = 0x00;
unsigned int OFFSET_PORT_CONF_HIGH = 0x04;
unsigned int OFFSET_INPUT_DATA = 0x08;
unsigned int OFFSET_BIT_SET_RESET = 0x10;

void enable_ports() {
  volatile unsigned int *REG_APB2_CLOCK = MEM_OFFSET(BASE_RCC, OFFSET_APB2_CLOCK);

  *REG_APB2_CLOCK = 0x3C;
}

void delay(){
  int i;
  for(i=0; i<10000000; i++){}
  
}

void set_input_output() {
  volatile unsigned int *REG_PORT_A_LOW = MEM_OFFSET(BASE_PORT_A, OFFSET_PORT_CONF_LOW);
  volatile unsigned int *REG_PORT_B_LOW = MEM_OFFSET(BASE_PORT_B, OFFSET_PORT_CONF_LOW);
  volatile unsigned int *REG_PORT_C_LOW = MEM_OFFSET(BASE_PORT_C, OFFSET_PORT_CONF_LOW);
  volatile unsigned int *REG_PORT_C_HIGH = MEM_OFFSET(BASE_PORT_C, OFFSET_PORT_CONF_HIGH);
  volatile unsigned int *REG_PORT_D_LOW = MEM_OFFSET(BASE_PORT_D, OFFSET_PORT_CONF_LOW);
  
  *REG_PORT_A_LOW = 0x44444448;
  *REG_PORT_B_LOW = 0x44844444;
  *REG_PORT_C_LOW = 0x44448444;
  *REG_PORT_C_HIGH = 0x44844444;
  *REG_PORT_D_LOW = 0x34433344;
}
// LED1: PD2, LED2: PD3, LED3: PD4, LED4: PD7
// reset 1 => turn on
void turn_on_led(int led1, int led2, int led3, int led4) {
  volatile unsigned int *PORT_D_SET = MEM_OFFSET(BASE_PORT_D, OFFSET_BIT_SET_RESET);
  
  volatile unsigned int val = *PORT_D_SET;
  
  if (led1 == 1) val |= 0x40000;
  if (led2 == 1) val |= 0x80000;
  
  *PORT_D_SET = val;
}

// set 1 => turn off
void turn_off_led(int led1, int led2, int led3, int led4) {
  volatile unsigned int *PORT_D_SET = MEM_OFFSET(BASE_PORT_D, OFFSET_BIT_SET_RESET);
  
  volatile unsigned int val = *PORT_D_SET;
  if (led1 == 1) val |= 0x4;
  if (led2 == 1) val |= 0x8;
  
  *PORT_D_SET = val;
  
  delay();
  turn_on_led(led1, led2, led3, led4);
}


// KEY1: PC4, KEY2: PB10, KEY3: PC13, KEY4: PA0
int is_button_pushed(int btn) {
  volatile unsigned int *PORT_A_INPUT = MEM_OFFSET(BASE_PORT_A, OFFSET_INPUT_DATA);
  volatile unsigned int *PORT_B_INPUT = MEM_OFFSET(BASE_PORT_B, OFFSET_INPUT_DATA);
  volatile unsigned int *PORT_C_INPUT = MEM_OFFSET(BASE_PORT_C, OFFSET_INPUT_DATA);
  
  if (btn == 1) {
    volatile unsigned int input = *PORT_C_INPUT;
    return (input & 0x10) != 0x10;
  }
  if (btn == 2) {
    volatile unsigned int input = *PORT_B_INPUT;
    return (input & 0x400) != 0x400;
  }
  if (btn == 3) {
    volatile unsigned int input = *PORT_C_INPUT;
    return (input & 0x2000) != 0x2000;
  }
  
  return 0;
}

int main(void)
{
  enable_ports();
  set_input_output();
  
  turn_on_led(1,1,1,1);
  
  while(1) {
    if (is_button_pushed(1)) {
      turn_off_led(1, 1, 0, 0);
    }
    if (is_button_pushed(2)) {
      turn_off_led(1, 0, 0, 0);
    }
    if (is_button_pushed(3)) {
      turn_off_led(0, 1, 0, 0);
    }
  }
  
  
  return 0;
}