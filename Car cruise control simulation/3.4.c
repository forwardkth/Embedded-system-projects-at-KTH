#include <stdio.h>
#include "system.h" 
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "LCD.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 50 /* 50ms */

/* Button Patterns */
#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02        //when button is pressed,its value is 0

/* Switch Patterns */
#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001           //when switch is on,it means 1

/* LED Patterns */
#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear
#define LED_RED_12 0x00001000 
#define LED_RED_13 0x00002000
#define LED_RED_14 0x00004000
#define LED_RED_15 0x00008000
#define LED_RED_16 0x00010000
#define LED_RED_17 0x00020000

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */
#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];

OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];

// Task Priorities
#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  14
#define CONTROLTASK_PRIO  16
#define SWITCHIOTASK_PRIO  12
#define BUTTONIOTASK_PRIO  10

// Task Periods
#define CONTROL_PERIOD  250
#define VEHICLE_PERIOD  250
#define SWITCHIO_PERIOD  150
#define BUTTONIO_PERIOD  50

/*
 * Definition of Kernel Objects 
 */
// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

//OS_EVENT *Mbox_Engine;
//OS_EVENT *Mbox_Gear;

// Semaphores
OS_EVENT *Sem_vehicle;
OS_EVENT *Sem_control;
OS_EVENT *Sem_switchio;
OS_EVENT *Sem_buttonio;

// SW-Timer
OS_TMR  *Vehicle_softTmr;
OS_TMR  *Control_softTmr;
OS_TMR  *SwitchIO_softTmr;
OS_TMR  *ButtonIO_softTmr;
/*
 * Types
 */
enum active {on, off};

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off; 

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
INT8U disp_velocity = 0; //velocity to show on seven segment display
INT32S target_velocity=0;//target velocity to show on seven segment display
INT16S Global_velocity=0;  // global value


int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(BUTTON_PIO_BASE);    //attention:the return value is conplement!

}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(SWITCH_PIO_BASE);    
}



/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */    //??? update the timer
  
  return delay;                 //next time to call the alarm_handler function. form a periodic 
                                //call.
                                   
}

void Vehicle_releaseSem()           //callback function of Vehicle_softTmr
{
    OSSemPost(Sem_vehicle);
}

void Control_releaseSem()           //callback function of Control_softTmr
{
    OSSemPost(Sem_control);
}

void SwitchIO_releaseSem()           //callback function of SwitchIO_softTmr
{
    OSSemPost(Sem_switchio);
}

void ButtonIO_releaseSem()           //callback function of ButtonIO_softTmr
{
    OSSemPost(Sem_buttonio);
}

/*
 * output current velocity on the seven segement display
 */
void show_number_on_sevenseg(void)
{
  IOWR_ALTERA_AVALON_PIO_DATA(SEG7_DISPLAY_BASE, disp_velocity);
}

/*
 * show the current velocity on the seven segment display
 */
void show_velocity_on_sevenseg(INT8U velocity)
{
  disp_velocity = disp_velocity & (0<<4) & 0;                     //clear all 8 bits
  INT8U high_bcd = velocity / 10;
  INT8U low_bcd = velocity - high_bcd * 10;
  
  disp_velocity = disp_velocity | (high_bcd << 4) | low_bcd;
  //show_number_on_sevenseg();             //solve the flash problem
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT16S target_vel)
{  
  target_velocity = disp_velocity & (0<<4) & 0;                     //clear all 8 bits
  INT8U high_bcd = target_vel / 10;
  INT8U low_bcd = target_vel - high_bcd * 10;
  
  target_velocity = target_velocity | (high_bcd << 20) | (low_bcd<<16)|disp_velocity;
  IOWR_ALTERA_AVALON_PIO_DATA(SEG7_DISPLAY_BASE, target_velocity);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
    if (position < 4000) 
        if (engine==on)
            if(top_gear==on)
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_17);
            else
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_17);
        else
            if(top_gear==on)
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_17);
            else
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_17);
                
    else
      if (position < 8000)
          if (engine==on)
              if(top_gear==on)
                  IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_16);
              else
                  IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_16);
          else
              if(top_gear==on)
                  IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_16);
              else
                  IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_16);
                  
      else 
        if (position < 12000)
            if (engine==on)
                if(top_gear==on)
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_15);
                else
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_15);
            else
                if(top_gear==on)
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_15);
                else
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_15);
                    
        else
          if (position < 16000)
                if (engine==on)
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_14);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_14);
                else
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_14);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_14);
                        
          else
            if (position < 20000)
                if (engine==on)
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_13);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_13);
                else
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_13);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_13);
            else
                if (engine==on)
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1|LED_RED_12);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_12);
                else
                    if(top_gear==on)
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1|LED_RED_12);
                    else
                        IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_12);
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
 INT16U adjust_position(INT16U position, INT16S velocity,
                        INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000   //divided by 1000 to change time_intervalfrom ms to s. 
    + acceleration / 2  * (time_interval / 1000) * (time_interval / 1000);

  if (new_position > 24000) {
    new_position -= 24000;
  }
  
  show_position(new_position);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
               enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200;

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 1000.0;
  else {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}

/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
  INT8U err,perr;  
  void* msg;
  INT8U* throttle; 
  *throttle = 0;
  INT8S acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
  INT8S retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
  INT16U position = 0; /* Value between 0 and 20000 (0.0 m and 2000.0 m)  */
  INT16S velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
  INT16S wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */

  printf("Vehicle task created!\n");

  while(1)
    {
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);  //send velocity value to the velocity mailbox,which 
                                                            //is received by control task                
      OSSemPend(Sem_vehicle, 0, &perr);
      
      /* Non-blocking read of mailbox:  because wait time is set to 1.
       - message in mailbox: update throttle
       - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err);    //receive throttle value from the throttle mailbox,which 
                                                    //sended by control task.

                                                   
      if (err == OS_NO_ERR) 
    throttle = (INT8U*) msg;

      /* Retardation : Factor of Terrain and Wind Resistance */
      if (velocity > 0)
    wind_factor = velocity * velocity / 10000 + 1;
      else 
    wind_factor = (-1) * velocity * velocity / 10000 + 1;

      if (position < 4000) 
        retardation = wind_factor; // even ground
      else
        if (position < 8000)
          retardation = wind_factor + 15; // traveling uphill
        else 
          if (position < 12000)
            retardation = wind_factor + 25; // traveling steep uphill
          else
            if (position < 16000)
              retardation = wind_factor; // even ground
            else
              if (position < 20000)
                retardation = wind_factor - 10; //traveling downhill
              else
                retardation = wind_factor - 5 ; // traveling steep downhill

      acceleration = *throttle / 2 - retardation;     
      position = adjust_position(position, velocity, acceleration, 200); 
      velocity = adjust_velocity(velocity, acceleration, brake_pedal, 200);
       
      Global_velocity=velocity;
      printf("Position: %dm\n", position / 10);
      printf("Velocity: %4.1fm/s\n", velocity /10.0);
      printf("Throttle: %dV\n", *throttle / 10);
      show_velocity_on_sevenseg((INT8U) (velocity / 10));
      show_position(position);    //dummy funtion: show position
    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err,perr;
  INT8U flag=1;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;
  INT16S maintain_velocity=0;

  printf("Control Task created!\n");

  while(1)
    {
      msg = OSMboxPend(Mbox_Velocity, 0, &err);    //receive velocity from mail box
      current_velocity = (INT16S*) msg;
      
      if (engine==on)
      {
        if (gas_pedal==on)
            throttle=40;
        else
            throttle=0;    
        
        if (top_gear==off)
           cruise_control=off; 
            
        if (cruise_control==off)
        {
            maintain_velocity=0;
            flag=1;
        }
        else
        
            //if (*current_velocity>=230)
            {
                throttle=40;
                if ((*current_velocity-maintain_velocity)>=5)
                    throttle=0;
                
                if ((maintain_velocity-*current_velocity)>=5)
                    throttle=80;

                if (flag==1)
                {
                    maintain_velocity=*current_velocity;
                    flag=0;
                }            
            }   
        show_target_velocity (maintain_velocity / 10);
            
        err = OSMboxPost(Mbox_Throttle, (void *) &throttle);         //send throttle to mail box
      }
      
      OSSemPend(Sem_control, 0, &perr);
    }
}

//task SwitchIO
void SwitchIOTask(void* pdata)
{ 
    INT8U perr;
    int ret;
    printf("SwitchIO task created!\n");
    while(1)
    {
      //msg = OSMboxPend(Mbox_Velocity, 0, &err);    //receive velocity from mail box
      ret=0x000f&switches_pressed();
      switch (ret)
      {
        case 0:
            top_gear=off;
            //cruise_control=off;
            printf("Gear is low!\n");
            if (Global_velocity==0)
            {
                engine=off;
                printf("engine is off!\n");
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,0);
            }
            else
            {
                engine=on;
                printf("engine is on!\n");
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0);               
            }  
            break;
                    
        case ENGINE_FLAG:       // ENGINE_FLAG         0x00000001
            engine=on;
            top_gear=off;
            //cruise_control=off;   
            printf("engine is on!\n");
            printf("Gear is low!\n");
            IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0);
            break;
            
        case ENGINE_FLAG|TOP_GEAR_FLAG:  // ENGINE_FLAG|TOP_GEAR_FLAG         0x00000003
            engine=on;
            top_gear=on;
            printf("engine is on!\n");
            printf("Gear is high!\n");
            IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1);
            break;
            
        case TOP_GEAR_FLAG:     // TOP_GEAR_FLAG         0x00000002
            top_gear=on;
            printf("Gear is high!\n");
            if (Global_velocity==0)
            {
                engine=off;
                printf("engine is off!\n");
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_1);
            }
            else
            {
                engine=on;
                printf("engine is on!\n");
                IOWR_ALTERA_AVALON_PIO_DATA (LED_RED_BASE,LED_RED_0|LED_RED_1);               
            }
            break;
         default:
            printf("no definition!\n");
            break;
      }     
      OSSemPend(Sem_switchio, 0, &perr);
    }
}

//task ButtonIO
void ButtonIOTask(void* pdata)
{ 
    INT8U perr;
    int ret;
    printf("ButtonIO task created!\n");
    
    while(1)
    {       
        ret=0x000f&buttons_pressed();
        //printf("test is %d\n", test);
        switch (ret)
        {       
           case CRUISE_CONTROL_FLAG:
                IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,LED_GREEN_2);
                if ((top_gear==on)&&(Global_velocity>=200)&&(gas_pedal==off)&&(brake_pedal==off))
                {
                   cruise_control=on;
                   printf("cruise control is on!\n");
                   IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,LED_GREEN_0);
                }
                
                break;
                
           case BRAKE_PEDAL_FLAG:
                brake_pedal=on;
                cruise_control=off;
                IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,LED_GREEN_4);
                break;
                
           case GAS_PEDAL_FLAG:
                gas_pedal=on;
                cruise_control=off;
                IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,LED_GREEN_6);
                break;   
                
           case 0x0: 
                gas_pedal=off;
                brake_pedal=off;
                if (cruise_control==on)
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,LED_GREEN_0);
                else
                    IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,0);
                break;
                
           default:    
                printf("no definition!\n");
                //IOWR_ALTERA_AVALON_PIO_DATA (LED_GREEN_BASE,0);   //will cause pause of program?why?
                break;          
        }       
                
        OSSemPend(Sem_buttonio, 0, &perr);
    }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err,perr;
  void* context;
  BOOLEAN  status; 
  
  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000;   //the number of ticks per 50ms the period of one tick of soft timer is 50ms
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
      delay,
      alarm_handler,                      //The function alarm_handler is called after delay have elapsed.


      context) < 0)
      {
          printf("No system clock available!n");
      }

  /* 
   * Create and start Software Timer 
   */
  Vehicle_softTmr = OSTmrCreate(0,  
                                VEHICLE_PERIOD/50, // this parameter must be integer! 5 soft timer ticks,
                                OS_TMR_OPT_PERIODIC,  
                                Vehicle_releaseSem, 
                                (void *)0, 
                                "Vehicle", 
                                &err); 
        if (err == OS_ERR_NONE) { 
            printf("software timer of task Vehicle generated!\n");
            status =OSTmrStart(Vehicle_softTmr, &perr);/* start the timer */
            if (perr == OS_ERR_NONE) 
            { 
                printf("software timer of task Vehicle started!\n");                          /* 

Timer was started */ 
            } 
        }

  Control_softTmr = OSTmrCreate(0,  
                                CONTROL_PERIOD/50, //5 soft timer ticks,
                                OS_TMR_OPT_PERIODIC,  
                                Control_releaseSem, 
                                (void *)0, 
                                "Control", 
                                &err); 
        if (err == OS_ERR_NONE) { 
            printf("software timer of task Control generated!\n");
            status =OSTmrStart(Control_softTmr, &perr);/* start the timer */
            if (perr == OS_ERR_NONE) 
            { 
                printf("software timer of task Control started!\n");                          /* 

Timer was started */ 
            }  
        }
        
  SwitchIO_softTmr = OSTmrCreate(0,  
                                SWITCHIO_PERIOD/50, //3 soft timer ticks,
                                OS_TMR_OPT_PERIODIC,  
                                SwitchIO_releaseSem, 
                                (void *)0, 
                                "Switch", 
                                &err); 
        if (err == OS_ERR_NONE) { 
            printf("software timer of task SwitchIO generated!\n");
            status =OSTmrStart(SwitchIO_softTmr, &perr);/* start the timer */
            if (perr == OS_ERR_NONE) 
            { 
                printf("software timer of task SwitchIO started!\n");                          /* 

Timer was started */ 
            }  
        }

  ButtonIO_softTmr = OSTmrCreate(0,  
                                BUTTONIO_PERIOD/50, //1 soft timer tick,
                                OS_TMR_OPT_PERIODIC,  
                                ButtonIO_releaseSem, 
                                (void *)0, 
                                "Button", 
                                &err); 
        if (err == OS_ERR_NONE) { 
            printf("software timer of task ButtonIO generated!\n");
            status =OSTmrStart(ButtonIO_softTmr, &perr);/* start the timer */
            if (perr == OS_ERR_NONE) 
            { 
                printf("software timer of task ButtonIO started!\n");                          /* 

Timer was started */ 
            }  
        }

  /*
   * Creation of Kernel Objects
   */
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
   
  /*
   * Create statistics task
   */
  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */
  err = OSTaskCreateExt(
       ControlTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       CONTROLTASK_PRIO,
       CONTROLTASK_PRIO,
       (void *)&ControlTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       VehicleTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       VEHICLETASK_PRIO,
       VEHICLETASK_PRIO,
       (void *)&VehicleTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
       SwitchIOTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &SwitchIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       SWITCHIOTASK_PRIO,
       SWITCHIOTASK_PRIO,
       (void *)&SwitchIOTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);  
       
  err = OSTaskCreateExt(
       ButtonIOTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       BUTTONIOTASK_PRIO,
       BUTTONIOTASK_PRIO,
       (void *)&ButtonIOTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);       
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */
  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */
int main(void) {
  printf("Lab: Cruise Control\n");

  LCD_Init();
  LCD_WriteLine1("Laboratory 1");
  LCD_WriteLine2("Cruise Control");

  Sem_vehicle=OSSemCreate(0);     //initiate semaphore
  Sem_control=OSSemCreate(0);          
  Sem_switchio=OSSemCreate(0);
  Sem_buttonio=OSSemCreate(0);
  
  
  OSTaskCreateExt(
     StartTask, // Pointer to task code
         NULL,      // Pointer to argument that is
                    // passed to task
         (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack 
         STARTTASK_PRIO,
         STARTTASK_PRIO,
         (void *)&StartTask_Stack[0],
         TASK_STACKSIZE,
         (void *) 0,  
         OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
         
  OSStart();
  
  return 0;
}
