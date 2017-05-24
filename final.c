#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/sem.h>

#include  <rtdk.h>

#define pin_pwm_motor_a 0
#define pin_pwm_motor_b 1
#define in1_motor_a 2
#define in2_motor_a 3
#define in3_motor_b 4
#define in4_motor_b 5

RT_TASK serial_communication_task;
RT_TASK converter_task;
RT_TASK pwm_motor_a_task;
RT_TASK pwm_motor_b_task;
RT_TASK dir_motor_a_task;
RT_TASK dir_motor_b_task;

RT_SEM semPosition;
RT_SEM semMotorA;
RT_SEM semMotorB;

RTIME one_second = 1000000000llu;

//Global variables
char position[64];
int dirA,dirB; //0->forward, 1->backward
float potA,potB;

//obtiene los datos a travez de comunicacion serial
void serial_communication_function(void *arg){
  // Find Serial device on Raspberry with ~ls /dev/tty*
  // ARDUINO_UNO "/dev/ttyACM0"
  // FTDI_PROGRAMMER "/dev/ttyUSB0"
  // HARDWARE_UART "/dev/ttyAMA0"
  char device[]= "/dev/ttyUSB0";
  // filedescriptor
  int fd,err,i;
  unsigned long baud = 9600;
  unsigned long time=0;

  //get filedescriptor
  if ((fd = serialOpen (device, baud)) < 0){
    rt_fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    exit(1);//error
  }

  //setting the task periodic 0.2s
  err =  rt_task_set_periodic(NULL,TM_NOW,0.2*one_second);
  if (err) {
    rt_printf("error on set periodic, %s\n", strerror(-err));
    exit(1);//error
  }

  int length,index;
  char msg[64],newChar;

  //loop of the periodic task
  while(1){
    index=0;
    // read signal
    while(serialDataAvail (fd)>0){
      if(index<63){
        newChar = serialGetchar (fd);
        msg[index] = newChar;
        index++;
        msg[index]='\0'; //Null terminate character
      }
    }
    /* Now, wait for a semaphore unit... */
    rt_sem_p(&semPosition,TM_INFINITE);

    for(i=0;i<64;i++){
      position[i] = msg[i];
    }
    //to watch position in console
    rt_printf("%s\n",position);
    // then release it. //
    rt_sem_v(&semPosition);

    //waiting for period time
    rt_task_wait_period(NULL);
  }
}

//obtiene la potencia para cada motor y su direccion
void converter_function(void *arg){
  char positionTemp[64];
  int err,i;
  float x,y; //x -> girar, y -> avanzar
  //x negativo -> a la derecha, y negativo -> hacia delante


  err =  rt_task_set_periodic(NULL,TM_NOW,0.2*one_second);
  if (err) {
    rt_printf("error on set periodic, %s\n", strerror(-err));
    exit(1);//error
  }

  while(1){

    /* Now, wait for a semaphore unit... */
    rt_sem_p(&semPosition,TM_INFINITE);

    for(i=0;i<64;i++){
      positionTemp[i] = position[i];
    }

    /* then release it. */
    rt_sem_v(&semPosition);

    //converting position string to float
    sscanf (positionTemp,"A%f %f*",&x,&y);

    if (y<0){
      dirA = 0;
      dirB = 0;
      y = -1.0*y;
    }else{
      dirA = 1;
      dirB = 1;
    }

    //potencia directamente proporcional a la inclinación de y
    if (y>45){
      potA = 1.0;
      potB = 1.0;
    }else if(y>2){
      potA = y/45.0;
      potB = y/45.0;
    }else{
      potA = 0;
      potB = 0;
    }

    //aplicando potencia diferencial lineal por inclinacion de x
    if (x<0){
      x = -1.0*x;
      if(x>45){
        potB = 0;
      }else{
        potB = potB*(-1.0*x/45.0+1.0);
      }
    }else{
      if(x>45){
        potA = 0;
      }else{
        potA = potA*(-1.0*x/45.0+1.0);
      }
    }

    dirA=0;
    dirB=0;
    potA = 0.5;
    potB = 0.5;
    
    //waiting for period time
    rt_task_wait_period(NULL);
  }
}

//aplica el sentido de rotacion a los motores
void dir_motor_function(void *arg){
  int motor = *(int*)arg;
  int dir,in_1,in_2;
  if (motor == 1){//motorA
    dir = dirA;
    in_1 = in1_motor_a;
    in_2 = in2_motor_a;
  }else{//motorB
    dir = dirB;
    in_1 = in3_motor_b;
    in_2 = in4_motor_b;
  }
  if (dir==0){//forward
    digitalWrite(in_1,HIGH);
    digitalWrite(in_2,LOW);
  }else{//backward
    digitalWrite(in_1,LOW);
    digitalWrite(in_2,HIGH);
  }
}

//aplica PWM a la salida que controla la potencia del motor
void pwm_motor_function(void *arg) {
  int motor = *(int*)arg, pin,err;
  float pot;

  //setting the task periodic
  err =  rt_task_set_periodic(NULL,TM_NOW,0.2*one_second);
  if (err) {
    rt_printf("error on set periodic, %s\n", strerror(-err));
    exit(1);//error
  }

  // Toggling the pins
  while(1){
    
    if (motor==1){
      pot = potA;
      pin = pin_pwm_motor_a;
    }else{
      pot = potB;
      pin = pin_pwm_motor_b;
    }
    //set_data_out has offset 0x94
    //set gpio pin to 1 (up)
    digitalWrite(pin,HIGH);
    // wait requested pulse width time (duty)
    rt_task_sleep(pot*0.2*one_second);

    //clear_data_out has offset 0x90
    //set gpio pin to 0 (down)
    digitalWrite(pin,LOW);

    // wait until the next pulse should start (20mS interval)
    rt_task_wait_period(NULL);
  }
}

//startup code
void startup(){
  int a=1,b=2;

  //setup GPIO in wiringPi mode
  if (wiringPiSetup () == -1){
    rt_fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    exit(1);
    }//error
  pinMode (pin_pwm_motor_a, OUTPUT);
  pinMode (in1_motor_a, OUTPUT);
  pinMode (in2_motor_a, OUTPUT);
  pinMode (pin_pwm_motor_b, OUTPUT);
  pinMode (in3_motor_b, OUTPUT);
  pinMode (in4_motor_b, OUTPUT);

  rt_task_create(&serial_communication_task, "comunication", 0, 50, 0);
  rt_task_create(&converter_task, "converter", 0, 70, 0);
  rt_task_create(&pwm_motor_a_task, "pwm_motorA", 0, 70, 0);
  rt_task_create(&dir_motor_a_task, "direction_motorA", 0, 70, 0);
  rt_task_create(&pwm_motor_b_task, "pwm_motorB", 0, 70, 0);
  rt_task_create(&dir_motor_b_task, "direction_motorB", 0, 70, 0);
  rt_task_start(&serial_communication_task, &serial_communication_function, NULL);
  rt_task_start(&converter_task, &converter_function, NULL);
  rt_task_start(&dir_motor_a_task, &dir_motor_function, &a);
  rt_task_start(&dir_motor_b_task, &dir_motor_function, &b);
  rt_task_start(&pwm_motor_a_task, &pwm_motor_function, &a);
  rt_task_start(&pwm_motor_b_task, &pwm_motor_function, &b);
}

void init_xenomai() {
 	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	rt_print_auto_init(1);
}

int main(int argc, char* argv[]){

  printf("\nType CTRL-C to end this program\n\n" );

  // code to set things to run xenomai
  init_xenomai();

  //startup code
  startup();

  pause();
}

