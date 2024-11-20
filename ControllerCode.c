/*
    CSCI291- Programming for Engineers
    Final Project 2024
    Done By: Leul Tilahun- 7933290
             Mohamad Karam Badawi- 
*/
#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/light_sensor.h>
#include <stdbool.h>
#include <webots/led.h>


// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28
#define NUM_LIGHT_SENSORS 8

static int left_speed;
static int right_speed;
static int end;
static bool second_iteration=false;
static double fullTimer;
static double difference;
int counter=1;
static bool displayed=false;
static bool checkerbool=false;

void turn_left();
void turn_right();

//Function to find light intensity
double find_max_intensity(double intensities[], int count) {
    double max_intensity = intensities[0];
    for (int i = 1; i < count; i++) {
        if (intensities[i] > max_intensity) {
            max_intensity = intensities[i];
        }
    }
    return max_intensity;
}



// entry point of the controller
int main(int argc, char **argv) {
    int Highest_Intensity_Location=0;

  // initialize the Webots API
  wb_robot_init();

  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  // initialize all of the proximity sensors
  for (i = 0; i < 8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  //WbDeviceTag tag= wb_robot_get_device("light sensor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  int counter_left=0;
  int counter_right=0;
  end=0;

  
  
   // Array to store light sensor devices
    WbDeviceTag light_sensors[NUM_LIGHT_SENSORS];
    const char *light_sensor_names[NUM_LIGHT_SENSORS] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};

    // Initialize light sensors
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        light_sensors[i] = wb_robot_get_device(light_sensor_names[i]);
        wb_light_sensor_enable(light_sensors[i], TIME_STEP);
   }
  
    // Array for storing the average light intensity for every dead end reached.
    double Light_Intinsity[10];
    int counter_light = 0;
    double Highest_light_Intensity = 0.0;
  
  // feedback loop: step simulaion until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {
    puts("");
 
   //Declare list to store the readings of all 9 proximity sensors
    double ps_values[8];
    //Storing values of all proximity sensors in the list above
    for (i = 0; i < 8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      
      
      
    /*
        Initialize right and left motor speeds to go straight
    */
    left_speed=6.8;
    right_speed=6.8;
    
    
   /*
     If there is free or empty space on the left side. This means that there is an opening on
     the maze that has to be touched.
   */
   if((ps_values[6]<70.0) ) {
   /*
     turn left by 
     */
     //Every left turn set the right turn counter to 0
     counter_right=0;
     turn_left();
     counter_left+=1;
   }   
   /*
     If front side is blocked
   */
   
 else if((ps_values[7] >75.0) && (ps_values[0] >75.0)){
 /*
    Turn right 
 */
      turn_right(left_motor, right_motor);
      //Adding 1 to the right counter every time right turn is made.
     counter_right+=1;
     //Getting time to get the the difference between two consecutive right turns
     if(counter_right==1){
       fullTimer=wb_robot_get_time();
     }
     else if(counter_right==2){
       difference=wb_robot_get_time()-fullTimer;
     }
     //If the time difference between the two right turns is significant, then it is not considered a dead end
     if((counter_right==2)&&difference>3.0){         
         counter_right=0;
         
     }
     counter+=1;
  }
  /*
  Incrementing the dead end count each time there is two consecutive right turns.
  */
  if((counter_right==2)){
    end+=1;
    //reseting the right counter to 0
    counter_right=0;
    //Printing the values if in the first iteration
    if(!second_iteration){
    printf("Dead end count: %d\n",end);
    
    double total_intensity = 0.0;
    // Read values from each light sensor and calculate total intensity
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        double value = wb_light_sensor_get_value(light_sensors[i]);
        total_intensity += value;
        printf("Sensor %d: %lf\n", i, value);
        }
     // Calculate average intensity
     double average_intensity = total_intensity / NUM_LIGHT_SENSORS;
     Light_Intinsity[counter_light] = average_intensity;
    printf("Average Light Intensity: %lf\n\n", average_intensity);
     printf("******\n");
     counter_light = counter_light + 1;  
  }}
  /*
    If the iteration is the first iteration and it finishes the maze, print the dead end count and print the junction
    that has the highest light intensity.
  */
  if(!second_iteration){
  if(end==10){
      fullTimer=wb_robot_get_time();
       Highest_light_Intensity = find_max_intensity(Light_Intinsity,counter_light);
      printf("Maximum Average Light Intensity after 10 dead ends: %lf\n", Highest_light_Intensity);
      for(int i=0; i<10;i++)
        {
          if(Light_Intinsity[i] == Highest_light_Intensity){
              Highest_Intensity_Location = i+1;
              printf("Junction num: %d",Highest_Intensity_Location);
              break;
          }
        }
      printf("******\n");
      second_iteration=true;
      end=0;  
      
  }}
  /*
    If in the second iteration display message that second iteration has started and
  */
  if(second_iteration){
  if(!checkerbool){
    printf("\n**********************************************\n");
  printf("Second iteration Has Begun!\n");
  printf("\n**********************************************\n");
  checkerbool=true;
  }
  
  WbDeviceTag led[10];
  double total_intensity = 0.0;
  char *led_names[10]= {"led0","led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
  for(int i=0; i<=9; i++){
      led[i]=wb_robot_get_device(led_names[i]);
  }
  if(end==Highest_Intensity_Location){
      if(!displayed){
      
      for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        double value = wb_light_sensor_get_value(light_sensors[i]);
        total_intensity += value;
        printf("Sensor %d: %lf\n", i, value);
        }

     // Calculate average intensity
     double average_intensity = total_intensity / NUM_LIGHT_SENSORS;
     Light_Intinsity[counter_light] = average_intensity;
    
    printf("Average Light Intensity: %lf\n\n", average_intensity);
     printf("******\n");
     displayed=true;
     }
     for(int i=0; i<=9; i++){
      wb_led_set(led[0], 1);}
      left_speed=0.0;
      right_speed=0.0;
  }
  }
  
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
    
  }
  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_
}
//Function to turn robot 90 degrees right
void turn_right(int left_motor, int right_motor){
    //Setting turn speed variable at half of the maximum speed
     double turn_speed = MAX_SPEED / 2; 
    //Setting left motor speed to turn speed and and right motor speed to 0(achieving right turn)
    wb_motor_set_velocity(left_motor, turn_speed);
    wb_motor_set_velocity(right_motor, 0.0);
  //Turn duration found using trial and error. 1000ms also works
    int turn_duration = (int)(1500 / TIME_STEP); 
    /*Variable to increment every time step. This ensures the function runs and basically exits the other function
    for the required amount of time*/
    int c = 0;
    while (c < turn_duration) {
        wb_robot_step(TIME_STEP);
        c=c+1;
    }

}
//Simple function to turn left. Setting gloabl variables of left and right motor.
void turn_left(){
    left_speed=0.0;
    right_speed=6.8;
}
