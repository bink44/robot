#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

int total_legs=4; //number of legs

uint8_t IDs[]={1,2,3,4}; // Leg ID corresponding to [RF,LF,LR,RR] legs.  if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; 
uint8_t Directions[]={1,0,0,1}; // Leg rotating direction corresponding to [RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; If not changed, leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; 
float gait[]={0,0,0};
//float gait[]={0.5,-0.5,0}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LR-RF,RR-RF]; 
float gait_deg[]={0,0,0,0};  //gait in deg; [RF,LF,LR,RR]; the value for RF will always be 0, the value for LF= 360*\phi_1, LR=360*\phi_2, RR=360*\phi_3; calculated in translate_gait_deg();

int Leg_zeroing_offset[]={120,60,60,120}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

//clock parameters belows need to be calculated in clock_init()///////////////////////////////////////////////
float time_slow_start=1; // in seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end=3; // in seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start=150; //in deg, the slow phase starting position
float degree_slow_end=210; //in deg, the slow phase starting position, if degree_slow_start=150 and degree_slow_end=210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//You can change the desired Buehler clock parameters here
float phi_s=1.1; //in rad, ϕ_s is the angular extent of the slow phase
float phi_0=2.7; //in rad, ϕ_0 is the center of the slow phase 
float d_c=0.45;//d_c is the duty factor of the slow phase (i.e. fraction of the period spent in the slow phase). 
float clock_period=4; //in seconds, time to complete 1 rotation

// Global variables for speed_slow and speed_fast
float speed_slow;
float speed_fast;

//change code to return desired speed in slow phase
float omega_slow(){ 
  float speed_slow = (180.0/3.14)*phi_s/(clock_period*d_c);  
  
  return speed_slow;  //return desired leg speed in slow phase in deg/s
}


//change code to return desired speed in fast phase
float omega_fast(){  
  float speed_fast = (180.0/3.14)*(6.28-phi_s)/(clock_period*(1-d_c));  
  
  return speed_fast; //return desired leg speed in fast phase in deg/s
}

//configure your timing parameters
void clock_init(){   
  //Insert your calculated time_slow_start and time_slow_end here. You do not have to use degree_slow_start and degree_slow_end, but they may be helpful.
  // at the beginning of each stride period, the desired angle, \phi, should be 0 degree (leg should point vertically upward if you have a leg installed). 
  // we suggest that you make sure that the deadzone (300deg to 360deg) is fully within the fast phase (i.e., 0<degree_slow_start<degree_slow_end<300). Also make sure 0<time_slow_start<time_slow_end<clock_period
  // notice degree_slow_start, degree_slow_end here are in deg  
  
  degree_slow_start= (180.0/3.14) * (phi_0 - (phi_s / 2)); //FIXME(optional)  //return the position that the the leg enters the slow phase in deg
  degree_slow_end= degree_slow_start + (180.0 / 3.14) * phi_s; //FIXME(optional)  //return the position that the the leg exits the slow phase in deg

  time_slow_start= degree_slow_start/omega_fast();  //FIXME  //return the time after the start of the period that the the leg enters the slow phase
  time_slow_end= time_slow_start + clock_period*d_c;  //FIXME  //return the time after the start of the period that the the leg exits the slow phase
 
  return;
}

// compute desired motor angle at any time instances
float get_desired_angle(int leg,                    // Robot's leg enum, in 0,1,2,3
                   long elapsed               // time elapsed since start, in ms
                   ) { // return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    
    
      float angle;
      float elapsed_seconds = elapsed / 1000.0;  // Convert elapsed time to seconds
      if (leg>0) {
        elapsed_seconds+=gait[leg-1]*4;
      }
      float time_cycle = fmod(elapsed_seconds,4);

      if (time_cycle < time_slow_start) {
        // Phase 1: y(angle) = speed_fast * x(time)
        angle = omega_fast() * time_cycle;
      } else if (time_cycle < time_slow_end) {
        // Phase 2: y(angle) = speed_slow * x(time) + degree_slow_start
        angle = omega_slow() * (time_cycle - time_slow_start) + degree_slow_start;
      } else {
        // Phase 3: y(angle) = speed_fast * x(time) + degree_slow_end
        angle = omega_fast() * (time_cycle - time_slow_end) + degree_slow_end;
      }

    angle = angle+Leg_zeroing_offset[leg];
    float angle_mod = fmod(angle,360.0);
    return angle_mod; //in deg
}

// print desired motor position and associated elapsed time to debug serial
void print_position(float t, int leg, float desired_pos){
      Serial.print(" Leg: ");
      Serial.print(leg);
      Serial.print("(");
      Serial.print(t/1000.0);

      Serial.print(", ");
      Serial.print(desired_pos); 
      Serial.print(")");     
  return 0;
}


void translate_gait_deg(){ //calculate the value of the array 'gait_deg[]' 
  //notice the unit of 'gait_deg[]' is in deg and has 4 elements
  gait_deg[0]=0;  //RF
  gait_deg[1]=gait[0]*360; //LF
  gait_deg[2]=gait[1]*360; //LB
  gait_deg[3]=gait[2]*360; //RB
  
   return;
}

int dead_zone_speed_tuning=-10; //adjustment to tune deadzone speed, MAGIC Variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
//int different_direction_offset=-110; //adjustment to compensate position offset between 2 legs with different rotating direction. Another MAGIC variable that requires manual observation. 


////////////////////////////////////////////// The code below should require none or minimal changes /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


long start;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<=0;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();
  translate_gait_deg(); 
}


long last_time=0;
int time_step=100;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in


  //Starting Position
    int target = 100;
  //Ending Position
    int end = 200;

void tail(){

  int pitch = 140;
  dxl.setGoalPosition(5,pitch, UNIT_DEGREE);

  if(target != end){
  DEBUG_SERIAL.println(target);
  dxl.setGoalPosition(6,target, UNIT_DEGREE);
  delay(10);
  target = target + 1;
  }
  

}


void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  long elapsed = millis() - start;
  DEBUG_SERIAL.println(elapsed);
  
//Lift Tail Up
  dxl.setGoalPosition(5,30, UNIT_DEGREE);

//Start Tail Movement
  if(elapsed > 6000){
    DEBUG_SERIAL.println("TAIL");
    dxl.setGoalPosition(6,target, UNIT_DEGREE);
    tail();
  }
  
//END ALL
  if(target == end)
    while(1);


  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
        
      print_position(elapsed, i,desired_pos);
      Serial.print("Result Desired Position: ");
      Serial.println(desired_pos); 
      if (in_dead_zone[i]==0){

        
        if (desired_pos<300)
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          if(Directions[i]==0)
            dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
          else
            dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
          
        }
        delay(10);   
      }
      else{
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool flag_temp=0;
        
        
        if(Directions[i]==0)
            flag_temp=current_pos>20;
         else
            flag_temp=current_pos<280;
          
        
        if (flag_temp && desired_pos>0 && desired_pos<300){
          in_dead_zone[i]=0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
        
      }
    }
  }
  
}
