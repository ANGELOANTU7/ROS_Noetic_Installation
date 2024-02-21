#include <Encoder.h>

#define Encoder_output_A1 53
#define Encoder_output_B1 52
#define Encoder_output_A2 49
#define Encoder_output_B2 47
#define Encoder_output_A3 15 // 44
#define Encoder_output_B3 16 // 42
#define Encoder_output_A4 39
#define Encoder_output_B4 37
#define Encoder_output_A5 35
#define Encoder_output_B5 33
#define Encoder_output_A6 31
#define Encoder_output_B6 29

// Define motor control pins
#define motor_front_right 8
#define motor_front_left 9
#define motor_middle_right 10
#define motor_middle_left 11
#define motor_back_right 12
#define motor_back_left 13

#define total_tick_1Rotation 3000
#define rpm_delay 10

// Define the speed parameters
#define noise_threshold 10
#define delta_speed 5
#define number_of_encoders 4 // this defines the number of encoders used

// Create encoder instances
Encoder back_left_Encoder(Encoder_output_A6, Encoder_output_B6);
//Encoder middle_left_Encoder(Encoder_output_A3, Encoder_output_B3);
Encoder front_left_Encoder(Encoder_output_A5, Encoder_output_B5);
Encoder front_right_Encoder(Encoder_output_A2, Encoder_output_B2);
//Encoder middle_right_Encoder(Encoder_output_A1, Encoder_output_B1);
Encoder back_right_Encoder(Encoder_output_A4, Encoder_output_B4);

Encoder WheelEncoders[number_of_encoders] = {back_left_Encoder,front_left_Encoder,front_right_Encoder,back_right_Encoder}; //Same as above but for ease of access of encoder objects


long prev_A[number_of_encoders]={0}; // stores the old values of encoders

void setup() {
  Serial.begin(57600);

  pinMode(motor_front_right,OUTPUT);
  pinMode(motor_front_left,OUTPUT);
  //pinMode(motor_middle_right,OUTPUT);
  //pinMode(motor_middle_left ,OUTPUT);
  pinMode(motor_back_right,OUTPUT);
  pinMode(motor_back_left,OUTPUT);
}

void loop() {
// Read encoder values
  unsigned long startTime = millis();
  long back_left_Encoder_Value = back_left_Encoder.read(); // index 0 
  //long middle_left_Encoder_Value = middle_left_Encoder.read();// index 1
  long front_left_Encoder_Value = front_left_Encoder.read();// index 2
  long front_right_Encoder_Value = front_right_Encoder.read();// index 3
  //long middle_right_Encoder_Value = middle_right_Encoder.read();// index 4
  long back_right_Encoder_Value = back_right_Encoder.read(); // index 5

  long A[number_of_encoders]={back_left_Encoder_Value,front_left_Encoder_Value,front_right_Encoder_Value,back_right_Encoder_Value}; //array of encoder values
  
  int direction[number_of_encoders];

  long meanSpeedForward = 0;
  long meanSpeedBackward = 0;

  for(int i = 0;i<number_of_encoders;i++){
    if(A[i] >= 0){
      meanSpeedForward = meanSpeedForward + abs(A[i]);  //for getting the mean speed of wheels rotating in same direction
      direction[i] = 1;
    }
    else{
      meanSpeedBackward = meanSpeedBackward + abs(A[i]);
      direction[i] = -1;
    }
  }

  meanSpeedForward = meanSpeedForward / 1;
  meanSpeedBackward = meanSpeedBackward / 1;


  // long meanSpeed = (abs(back_left_Encoder_Value) + abs(middle_left_Encoder_Value) + abs(front_left_Encoder_Value) + abs(front_right_Encoder_Value) + abs(middle_right_Encoder_Value) + abs(back_right_Encoder_Value)) / 6 ;

  float speedVariance[number_of_encoders];
  for(int i = 0;i<number_of_encoders;i++){
    if(A[i] >= 0){
      speedVariance[i] = variance(A[i],meanSpeedForward);
    }
    else{
      speedVariance[i] = variance(A[i],meanSpeedBackward) * -1;
    }
  }

  for(int i=0;i<number_of_encoders;i++)  // this part is for adjusting the speed of each wheels according to their variance
  {
    if(speedVariance[i] < noise_threshold)
    {
      A[i] = A[i] + delta_speed;
    }
    else
    {
      A[i] = A[i] - delta_speed;
    }
  }

  // Serial.print("back_left_Encoder: ");
  // Serial.print(back_left_Encoder_Value);
  // Serial.print("middle_left_Encoder: ");
  //Serial.print(middle_left_Encoder_Value);
  // Serial.print("front_left_Encoder: ");
  // Serial.print(front_left_Encoder_Value);
  // Serial.print("front_right_Encoder: ");
  // Serial.print(front_right_Encoder_Value);
  // Serial.print("middle_right_Encoder: ");
  //Serial.print(middle_right_Encoder_Value);
  // Serial.print("back_right_Encoder: ");
  // Serial.print(A[3]);
  // Serial.print("prev_back_right_Encoder: ");
  // Serial.print(prev_A[3]);
  long delta_tick[number_of_encoders]= {0};
  for(int i = 0;i<number_of_encoders;i++){
   delta_tick[i] = A[i] - prev_A[i];
  }
  // Serial.print("\tMean Speed forward: ");
  // Serial.print(meanSpeedForward);
  // Serial.print("\tMean Speed backward: ");
  // Serial.print(meanSpeedBackward);
  Serial.println("");

  delay(rpm_delay);
  for(int i = 0;i<number_of_encoders;i++){
    prev_A[i] = A[i];
  }
  unsigned long Timediff = millis() - startTime;
  float *ptr;
  ptr = getrpm(Timediff,delta_tick);
  for(int i = 0;i<number_of_encoders;i++){
   Serial.print(" " + String(ptr[i]));
  }
}




float variance(int speed, float meanSpeed)
{
  return abs(meanSpeed - speed);
}

int[] getrpm(unsigned long Timediff,long int delta_tick[])
{
  Serial.println(delta_tick[3]);
  float rpm[number_of_encoders]={0};
  for(int i =0;i<number_of_encoders;i++)
  {
    double num=delta_tick[i]*60000;
    double deno=total_tick_1Rotation*Timediff;
    rpm[i]=num/deno;
  }
  // for(int i = 0;i<number_of_encoders;i++){
  //  Serial.print(" " + String(rpm[i]));
  // }
  return rpm;
}









