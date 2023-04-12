/* ScanyBot is a simple 3d scanning robot
Its main job to 3d scan a mapping the area using a set of points
thanks to webgl to view the points through any browser without the need for an external tools or plugins

By Abdulsalam Abbod
With a help from bitluni using his code of rendering webgl
2023/4/12 9:44pm

my github page :- https://github.com/AbdulsalamAbbod
bitluni youtube channel :- https://youtube.com/@bitlunislab


*/

#include <MPU6050_light.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <FS.h>

#define m1a D3
#define m1b D4
#define m2a D6
#define m2b D7

// a fancy way to include the 3d rendering webpage 
char *renderer = 
#include "Renderer.h"

MPU6050 mpu(Wire);// SCL => D1, SDA => D2 : On D1 Mini Board

void forward();
void right();
void left();
void back();
void stop();
// start a webpage on port 80 
ESP8266WebServer server(80);
 
// speed coulde effect the accurecy of the robot, make sure to set it well
int power = 700;// 1024-512-0 :600
int pos_power = 700; // 800

float resolution=1;
int length = 100; // line length at cm 
int width = 100; // at cm 
int sensitivity;
int power_html;
// values inside the code
 int stepvalue =  59 *2;// number of steps at each line, it should multiplied by 2 because the encoders sends two pulses at one time
 int line_range = 10; // number of lines
 int pitch_threshold = 10;// pitch center_threshold 10 good, less is more

 int path_range = 8;// 15, for robot line step
const volatile float  step_size= 1.000;//step size 
//float max_range = 0.300;// number of Lines


int points =0;
volatile int intstep=0;
volatile int intstep_mode=0;
volatile int intline =0;
 
 
const int step_sensor = D5;//interrupt sensor value from UNO
bool transition=false;
bool debug2=false;
bool debug = debug;
bool Done = false;
volatile bool shift = true;
volatile bool mode_now=false;
bool isscan=false;
volatile bool hold=false;
unsigned long current = 0;
unsigned long current2 = 0;
unsigned long current3 = 0;
unsigned long prev = 0;
unsigned long prev2 = 0;
unsigned long prev3 =0;
volatile unsigned long cycle =0;
volatile unsigned long cycle_ps =0;

int current_position = -9;                             
volatile int mode = 1;
// path correction error
const float center_threshold = 2.500;// 2.5 is good
const float angle_90 = 2.500; // 3 is good
const float angle_180 = 2.500;

int move=0;// for Calibration direction

float step =0.000;
float pitch=0.000;
volatile float line =0.000;
volatile float Z=0.000, Y=0.000;
float x=0.000, y=0.000, z=0.000;
/*x = step  pitch = y  line = z */

 
/*********************************************************/


// the interrupt function which called only when the encoder is triggered and the robot is moving 
void ICACHE_RAM_ATTR interruptt(){         
       hold=true;// to scan points only when the robot is moving
    
    if(intstep_mode>=path_range && (mode==2 || mode ==4) && current_position==0){     
        switch(mode){
          case 2: mode = 3;  transition = false;  break;
          case 4: mode = 1;  transition = false;  break;
          default : break;
        } 
    
        intstep_mode=0;
    }
     else if(transition==true) { ++intstep_mode;}
 
      //   3         3
    if(intstep > stepvalue) {// hey i added = here!
        intstep=0;// intstep is 0
        intstep_mode=0;
        shift = !shift;
        ++intline;
        line = line + step_size; 
        transition = true;
        ++mode;// mode should = 2 here at first run.
     //  Serial.print("mde ");
     //  Serial.println(mode);
       
      }     
    else if(transition==false) { ++intstep; } // and done bool that tell repositioning is done and ready to scan a new line         
      
    
    ++cycle;

}


void scan() { 
// open a file at the memory of the esp8266 and record the data
  File f = SPIFFS.open("/distance.js", "w");
  if (!f) {  Serial.println("file open failed");   return; }
  f.print("var vertices = new Float32Array([");
    
  
   while(true){         
     
     current = millis();   
     current2 = millis();     
     current3 = millis();   
   // do the scanning algorithm
      pattern(); 
        
            
    mpu.update();  
    // get the y and z values at dgrees    
    Y = mpu.getAngleY();
    Z = mpu.getAngleZ();   

            
    if(current - prev > 500 || debug){  
     Serial.print("Y : ");
     Serial.print(Y);
     Serial.print(", Z : ");
     Serial.println(Z);
     Serial.print("Points : ");
     Serial.println(cycle/2);     
     Serial.print("intstep : ");
     Serial.println(intstep/2); 
     Serial.print("direc : ");
     Serial.println(current_position);    
     Serial.print("mode : ");
     Serial.println(mode);   
     Serial.print("intstep_mode : ");
     Serial.println(intstep_mode/2);   
             
            prev = current; 

    }  
      
// scan each point at one line
 if((round(intstep*resolution)) <= stepvalue )&& hold && transition==false)  {
         hold=false;
    // reverse the sscanning
       if(shift)  step = step + step_size;      
         else    step = step - step_size;            
     // set the sensitivity of the robot to obsticales    
    pitch = (pitch+Y)/pitch_threshold;//Y value is the amplitude                             
         // logging data             
    String vertex = String(step, 3) + ", " +// every step is 0.010 value 
                    String(-line, 3) + ", " +// line is the same size as step which 0.010
                    String(-pitch, 3);
    
         f.print(vertex + ", "); 
            
      if(current2 - prev2 > 500 || debug){
        Serial.println(vertex);      
        Serial.print("Pitch : ");
        Serial.println(pitch);
        Serial.print("direc : ");
        Serial.println(current_position);
 
       Serial.print("Lines : ");
       Serial.println(line);
      prev2 = current2;
             }
  // when reach the last line in scanning stop  
   if(intline >= line_range){
        stop();
        detachInterrupt(digitalPinToInterrupt(step_sensor));
        Serial.println("");
        Serial.println("Done Scanning!");
        Serial.print("Number of points : ");
        Serial.println(cycle/2);
        Serial.print("Size is : ");
        Serial.print(stepvalue/2);
        Serial.print(" Step BY ");
        Serial.print(line_range);
        Serial.println(" line");
        intline=0;
        Done=true;
        break;// exit to start the webpage and render tue results
       }                  
  }
        
        delay(20);
 }
    
  //closing arrays and files
  f.print("]);");
  f.close();
}


// the algorithm that the robot follows to scan the area 
void pattern(){

    current_position = mods();
      

    switch(current_position){
        
    case 0: 
         forward();
        if(current3 - prev3 > 500 || debug2)  {
         debug?Serial.println("forward"):0; 
            prev3 = current3;
            } 
        break;
    case 1: left(); 
        if(current3 - prev3 > 500 || debug2)  {        
        debug?Serial.println("left"):0; 
            prev3 = current3;
            }   
        break;
    case -1:  right();
        if(current3 - prev3 > 500|| debug2)  {
         debug?Serial.println("right"):0; 
          prev3 = current3;
              }  
        break;
    case -9:  stop();
        if(current3 - prev3 > 500 || debug2)  {
         debug?Serial.println("stop"):0;    
          prev3 = current3;
              }
        break;
        default : break;
      }
    
}
    


       
// we have 4 modes, each one has a correction statement to correct its path. 
int mods(){
    if(mode == 1){ 
     if(Z>=-center_threshold && Z<=center_threshold){/* center*/// transition = false;
           return 0;           
        }
        else if(Z<=-center_threshold){// to the right
          
           return 1;
        }  
        else if(Z>=center_threshold){// to the left
           return -1;
        } 
     }    /////////////////// 
    
    
    else if(mode == 2){ 
         // mode 2      -100 < -90 < -80
        //      100              80
         if(Z<=90+angle_90 && Z>=90-angle_90){// to -90 dgree 
           return 0;
        } //      Z 80
        else if(Z<=90-angle_90){// 90 to right 
           return 1;
        } 
        else if(Z>=90+angle_90){// 90 left
           return -1;
        } 
      }   ///////////////////
    
    else if(mode == 3){
         // mode 3     -190 < -180 < -170
         if(Z<=180+angle_180 && Z>=180-angle_180){// to -180 dgree
           return 0;
        } 
        else if(Z<=180-angle_180){// -180 to right
           return 1;
        } 
        else if(Z>=180+angle_180){// -180 to left
          return -1;
        } 
     }    /////////////////////
    
    else if(mode == 4){
        // mode 4      -100 < -90 < -80
         if(Z<=90+angle_90 && Z>=90-angle_90){// to -90 dgree 
           return 0;
        } 
        else if(Z<=90-angle_90){// 90 to right 
           return 1;
        } 
        else if(Z>=90+angle_90){// 90 left
           return -1;
        } 
     }    /////////////////////

    
 }



//serve main page
void returnRenderer(){
  server.send (200, "text/html", renderer);
}

//serve stored vertices
void returnVertices(){
  File file = SPIFFS.open("/distance.js", "r");
  server.streamFile(file, "script/js");
  file.close();
}  



void setup() {

 
  Serial.begin(9600);
  Wire.begin();
 // define pins state
    pinMode(m1a, OUTPUT);
    pinMode(m1b, OUTPUT);
    pinMode(m2a, OUTPUT);
    pinMode(m2b, OUTPUT);
    pinMode(step_sensor, INPUT);
    
   // attach an interrupt to read the encoder sensor state and do something with it.
 attachInterrupt(digitalPinToInterrupt(step_sensor), interruptt, RISING); // Attach the interrupt handler function to the interrupt pin

   Serial.print(F("MPU6050 status: "));
   Serial.println(status);
   
  SPIFFS.begin();

// my webpage
  Serial.println(WiFi.softAP("ScannyBot") ? "Ready" : "Failed!");
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/submit", handleSubmit);
  server.begin();
  Serial.println("Server started");
    
// serve the webpage until the user fill all the data and press go
 while(isscan==false) { server.handleClient();}
 
   byte status = mpu.begin();
   
  while(status != 0) {
      Serial.println("The MPU Not connect!"); delay(1000); } // stop everything if could not connect to MPU6050
 
 Serial.println(F("Calibration, do not move MPU6050"));
 
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");

 //turn off wifi to get precise measurements
   WiFi.mode(WIFI_OFF);
   
  //perform a scan
    scan();
   
// start the wifi and serve the page to view the results 
  Serial.println(WiFi.softAP("ScannyBot") ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

//let server serve the pages
  server.on("/view", returnRenderer);
  server.on("/vertices.js", returnVertices);
  server.begin();  
}


void loop() 
{
  //handle the server
  server.handleClient();
}

// webpage interface
void handleRoot() {
  String html = "<html>\
    <head>\
      <style>\
        body {\
          font-family: Arial, sans-serif;\
          background-color: #f2f2f2;\
        }\
        h1 {\
          text-align: center;\
          margin-top: 50px;\
        }\
        form {\
          max-width: 500px;\
          margin: 0 auto;\
          background-color: #ffffff;\
          padding: 20px;\
          border-radius: 10px;\
          box-shadow: 0px 0px 10px #999999;\
        }\
        input[type=text] {\
          width: 100%;\
          padding: 12px 20px;\
          margin: 8px 0;\
          box-sizing: border-box;\
          border: 2px solid #ccc;\
          border-radius: 4px;\
        }\
        input[type=submit], input[type=button] {\
          background-color: #4CAF50;\
          color: white;\
          padding: 12px 20px;\
          border: none;\
          border-radius: 4px;\
          cursor: pointer;\
        }\
        input[type=submit]:hover, input[type=button]:hover {\
          background-color: #45a049;\
        }\
        .results {\
          text-align: center;\
          margin-top: 50px;\
          font-size: 24px;\
        }\
      </style>\
    </head>\
    <body>\
      <h1>ScannyBot</h1>\
      <form action=\"/submit\" method=\"get\">\
        <label for=\"input1\">Length:</label>\
        <input type=\"text\" id=\"input1\" name=\"input1\" placeholder=\"Value in cm\">\
        <label for=\"input2\">Lines:</label>\
        <input type=\"text\" id=\"input2\" name=\"input2\" placeholder=\"Number of lines\">\
        <label for=\"input3\">Sensitivity:</label>\
        <input type=\"text\" id=\"input3\" name=\"input3\" placeholder=\"From 0 to 20\">\
        <label for=\"input4\">Speed:</label>\
        <input type=\"text\" id=\"input4\" name=\"input4\" placeholder=\"From 0 to 100\">\
        <label for=\"input5\">Resolution:</label>\
        <input type=\"text\" id=\"input5\" name=\"input5\" placeholder=\"From 0 to 10\">\
        <br>\
        <input type=\"submit\" value=\"Go\">\
        <input type=\"button\" value=\"View Results\" onclick=\"window.location.href='/view'\">\
      </form>\
      <div class=\"results\"></div>\
    </body>\
  </html>";

  server.send(200, "text/html", html);
}


// handle the inputs that got from the user
void handleSubmit() {
    // convert the value that came from the webpage at cm to points
   stepvalue = round(server.arg("input1").toInt())*1.48;
  
   line_range = server.arg("input2").toInt();
   
   pitch_threshold = server.arg("input3").toInt();
   // convert the percentage value to value from 0 to 1023
   power = round((server.arg("input4").toInt())/0.097);
   resolution = server.arg("input5").toInt();
   pos_power=power;
   // scanning done flag
   isscan=true;
   // go to results page after done scanning
  server.sendHeader("Location", "/view");
  server.send(302);

}


// directions functions, it may change depending on the way you connect the wires!
void right(){
    analogWrite(m1a, pos_power);// 
    digitalWrite(m1b, LOW);
     
    analogWrite(m2a, pos_power);
    digitalWrite(m2b, LOW);
}


void forward(){
    digitalWrite(m1a, LOW);// 
    analogWrite(m1b, power);
    
    analogWrite(m2a, power);
    digitalWrite(m2b, LOW);
}


void left(){
    digitalWrite(m1a, LOW);// 
    analogWrite(m1b, pos_power);
    
    digitalWrite(m2a, LOW);
    analogWrite(m2b, pos_power);
}

void stop(){
    digitalWrite(m1a, 0);
    digitalWrite(m1b, 0);
    
    digitalWrite(m2a, 0);
    digitalWrite(m2b, 0);
}

