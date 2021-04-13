/*Block Name    :   Display Audio Processor Interface
 *Owner/Author  :   Abdulla Al-Ansari 
 *Date          :   18/02/2021 
 *Purpose       :   Block Checkoff 
 *Environment   :   Arduino IDE``
 *
 *8bit Command codes used:
 *      A    :   Sending coordinates
 *      B    :   Begin Location command 
 *      C    :   Begin Recording
 *      D    :   Stop Recording
 *      E    :   Set Frequency Range
 */
int coordArray[2]={576,1047};
int x_coord,y_coord,freq=1025;
char incomingComd; 
bool mode=true;//True for recording
void dsply_ad_prcssr_ntrfc_g_data(int x,int y){
  Serial.println('A');//Sending coordinates command with 8bit ascii of A 
  Serial.println(x);
  Serial.println(y);
}
void dsply_ad_prcssr_ntrfc_frqncy_dtctn_data(int f){
  Serial.println(f);//Send frequency value 
  Serial.println('B');//Begin Location command with 8bit ascii of B 
}
void dsply_ad_prcssr_ntrfc_rcrdd_d_dtctn_data(){
  Serial.println('C');//Begin Recording command with 8bit ascii of C
  Serial.println('D');//Stop Recording command with 8bit ascii of D
  Serial.println('B');//Begin Location command with 8bit ascii of B       
}
void snd_lclztn_dsply_ad_prcssr_ntrfc_comm(){
  x_coord= coordArray[0];//Take first coordinate value from previous block
  y_coord= coordArray[1];//Take 2nd coordinate value from previous block
}
void g_dsply_ad_prcssr_ntrfc_data(){
  if (Serial1.available()>0) {
    incomingComd = Serial1.read();
    Serial.print("I have received command : ");
    Serial.println(incomingComd);
  }  
}
void setup() {
  Serial.begin(57600);//Initialize Serial Communication
  Serial1.begin(57600);
}
//Loop function, iterates every 100ms thus making a frequency of 10Hz
void loop() {
int f=-1;

snd_lclztn_dsply_ad_prcssr_ntrfc_comm();  

if(mode==true)
  dsply_ad_prcssr_ntrfc_rcrdd_d_dtctn_data();//Output interface iterating every 10Hz and will be a function call  
else  
  f=freq;
if(freq>=0 && freq<=2000)
  dsply_ad_prcssr_ntrfc_frqncy_dtctn_data(freq); //Output Frequency in range 20-2000Hz and begin location command, Iterating with 10Hz frequency
  
g_dsply_ad_prcssr_ntrfc_data();

dsply_ad_prcssr_ntrfc_g_data(x_coord,y_coord);//Output Interface,running with frequency of 10Hz

delay(100);//Delay of 100ms producing an iteration frequency of 10Hz 
}
