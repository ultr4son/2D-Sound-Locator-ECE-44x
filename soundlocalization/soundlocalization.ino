/*Block Name    :   Sound Localization
 *Owner/Author  :   Abdulla Al-Ansari 
 *Date          :   29/01/2021 
 *Purpose       :   Block Checkoff 
 *Environment   :   Arduino IDE
 */

int dataArray[4]={1,2,300,500};//Data from Tristan's block will be stored in this array every 100ms approx
int sensor1,sensor2,sensor3,sensor4;
//Function definition for to take new data from previous block
void getData(){
  sensor1=dataArray[0];//Store data in new variables
  sensor2=dataArray[1];
  sensor3=dataArray[2];
  sensor4=dataArray[3];
}
//Function for usb serial protocol communication
void initializeUSBSerialComm(void){
  //nothing here right now
  // part of code would be in anothor block
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);//Initialize Serial Communication
}
//Loop function, iterates every 100ms thus making a frequency of 10Hz
void loop() {
  getData();//Get data stored by the code of previous block
  Serial.print("Iteration time stamp in ms is : ");
  Serial.println(millis());//print the current time stamp in milliseconds on serial monitor
  if(sensor1>50 || sensor2>50 || sensor3>50 || sensor4>50){//Check if the sound is heard by any of sensors and its not noise. Value less than 50 value is considered noise
      if(sensor1>sensor2 && sensor1>sensor3 && sensor1>sensor4){
        Serial.print("Sound near sensor1 felt  ");
        Serial.print(sensor1/20);
        Serial.println("  ft away from sensor 1");
        if(sensor2>50 && sensor2>sensor3 && sensor2>sensor4){
            Serial.print("Sound near sensor2 felt  ");
            Serial.print(sensor2/20);
            Serial.println("  ft away from sensor 2");            
        }
        else
        if(sensor3>50 && sensor3>sensor2 && sensor3>sensor4){
            Serial.print("Sound near sensor3 felt  ");
            Serial.print(sensor3/20);
            Serial.println("  ft away from sensor 3");            
        }    
        else
        if(sensor4>50 && sensor4>sensor2 && sensor4>sensor3){
            Serial.print("Sound near sensor4 felt  ");
            Serial.print(sensor4/20);
            Serial.println("  ft away from sensor 4");            
        }            
      }
      else
      if(sensor2>sensor1 && sensor2>sensor3 && sensor2>sensor4){
        Serial.print("Sound near sensor2 felt  ");
        Serial.print(sensor2/20);
        Serial.println("  ft away from sensor 2");
        if(sensor1>50 && sensor1>sensor3 && sensor1>sensor4){
            Serial.print("Sound near sensor1 felt  ");
            Serial.print(sensor1/20);
            Serial.println("  ft away from sensor 1");            
        }
        else
        if(sensor3>50 && sensor3>sensor1 && sensor3>sensor4){
            Serial.print("Sound near sensor3 felt  ");
            Serial.print(sensor3/20);
            Serial.println("  ft away from sensor 3");            
        }    
        else
        if(sensor4>50 && sensor4>sensor1 && sensor4>sensor3){
            Serial.print("Sound near sensor4 felt  ");
            Serial.print(sensor4/20);
            Serial.println("  ft away from sensor 4");            
        }                  
      }
      else
      if(sensor3>sensor1 && sensor3>sensor2 && sensor3>sensor4){
        Serial.print("Sound near sensor3 felt  ");
        Serial.print(sensor3/20);
        Serial.println("  ft away from sensor 3");  
        if(sensor1>50 && sensor1>sensor3 && sensor1>sensor4){
            Serial.print("Sound near sensor1 felt  ");
            Serial.print(sensor1/20);
            Serial.println("  ft away from sensor 1");            
        }
        else
        if(sensor2>50 && sensor2>sensor1 && sensor2>sensor4){
            Serial.print("Sound near sensor2 felt  ");
            Serial.print(sensor2/20);
            Serial.println("  ft away from sensor 2");            
        }    
        else
        if(sensor4>50 && sensor4>sensor1 && sensor4>sensor3){
            Serial.print("Sound near sensor4 felt  ");
            Serial.print(sensor4/20);
            Serial.println("  ft away from sensor 4");                      
        }            
      }  
      else
      if(sensor4>sensor1 && sensor4>sensor2 && sensor4>sensor3){
        Serial.print("Sound near sensor4 felt  ");
        Serial.print(sensor4/20);
        Serial.println("  ft away from sensor 4");    
        if(sensor1>50 && sensor1>sensor3 && sensor1>sensor4){
            Serial.print("Sound near sensor1 felt  ");
            Serial.print(sensor1/20);
            Serial.println("  ft away from sensor 1");            
        }
        else
        if(sensor2>50 && sensor2>sensor1 && sensor2>sensor4){
            Serial.print("Sound near sensor2 felt  ");
            Serial.print(sensor2/20);
            Serial.println("  ft away from sensor 2");            
        }    
        else
        if(sensor3>50 && sensor3>sensor1 && sensor3>sensor3){
            Serial.print("Sound near sensor3 felt  ");
            Serial.print(sensor3/20);
            Serial.println("  ft away from sensor 3");                      
        }              
      }  
  }
  else{
    Serial.println("No sound felt other than noise");
  }
  Serial.println("\n");
  initializeUSBSerialComm();//Initiate usb serial comm sequence
  delay(86);//Delay of 100ms means frequency of 10Hz but due delay done due to processing instructions is considered aswell
 
}
