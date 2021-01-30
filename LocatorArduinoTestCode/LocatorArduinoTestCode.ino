int16_t coordnates_x[] = {180};
int16_t coordnates_y[] = {180};

int x_i = 0;
int y_i = 0;
int send_i = 0;

const byte SET_COORDNATES = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void write_16(int16_t value) {
  Serial.write((value >> 8)); // Send the upper byte first
  Serial.write((value)); // Send the lower byte
}

void loop() {

  while(Serial.available() > 0) {
     Serial.write(Serial.read());
  }
  send_i++;
  if(send_i == 0) {

    Serial.write(SET_COORDNATES);
   
    write_16(coordnates_x[x_i]);
    write_16(coordnates_y[y_i]);
    
    x_i = (x_i + 1) % 4;
    y_i = (y_i + 1) % 4;
  }
  
}
