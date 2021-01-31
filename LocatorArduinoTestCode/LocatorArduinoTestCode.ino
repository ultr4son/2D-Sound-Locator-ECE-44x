int16_t coordnates_x[24];
int16_t coordnates_y[24];

int x_i = 0;
int y_i = 0;
int send_i = 0;

const byte SET_COORDNATES = 7;

void setup() {
  int a = -90;
  for(int i = 0; i < 12; i++) {
    coordnates_x[i] = a;
    coordnates_y[i] = 0;
    a += 15;
  }
  a = -90;
  for(int i = 12; i < 24; i++) {
    coordnates_x[i] = 0;
    coordnates_y[i] = a;
    a += 15;
  }
  
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void write_16(int16_t value) {
  Serial.write((value >> 8)); // Send the upper byte first
  Serial.write((value)); // Send the lower byte
}
long prev_millis = 0;
void loop() {

  while(Serial.available() > 0) {
     Serial.write(Serial.read());
  }
  long current_millis = millis();
  if(current_millis - prev_millis > 100) {
    prev_millis = current_millis;
    Serial.write(SET_COORDNATES);
   
    write_16(coordnates_x[x_i]);
    write_16(coordnates_y[y_i]);
    
    x_i = (x_i + 1) % 24;
    y_i = (y_i + 1) % 24;
  }
  
}
