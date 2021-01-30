void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

int16_t coordnates_x[] = {0, 90, 45, 30};
int16_t coordnates_y[] = {0, 45, 90, 10};

int x_i = 0;
int y_i = 0;
void loop() {

  bool did_read = false;
  while(Serial.available() > 0) {
     Serial.write(Serial.read());
     did_read = true;
  }
  if(did_read) {
    Serial.write(7);
    Serial.write(coordnates_x[x_i]);
    Serial.write(coordnates_y[y_i]);

    x_i = (x_i + 1) % 4;
    y_i = (y_i + 1) % 4;
  }

  
}
