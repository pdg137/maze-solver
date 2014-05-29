int8_t count1 = 0;
int8_t count2 = 0;
int8_t last_count1 = 1;
int8_t last_count2 = 1;
uint8_t last11, last12, last21, last22;
uint32_t error1 = 0;
uint32_t error2 = 0;

uint16_t started_following_time = 0;
uint8_t started_following = 0;

#define ANGLE_SCALE 20000
#define STEPS_PER_RADIAN 410
#define ENCODER_CALIBRATION1 160
#define RADIUS 10000000L
#define FOLLOW_MAX_S 16000L
#define MAZE_UNIT_DISTANCE 22000000L
#define FOLLOW_MAX_Y (MAZE_UNIT_DISTANCE/3)
#define STOPPING_DISTANCE    800000L

#define MAX_PATH 255

#define FORWARD 0
#define LEFT 1
#define BACKWARD 2
#define RIGHT 3
#define DIR_MAX 3

#define BUTTON_PIN 7
#define LED_PIN 13

#define DARK_VALUE 900

//uint8_t path[MAX_PATH] = { FORWARD, FORWARD, FORWARD, FORWARD, RIGHT };

uint8_t path[MAX_PATH] = {
FORWARD, FORWARD, FORWARD, FORWARD, FORWARD,
LEFT, LEFT, RIGHT, RIGHT, LEFT,
LEFT, FORWARD, RIGHT, LEFT, FORWARD,
RIGHT, LEFT, RIGHT, LEFT, LEFT,
FORWARD, FORWARD };

uint8_t path_size = 22;
uint8_t path_pos = 0;
uint8_t steps_since_last_calibrated = 99;
  
#define SPEED 120
#define STOPPING_TIME_MS 100

#define SPIN_SPEED 50
#define SPIN_STOPPING_SINE 500

#define sign(x) ((x)<0?-1:1)
#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))

int16_t calibration_count1 = 0;
int16_t c=ANGLE_SCALE;
int16_t s=0;
int32_t x=0, y=0;
uint8_t state = 0;
uint32_t state_start_millis = 0;

// Encoder 1 uses INT0 and INT1
ISR(INT1_vect,ISR_ALIASOF(INT0_vect));
ISR(INT0_vect)
{
  uint8_t new11 = ((PIND & 0x01) != 0);
  uint8_t new12 = ((PIND & 0x02) != 0);
  
  count1 += (last11 ^ new12) - (int)(new11 ^ last12);
  
  if((last11 ^ new11) & (last12 ^ new12))
    error1 ++;
    
  last11 = new11;
  last12 = new12;
}

// Encoder two uses PCINT4 and PCINT7 (which trigger the PCINT0 interrupt)  
ISR(PCINT0_vect)
{
  uint8_t new21 = ((PINB & 0x10) != 0);
  uint8_t new22 = ((PINB & 0x80) != 0);
  
  count2 += (last21 ^ new22) - (int)(new21 ^ last22);  
  
  if((last21 ^ new21) & (last22 ^ new22))
    error2 ++;
  
  last21 = new21;
  last22 = new22;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  cli();
  PCMSK0 = 0x90; // enable pin change interrupts on PCINT4 and 7 which are Arduino pins 8, 11, PB4 and PB7
  PCICR = 0xff; // turns on pin change interrupts in general
  PCIFR = 0; // clear interrupt flags
  
  EICRA = 0x05; // set INT0 and INT to interrupt on all edges
  EIMSK = 0x03; // enable INT0 and INT1
  EIFR = 0; // clear interrupt flags
  
  // 20kHz PWM copied from Zumo shield library
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  sei();
  
  setMotors(0,0);
  
  delay(200);
}

void setMotors(int left, int right)
{
  if(left < 0)
  {
    digitalWrite(4, LOW);
    OCR1A = min(-left, 400);
  }
  else
  {
    digitalWrite(4, HIGH);
    OCR1A = min(left, 400);
  }
  
  if(right < 0)
  {
    digitalWrite(5, LOW);
    OCR1B = min(-right, 400);
  }
  else
  {
    digitalWrite(5, HIGH);
    OCR1B = min(right, 400);
  }
}

int16_t divide(int16_t a, int16_t b)
{
  return (a + sign(a)*(b/2-1)) / b;
}

void ticks1(int8_t n) {
  n = sign(n);
  
  int16_t dc = + divide(n*s - n*c/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  int16_t ds = - divide(n*c + n*s/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  
  c += dc;
  s += ds;
  x += n * c;
  y += n * s;
  
  last_count1 += n;
  
  // do it again every X times
  calibration_count1 ++;
  if(calibration_count1 >= ENCODER_CALIBRATION1 || calibration_count1 <= -ENCODER_CALIBRATION1)
  {
    last_count1 -= n;
    calibration_count1 = 0;
  }
}

void ticks2(int8_t n) {
  n = sign(n);
  
  int16_t dc = - divide(n*s + n*c/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  int16_t ds = + divide(n*c - n*s/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  c += dc;
  s += ds;
  x += n * c;
  y += n * s;
  
  last_count2 += n;
}

void positionUpdate() {
  if(count1 != last_count1) ticks1(count1 - last_count1);
  if(count2 != last_count2) ticks2(count2 - last_count2);
}

int16_t last_pos[DIR_MAX+1];

#define FRONT_LEFT_SENSOR_CHANNEL 1
#define FRONT_RIGHT_SENSOR_CHANNEL 0

void readLine(uint8_t dir, int16_t *pos, uint8_t *on_line)
{
  int16_t s0 = 0; // left sensor
  int16_t s1 = 0; // right sensor
  
  switch(dir)
  {
  case FORWARD:
    s0 = analogRead(FRONT_LEFT_SENSOR_CHANNEL);
    s1 = analogRead(FRONT_RIGHT_SENSOR_CHANNEL);
    break;
  case LEFT:
    s0 = analogRead(5);
    s1 = analogRead(4);
    break;
  case RIGHT:
    s0 = analogRead(2);
    s1 = analogRead(3);
    break;
  }
  
  s1 = max(s1, 640) - 640;
  s0 = max(s0, 640) - 640;
  
  if(s1 > 20 || s0 > 20)
  {
    *pos = 1000*(int32_t)(s1 - s0)/(s1+s0); // positive is to the left of the line
    last_pos[dir] = *pos;
    *on_line = 1;
  }
  else
  {
    if(last_pos[dir] > 0)
      *pos = 1000;
    else if(last_pos[dir] < 0)
      *pos = -1000;
    else
      *pos = 0;
    *on_line = 0;
  }
}

void followLine()
{
  static int16_t last_p = 0;
  int16_t p = 0;
  uint8_t on_line = 0;
  readLine(FORWARD, &p, &on_line);
  
  int16_t d = p - last_p;
  last_p = p;
  static int32_t i = 0;
  
  i += p;
  i = max(min(p, 1000000), -1000000);
  
  int16_t pid = p/7 + d*10;
  pid = max(min(pid, SPEED), -SPEED);
  if(pid < 0)
  {
    setMotors(SPEED + pid, SPEED);
  }
  else
  {
    setMotors(SPEED, SPEED - pid);
  }
  
  last_p = p;
  
  if(on_line)
  {
    if(!started_following)
    {
      started_following = 1;
      started_following_time = millis();
    }
    digitalWrite(13, 1);
  }
  else
  {
    resetStartedFollowing();
  }
}

uint8_t saw_line_forward = 0;
uint8_t saw_line_left = 0;
uint8_t saw_line_right = 0;

void watchForLine()
{
  int16_t pos;
  uint8_t on_line;
  
  readLine(FORWARD, &pos, &on_line);
  if(on_line)
  {
    saw_line_forward = 1;
  }
  
  readLine(LEFT, &pos, &on_line);
  if(on_line)
  {
    saw_line_left = 1;
  }
  
  readLine(RIGHT, &pos, &on_line);
  if(on_line)
  {
    saw_line_right = 1;
  }  
}

void resetSawLine()
{
  saw_line_left = 0;
  saw_line_forward = 0;
  saw_line_right = 0;
}

void resetStartedFollowing()
{
  started_following = 0;
  digitalWrite(13, 0);
}

uint8_t checkForEnd() {
  // watch for the end in the last four inches
  static int32_t dark_start_x = 0;
  static uint8_t on_dark = 0;
  
  if(x > -MAZE_UNIT_DISTANCE*4/6 &&
     analogRead(FRONT_LEFT_SENSOR_CHANNEL) > DARK_VALUE &&
     analogRead(FRONT_RIGHT_SENSOR_CHANNEL) > DARK_VALUE)
  {
    // we are on a very dark region - either the end or crossing a line
    if(!on_dark)
    {
      on_dark = true;
      dark_start_x = x;
    }
    if(x - dark_start_x > MAZE_UNIT_DISTANCE*2/6)
    {
      return 1; // 2 inches dark - must be the end!
    }
  }
  else
  {
    on_dark = false;
  }
  
  return 0;
}

uint8_t goHome(uint8_t allow_following, uint8_t stop_at_end) {
  int16_t speed = SPEED;
  int32_t err;
  static uint8_t started_stopping = 0;
  static uint16_t started_stopping_time = 0;
  int16_t pos;
  uint8_t on_line;
  
  // watch for the line in the last inch
  if(x > -MAZE_UNIT_DISTANCE/6)
    watchForLine();
  else
    readLine(FORWARD, &pos, &on_line);
  
  // consider following if it is allowed and we are close to the line
  if(allow_following && y > -MAZE_UNIT_DISTANCE/6 && y < MAZE_UNIT_DISTANCE/6 && s > -ANGLE_SCALE/2 && s < ANGLE_SCALE/2)
  {
    // follow the line for the moddle four inches
    if( (x > -MAZE_UNIT_DISTANCE*8/6 && x < -MAZE_UNIT_DISTANCE*4/6) ||
      (x > -MAZE_UNIT_DISTANCE*14/6 && x < -MAZE_UNIT_DISTANCE*10/6) ) // if it is looking ahead to the next segment
    {
      followLine();
      started_stopping = 0;
      return 0;
    }
    else if(started_following && ((uint16_t)millis() - started_following_time > 200))
    {
      // assume we are on the line
      y = 0;
      s = 0;
      c = ANGLE_SCALE;
      steps_since_last_calibrated = 0;
    }
  }
  
  // not following
  resetStartedFollowing();
  
  if(x > -STOPPING_DISTANCE && stop_at_end)
  {
    // work on stopping
    setMotors(0,0);
    if(!started_stopping)
    {
      started_stopping = 1;
      started_stopping_time = millis();
    }
    
    if(((uint16_t)millis()) - started_stopping_time > STOPPING_TIME_MS)
    {
      started_stopping = 0;
      return 1;
    }
    return 0;
  }
  else if(x >= 0 && !stop_at_end)
  {
    started_stopping = 0;
    return 1;
  }
  
  started_stopping = 0;
  
  if(stop_at_end && x > -MAZE_UNIT_DISTANCE/3)
    speed = speed/2;
  
  if(c < 0)
  {
    // pointed backwards
    err = (s > 0 ? speed : -speed);
  }
  else
  {    
    int32_t target_s = -max(min(y / 10000 * FOLLOW_MAX_S / (FOLLOW_MAX_Y / 10000), FOLLOW_MAX_S), -FOLLOW_MAX_S);
    err = (s - target_s)/100;
    err = max(min(err,speed),-speed);
  }
  if(err > 0)
    setMotors(speed, speed - err);
  else
    setMotors(speed + err, speed);
    
  return 0;
}

uint8_t spinToAngleZero() {
  int16_t speed = SPIN_SPEED;
  int32_t err;
  static uint8_t started_stopping = 0;
  static uint16_t started_stopping_time = 0;
  
  // not following
  resetStartedFollowing();
  
  if(c > 0 && s > -SPIN_STOPPING_SINE && s < SPIN_STOPPING_SINE)
  {
    // work on stopping
    setMotors(0,0);
    if(!started_stopping)
    {
      started_stopping = 1;
      started_stopping_time = millis();
    }
    
    if(((uint16_t)millis()) - started_stopping_time > STOPPING_TIME_MS)
    {    
      started_stopping = 0;
      return 1;
    }
    else
    {
      return 0;
    }
  }
  
  started_stopping = 0;
  
  if(s > 0)
    setMotors(speed, -speed);
  else
    setMotors(-speed, speed);
    
  return 0;
}

// direct-to the origin
void transform() {
  double r = hypot((double)x, (double)y);
  double nx = (double)x/r;
  double ny = (double)y/r;
  int16_t new_c = -nx*c-ny*s;
  int16_t new_s = ny*c-nx*s;
  c = new_c;
  s = new_s;
  y = 0;
  x = -r;
}

void forward_unit() {
  x -= MAZE_UNIT_DISTANCE;
}

void turn(uint8_t dir) {
  int16_t tmp_s;
  int32_t tmp_y;
  
  switch(dir) {
  case LEFT:
    tmp_s = -c;
    c = s;
    s = tmp_s;  
    tmp_y = -x;
    x = y;
    y = tmp_y;
    break;
  case RIGHT:
    tmp_s = c;
    c = -s;
    s = tmp_s;  
    tmp_y = x;
    x = -y;
    y = tmp_y;
    break;
  case BACKWARD:
    c = -c;
    s = -s;
    x = -x;
    y = -y;
    break;
  case FORWARD:
    break;
  }
}

void simplifyPath() {
  if(path_size >= 3 && path[path_size-2] == BACKWARD && path[path_size-3] != BACKWARD && path[path_size-1] != BACKWARD)
  {
    // if the second-to-last move was B, we can always simplify
    // e.g. RBS -> L
    // the general algorithm is to count L as -1 and R as +1 as follows:
    int8_t count = 0;
    count -= (path[path_size-3] == LEFT);
    count += (path[path_size-3] == RIGHT);
    count -= (path[path_size-1] == LEFT);
    count += (path[path_size-1] == RIGHT);
    
    path_size -= 2;
    switch(count) {
    case 2:
    case -2:
      path[path_size-1] = FORWARD;
      break;
    case -1:
      path[path_size-1] = RIGHT;
      break;
    case 0:
      path[path_size-1] = BACKWARD;
      break;
    case 1:
      path[path_size-1] = LEFT;
      break;
    }
  }
}
void addToPath(uint8_t dir) {
  path[path_size] = dir;
  path_size += 1;
  simplifyPath();
}

void turnLeftmost() {
  watchForLine();
  
  if(saw_line_left)
  {
    addToPath(LEFT);
    turn(LEFT);
    return;
  }
  
  if(saw_line_forward)
  {
    addToPath(FORWARD);
    return;
  }
  
  if(saw_line_right)
  {
    addToPath(RIGHT);
    turn(RIGHT);
    return;
  }
  
  addToPath(BACKWARD);
  turn(BACKWARD);
  return;
}

uint32_t last_millis = 0;
uint8_t led = 0;

void encoderUpdate() {
  if(last_count1 != count1 || last_count2 != count2)
  {
    positionUpdate();
  }
}

uint16_t getBatteryVoltage_mv() {
  return analogRead(11) * 10;
}

void debug() {
  int16_t pos = 0;
  uint8_t on_line = 0;
  if(millis() - last_millis > 100)
  {
    led = !led;
    digitalWrite(13, led);
    
    Serial.print(getBatteryVoltage_mv());
    Serial.write("mV\t");
    
    Serial.print(analogRead(1));
    Serial.write(",");
    Serial.print(analogRead(0));
    readLine(FORWARD,&pos,&on_line);
    if(on_line)   
      Serial.write(":");
    else
      Serial.write("!");
    Serial.print(pos);
    Serial.write(" ");
    
    Serial.print(analogRead(5));
    Serial.write(",");
    Serial.print(analogRead(4));
    Serial.write(":");
    readLine(LEFT,&pos,&on_line);
    Serial.print(pos);
    Serial.write(" ");
    
    Serial.print(analogRead(2));
    Serial.write(",");
    Serial.print(analogRead(3));
    Serial.write(":");
    readLine(RIGHT,&pos,&on_line);
    Serial.print(pos);
    Serial.write(" ");
    
    Serial.write("\t");
    Serial.print(c);
    Serial.write(",");
    Serial.print(s);
    Serial.write("\t");
    Serial.print(x);
    Serial.write(",");
    Serial.print(y);
    Serial.write("\t");
    Serial.print(error1);
    Serial.write(",");  
    Serial.print(error2);
    
    Serial.println("");
    last_millis = millis();
  }
}

uint8_t okToLookAhead() {
  return x > -MAZE_UNIT_DISTANCE*3/6 && y > -MAZE_UNIT_DISTANCE*4/6 && y < MAZE_UNIT_DISTANCE*4/6 ||
    x > -MAZE_UNIT_DISTANCE*4/6 && y > -MAZE_UNIT_DISTANCE*3/6 && y < MAZE_UNIT_DISTANCE*3/6 ||
    x > -MAZE_UNIT_DISTANCE*5/6 && y > -MAZE_UNIT_DISTANCE*2/6 && y < MAZE_UNIT_DISTANCE*2/6 ||
    x > -MAZE_UNIT_DISTANCE && y > -MAZE_UNIT_DISTANCE/6 && y < MAZE_UNIT_DISTANCE/6;
}

void loop() {
  static uint16_t battery_voltage_low_millis = 0;
  static uint8_t last_state = 255;
  static uint8_t last_turn_was_fast = 0;
  uint8_t calibration_cutoff;
    
  if(last_state != state)
  {
    // in a new state!
    state_start_millis = millis();
    last_state = state;
  }
  
  encoderUpdate();
  switch(state) {
  case 0:
    setMotors(0,0);
    if(!digitalRead(BUTTON_PIN))
    {
      delay(1000);
      if(path_size)
      {
        state = 6;
        path_pos = 0;
      }
      else
      {
         state ++;
      }
    }
    debug();
    break;
  case 1:
    digitalWrite(13, 0);
    forward_unit();
    resetSawLine();
    state++;
    break;
  case 2:
    if(checkForEnd())
      state = 0;
    else if(goHome(1, 1))   
      state ++;
    break;
  case 3:
    turnLeftmost();
    state ++;
    break; 
  case 4:
    if(spinToAngleZero())
      state = 1;
    break;
  case 5:
    digitalWrite(13, 1);
    setMotors(0,0);
    debug();
    break;
  case 6:
    digitalWrite(13, 0);
    forward_unit();
    steps_since_last_calibrated ++;
    
    calibration_cutoff = 10;
    
    // if the next two turns are FORWARD, try calibration, since it will not slow us down
    if(path_pos+1 < path_size && path[path_pos] == FORWARD && path[path_pos+1] == FORWARD)
      calibration_cutoff = 0;

    // if we have a BACKWARD approaching (which shouldn't happen), also calibrate
    else if(path_pos+1 < path_size && path[path_pos+1] == BACKWARD ||
      path_pos < path_size && path[path_pos] == BACKWARD)
      calibration_cutoff = 0;
    
    // if we have a forward coming up, make calibration more likely
    else if(path_pos < path_size+1 &&
      (path[path_pos] == FORWARD || path[path_pos+1] == FORWARD))
      calibration_cutoff = 5;
      
    state = (steps_since_last_calibrated < calibration_cutoff ? 7 : 9);
    break;
  case 7: // follow path without calibrating
    if(goHome(0, 0) || okToLookAhead())
      state ++;
    break;
  case 8:
    if(path_size == path_pos)
      state = 12;
    else
    {
      turn(path[path_pos]);
      last_turn_was_fast = (path[path_pos] != FORWARD);
      state = 6;
      path_pos ++;
    }
    break;
  case 9: // do a slow turn
    if(goHome(last_turn_was_fast ? 0 : 1, path_size != path_pos && path[path_pos] != FORWARD) ||
      (path[path_pos] == FORWARD && okToLookAhead()))
      state ++;
    break;
  case 10:  
    if(path_size == path_pos)
    {
      state = 12;
      break;
    }
    turn(path[path_pos]);
    if(path[path_pos] == FORWARD)
    {
        state = 6;
        last_turn_was_fast = 0;
    }
    else
    {
      state ++;
    }
    path_pos ++;
    break;
  case 11:
    if(spinToAngleZero())
    {
      last_turn_was_fast = 0;
      state = 6;
    }
    break;
  case 12:
    if(goHome(0, 0))
      state = 0;
    break;
  }
}
