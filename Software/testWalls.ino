#define DIST_3_PIN      A3
#define DIST_2_PIN      A2
#define DIST_1_PIN      A1

#define NORTH 1
#define SOUTH 2
#define EAST 4
#define WEST 8


inline bool isWall(float d){
    return ( d <= 15 && d >=140);
}

#define ir_K = 4419.36 // Linearization of the sensor response
#define ir_C = 32.736
float getDistanceCM(int pin) {
  float measurement = analogRead(pin);
  if(measurement <= ir_C) return 150;
  float res = ir_K/(measurement-ir_C);
  if(res < 0 || res > 150) res = 150;
  return res;
}

uint8_t wallsAtSensors(uint8_t headingRobot){
  boolean pF=isWall(getDistanceCM(DIST_1_PIN));
  boolean pR=isWall(getDistanceCM(DIST_1_PIN));
  boolean pL=isWall(getDistanceCM(DIST_1_PIN));
  uint8_t ret=0;
  switch(headingRobot){
    case NORTH:
        if(!pF)ret+=NORTH;
        if(!pR)ret+=EAST;
        if(!pL)ret+=WEST;
        return ret+SOUTH; //devuelvo las paredes detectadas mas la ausencia de pared por donde he venido
    case EAST:
        if(!pF)ret+=EAST;
        if(!pR)ret+=SOUTH;
        if(!pL)ret+=NORTH;
        return ret+WEST; //devuelvo las paredes detectadas mas la ausencia de pared por donde he venido
    case SOUTH:
        if(!pF)ret+=SOUTH;
        if(!pR)ret+=WEST;
        if(!pL)ret+=EAST;
        return ret+NORTH; //devuelvo las paredes detectadas mas la ausencia de pared por donde he venido
    case WEST:
        if(!pF)ret+=WEST;
        if(!pR)ret+=NORTH;
        if(!pL)ret+=SOUTH;
        return ret+EAST; //devuelvo las paredes detectadas mas la ausencia de pared por donde he venido
    default:
        return -1;
  }
}


void setup(){
  //instantiate an empty maze
  Serial.begin(115200);
  delay(1000);

}

void loop(){
    Serial.println("ORIENTACION NORTE")
    Serial.println(wallsAtSensors(NORTH))
    delay(1000);
    Serial.println("GIRO AL ESTE");
    Serial.println(wallsAtSensors(EAST))
    delay(1000);
    Serial.println("GIRO AL OESTE");
    Serial.println(wallsAtSensors(WEST))
    delay(1000);
    Serial.println("GIRO AL SUR");
    Serial.println(wallsAtSensors(SOUTH))
    delay(1000);
}
