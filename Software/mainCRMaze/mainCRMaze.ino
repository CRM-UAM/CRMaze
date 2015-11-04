#include <StackList.h>
#include <stdarg.h>

//Include custom data types
#include "Coord.h"
#include "entry.h"

#define DIST_3_PIN      A3
#define DIST_2_PIN      A2
#define DIST_1_PIN      A1

#define NORTH 1
#define SOUTH 2
#define EAST 4
#define WEST 8
  //Define some global constants
  #define X 13
  #define Y 13

float getDistanceCM(int);
  entry maze[Y][X]; //maze matrix ¡¡Cuidado usa la Y y la X al reves ya que maze[0] es la FILA superior!!
  uint8_t headings[] = {1,2,4,8}; //N,S,E,W

//Variables para simulacion de un laberinto y comprobar la correccion
  uint8_t posXrobot=0;
  uint8_t posYrobot=0;
  uint8_t headingRobot=2;
  int totalMove=0;



byte mazeSol[13][13]={
{4,12,10,4,12,12,12,14,10,6,10,4,10},
{4,10,5,10,6,12,12,9,5,9,5,12,11},
{6,13,10,3,3,4,14,14,12,8,6,12,9},
{3,6,9,5,9,6,9,1,6,12,9,6,10},
{3,5,12,8,6,13,12,12,9,6,8,3,3},
{7,12,10,6,9,6,14,10,2,7,12,9,3},
{1,6,11,5,10,7,15,11,5,9,6,12,11},
{6,9,1,6,9,5,15,9,6,12,9,6,9},
{5,10,6,9,4,14,13,10,7,10,4,11,2},
{6,11,3,6,10,5,10,5,9,5,10,3,3},
{3,1,5,9,3,2,7,12,12,8,3,3,3},
{7,14,8,6,9,5,9,6,12,12,9,5,11},
{1,5,12,13,12,12,12,9,4,12,12,12,9}
}; //matriz con posibles paredes en los indices pares y celdas en los impares. 0 si no hay pared 1 si si.




void pf(char *fmt, ... ){
        char buf[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 128, fmt, args);
        va_end (args);
        Serial.print(buf);
}

uint8_t wallsAt(uint8_t posX, uint8_t posY){
  return mazeSol[posY][posX];
  /*uint8_t ret=0;
  uint8_t nX=posX*2+1;
  uint8_t nY=posY*2+1;
  //W=8,E=4,S=2,N=1
  if(!mazeSol[nY-1][nX])ret+=1; //no hay pared al norte
  if(!mazeSol[nY+1][nX])ret+=2; //no hay pared al sur
  if(!mazeSol[nY][nX-1])ret+=8; //no hay pared al oeste
  if(!mazeSol[nY][nX+1])ret+=4; //no hay pared al este
  //pf("DEBUG WALLSAT (%d,%d)=%d\n",posX,posY,ret);
  return ret;*/
}

inline bool isWall(float d){
    return ( d <= 15 && d >=140);
}

uint8_t wallsAtSensors(){
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

/*
Imprime el esado actual de maze con las sitancias en las celdas
+----+
| 9  |
+----+
*/
void printMaze(){
  uint8_t i,j;
  for(i=0;i<X;i++)
    pf("+%.3d+",i);
  Serial.println("");
  for (j=0;j<Y;j++){
    for(i=0;i<X;i++){
      if(!(maze[j][i].walls & NORTH))
        Serial.print("+---+");
      else
        Serial.print("+   +");
    }
    Serial.println("");
    for(i=0;i<X;i++){
      if(!(maze[j][i].walls & WEST))
        Serial.print("|");
      else
        Serial.print(" ");
      if (maze[j][i].distance/100) pf("%d",maze[j][i].distance);
      else
        if(i==posXrobot && j==posYrobot)
          pf("^%d",maze[j][i].distance);
        else
          pf(" %d",maze[j][i].distance);
        if(maze[j][i].distance < 10) pf(" ");
      if(!(maze[j][i].walls & EAST))
        Serial.print("|");
      else
        Serial.print(" ");
    }
    pf(" -%d\n",j);
    for(i=0;i<X;i++){
      if(!(maze[j][i].walls & SOUTH))
        Serial.print("+---+");
      else
        Serial.print("+   +");
    }
    Serial.println("");

  }
}


void setup(){
  //instantiate an empty maze
  Serial.begin(115200);
  instantiate();
delay(1000);

}



void instantiate(){
  for(uint8_t j = 0; j<Y; j++){
    for(uint8_t i = 0; i<X; i++){
      maze[j][i].distance = calcCenter(i, j, X);
      maze[j][i].walls = 15;
      //If this is the left column (0,x)
      if(i==0){
        maze[j][i].walls = 7;
      }
      //if this is the top row
      if(j==0){
        maze[j][i].walls = 14;
      }
      //if this is the bottom row
      if(j==(Y-1)){
        maze[j][i].walls = 13;
      }
      //If this is the righ column
      if(i==(X-1)){
        maze[j][i].walls = 11;
      }
      maze[0][0].walls = 6;
      maze[Y-1][0].walls = 5;
      maze[0][X-1].walls = 10;
      maze[X-1][Y-1].walls = 9;
    }
  }
}
//Get the most optimistic distance between two coordinates in a grid
uint8_t calcDist(uint8_t posx, uint8_t posy, uint8_t desireX, uint8_t desireY){
  uint8_t dist = abs(desireY-posy)+abs(desireX-posx);
  return dist;
}

//Get the most optimistic distance between a given coordinate and a
//3x3 square in the center of a maze of dimension dim (dim must be even)
uint8_t calcCenter(uint8_t posx, uint8_t posy, uint8_t dim){
  uint8_t center = dim/2;
  uint8_t dist = 0;

  if(posy < center){
    if(posx < center){
      //You're in the top left of the maze
      dist=calcDist(posx, posy, (center-1), (center-1));
    }else if(posx == center){
      dist=calcDist(posx, posy, center, (center-1));
    }else{
      //You're in the bottom left of the maze
      dist=calcDist(posx,posy, (center+1),(center-1));
    }
  }else if(posy == center){
    if(posx < center){
      //You're in the top center of the maze
      dist=calcDist(posx, posy, center-1, center);
    }else if(posx == center){
      dist=0;
    }else{
      //You're in the bottom center of the maze
      dist=calcDist(posx,posy, center+1,center);
    }
  }else{
    if(posx < center){
      //You're in the bottom left of the maze
      dist=calcDist(posx,posy,(center - 1),(center + 1));
    }else if(posx == center){
      dist=calcDist(posx, posy, center, (center + 1));
    }else{
      //You're in the bottom right of the maze
      dist=calcDist(posx,posy,(center + 1),(center + 1));
    }
  }
return dist;
}

/*
INPUT: a coordinate representing a current position, and a heading
OUTPUT: the coordinates of the next desired position based on the heading and current position
*/
Coord bearingCoord(Coord currCoord, uint8_t heading){
  Coord nextCoord (0,0);
  switch (heading){
    case 1:
      //code
      nextCoord.setX(currCoord.getX());
      nextCoord.setY(currCoord.getY()-1);
      break;
    case 2:
      nextCoord.setX(currCoord.getX());
      nextCoord.setY(currCoord.getY()+1);
      break;
    case 4:
      nextCoord.setX(currCoord.getX()+1);
      nextCoord.setY(currCoord.getY());
      break;
    case 8:
      nextCoord.setX(currCoord.getX()-1);
      nextCoord.setY(currCoord.getY());
      break;
  }
  return nextCoord;
}

/*
INPUT: A Coord representing the current coordiante and the robots current heading
OUTPUT: An optimal direction away from the current coordinate.
*/
uint8_t orient(Coord currCoord, uint8_t heading){
  //printMaze();
  //pf("DEBUG orient (%d,%d-%d)\n",currCoord.x,currCoord.y,heading);
  Coord leastNext (0,0);
  //This is the absolute largest value possible (dimension of maze squared)
  uint8_t leastNextVal = 255;
  uint8_t leastDir = heading;

  //If there is a bitwise equivalence between the current heading and the cell's value, then the next cell is accessible
  //pf("DEBUG orient if w(%d,%d)=%d(=%d) h=%d\n",currCoord.x,currCoord.y,maze[currCoord.y][currCoord.x].walls,wallsAt(currCoord.x,currCoord.y), heading);
  if((maze[currCoord.getY()][currCoord.getX()].walls & heading) != 0){
    //Define a coordinate for the next cell based onthis heading and set the leastNextVal t its value
    //pf("\tDEBUG orient test1 (%d,%d-%d)->", currCoord.getX(),currCoord.getY(),heading);
    Coord leastnextTemp = bearingCoord(currCoord, heading);
    //pf("(%d,%d)\n",leastnextTemp.getX(),leastnextTemp.getY());
    if(checkBounds(leastnextTemp)){
      leastNext = leastnextTemp;
      leastNextVal = maze[leastNext.getY()][leastNext.getX()].distance;
    }
  }
  //pf("DEBUG orient partial result (%d,%d)=%d\n",leastNext.getX(),leastNext.getY(),leastNextVal);
  for(uint8_t i=0; i<4; i++){
    uint8_t dir = headings[i];
    //if this dir is accessible
    if((maze[currCoord.getY()][currCoord.getX()].walls & dir) != 0){
      //define the coordiante for this dir
      Coord dirCoord = bearingCoord(currCoord,dir);

      if(checkBounds(dirCoord)){
        //if this dir is more optimal than continuing straight
        if(maze[dirCoord.getY()][dirCoord.getX()].distance < leastNextVal){
          //update teh value of leastNextVal
          leastNextVal = maze[dirCoord.getY()][dirCoord.getX()].distance;
          //update the value of leastnext to this dir
          leastNext = dirCoord;
          leastDir = dir;
          //pf("DEBUG orient actualizar minimo to h=%d, (%d,%d-%d)\n",leastDir,leastNext.getX(),leastNext.y,leastNextVal);
        }
      }
    }
  }
  return leastDir;
}

//Take a coordinate and test if it is within the allowable bounds
boolean checkBounds(Coord c){
  if((c.getX() >= X) || (c.getY() >= Y) || (c.getX() < 0) || (c.getY() < 0))
    return false;
  else
    return true;
}

/*
INPUT: c
OUTPUT: An integer that is the least neighbor
*/


uint8_t checkNeighs(Coord c){
  uint8_t minVal =  255;

  for(int i=0; i<4; i++){
    uint8_t dir = headings[i];
    //if this dir is accessible
    /*if(c.getX()==4 && c.getY()==3){
        pf("DEBUG checkNeighs (4,3) walls %d, dir%d\n",maze[c.getY()][c.getX()].walls,dir);
        printMaze();
    }*/
    if((maze[c.getY()][c.getX()].walls & dir) != 0){
      //Get the coordinate of the accessible neighbor
      Coord neighCoord = bearingCoord(c, dir);
      //Check the value of the accessible neighbor
      if (checkBounds(neighCoord)){
        //if the neighbore is less than the current recording minimum value, update the minimum value
        //If minVal is null, set it right away, otherwise test
        if(maze[neighCoord.getY()][neighCoord.getX()].distance < minVal){
            minVal = maze[neighCoord.getY()][neighCoord.getX()].distance;
           // if(c.getX()==4 && c.getY()==3)pf("DEBUG checkNeighs (4,3)=%d (dir=%d, {%d,%d})\n",minVal,dir,neighCoord.getX(),neighCoord.getY());
        }
      }
    }
  }

  return minVal;
}

//Given a coordinate, test and return if the coordinate is bounded on three sides
boolean isDead(Coord c){
  boolean deadEnd = false;
  if(checkBounds(c)){
    uint8_t bounds = maze[c.getY()][c.getX()].walls;
    //bounds is the integer from the exploratory maze that represents the known walls of the coordinate
    if((bounds == 1)||(bounds == 2)||(bounds == 4) || (bounds == 8)){deadEnd=true;}
  }
  return deadEnd;
}

boolean isEnd(Coord c, Coord DesiredArray[],int lenArr){
  for(uint8_t i=0; i<lenArr;i++){

    Coord Desired = DesiredArray[i];
    //pf("DEBUG isEND: intento %d (%d,%d)==(%d,%d)\n",i,c.x,c.y,Desired.x,Desired.y);
    if(checkBounds(c)){
      if((c.getX() == Desired.getX())&&(c.getY()==Desired.getY())){
        return true;
      }
    }
  }
  return false;
}

/*
This function makes calls to the dispatcher to get the following info
  -orientation
  -surrounding walls
Using orientation and walls, this information is mapped to a map integer in the global coordinate frame
*/
uint8_t readCurrent(){
  //TODO: Fill this bitch out
  //return 0;

  return wallsAt(posXrobot, posYrobot);
}

uint8_t readAhead(){
  return 0;
}


/*
INPUT: Coordindate to update, and a direction representing the wall to add
OUTPUT: Update to coordinate adding the wall provided as an argument
*/
void coordUpdate(Coord coordinate, uint8_t wallDir){
  if(checkBounds(coordinate)){
    if((maze[coordinate.getY()][coordinate.getX()].walls & wallDir) != 0){
      maze[coordinate.getY()][coordinate.getX()].walls = maze[coordinate.getY()][coordinate.getX()].walls-wallDir;
    }
  }
}

/*
INPUT: Current Robot coordinate
OUTPUT: Update maze for learned walls
*/
void floodFillUpdate(Coord currCoord, Coord desired[], int lenDesired){
  StackList<Coord> entries;

  maze[currCoord.getY()][currCoord.getX()].walls=readCurrent();
  entries.push(currCoord);
  //printMaze();
  //pf("DEBUG FFUpdate (%d,%d)\tNumero en la cola: %d\t",currCoord.getX(),currCoord.getY(),entries.count());
  for(uint8_t i=0; i<4; i++){
    uint8_t dir = headings[i];
    //If there's a wall in this dir
    if((maze[currCoord.getY()][currCoord.getX()].walls & dir) == 0){
      //pf("direccion %d con paredes=%d\n",dir,maze[currCoord.getY()][currCoord.getX()].walls);
      Coord workingCoord = Coord(currCoord.getX(),currCoord.getY());
      switch(dir){
        case 1:
          workingCoord.setY(workingCoord.getY()-1);
          coordUpdate(workingCoord,2);
          break;
        case 2:
          workingCoord.setY(workingCoord.getY()+1);
          coordUpdate(workingCoord,1);
          break;
        case 4:
          workingCoord.setX(workingCoord.getX()+1);
          coordUpdate(workingCoord, 8);
          break;
       case 8:
         workingCoord.setX(workingCoord.getX()-1);
         coordUpdate(workingCoord,4);
         break;
      }

      //If the workingEntry is a valid entry and not a dead end, push it onto the stack
      if(checkBounds(workingCoord)&&(!isEnd(workingCoord, desired,lenDesired))){
       // pf("DEBUG\t a la cola-> (%d,%d)\n",workingCoord.x,workingCoord.y);
        entries.push(workingCoord);
        //pf("push (%d,%d)\t",workingCoord.getX(),workingCoord.getY());
      }
    }
  }
  //pf("DEBUG FFUpdate (%d,%d)\tNumero en la cola: %d\t",currCoord.getX(),currCoord.getY(),entries.count());

  //While the entries stack isn't empty

  while(!entries.isEmpty()){
    //pf("pila size: %d\n",entries.count());
    //Pop an entry from the stack
    Coord workingEntry = entries.pop();
    uint8_t neighCheck = checkNeighs(workingEntry);
    //pf("DEBUG\t vecino minimo de (%d,%d) es: %d\n",workingEntry.getX(),workingEntry.getY(),neighCheck);
    //If the least neighbor of the working entry is not one less than the value of the working entry
    //if(workingEntry.getX()==4 && workingEntry.getY()==6)pf("DEBUG ALERTA (4,6): vecinominimo=%d isEnd=%d\n",neighCheck,isEnd(workingEntry,desired,lenDesired));
    if(neighCheck+1!=maze[workingEntry.getY()][workingEntry.getX()].distance && !isEnd(workingEntry,desired,lenDesired)){
      maze[workingEntry.getY()][workingEntry.getX()].distance=neighCheck+1;
      for(uint8_t i=0;i<4;i++){
        uint8_t dir = headings[i];
        if((maze[workingEntry.getY()][workingEntry.getX()].walls & dir) != 0){
          Coord nextCoord = bearingCoord(workingEntry,dir);
          if(checkBounds(nextCoord)){
            if(!isEnd(nextCoord, desired,lenDesired)){
              entries.push(nextCoord);
            }
          }
        }
      }
    }
  }
}

void floodFill(Coord desired[],int lenDesired, Coord currCoord, boolean isMoving){

  uint8_t heading = 4;
  /*Integer representation of heading
  * 1 = N
  * 4 = E
  * 2 = S
  * 8 = W
  */
  while(maze[currCoord.getY()][currCoord.getX()].distance != 0){

      floodFillUpdate(currCoord, desired,lenDesired);
      //if(!isMoving)printMaze();
      //Serial.println("END UPDATE");
      //printMaze();
      uint8_t nextHeading = orient(currCoord, heading);
      Coord nextCoord = bearingCoord(currCoord, nextHeading);
      //TODO: ADD MOVING INSTRUCTIONS HERE
      if(isMoving){
         //pf("\tmovimiento: (%d,%d)->(%d,%d) [heading %d->%d]\n",currCoord.x,currCoord.y,nextCoord.x,nextCoord.y,heading,nextHeading);
        totalMove++;
      }
      posXrobot=nextCoord.getX();
      posYrobot=nextCoord.getY();

      //This should occur as a callback of the moving finishing
      currCoord = nextCoord;
      heading = nextHeading;
  }
}


void resetToCoord(Coord desiredCoord){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcDist(i, j, desiredCoord.getX(), desiredCoord.getY());
    }
  }
}
void instantiateReflood(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcCenter(i, j, X);
    }
  }
}


long createSpeedQueue(Coord workingCoord, Coord desired[], int lenDesired, long maxLengthPath){
  byte workingDir = EAST;
  int workingDist = 1;
  long len=0;
  while(!isEnd(workingCoord,desired,lenDesired) && len<(maxLengthPath+5)){
    byte optimalDir = orient(workingCoord, workingDir);

    //If the direction is the same, accumulate distance
    if(optimalDir==workingDir){
      workingDist++;
    }else{
      //if the optimal is different from the working, add the working and the accumulated distance
      //instruction nextInstruction = {workingDist, optimalDir};
      //instructions.push(nextInstruction);
      pf("avanzar: %d celdas\n",workingDist);
      switch(optimalDir){
        case 1:
          Serial.println("tomar orientacion: NORTE");
          break;
        case 2:
          Serial.println("tomar orientacion: SUR");
          break;
        case 4:
          Serial.println("tomar orientacion: ESTE");
          break;
        case 8:
          Serial.println("tomar orientacion: OESTE");
          break;
      }
      //Reset the distance to one square and update the workingDir
      workingDist = 1;
      workingDir = optimalDir;
      len++;
    }

    //update workingCoord to the next optimal coord
    workingCoord = bearingCoord(workingCoord, optimalDir);
  }
  pf("avanzar: %d celdas\n",workingDist);
  if(!isEnd(workingCoord,desired,lenDesired))Serial.println("*RUTA NO VALIDA*");
  return len;
}

void reflood(Coord currCoord, Coord desired[], int lenDesired, long lenPath){
  //Refill the maze for most optimistic values, but now the maze has walls
  instantiateReflood();
 printMaze();
  //Run flood fill but without actual motion
  floodFill(desired,9, currCoord, false);
  printMaze();
  //Now, the robot is still at the start, but the maze distance values have been updated with the walls discovered
  //So we follow the maze creating instructions
  createSpeedQueue(currCoord,desired,lenDesired,lenPath);
  //We now have a queue of instructions.

}

void loop(){


  Coord desired[] = {Coord(5,5),Coord(5,6),Coord(5,7),Coord(6,5),Coord(6,6),Coord(6,7),Coord(7,5),Coord(7,6),Coord(7,7)};

  floodFill(desired,9, Coord(0,0),true);
  Serial.println("**FIN FLOOD_FILL 1");
  pf("movimientos: %d***\n",totalMove);
  printMaze();
  long lenPath=createSpeedQueue(Coord(0,0),desired,9,1000);

  Serial.println("***VOLVIENDO AL ORIGEN****");
  Coord returnCoord3[] = {Coord(0,0)};
  resetToCoord(returnCoord3[0]);
  floodFill(returnCoord3,1, Coord(posXrobot,posYrobot),true);
  Serial.println("*****FIN FLOOD_FILL Retorno*******");
  printMaze();



  //pf("**SOLUCION (Realizados %d mov para rastrear):\n");
  reflood(Coord(0,0),desired,9,lenPath);
  //Serial.println("*****FIN FLOOD_FILL Reflood****");
  //printMaze();
  Serial.println("****END PROGRAM*******");

  while(1){
    digitalWrite(13,HIGH);
    delay(10000);
  }
}
