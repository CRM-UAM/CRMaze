#include <QueueList.h>
#include <StackList.h>

//Include custom data types
#include "coord.h"
#include "entry.h"

#define NORTH 1
#define SOUTH 2
#define EAST 4
#define WEST 8

//Navigation info
QueueList<instruction> instructions;


byte globalHeading = 4;
coord globalCoord = {0,12};
coord globalEnd = {0,12};
int actualAng=0;

  //Define some global constants
  #define X 13
  #define Y 13

  entry maze[Y][X];
  //N,S,E,W
  byte headings[] = {1,2,4,8};

boolean mazeSol[27][27]={
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1},
  {1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
  {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1},
  {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1},
  {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1},
  {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
}; //matriz con posibles paredes en los indices pares y celdas en los impares. 0 si no hay pared 1 si si.

void pf(char *fmt, ... ){
        char buf[512]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 512, fmt, args);
        va_end (args);
        Serial.print(buf);
}


byte wallsAt(byte posX, byte posY){
  byte ret=0;
  byte nX=posX*2+1;
  byte nY=posY*2+1;
  //W=8,E=4,S=2,N=1
  if(!mazeSol[nY-1][nX])ret+=1; //no hay pared al norte
  if(!mazeSol[nY+1][nX])ret+=2; //no hay pared al sur
  if(!mazeSol[nY][nX-1])ret+=8; //no hay pared al oeste
  if(!mazeSol[nY][nX+1])ret+=4; //no hay pared al este
  //pf("DEBUG WALLSAT (%d,%d)=%d\n",posX,posY,ret);
  return ret;
}

/*
Imprime el esado actual de maze con las sitancias en las celdas
+----+
| 9  |
+----+
*/
void printMaze(){
  byte i,j;
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
      if(i==globalCoord.x && j==globalCoord.y)
        pf("^%d",maze[j][i].distance);
      else
        pf(" %d",maze[j][i].distance);
      if(maze[j][i].distance < 10) pf(" ");
      if(!(maze[j][i].walls & EAST))
        Serial.print("|");
      else
        Serial.print(" ");
    }
    Serial.println("");
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
  Serial.begin(115200);
  delay(2000);
  instantiate();
  delay(100);
  printMaze();

}

byte readCurrent(){
  return wallsAt(globalCoord.x,globalCoord.y);
  /*
  byte wallReading = 15;
  byte north = 0;
  byte south = 0;
  byte east = 0;
  byte west = 0;
  switch(globalHeading){
    case 1:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set north to 1
        north = 1;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set east to 4
        east = 4;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set west to 9
        west = 8;
      }
      //Subtract the sum of north east and west from the value of wall reading
      wallReading -= (north+east+west);
      break;
    case 2:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set south to 2
        south = 2;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set west to 8
        west = 8;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set east to 4
        east = 4;
      }
      //subtract the sum from 15
      wallReading-=(south+east+west);
      break;
    case 4:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set east to 4
        east = 4;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set south to 2
        south = 2;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set north to 1
        north = 1;
      }
      //subtract the sum from 15
      wallReading-=(north+south+east);
      break;
    case 8:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set east to 8
        west = 8;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set north to 1
        north = 1;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set south to 1
        south = 2;
      }
      //subtract the sum from 15
      wallReading-=(west+north+south);
      break;
    }
    return wallReading;
    */
  }


//Instantiate the reflood maze with the most optimistic values
void instantiateReflood(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcCenter(i, j, X);
    }
  }
}

void instantiate(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
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
      maze[Y-1][X-1].walls = 9;
    }
  }
}

void resetToCoord(coord desiredCoord){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcDist(i, j, desiredCoord.x, desiredCoord.y);
    }
  }
}

//Get the most optimistic distance between two coordinates in a grid
int calcDist(byte posx, byte posy, byte desireX, byte desireY){
  int dist = (int) (abs(desireY-posy)+abs(desireX-posx));
  return dist;
}

//Get the most optimistic distance between a given coordinate and a
//3x3 square in the center of a maze of dimension dim (dim must be even)
int calcCenter(int posx, int posy, int dim){
  int center = dim/2;
  int dist = 0;

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
coord bearingCoord(coord currCoord, byte heading){
  coord nextCoord = {0,0};
  switch (heading){
    case 1:
      //code
      nextCoord.x=currCoord.x;
      nextCoord.y=currCoord.y-1;
      break;
    case 2:
      nextCoord.x=currCoord.x;
      nextCoord.y=currCoord.y+1;
      break;
    case 4:
      nextCoord.x=currCoord.x+1;
      nextCoord.y=currCoord.y;
      break;
    case 8:
      nextCoord.x=currCoord.x-1;
      nextCoord.y=currCoord.y;
      break;
  }
  return nextCoord;
}

/*
INPUT: A Coord representing the current coordiante and the robots current heading
OUTPUT: An optimal direction away from the current coordinate.
*/
byte orient(coord currCoord, byte heading){

  coord leastNext = {0,0};
  //This is the absolute largest value possible (dimension of maze squared)
  int leastNextVal = sizeof(maze)*sizeof(maze);
  byte leastDir = heading;

  //If there is a bitwise equivalence between the current heading and the cell's value, then the next cell is accessible
  if((maze[currCoord.y][currCoord.x].walls & heading) != 0){
    //Define a coordinate for the next cell based onthis heading and set the leastNextVal t its value
    coord leastnextTemp = bearingCoord(currCoord, heading);

    if(checkBounds(leastnextTemp)){
      leastNext = leastnextTemp;
      leastNextVal = maze[leastNext.y][leastNext.x].distance;
    }
  }

  for(int i=0; i<sizeof(headings); i++){
    byte dir = headings[i];
    //if this dir is accessible
    if((maze[currCoord.y][currCoord.x].walls & dir) != 0){
      //define the coordiante for this dir
      coord dirCoord = bearingCoord(currCoord,dir);

      if(checkBounds(dirCoord)){
        //if this dir is more optimal than continuing straight
        if(maze[dirCoord.y][dirCoord.x].distance < leastNextVal){
          //update teh value of leastNextVal
          leastNextVal = maze[dirCoord.y][dirCoord.x].distance;
          //update the value of leastnext to this dir
          leastNext = dirCoord;
          leastDir = dir;
        }
      }
    }
  }
  return leastDir;
}

//Take a coordinate and test if it is within the allowable bounds
boolean checkBounds(coord Coord){
  if((Coord.x >= X) || (Coord.y >= Y) || (Coord.x < 0) || (Coord.y < 0)){return false;}else{return true;}
}

/*
INPUT: Coord
OUTPUT: An integer that is the least neighbor
*/
int checkNeighs(coord Coord){
  int minVal =  sizeof(maze)*sizeof(maze);
  for(int i=0; i<sizeof(headings); i++){
    byte dir = headings[i];
    //if this dir is accessible
    if((maze[Coord.y][Coord.x].walls & dir) != 0){
      //Get the coordinate of the accessible neighbor
      coord neighCoord = bearingCoord(Coord, dir);
      //Check the value of the accessible neighbor
      if (checkBounds(neighCoord)){
        //if the neighbore is less than the current recording minimum value, update the minimum value
        //If minVal is null, set it right away, otherwise test
        if(maze[neighCoord.y][neighCoord.x].distance < minVal){minVal = maze[neighCoord.y][neighCoord.x].distance;}
      }
    }
  }
  return minVal;
}

//Given a coordinate, test and return if the coordinate is bounded on three sides
boolean isDead(coord coord){
  boolean deadEnd = false;
  if(checkBounds(coord)){
    byte bounds = maze[coord.y][coord.x].walls;
    //bounds is the integer from the exploratory maze that represents the known walls of the coordinate
    if((bounds == 1)||(bounds == 2)||(bounds == 4) || (bounds == 8)){deadEnd=true;}
  }
  return deadEnd;
}

boolean isEnd(coord Coord, coord DesiredArray[]){
  boolean End = false;
  for(int i=0; i<sizeof(DesiredArray);i++){
    coord Desired = DesiredArray[i];
    if(checkBounds(Coord)){
      if((Coord.x == Desired.x)&&(Coord.y==Desired.y)){
        End = true;
      }
    }
  }
  return End;
}

/*
int readAhead(){
  return 0;
}
  */

/*
INPUT: Coordindate to update, and a direction representing the wall to add
OUTPUT: Update to coordinate adding the wall provided as an argument
*/

void coordUpdate(coord coordinate, byte wallDir){
  if(checkBounds(coordinate)){
    if((maze[coordinate.y][coordinate.x].walls & wallDir) != 0){
      maze[coordinate.y][coordinate.x].walls = maze[coordinate.y][coordinate.x].walls-wallDir;
    }
  }
}

/*
INPUT: Current Robot coordinate
OUTPUT: Update maze for learned walls
*/
void floodFillUpdate(coord currCoord, coord desired[]){
  StackList<coord> entries;
  pf("D\tffUp(%d,%d){\n",currCoord.x,currCoord.y);
  maze[currCoord.y][currCoord.x].walls=readCurrent();
  entries.push(currCoord);

  for(int i=0; i<sizeof(headings); i++){
    byte dir = headings[i];
    //If there's a wall in this dir
    if((maze[currCoord.y][currCoord.x].walls & dir) == 0){
      //create a temporary working coordinate
      coord workingCoord = {currCoord.x,currCoord.y};
      switch(dir){
        case 1:
          workingCoord.y=workingCoord.y-1;
          coordUpdate(workingCoord,2);
          break;
        case 2:
          workingCoord.y=workingCoord.y+1;
          coordUpdate(workingCoord,1);
          break;
        case 4:
          workingCoord.x=workingCoord.x+1;
          coordUpdate(workingCoord, 8);
          break;
       case 8:
         workingCoord.x=workingCoord.x-1;
         coordUpdate(workingCoord,4);
         break;
      }
      //If the workingEntry is a valid entry and not a dead end, push it onto the stack
      if(checkBounds(workingCoord)&&(!isEnd(workingCoord, desired))){
        entries.push(workingCoord);
      }
    }
  }
  pf("sPila:%d\n",entries.count());
  //While the entries stack isn't empty
  while(!entries.isEmpty()){
    //Pop an entry from the stack
    coord workingEntry = entries.pop();
    pf("pila saco (%d,%d)\n",workingEntry.x,workingEntry.y);
    int neighCheck = checkNeighs(workingEntry);
    //If the least neighbor of the working entry is not one less than the value of the working entry
    if(neighCheck+1!=maze[workingEntry.y][workingEntry.x].distance){
      maze[workingEntry.y][workingEntry.x].distance=neighCheck+1;
      for(int i=0;i<sizeof(headings);i++){
        byte dir = headings[i];
        if((maze[workingEntry.y][workingEntry.x].walls & dir) != 0){
          coord nextCoord = bearingCoord(workingEntry,dir);
          if(checkBounds(nextCoord)){
            if(!isEnd(nextCoord, desired)){
              entries.push(nextCoord);
              pf("pila meto (%d,%d)\n",nextCoord.x,nextCoord.y);
            }
          }
        }
      }
    }
  }

}



instruction createInstruction(coord currCoord, coord nextCoord, byte nextHeading){
  float change = 0.0;
  switch(nextHeading){
    case 1:
      if(globalHeading==4){
        change = -90.0;
      }
      if(globalHeading==8){
        change = 90.0;
      }
      if(globalHeading==2){
        change = 180.0;
      }
      break;
    case 2:
      if(globalHeading==4){
        change = 90.0;
      }
      if(globalHeading==8){
        change = -90.0;
      }
      if(globalHeading==1){
        change = 180.0;
      }
      break;
    case 4:
      if(globalHeading==1){
        change = 90.0;
      }
      if(globalHeading==2){
        change = -90.0;
      }
      if(globalHeading==8){
        change = 180.0;
      }
      break;
    case 8:
      if(globalHeading==1){
        change = -90.0;
      }
      if(globalHeading==2){
        change = 90.0;
      }
      if(globalHeading==4){
        change = 180.0;
      }
      break;
  }
  float desiredHeading = actualAng+change;
  //fix over or underflow

  if(((desiredHeading<45.0)||(desiredHeading>315.0))){
    desiredHeading=0.0;
  }
  if((desiredHeading>45.0)&&(desiredHeading<135.0)){
    desiredHeading = 90.0;
  }
  if((desiredHeading>135.0)&&(desiredHeading<225.0)){
    desiredHeading = 180.0;
  }
  if((desiredHeading>225.0)&&(desiredHeading<315.0)){
    desiredHeading = 270.0;
  }

  instruction turnMove = {7.74, desiredHeading};
  return turnMove;
}

void executeInstruction(instruction instruct){
  turn(instruct.desiredHeading);
  moveDist(instruct.desiredPos);
}

void floodFill(coord desired[], coord current, boolean isMoving){
  coord currCoord = current;
  byte heading = globalHeading;
  /*Integer representation of heading
  * 1 = N
  * 4 = E
  * 2 = S
  * 8 = W
  */
  while(maze[currCoord.y][currCoord.x].distance != 0){
    Serial.println("DEBUG0");
    floodFillUpdate(currCoord, desired);
    Serial.println("DEBUG1");
    byte nextHeading = orient(currCoord, heading);
    coord nextCoord = bearingCoord(currCoord, nextHeading);
    Serial.println("DEBUG2");
    if(isMoving){
      //Call createInstruction to push a new instruction to the stack
      instructions.push(createInstruction(currCoord, nextCoord, nextHeading));

      //Pop the next instruction from the instructions queue and execute it
      executeInstruction(instructions.pop());
    }

    //After exectuing the instruction update the values of the local and global variables
    currCoord = nextCoord;
    heading = nextHeading;
    //If the robot has actually moved, update the global position variables
    if(isMoving){
      globalHeading = heading;
      globalCoord = currCoord;
    }
  }
  //Set the global end as the current coordinate.
  globalEnd = currCoord;
}



void turn(float desiredPosition){
    pf("\tgirarA:%d\n",desiredPosition);
    actualAng=desiredPosition;
}

void moveDist(float dist){
    pf("\tMoverDist:%d\n",dist);
}



void reflood(){
  //Refill the maze for most optimistic values, but now the maze has walls
  instantiateReflood();

  //Run flood fill but without actual motion
  coord desired[] = {{X-1,Y-1},{X-1,Y},{X,Y-1},{X,Y}};
  coord currCoord = {0,0};
  floodFill(desired, currCoord, false);

  //Now, the robot is still at the start, but the maze distance values have been updated with the walls discovered
  //So we follow the maze creating instructions
  createSpeedQueue();
  //We now have a queue of instructions.

}

//Trace the maze back to the end creating instructions and adding them to the queue
void createSpeedQueue(){
  coord workingCoord = globalCoord;
  byte workingDir = globalHeading;
  int workingDist = 7.74;
  while((workingCoord.x!=globalEnd.x)&&(workingCoord.y!=globalEnd.y)){
    byte optimalDir = orient(workingCoord, workingDir);

    //If the direction is the same, accumulate distance
    if(optimalDir==workingDir){
      workingDist+=7.74;
    }else{
      //if the optimal is different from the working, add the working and the accumulated distance
      instruction nextInstruction = {workingDist, optimalDir};
      instructions.push(nextInstruction);
      //Reset the distance to one square and update the workingDir
      workingDist = 7.74;
      workingDir = optimalDir;
    }

    //update workingCoord to the next optimal coord
    workingCoord = bearingCoord(workingCoord, optimalDir);
  }
}
byte n_loop=0;
void loop(){

  pf("\t\tINI_LOOP%d\n",n_loop++);
  coord desired[] = {{4,5},{4,6},{4,4},{5,5},{5,6},{5,4},{6,5},{6,6},{6,4}};

  floodFill(desired, globalCoord, true);

  printMaze();
  Serial.println("VOLVIENDO AL ORIGEN");
  coord returnCoord[] = {{0,12}};
  resetToCoord(returnCoord[0]);
  //Run fill to return to the start coord
  floodFill(returnCoord, globalCoord, true);
  printMaze();
  //Reflood the maze
  reflood();

  //Pop instructions from the front of the queue until its empty.
  Serial.println("RESULTADO FINAL");
  Serial.println("Instrucciones a ejecutar:");
  while(!instructions.isEmpty()){
    executeInstruction(instructions.pop());
  }
}
