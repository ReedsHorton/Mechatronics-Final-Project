/*
 * AutoNav.c
 *
 * This program navigates a robot from point A to point B detecting 
 * and avoiding obstacles in its path through the use of a BFS 
 * algorithm. It is programmed for a BL2600 board, US Digital 
 * encoders, and a robot specifically made for this project. All 
 * distances are in Centimeters and all angles are in degrees unless 
 * otherwise stated. 
 *
 * Elias Bello, Scott Calnan, and Reed Horton
 * Obstacle Course Mapping and Navigation
 * ENGS 147, 18S
 * Dartmouth College
 */

#define DATA  0x00ff
#define RD     8
#define WR     9
#define CS1    10
#define CS2    11
#define YX     12
#define CD     13

#define BP_RESET          0X01      // reset byte pointer
#define EFLAG_RESET   0X86      // reset E bit of flag register
#define CNT         0x01       // access control register
#define DAT         0x00       // access data register
#define BP_RESETB     0X81      // reset byte pointer (x and y)
#define CLOCK_DATA    2     // FCK frequency divider
#define CLOCK_SETUP     0X98    // transfer PR0 to PSC (x and y)
#define INPUT_SETUP       0XC7    // enable inputs A and B (x and y)
                        // set indexing function to load OL and
                                    // A and B enable gate TWP 4/2/12
#define QUAD_X1           0XA8 // quadrature multiplier to 1 (x and y)
#define QUAD_X2           0XB0 // quadrature multiplier to 2 (x and y)
#define QUAD_X4           0XB8 // quadrature multiplier to 4 (x and y)
#define CNTR_RESET        0X02    // reset counter
#define CNTR_RESETB       0X82    // reset counter (x and y)
#define TRSFRPR_CNTR      0X08  // transfer preset register to counter
#define TRSFRCNTR_OL      0X90    // transfer CNTR to OL (x and y)
#define XYIDR_SETUP     0XE1 // set index cntrl register to active low
                        // input. index input is pulled up to +5V
                            // in hardware to disable index functions
                                    // TWP 4/2/2012
#define HI_Z_STATE         0xFF

#define HI_Z_STATE         0xFF
#define GridSize    16          //Size of the grid AxA
#define SquareSize    22.83     //Length of a side of a grid space
#define WheelRadius    13.3     //Turning radius of the car 
#define Filter    25            //Filter for extraneous signal noise


//Force functions to be compiled to extended memory.  Helps when the
// program gets large
#memmap xmem

//Import Library for BL2600
#use BL26XX.lib


int EncRead(int channel, int reg);
void EncWrite(int channel, int data, int reg);
long encoder(int j);
int init(void);
void driveToFwd(float dist, float *x, float *y, float heading);
float Turn(float angle, float *heading, float *x, float *y);
void characterize(long ipos, int grid[][30], float currX, 
                                          float currY, float heading);
void rotate360(void);
void BFS(float x, float y, int grid[][GridSize], 
            int goalX, int goalY, int *x1, int *y1, int *x2, int *y2);
void CalculateAngleDist(float *angle2, float *dist2, 
          int gridX, int gridY, int nextX, int nextY, float heading);

int main(void){
//structure to hold the car's xy coordinates and heading
struct ROBOT
{
  float x;
  float y;
  float heading;
};

//strucutre to hold two XY coordinates
struct Point
	{
  int x;
  int y;
  int prevX;
  int prevY;
};

  unsigned long int T0;
  float dist, angle, angle2, dist2;
  float currX, currY, nextSquare;
  int i, f, test, grid[GridSize][GridSize], goalX, goalY;
  int gridX, gridY, solved;
  struct ROBOT car;// car;
  struct Point next;
  long ipos;

  //INITIALIZE SUBSYSTEMS
  brdInit();        //default processor configuration
  digOutConfig(0xff00);
  anaOutConfig(1, DAC_SYNC);//Channel 1 Bipolar operation
  anaOutPwr(1);       //Enable BL2600 power supply to drive D/A 
  anaOutVolts(1, 0);    //Send 0 volts to prevent indeterminate state
  anaOutVolts(0, 0);    //Send 0 volts to prevent indeterminate state
  anaOutVolts(2, 0);    //Send 0 volts to prevent indeterminate state
  anaOutStrobe();
  digOut(CS1,1);
  digOut(CS2,1);
  digOut(RD,1);
  digOut(WR,1);
  anaInConfig(0, DIFF_MODE); //Configure analog input  0 for Diff mode
  init();

  //Set initial car parameters, heading and cartesian coordinates
  car.heading = 0;
  car.x = (1+1)*SquareSize-SquareSize/2;
  car.y = (1+1)*SquareSize-SquareSize/2;

  //Get the grid coordinates for the car
  gridX = car.x/SquareSize;
  gridY = car.y/SquareSize;

  //goal grid space
  goalX = 15; //top right corner
  goalY = 15;

  //initialize
  solved = 0;
  next.x = 0;
  next.y = 0;
  next.prevX = 0;
  next.prevY = 0;


  //Set grid to empty
  for (i=0; i<GridSize; i++){
    for (f=0; f<GridSize; f++){
      grid[i][f] = 0;
    }
  }

  //rotate encoder to avoid skip from 0-240 counts
  rotate360();
  ipos = encoder(0); 

  //Loop for navigation
  while(solved != 1){
    //Characterize the map
    characterize(ipos, grid, car.x, car.y, car.heading);  
    //print map
    for (i=GridSize-1; i>=0; i--){
  		for (f=0; f<GridSize; f++){
        if (f == gridX && i == gridY) {
          printf("O "); //car position
        }
        else if (f == goalX && i == goalY) {
          printf("H "); //goal
        }
        else if(grid[f][i] > Filter ) {
  	 			printf("X "); // obstacle
    		}
    		else {
  	 			printf(". "); //empty grid space
    		}
      }
  		printf("\n");
		}
		printf("\n");               //Search the map for the next move
    BFS(car.x, car.y, grid, goalX, 
                  goalY, &next.x, &next.y, &next.prevX, &next.prevY);
                            //calculate distance/angle needed to move
    CalculateAngleDist(&angle, &dist, gridX,
                                  gridY, next.x, next.y, car.heading);
    //Turn to that angle
    car.heading = Turn(angle, &car.heading, &car.x, &car.y);
    //re-characterize map from new vantage point
    characterize(ipos, grid, car.x, car.y, car.heading);
    //while the next position now has an obstacle in it
    while(grid[next.x][next.y] > Filter){
      //re-search for next move, turn to, and character
      BFS(car.x, car.y, grid, goalX, goalY, &next.x, &next.y, 
                                            &next.prevX, &next.prevY);
      CalculateAngleDist(&angle, &dist, gridX, gridY, next.x, 
                                                next.y, car.heading);
      car.heading = Turn(angle, &car.heading, &car.x, &car.y);
      characterize(ipos, grid, car.x, car.y, car.heading);
    }
    //drive to next grid space
    driveToFwd(dist, &car.x, &car.y, car.heading);
    gridX = car.x/SquareSize;                //update grid coordinates
    gridY = car.y/SquareSize;
    if (gridX == goalX && gridY == goalY){   //check if maze is sovled
      solved = 1;
    }
  }
}

/***************** CalculateAngleDist() *****************
* This function takes the heading the car is currently at, the grid
* space the car is currently at, the grid space the car is going to
* and pointers to the distance the car should move/angle it should 
* turn to and returns nothing.
*/
void CalculateAngleDist(float *angle2, float *dist2, int gridX, 
                      int gridY, int nextX, int nextY, float heading){
  //Going to a grid space diagonal from the current space
  if(nextX != gridX && nextY != gridY){
    //distance is hypotenuse from 45,45,90 triangle
    *dist2 = sqrt(2*(SquareSize*SquareSize));
    //calculate heading needed to get to that space
    if(nextY>gridY && nextX>gridX){
      *angle2 = 45;
    }
    else if(nextY<gridY && nextX>gridX){
      *angle2 = 135;
    }
    else if (nextY<gridY && nextX<gridX) {
      *angle2 = 225;
    }
    else {
      *angle2 = 315;
    }
  }
  //Going to a grid space adjacent in X or Y to current gridspace
  else {
    *dist2 = SquareSize;
    if(nextY>gridY){
      *angle2 = 0;
    }
    else if(nextY <gridY){
      *angle2 = 180;
    }
    else if (nextX>gridX) {
      *angle2 = 90;
    }
    else {
      *angle2 = 270;
    }
  }
  //calculate angle needed to turn to, only turn clockwise
  *angle2 = *angle2 - heading;
  if (*angle2 < 0){
    *angle2 = *angle2+360;
  }
}

/***************** BFS() *****************
* This function takes the current map of the grid, the goal XY grid 
* space, the current cartesian coordinate of the robot, and pointers 
* to the next x/y grid space to move to. It conducts a Breadth First
* Search of the grid, not including the robots own position and any
* grid space with an obstacle, to find a short (but not nessicarily 
* the shortest) path to the goal. It then sets the pointers to the 
*next x/y grid space to next gridsace in the path and returns nothing.
*/
void BFS(float x, float y, int grid[][GridSize], int goalX, 
                      int goalY, int *x1, int *y1, int *x2, int *y2) {
  //Structure to hold the current gridspace and the previous gridspace
  //in the tree
  struct Point
	{
  int x;
  int y;
  int prevX;
  int prevY;
  } ;

  int X, Y, i, j, goal, start;
  int curr, end, visited[GridSize][GridSize]; 
  struct Point queue[GridSize*GridSize];
  struct Point path[GridSize*GridSize/2];
  struct Point steps[GridSize][GridSize];

  //Set grid of visited cells to not visited
  for (i=0; i<GridSize; i++){
    for (j=0; j<GridSize; j++){
      visited[i][j] = 0;
    }
  }

  //calculate current grid space for robot
  X = x/SquareSize;
  Y = y/SquareSize;

  //initialize variables 
  curr = 0;
  end = 0;
  goal = 0;
  start = 0;

  //Put the current grid space onto the queue and set it to visited
  queue[end].x = X;
  queue[end].y = Y;
  visited[X][Y] = 1;
  end++; //keep track of array position of end of list
  //while there are more points to look at not looking at the goal 
  while(end != curr && goal != 1){ 
    //for all its neighbors that are on the grid
    for(i=queue[curr].x-1; i < queue[curr].x+2; i++){
      for(j=queue[curr].y-1; j < queue[curr].y+2; j++){
        if (i>=0 && j>=0 && j<GridSize && i<GridSize){
  //if the gridspace does not have an obstacle and hasn't been visited
        	if(grid[i][j] < (Filter) && visited[i][j] == 0){
	          //add it to the queue, remember who added it, 
                                              //and set it to visited
            queue[end].x = i;
	          queue[end].y = j;
	          queue[end].prevX = queue[curr].x;
            steps[i][j].prevX = queue[curr].x;
	          queue[end].prevY = queue[curr].y;
            steps[i][j].prevY = queue[curr].y;
	          visited[i][j] = 1;
            end++; //keep track of end
	        }
        }
      }
    }
    curr++;//get the next point in the queue and check if its the goal
    if(queue[curr].x == goalX && queue[curr].y == goalY){
      goal = 1;
    }
  }

  //from the goal, work back to the start and save the path
  i = 0;
  path[i]=queue[curr];
  (steps[queue[curr].x][queue[curr].y]).prevX = queue[curr].prevX;
  (steps[queue[curr].x][queue[curr].y]).prevY = queue[curr].prevY;
  if (path[i].x == X && path[i].y == Y) {
  	start = 1;
  }

  while (start == 0){
    path[i+1].x = steps[path[i].x][path[i].y].prevX;
    path[i+1].y = steps[path[i].x][path[i].y].prevY;
    i++;
    if (path[i].x == X && path[i].y == Y) {
  		 start = 1;
  	}

  }

  //get the position for the first one or two moves on the path
  if(i>1){
  *x1 = path[i-1].x;
  *y1 = path[i-1].y;
  *x2 = path[i-2].x;
  *y2 = path[i-2].y;
  }
  else if (i>0) {
   *x1 = path[i-1].x;
  	*y1 = path[i-1].y;
  }
  else {
  printf("error calulating next move\n");
  }
}

/***************** rotate360() *****************
* This function rotates the IR sensor 360 degrees to start the
* program. The purpose is to skip over the encoder counts where the
* encoder sometimes skips from 240 back to 0. It does this by spinning
* until it has travled 360 degrees worth of encoder counts.
*/
void rotate360(void) {
  long  pos, oldpos, diff;
  unsigned long int T0;

 	anaOutVolts(2, -2);     //Spin motor
 	anaOutStrobe();
  pos = encoder(0);       //Get Position
  diff = 0;
  oldpos = 0;
  
  while (diff < 1440 ){          //Less than half a full turn
    oldpos = pos;
    pos = encoder(0);
    if ((oldpos > 200) && (pos < 100)) {    //account for a skip
      oldpos = 254 - oldpos;
    }
    if ((pos > 200) && (oldpos < 100)) {
      oldpos = 254 + oldpos;
    }
    diff = diff + (pos - oldpos);   //calculate counts moved
	}

  anaOutVolts(2, 0);          //Stop the motor
 	anaOutStrobe();

  T0 = TICK_TIMER;        //Wait .03125 seconds
	while ((TICK_TIMER - T0) < 32) {}
}

/***************** characterize() *****************
* This function takes the initial IR position (in counts), the current
* map, and the current X/Y position and heading of the robot. The
* function then rotates the IR sensor 180 degrees and recorded IR
* Voltages. If the IR sensor detects an object betwen 60cm and .75cm
* away it calculates what grid space the object is in by taking the
* distance from the IR to the object and angle the IR is currently at.
*/
void characterize(long ipos, int grid[][GridSize], float currX, 
                                        float currY, float heading) {
  unsigned long int T0;
  int x, y, i, f;
  float sens, angle, distance;
  long  pos, oldpos, diff;

  //initialze
  angle = 90;
  diff = 0;
  pos = 0;
  oldpos = 0;


  while(angle > .1 && angle < 179.9) {   //in between 0 and 180 for IR
    pos = encoder(0);
    if (pos < (ipos+360)) {           //Turn IR clockwise
      anaOutVolts(2, -2);
      anaOutStrobe();
      while (diff < 720 && pos < ipos+720) {   //Less than half a turn
        sens = anaInDiff(0, 0);         //calculate distance to object
        if (sens > .62) {
          sens = -log((sens-.6171)/2.8400)*10.9775;
        }
        oldpos = pos;
        pos = encoder(0);
        diff = diff + (pos - oldpos);
        angle = (diff)*360/1440;
        // get angle of IR (from straight ahead)
        //If the object is within the current gridspace, but it in the 
              //gridspace directly in front of the robot
        if (sens*.7072 < SquareSize && sens > 1) {
        	sens = (sens + (SquareSize-sens*.7072)+.01)/.7072;
        }

        //calculate the grid position
        if(sens < 60 && sens > .75) {
          y = (int)(currY +
              (cos(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          x = (int)(currX + 
              (sin(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;

          //If the object is within the grid, place it their
          if(x>=0 && x<GridSize){
            if(y>=0 && y<GridSize){
              grid[x][y]++;
            }
          }
        }
      }
      diff = 0;
    }

    //same process for the IR turning the opposite way 
                                    //(angle is slightly different)
    else if (pos > (ipos-360)) {
      anaOutVolts(2, 2);
      anaOutStrobe();
      while (diff > -720 && pos >= ipos) {
        sens = anaInDiff(0, 0);
        oldpos = pos;
        pos = encoder(0);
        if (sens > .62) {
          sens = -log((sens-.6171)/2.8400)*10.9775;
        }
        diff = diff + (pos - oldpos);
        angle = 180 - (-diff)*360/1440;
        if (sens*.7072 < SquareSize && sens > 1) {
        	sens = (sens + (SquareSize-sens*.7072)+.01)/.7072;
        }
        if(sens < 60 && sens > .75) {
          y = (int)(currY + 
              (cos(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          x = (int)(currX + 
              (sin(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          if(x>=0 && x<GridSize){
            if(y>=0 && y<GridSize){
              grid[x][y]++;
            }
          }
        }
      }
      diff = 0;
    }
  }
  anaOutVolts(2, 0);
  anaOutStrobe();
}


/***************** driveToFwd() *****************
* This function takes the distance to drive, the current x/y
* coordinate of the robot, and its heading. It drives the car the 
* desired distance by counting encoder ticks, while also comparing
* enocder ticks for each wheel and asjusting Power to the left wheel
* so as to maintain straight travel. Once close to desired distance,
* the robot increments distance travlled slowly in order to not 
* overshoot the target. Finally, it calculates the change in
* cartesian coordinates for the robot.
*/
void driveToFwd(float dist, float *x, float *y, float heading){
  long turns, rollover, negRollover, Rwheel, Rdesired, Lwheel;
  long oldR, oldL, diffL, diffR, diff;
  int timesRollover, timesRolloverDes, close, rolls;
  unsigned long int T0;
  float driveR, driveL, rolloverDist, dist1;

  //rollover values for the encoders
  rollover = 32767;
  negRollover= -32768;
  rolloverDist = 58.7291; //cm to rollover

  //initialize
  timesRollover = 0;
  timesRolloverDes = 0;
  close = 0;
  driveR = 2;
  driveL = 1.86;

  turns = (long) dist*1115.95; //counts/cm, number of counts needed
  Rwheel = encoder(3);

  //If counts to rollover<counts needed, calculate # of rollovers
  if((rollover-Rwheel) < turns){  
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }

  //Final encoder value desired
  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;

  //drive the motors
  anaOutVolts(1, driveR); 
  anaOutVolts(0, driveL);
  anaOutStrobe();

  while(close!=1){
    oldR = Rwheel;      //calculate the change in encoders
    oldL = Lwheel;
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR+diffL;

    //check for a rollover
    if(oldR>0 && Rwheel < 0) {
      timesRollover++;
    }

    //if within ~1cm of final target, start precision movement
    if(timesRollover==timesRolloverDes){
      if (Rdesired-Rwheel < 1000){
        close=1;
      }
    }

    //check for straight driving
    if(close!=1){
      if(diff > .99){     //If the right wheel moves faster
        driveL = driveL+.01;  //increase left wheel power
      }
      if(diff < -.99){      //If the left wheel moves faster
        driveL = driveL-.01;  //decrease left wheel power
      }
      anaOutVolts(1, driveR); //update power to motors
      anaOutVolts(0, driveL);
      anaOutStrobe();

      T0 = TICK_TIMER;        //Wait .0039 seconds
      while ((TICK_TIMER - T0) < 4) {}
    }
  }

  //Once close, stop the car
  anaOutVolts(1, 0);
  anaOutVolts(0, 0);
  anaOutStrobe();

  
  T0 = TICK_TIMER;        //Wait .0625 seconds
  while ((TICK_TIMER - T0) < 64) {}
  Rwheel = encoder(3);

  //Drive the wheels for .0078 seconds and check distance travelled
    //stop once within .3mm
  while(Rdesired-Rwheel > 40) {
    anaOutVolts(1, driveR);
    anaOutVolts(0, driveL);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait 0.0078 seconds
    while ((TICK_TIMER - T0) < 8) {}
    anaOutVolts(1, 0);
    anaOutVolts(0, 0);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0625 seconds
    while ((TICK_TIMER - T0) < 64) {}
    
    //maintain straight driving
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR+diffL;
    if(diff > .99){     //If the right wheel moves faster
      driveL = driveL+.01;  //increase left wheel power
    }
    if(diff < -.99){      //If the left wheel moves faster
      driveL = driveL-.01;  //decrease left wheel power
    }
  }
  //cauclate new x/y coordinate
  *x = sin(heading/180*3.1415)*dist + *x;
  *y = cos(heading/180*3.1415)*dist + *y;
}

/***************** Turn() *****************
* This function takes the angle to turn to, the cars current heading
* and the cars current x/y coordinate. It turns the car the angle 
* specified, and then updates the new heading and new coordinates.
* It returns the new heading of the car.
*/
float Turn(float angle, float *heading, float *x, float *y){
  long turns, rollover, negRollover, Rwheel, Rdesired, Lwheel;
  long oldR, oldL, diffL, diffR, diff;
  int timesRollover, timesRolloverDes, close, rolls;
  unsigned long int T0;
  float driveR, driveL, rolloverDist, dist1, dist;

  //rollover counts for the car
  rollover = 32767;
  negRollover= -32768;
  rolloverDist = 58.7291; //cm to rollover

  //initialize variables
  timesRollover = 0;
  timesRolloverDes = 0;
  close = 0;
  driveR = 2;
  driveL = 1.84;

  //calculate distance needed to travel and encoder counts
  dist = (angle/180)*3.14159*11.05;
  turns = (long) dist*1115.95;///1.006780792;

  Rwheel = encoder(3);

  //If counts to rollover<counts needed, calculate # of rollovers
  if((rollover-Rwheel) < turns){
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }
  //desired encoder count
  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;

  anaOutVolts(1, driveR);   //drive wheels
  anaOutVolts(0, -driveL);
  anaOutStrobe();
  while(close!=1){
    oldR = Rwheel;      //calculate the change in encoders
    oldL = Lwheel;
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR-diffL;

    //check for a rollover
    if(oldR>0 && Rwheel < 0) {
      timesRollover++;
    }
    if(timesRollover==timesRolloverDes){
      if (Rdesired-Rwheel < 1000){ //within ~1cm of final
        close=1;
      }
    }

    //drive straight
    if(close!=1){
      if(diff < .99){     //If the right wheel moves faster
        driveL = driveL-.01;  //increase left wheel power
      }
      if(diff > -.99){      //If the left wheel moves faster
        driveL = driveL+.01;  //decrease left wheel power
      }
      anaOutVolts(1, driveR);   //should ramp up here
      anaOutVolts(0, -driveL);
      anaOutStrobe();
      T0 = TICK_TIMER;        //Wait .0156 seconds
      while ((TICK_TIMER - T0) < 4) {}
    }
  }

  //once close, stop
  anaOutVolts(1, 0);
  anaOutVolts(0, 0);
  anaOutStrobe();

  //until within .5mm of goal, increment distance moved 
  T0 = TICK_TIMER;        //Wait .0625seconds
  while ((TICK_TIMER - T0) < 64) {}
  Rwheel = encoder(3);
  while(Rdesired-Rwheel > 55) {
    anaOutVolts(1, driveR);
    anaOutVolts(0, -driveL);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0078 seconds
    while ((TICK_TIMER - T0) < 8) {}
    anaOutVolts(1, 0);
    anaOutVolts(0, 0);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0625 seconds
    while ((TICK_TIMER - T0) < 64) {}
    
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR-diffL;

    //drive straight
    if(diff < .99){     //If the right wheel moves faster
        driveL = driveL-.01;  //increase left wheel power
      }

      if(diff > -.99){      //If the left wheel moves faster
        driveL = driveL+.01;  //decrease left wheel power
    }
  }

  //calculate new x/y coordinate of IR sensor given the turn
  *x = *x + (sin((angle+*heading)/180*3.14159265) - 
                          sin(*heading/180*3.13159265))*WheelRadius;
  *y = *y + (cos((angle+*heading)/180*3.14159265) - 
                          cos(*heading/180*3.14159265))*WheelRadius;
  //calculate the new heading of the robot
  if (*heading+angle > 360){
    *heading = *heading+angle-360;
  }
  else {
    *heading = *heading+angle;
  }
 return(*heading);
}

/***************** encoder() *****************
* This function takes encoder port to read and returns the counts at
* that encoder. It does this by using the given EncRead and EncWrite
* functions
*/
long encoder(int j) {
  int asb, bsb, csb;
  long position;
  asb = 0;
  bsb = 0;
  csb = 0;
  position = 0;

  EncWrite(j, TRSFRCNTR_OL, CNT);
  EncWrite(j,BP_RESETB,CNT);
  asb = EncRead(j,DAT);
  bsb = EncRead(j,DAT);
  csb = EncRead(j,DAT);
  position  = (long)asb;      // least significant byte
  position += (long)(bsb << 8);
  position += (long)(csb <<16);

  return position;
}


int init(void)
{

  int fail;
  int i,j,k,delayvar;
  fail = 0;

  for (i = 0; i<4; i++)
  {
    EncWrite(i, XYIDR_SETUP,CNT);   // Disable Index
    EncWrite(i,EFLAG_RESET,CNT);
    EncWrite(i,BP_RESETB,CNT);
    EncWrite(i,CLOCK_DATA,DAT);
      EncWrite(i,CLOCK_SETUP,CNT);
      EncWrite(i,INPUT_SETUP,CNT);
      EncWrite(i,QUAD_X4,CNT);

      EncWrite(i,BP_RESETB,CNT);
      EncWrite(i,0x12,DAT);
      EncWrite(i,0x34,DAT);
      EncWrite(i,0x56,DAT);

      EncWrite(i,TRSFRPR_CNTR,CNT);
      EncWrite(i,TRSFRCNTR_OL,CNT);

      EncWrite(i,BP_RESETB,CNT);
      printf("written = %d, read = %d\n",0x12,EncRead(i,DAT));
      printf("written = %d, read = %d\n",0x34,EncRead(i,DAT));
      printf("written = %d, read = %d\n",0x56,EncRead(i,DAT));

      // Reset the counter now so that starting position is 0  TWP 4/2/12
      EncWrite(i,CNTR_RESET,CNT);
    }
    return fail;
}

// channel is an int from 0 to 3 indicating which encoder
// reg is an int which is 1 or 0 indicating whether control 
          //or data is desired
int EncRead(int channel, int reg)
{
  int EncData;
    int i, delayvar;
    EncData = 0;

    digOutConfig(0xff00); // set data lines as inputs and everything else as outputs

    // select which chip
    if (channel <= 1)
      digOut(CS1,0);
    else
      digOut(CS2,0);

    // Select control or data register
    digOut(CD,reg);

    // select which channel, X or Y
    if ((channel == 0) | (channel == 3) )
      digOut(YX,0);
    else
      digOut(YX,1);
      // assert Read low
    digOut(RD,0);

    EncData = digInBank(0);     // read the data from the data lines

    //deassert read reads the data.  Deassert, delay to allow rise
   // then deselect chips
    digOut(RD,1);

    digOut(CS1,1);
    digOut(CS2,1);

    return EncData;
}

void EncWrite(int channel, int data, int reg)
{
  int i, delayvar;

    // select which chip - channel 0 & 1 are chip 1 and channel 2 & 3 are chip 2
    if (channel <= 1)
      digOut(CS1,0);
    else
      digOut(CS2,0);
    // select which channel, X or Y  X = 0 and 2, Y = 1 and 3

    digOut(CD,reg);

    if ((channel == 0) | (channel == 3) )
      digOut(YX,0);
    else
      digOut(YX,1);
    // assert write
    digOut(WR,0);   //First assert WR before driving outputs to avoid bus
              //contention with encoder board  TWP 4/2/12

    digOutConfig(0xffff);// set all digI/O lines as outputs
    digOutBank((char)0,(char)data);
    // deassert write
    digOut(WR,1);

    // deselect chip
    digOut(CS1,1);
    digOut(CS2,1);
    //Set all outputs to 1 so that open collector transistor is off
    digOutBank((char)0,(char)HI_Z_STATE);
    digOutConfig(0xff00);
}