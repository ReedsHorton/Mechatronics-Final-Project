
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
#define QUAD_X1           0XA8    // quadrature multiplier to 1 (x and y)
#define QUAD_X2           0XB0    // quadrature multiplier to 2 (x and y)
#define QUAD_X4           0XB8    // quadrature multiplier to 4 (x and y)
#define CNTR_RESET        0X02    // reset counter
#define CNTR_RESETB       0X82    // reset counter (x and y)
#define TRSFRPR_CNTR      0X08    // transfer preset register to counter
#define TRSFRCNTR_OL      0X90    // transfer CNTR to OL (x and y)
#define XYIDR_SETUP     0XE1    // set index cntrl register to active low
                        // input. index input is pulled up to +5V
                                    // in hardware to disable index functions
                                    // TWP 4/2/2012
#define HI_Z_STATE         0xFF

#define HI_Z_STATE         0xFF
#define GridSize    16
#define SquareSize    22.83
#define WheelRadius    13.3
#define Filter    25


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
void characterize(long ipos, int grid[][30], float currX, float currY, float heading);
void rotate360(void);
void BFS(float x, float y, int grid[][GridSize], int goalX, int goalY, int *x1, int *y1, int *x2, int *y2);
void CalculateAngleDist(float *angle2, float *dist2, int gridX, int gridY, int nextX, int nextY, float heading);

int main(void){

struct ROBOT
{
   float x;
   float y;
   float heading;
};

struct Point
	{
  int x;
  int y;
  int prevX;
  int prevY;
} ;

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
  anaOutPwr(1);       //Enable BL2600 power supply to drive D/A channels
  anaOutVolts(1, 0);    //Send 0 volts to prevent indeterminate state
  anaOutVolts(0, 0);    //Send 0 volts to prevent indeterminate state
  anaOutVolts(2, 0);    //Send 0 volts to prevent indeterminate state
  anaOutStrobe();
  digOut(CS1,1);
  digOut(CS2,1);
  digOut(RD,1);
  digOut(WR,1);
  anaInConfig(0, DIFF_MODE); // Configure analog input channel 0 for Diff mode
  init();


  car.heading = 0;
  car.x = (1+1)*SquareSize-SquareSize/2;
  car.y = (1+1)*SquareSize-SquareSize/2;

  gridX = car.x/SquareSize;
  gridY = car.y/SquareSize;

  goalX = 15; //top right corner
  goalY = 15;

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


  //grid[2][2] = 5;
  //grid[3][2] = 5;
  //grid[2][3] = 5;
  //grid[2][4] = 5;

  rotate360();
  ipos = encoder(0);

  while(solved != 1){
    characterize(ipos, grid, car.x, car.y, car.heading);

    for (i=GridSize-1; i>=0; i--){
    		for (f=0; f<GridSize; f++){
            if (f == gridX && i == gridY) {
            	printf("O ");
            }
            else if (f == goalX && i == goalY) {
            	printf("H ");
            }
            else if(grid[f][i] > Filter ) {
    	 			printf("X ");
      		}
      		else {
    	 			printf(". ");
      		}
            //else {
        		//printf("%d ",grid[f][i]);
            //}
         }
    		printf("\n");
  		}
  		printf("\n");



    BFS(car.x, car.y, grid, goalX, goalY, &next.x, &next.y, &next.prevX, &next.prevY);
    printf("Going to (%d,%d)\n", next.x, next.y);
    CalculateAngleDist(&angle, &dist, gridX, gridY, next.x, next.y, car.heading);
    //angle = angle2;
    //dist = dist2;
    car.heading = Turn(angle, &car.heading, &car.x, &car.y);
    characterize(ipos, grid, car.x, car.y, car.heading);

    while(grid[next.x][next.y] > Filter){
      BFS(car.x, car.y, grid, goalX, goalY, &next.x, &next.y, &next.prevX, &next.prevY);
      printf("Psych!!! Going to (%d,%d)\n", next.x, next.y);
      CalculateAngleDist(&angle, &dist, gridX, gridY, next.x, next.y, car.heading);
      //angle = angle2;
      //dist = dist2;
      car.heading = Turn(angle, &car.heading, &car.x, &car.y);
      characterize(ipos, grid, car.x, car.y, car.heading);
      printf("New Grid!!\n\n");
      for (i=GridSize-1; i>=0; i--){
        for (f=0; f<GridSize; f++){
            if (f == gridX && i == gridY) {
              printf("O ");
            }
            else if (f == goalX && i == goalY) {
              printf("H ");
            }
            else if(grid[f][i] > Filter ) {
            printf("X ");
          }
          else {
            printf(". ");
          }
            //else {
            //printf("%d ",grid[f][i]);
            //}
         }
        printf("\n");
      }
      printf("\n");

    }


    //printf("Car at (%f,%f) Angle%f\n", car.x, car.y, car.heading);
    driveToFwd(dist, &car.x, &car.y, car.heading);
    printf("Car at (%f,%f) Angle%f\n", car.x, car.y, car.heading);
    gridX = car.x/SquareSize;
    gridY = car.y/SquareSize;
    printf("Now in position (%d,%d)\n", gridX, gridY);
    if (gridX == goalX && gridY == goalY){
      solved = 1;
      printf("Solved the mf maze!!!\n");
    }

  }
}

void CalculateAngleDist(float *angle2, float *dist2, int gridX, int gridY, int nextX, int nextY, float heading){

  if(nextX != gridX && nextY != gridY){
      *dist2 = sqrt(2*(SquareSize*SquareSize));
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
    printf("Going to (%d,%d), angle:%f, heading %f \n", nextX, nextY, *angle2, heading);
    *angle2 = *angle2 - heading;
    if (*angle2 < 0){
      *angle2 = *angle2+360;
    }
    printf("Going %f cm after turn %f degrees\n", *dist2, *angle2);
}


void BFS(float x, float y, int grid[][GridSize], int goalX, int goalY, int *x1, int *y1, int *x2, int *y2) {
  struct Point
	{
  int x;
  int y;
  int prevX;
  int prevY;
  } ;

  int X, Y, i, j, goal, start;
  int curr, end, visited[GridSize][GridSize] ;
  struct Point queue[GridSize*GridSize];
  struct Point path[GridSize*GridSize/2];
  struct Point steps[GridSize][GridSize];

  //printf("before the 0\n");
  //Set grid to empty
  for (i=0; i<GridSize; i++){
    for (j=0; j<GridSize; j++){
      visited[i][j] = 0;
    }
  }

  X = x/SquareSize;
  Y = y/SquareSize;


  curr = 0;
  end = 0;
  goal = 0;
  start = 0;

  queue[end].x = X;

  queue[end].y = Y;
  //printf("(%d,%d)\n", X, Y);
  visited[X][Y] = 1;
  printf("After\n");

  end++;


  while(end != curr && goal != 1){
    //for all its neighbors
   //printf("Entering for loop for (%d,%d)\n", queue[curr].x, queue[curr].y);
    for(i=queue[curr].x-1; i < queue[curr].x+2; i++){
      //printf("First Loop - (%d,%d)\n",queue[curr].x-1,queue[curr].x+1);
      for(j=queue[curr].y-1; j < queue[curr].y+2; j++){
      //printf("looking at point (%d,%d)\n",i,j);
        if (i>=0 && j>=0 && j<GridSize && i<GridSize){
        	//printf("more than 0, %d, %d\n", grid[i][j], visited[i][j]);
        	if(grid[i][j] < (Filter) && visited[i][j] == 0){
	          queue[end].x = i;
	          queue[end].y = j;
             //printf("Adding: (%d,%d)\n", queue[end].x, queue[end].y);
	          queue[end].prevX = queue[curr].x;
             steps[i][j].prevX = queue[curr].x;
	          queue[end].prevY = queue[curr].y;
             steps[i][j].prevY = queue[curr].y;
	          visited[i][j] = 1;
             end++;

	        }
        }
      }
    }
    curr++;
    if(queue[curr].x == goalX && queue[curr].y == goalY){
      goal = 1;
    }
  }
  printf("Found\n");
  i = 0;
  path[i]=queue[curr];
  (steps[queue[curr].x][queue[curr].y]).prevX = queue[curr].prevX;
  (steps[queue[curr].x][queue[curr].y]).prevY = queue[curr].prevY;
  if (path[i].x == X && path[i].y == Y) {
  	start = 1;
  }

  while (start == 0){
    //printf("(%d,%d) from (%d,%d)  \n", path[i].x, path[i].y, steps[path[i].x][path[i].y].prevX, steps[path[i].x][path[i].y].prevY);
    path[i+1].x = steps[path[i].x][path[i].y].prevX;
    path[i+1].y = steps[path[i].x][path[i].y].prevY;
    //printf("(%d,%d)\n", path[i].x, path[i].y);
    i++;
    if (path[i].x == X && path[i].y == Y) {
  		 start = 1;
  	}

  }
  //printf("Returning (%d,%d)\n", path[i-1].x,path[i-1].y);
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

void rotate360(void) {
  long  pos, oldpos, diff;
  unsigned long int T0;
 	anaOutVolts(2, -2);
 	anaOutStrobe();
  pos = encoder(0);
  diff = 0;
  oldpos = 0;
  while (diff < 1440 ){          //Less than half a turn
    oldpos = pos;
    pos = encoder(0);
    //printf("Clockwise: %ld\n", pos);
    if ((oldpos > 200) && (pos < 100)) {
      oldpos = 254 - oldpos;
      //printf("1 pos: %ld\noldpos: %ld\n", pos,oldpos);
    }
    if ((pos > 200) && (oldpos < 100)) {
      oldpos = 254 + oldpos;
      //printf("2 pos: %ld\noldpos: %ld\n", pos,oldpos);
    }
    diff = diff + (pos - oldpos);
    //printf("diff: %ld, pos: %ld oldpos: %ld\n",diff, pos, oldpos);
	}
  anaOutVolts(2, 0);
 	anaOutStrobe();

  T0 = TICK_TIMER;        //Wait .0156 seconds
	while ((TICK_TIMER - T0) < 32) {}
}

void characterize(long ipos, int grid[][GridSize], float currX, float currY, float heading) {
  unsigned long int T0;
  int x, y, i, f;
  float sens, angle, distance;
  long  pos, oldpos, diff;

  angle = 90;
  diff = 0;
  pos = 0;
  oldpos = 0;
  while(angle > .1 && angle < 179.9) {   //in between 0 and 180 for IR
    pos = encoder(0);
    //printf("pos: %ld\n", pos);
    //printf("ipos: %ld\n", ipos);
    if (pos < (ipos+360)) {           //Turn IR clockwise
      anaOutVolts(2, -2);
      anaOutStrobe();
      while (diff < 720 && pos < ipos+720) {          //Less than half a turn
        sens = anaInDiff(0, 0);
        if (sens > .62) {
          sens = -log((sens-.6171)/2.8400)*10.9775;
        }
        oldpos = pos;
        pos = encoder(0);
        //printf("Clockwise: %ld\n", pos);

        /**
        if ((oldpos > 200) && (pos < 100)) {
          oldpos = 254 - oldpos;
          //printf("1 pos: %ld\noldpos: %ld\n", pos,oldpos);
        }
        if ((pos > 200) && (oldpos < 100)) {
          oldpos = 254 + oldpos;
          //printf("2 pos: %ld\noldpos: %ld\n", pos,oldpos);
        } */
        diff = diff + (pos - oldpos);
        //printf("diff: %ld, pos: %ld oldpos: %ld\n",diff, pos, oldpos);
        angle = (diff)*360/1440;
        //printf("Seeing object at %f, angle %f heading: %f\n", sens, angle, heading);
        if (sens*.7072 < SquareSize && sens > 1) {
        	sens = (sens + (SquareSize-sens*.7072)+.01)/.7072;
        }
        if(sens < 60 && sens > .75) {
        	//printf("x =%f  , currX=%f\n",sin(((angle+heading-90)/180)*3.1415) ,currX);
          y = (int)(currY +(cos(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          x = (int)(currX + (sin(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          //printf("seeing an object at (%d,%d)\n",x,y);

          if(x>=0 && x<GridSize){
            if(y>=0 && y<GridSize){
              grid[x][y]++;
            }
          }
        }
      }
      diff = 0;
    }
    else if (pos > (ipos-360)) {
      anaOutVolts(2, 2);
      anaOutStrobe();
      while (diff > -720 && pos >= ipos) {
        sens = anaInDiff(0, 0);
        oldpos = pos;
        pos = encoder(0);
        //printf("CounterClockwise: %ld\n", pos);
        if (sens > .62) {
          sens = -log((sens-.6171)/2.8400)*10.9775;
        }
        /**
        if ((oldpos > 200) && (pos < 100)) {
          oldpos = 254 - oldpos;
          //printf("3pos: %ld\noldpos: %ld\n", pos,oldpos);
        }
        if ((pos > 200) && (oldpos < 100)) {
          oldpos = 254 + oldpos;
          //printf("4pos: %ld\noldpos: %ld\n", pos,oldpos);
        }  */
        diff = diff + (pos - oldpos);
        //printf("diff: %ld, pos: %ld oldpos: %ld\n",diff, pos, oldpos);
        angle = 180 - (-diff)*360/1440;
        //printf("Seeing object at %f, angle %f\n", sens, angle);
        if (sens*.7072 < SquareSize && sens > 1) {
        	sens = (sens + (SquareSize-sens*.7072)+.01)/.7072;
        }
        if(sens < 60 && sens > .75) {
          y = (int)(currY + (cos(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          x = (int)(currX + (sin(((angle+heading-90)/180)*3.1415)*sens))/SquareSize;
          //printf("seeing an object at (%d,%d)\n",x,y);
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


void driveToFwd(float dist, float *x, float *y, float heading){
  long turns, rollover, negRollover, Rwheel, Rdesired, Lwheel;
  long oldR, oldL, diffL, diffR, diff;
  int timesRollover, timesRolloverDes, close, rolls;
  unsigned long int T0;
  float driveR, driveL, rolloverDist, dist1;

  rollover = 32767;
  negRollover= -32768;
  rolloverDist = 58.7291; //cm to rollover

  timesRollover = 0;
  timesRolloverDes = 0;

  close = 0;
  driveR = 2;
  driveL = 1.86;

  turns = (long) dist*1115.95; //counts/cm, number of turns needed

  Rwheel = encoder(3);

  if((rollover-Rwheel) < turns){  //If number of turns to rollover<turns needed
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }

  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;

  anaOutVolts(1, driveR);   //should ramp up here
  anaOutVolts(0, driveL);
  anaOutStrobe();
  while(close!=1){
    oldR = Rwheel;      //calculate the chnage in encoders
    oldL = Lwheel;

    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR+diffL;

    if(oldR>0 && Rwheel < 0) {
      timesRollover++;
    }

    if(timesRollover==timesRolloverDes){
      if (Rdesired-Rwheel < 1000){ //within 2mm of final
        close=1;
      }
    }

    if(close!=1){
      if(diff > .99){     //If the right wheel moves faster
        driveL = driveL+.01;  //increase left wheel power
      }
      if(diff < -.99){      //If the left wheel moves faster
        driveL = driveL-.01;  //decrease left wheel power
      }

      anaOutVolts(1, driveR);   //should ramp up here
      anaOutVolts(0, driveL);
      anaOutStrobe();

      T0 = TICK_TIMER;        //Wait .0156 seconds
      while ((TICK_TIMER - T0) < 4) {}
    }
  }

  anaOutVolts(1, 0);
  anaOutVolts(0, 0);
  anaOutStrobe();

  T0 = TICK_TIMER;        //Wait .0156 seconds
  while ((TICK_TIMER - T0) < 64) {}
  Rwheel = encoder(3);
  while(Rdesired-Rwheel > 40) {
    anaOutVolts(1, driveR);
    anaOutVolts(0, driveL);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0156 seconds
    while ((TICK_TIMER - T0) < 8) {}
    anaOutVolts(1, 0);
    anaOutVolts(0, 0);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0156 seconds
    while ((TICK_TIMER - T0) < 64) {}
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
  *x = sin(heading/180*3.1415)*dist + *x;
  *y = cos(heading/180*3.1415)*dist + *y;
}

//needs to update pos...
float Turn(float angle, float *heading, float *x, float *y){
  long turns, rollover, negRollover, Rwheel, Rdesired, Lwheel;
  long oldR, oldL, diffL, diffR, diff;
  int timesRollover, timesRolloverDes, close, rolls;
  unsigned long int T0;
  float driveR, driveL, rolloverDist, dist1, dist;

  rollover = 32767;
  negRollover= -32768;

  rolloverDist = 58.7291; //cm to rollover

  timesRollover = 0;
  timesRolloverDes = 0;

  close = 0;
  driveR = 2;
  driveL = 1.84;

  dist = (angle/180)*3.14159*11.05;
  turns = (long) dist*1115.95;///1.006780792;


  Rwheel = encoder(3);

  if((rollover-Rwheel) < turns){  //If number of turns to rollover<turns needed
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }

  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;

  anaOutVolts(1, driveR);   //should ramp up here
  anaOutVolts(0, -driveL);
  anaOutStrobe();
  while(close!=1){
    oldR = Rwheel;      //calculate the chnage in encoders
    oldL = Lwheel;
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR-diffL;

    if(oldR>0 && Rwheel < 0) {
      timesRollover++;
    }

    if(timesRollover==timesRolloverDes){
      if (Rdesired-Rwheel < 1000){ //within 2mm of final
        close=1;
      }
    }

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


   //T0 = TICK_TIMER;        //Wait .0156 seconds
    //while ((TICK_TIMER - T0) < 64) {}
  anaOutVolts(1, 0);
  anaOutVolts(0, 0);
  anaOutStrobe();


  T0 = TICK_TIMER;        //Wait .0156 seconds
  while ((TICK_TIMER - T0) < 64) {}
  Rwheel = encoder(3);
  while(Rdesired-Rwheel > 55) {
    anaOutVolts(1, driveR);
    anaOutVolts(0, -driveL);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0156 seconds
    while ((TICK_TIMER - T0) < 8) {}
    anaOutVolts(1, 0);
    anaOutVolts(0, 0);
    anaOutStrobe();
    T0 = TICK_TIMER;        //Wait .0156 seconds
    while ((TICK_TIMER - T0) < 64) {}
    Rwheel = encoder(3);
    Lwheel = encoder(2);
    diffL = Lwheel-oldL;
    diffR = Rwheel-oldR;
    diff = diffR-diffL;

    if(diff < .99){     //If the right wheel moves faster
        driveL = driveL-.01;  //increase left wheel power
      }

      if(diff > -.99){      //If the left wheel moves faster
        driveL = driveL+.01;  //decrease left wheel power
    }
 }

   *x = *x + (sin((angle+*heading)/180*3.14159265) - sin(*heading/180*3.13159265))*WheelRadius;
   *y = *y + (cos((angle+*heading)/180*3.14159265) - cos(*heading/180*3.14159265))*WheelRadius;

   if (*heading+angle > 360){
    *heading = *heading+angle-360;
   }
   else {
    *heading = *heading+angle;
   }

   //needs help!!!!!!!!!!!!!!!!!!!!!!!!!


   return(*heading);
}


long encoder(int j) {
  int asb, bsb, csb;
  long position;
  asb = 0;
  bsb = 0;
  csb = 0;
  position = 0;

  EncWrite(j, TRSFRCNTR_OL, CNT);
  // EncWrite(j,BP_RESET,CNT);
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
// reg is an int which is 1 or 0 indicating whether control or data is desired
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