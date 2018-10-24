
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
#define GridSize    5.08

//Force functions to be compiled to extended memory.  Helps when the
// program gets large
#memmap xmem

//Import Library for BL2600
#use BL26XX.lib


int EncRead(int channel, int reg);
void EncWrite(int channel, int data, int reg);
long encoder(int j);
int init(void);
void driveToFwd(float dist);
float Turn(float angle, float heading);
void characterize(long ipos, int grid[][30], float currX, float currY, float heading);


int main(void){
  unsigned long int T0;
  long max, min, rwheel;
  float dist, heading;
  float currX, currY;
  int i, f, test, grid[30][30];
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


  currX = 50;
  currY = 25;
  heading = 0;



  ipos = encoder(0);


  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      grid[i][f] = 0;
    }
  }

  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(25);
  currX = sin(heading/180*3.1415)*25 + currX;
  currY = cos(heading/180*3.1415)*25 + currY;
  test = (int) currY/GridSize;
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(25);
  currX = sin(heading/180*3.1415)*25 + currX;
  currY = cos(heading/180*3.1415)*25 + currY;
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(26.2);
  currX = sin(heading/180*3.1415)*26.2 + currX;
  currY = cos(heading/180*3.1415)*26.2 + currY;

  /**
  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
    	printf("%d ",grid[f][i]);
    }
    printf("\n");
  }
   printf("\n");
  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      grid[i][f] = 0;
    }
  } */

  heading = Turn(90, heading);
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(25);
  currX = sin(heading/180*3.1415)*25 + currX;
  currY = cos(heading/180*3.1415)*25 + currY;
 	printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(26.2);
  currX = sin(heading/180*3.1415)*26.2 + currX;
  currY = cos(heading/180*3.1415)*26.2 + currY;
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  /*
  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
    	printf("%d ",grid[f][i]);
    }
    printf("\n");
  }
   printf("\n");
  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      grid[i][f] = 0;
    }
  }*/
  heading = Turn(90, heading);
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(25);
  currX = sin(heading/180*3.1415)*25 + currX;
  currY = cos(heading/180*3.1415)*25 + currY;
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);
  driveToFwd(15.64);
  currX = sin(heading/180*3.1415)*15.64 + currX;
  currY = cos(heading/180*3.1415)*15.64 + currY;
  printf("heading %.1f, (%.1f,%.1f), [%d,%d]\n", heading, currX, currY, ((int) (currX/GridSize)), ((int) (currY/GridSize)));
  characterize(0, grid, currX, currY, heading);


  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
    if(grid[f][i] > 5) {
    	printf("1 ");
    }
    else {
    	printf("0 ");
    }
      //printf("%d ",grid[f][i]);
    }
    printf("\n");
  }
   printf("\n");


}

void characterize(long ipos, int grid[][30], float currX, float currY, float heading) {
  unsigned long int T0;
  int x, y, i, f;
  float sens, angle, distance;
  long  pos, oldpos, diff;

  angle = 90;
  diff = 0;
  pos = 0;
  oldpos = 0;
  while(angle > .1 && angle < 179.9) {
    pos = encoder(0);
    //printf("pos: %ld\n", pos);
    //printf("ipos: %ld\n", ipos);
    if (pos < (ipos+360)) {
      anaOutVolts(2, -1.4);
      anaOutStrobe();
      while (diff < 720) {
        sens = anaInDiff(0, 0);
        if (sens > .65) {
          sens = -log((sens-.6225)/3.1121)*10.4517;
        }
        oldpos = pos;
        pos = encoder(0);
        if ((oldpos > 200) && (pos < 100)) {
          oldpos = 250 - oldpos;
        }
        if ((pos > 200) && (oldpos < 100)) {
          oldpos = 250 + oldpos;
        }
        diff = diff + (pos - oldpos);
        //printf("diff: %ld, pos: %ld oldpos: %ld\n",diff, pos, oldpos);
        angle = (diff)*360/1440;
        if(sens < 70 && sens > .75) {
          y = (int)(currX +(cos(((angle+heading-90)/180)*3.1415)*sens))/GridSize;
          x = (int)(currY + (sin(((angle+heading-90)/180)*3.1415)*sens))/GridSize;
          printf("seeing an object at (%d,%d)\n",x,y);

          if(x>=0 && x<30){
            if(y>=0 && x<30){
              grid[x][y]++;
            }
          }
        }
      }
      diff = 0;
    }
    else if (pos > (ipos-360)) {
      anaOutVolts(2, 1.4);
      anaOutStrobe();
      while (diff > -720) {
        sens = anaInDiff(0, 0);
        oldpos = pos;
        pos = encoder(0);
        if (sens > .65) {
          sens = -log((sens-.6225)/3.1121)*10.4517;
        }
        if ((oldpos > 200) && (pos < 100)) {
          oldpos = 250 - oldpos;
        }
        if ((pos > 200) && (oldpos < 100)) {
          oldpos = 250 + oldpos;
        }
        diff = diff + (pos - oldpos);
        //printf("diff: %ld, pos: %ld oldpos: %ld\n",diff, pos, oldpos);
        angle = 180 - (-diff)*360/1440;
        if(sens < 40 && sens > .75) {
          y = (int)(currX +(cos(((angle+heading-90)/180)*3.1415)*sens))/GridSize;
          x = (int)(currY + (sin(((angle+heading-90)/180)*3.1415)*sens))/GridSize;
          printf("seeing an object at (%d,%d)\n",x,y);
          if(x>=0 && x<30){
            if(y>=0 && x<30){
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


void driveToFwd(float dist){
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
  //printf("starting at Rhweel:%ld\n",Rwheel);

  if((rollover-Rwheel) < turns){  //If number of turns to rollover<turns needed
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }

   /**if ((Rwheel+turns) > rollover){
    timesRolloverDes = ((turns-(rollover-Lwheel))%(rollover-negRollover))+1;
  } */

  //Rdesired = turns-timesRolloverDes*(rollover-negRollover)+Rwheel; //desired encoder counts
  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;
  //printf("Rollovers:%d   Desired Enconcer:%ld\n",timesRolloverDes,Rdesired);

  anaOutVolts(1, driveR);   //should ramp up here
    anaOutVolts(0, driveL);
    anaOutStrobe();
    while(close!=1){

      oldR = Rwheel;      //calculate the chnage in encoders
      oldL = Lwheel;
      //printf("Rhweel:%ld\n",Rwheel);
      Rwheel = encoder(3);
      Lwheel = encoder(2);
      diffL = Lwheel-oldL;
      diffR = Rwheel-oldR;
      diff = diffR+diffL;
      //printf("Rhweel:%ld, Lwheel:%ld\n",Rwheel, Lwheel);
      //printf("DiffL: %ld, DiffR: %ld, diff:%ld\n", diffL, diffR, diff);

      if(oldR>0 && Rwheel < 0) {
        timesRollover++;
        //printf("Rolled Over, %d\n", timesRollover);
      }

      if(timesRollover==timesRolloverDes){
        if (Rdesired-Rwheel < 1000){ //within 2mm of final
          //printf("We're close\n");
          close=1;
        }
      }

      if(close!=1){
        if(diff > .99){     //If the right wheel moves faster
          driveL = driveL+.01;  //increase left wheel power
          //printf("Incrementing L\n");
        }

        if(diff < -.99){      //If the left wheel moves faster
          driveL = driveL-.01;  //decrease left wheel power
          //printf("Decrementing L\n");
        }

        anaOutVolts(1, driveR);   //should ramp up here
        anaOutVolts(0, driveL);
        anaOutStrobe();

        T0 = TICK_TIMER;        //Wait .0156 seconds
        while ((TICK_TIMER - T0) < 4) {}
      }
    }

   //anaOutVolts(1, -7);
    //anaOutVolts(0, (-7*(driveL/driveR)));
    //anaOutStrobe();
   //T0 = TICK_TIMER;        //Wait .0156 seconds
    //while ((TICK_TIMER - T0) < 64) {}
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
          //printf("Incrementing L\n");
        }

        if(diff < -.99){      //If the left wheel moves faster
          driveL = driveL-.01;  //decrease left wheel power
          //printf("Decrementing L\n");
        }
      //printf("Close! %ld\n",(Rdesired-Rwheel));
   }

   //fine tune position?

}


float Turn(float angle, float heading){
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
  turns = (long) dist*1115.95;


  Rwheel = encoder(3);
  //printf("starting at Rhweel:%ld\n",Rwheel);

  if((rollover-Rwheel) < turns){  //If number of turns to rollover<turns needed
    dist1 = (rollover-Rwheel)*.00089615;
    rolls = (int) (dist-dist1)/(rolloverDist);
    timesRolloverDes = rolls+1;
  }

  Rdesired = (dist-(timesRolloverDes*rolloverDist))*1115.95+Rwheel;
  //printf("Rollovers:%d   Desired Enconcer:%ld\n",timesRolloverDes,Rdesired);

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
      //printf("Rhweel:%ld, Lwheel:%ld\n",Rwheel, Lwheel);
      //printf("DiffL: %ld, DiffR: %ld, diff:%ld\n", diffL, diffR, diff);

      if(oldR>0 && Rwheel < 0) {
        timesRollover++;
        //printf("Rolled Over, %d\n", timesRollover);
      }

      if(timesRollover==timesRolloverDes){
        if (Rdesired-Rwheel < 1000){ //within 2mm of final
          //printf("We're close\n");
          close=1;
        }
      }

      if(close!=1){
        if(diff < .99){     //If the right wheel moves faster
          driveL = driveL-.01;  //increase left wheel power
          //printf("Incrementing L\n");
        }

        if(diff > -.99){      //If the left wheel moves faster
          driveL = driveL+.01;  //decrease left wheel power
          //printf("Decrementing L\n");
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
          //printf("Incrementing L\n");
        }

        if(diff > -.99){      //If the left wheel moves faster
          driveL = driveL+.01;  //decrease left wheel power
          //printf("Decrementing L\n");
      }
      //printf("Close! %ld\n",(Rdesired-Rwheel));
   }
   //T0 = TICK_TIMER;        //Wait .0156 seconds
    //while ((TICK_TIMER - T0) < 64) {}

   //fine tune position?
   if (heading+angle > 360){
    heading = heading+angle-360;
   }
   else {
    heading = heading+angle;
   }
   return(heading);
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