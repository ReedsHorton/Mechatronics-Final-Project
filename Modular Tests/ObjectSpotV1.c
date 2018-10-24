
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

//Force functions to be compiled to extended memory.  Helps when the
// program gets large
#memmap xmem

//Import Library for BL2600
#use BL26XX.lib
//#use MATH.LIB
//#include <math.lib>



int EncRead(int channel, int reg);
void EncWrite(int channel, int data, int reg);
long encoder(int j);
int init(void);
void characterize(long ipos, int grid[][30], int currX, int currY);


int main(void)
{
	int i, f, grid[30][30];
  long ipos;

	brdInit();				//default processor configuration
  digOutConfig(0xff00);
  anaOutConfig(1, DAC_SYNC);//Channel 1 Bipolar operation
  anaOutPwr(1);  			//Enable BL2600 power supply to drive D/A channels
  anaOutVolts(1, 0);		//Send 0 volts to prevent indeterminate state
  anaOutVolts(0, 0);    //Send 0 volts to prevent indeterminate state
  anaOutVolts(2, 0);    //Send 0 volts to prevent indeterminate state
  anaOutStrobe();
  digOut(CS1,1);
  digOut(CS2,1);
  digOut(RD,1);
  digOut(WR,1);
  anaInConfig(0, DIFF_MODE); // Configure analog input channel 0 for Diff mode
  init();

  ipos = encoder(0);


  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      grid[i][f] = 0;
    }
  }
  /**
  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      printf("%d ",grid[i][f]);
    }
    printf("\n");
  }
   printf("\n");*/

  //printf("iPos: %ld", ipos);

  //for(i=0; i<100; i++) {
  characterize(ipos, grid, 15, 0);


  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
      printf("%d ",grid[i][f]);
    }
    printf("\n");
  }
   printf("\n");

   characterize(ipos, grid, 15, 5);


  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
      printf("%d ",grid[i][f]);
    }
    printf("\n");
  }
   printf("\n");

   characterize(ipos, grid, 20, 10);


  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
      printf("%d ",grid[i][f]);
    }
    printf("\n");
  }
   printf("\n");



  //}

  /**
  characterize();
  printf("1\n");
  characterize();
  printf("2\n");
  characterize();
  printf("3\n");
  characterize();
  printf("4\n");
   */

}

void characterize(long ipos, int grid[][30], int currX, int currY){
unsigned long int T0;
  int m, flag, x, y, i, f, grid2[30][30];
  float sens, angle, distance;
  long  pos, oldpos, diff;

  //T0 = TICK_TIMER;        //Wait 2 seconds
  //while ((TICK_TIMER - T0) < 2048) {}
  //ipos = encoder(0); //get initial encoder position
  /*
  for (i=0; i<30; i++){
    for (f=0; f<30; f++){
      grid2[i][f] = 0;
    }
  } */
  /**
  for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
      printf("%d ",grid2[i][f]);
    }
    printf("\n");
  }
   printf("\n");
  */

  angle = 90;
  diff = 0;
  pos = 0;
  oldpos = 0;
  while(angle > .1 && angle < 179.9) {
  		flag = 1;

	   pos = encoder(0);
      //printf("pos: %ld\n", pos);

	   if (pos < (ipos+360)) {
	      anaOutVolts(2, -1);
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

	         	//if (sens > .75) {
	            	angle = (diff)*360/1440;
	            	if(sens < 70 && sens > .75) {
	              	 	//printf("Distance: %f, Angle: %f\n", sens, angle);

                  /*if (angle > 90) {
                    x = (int)(cos((angle-90)/180*3.1415)*sens)/10.16;
                    y = (int)(sin((angle-90)/180*3.1415)*sens)/10.16;
                    //printf("Distance: %f, Angle: %f\n", sens, angle);
                    //printf("grid[%d][%d]\n", x+3, y);
                    //x = x%10.16;
                    //y = y%10.16;
                    x = x+3;
                    grid[x][y] = 1;
                  } */
                  //else {
                    x = (int)currX +(cos((angle)/180*3.1415)*sens)/10.16;
                    y = (int)currY + (sin((angle)/180*3.1415)*sens)/10.16;
                    //printf("Distance: %f, Angle: %f\n", sens, angle);
                    //printf("object at (%d,%d)\n", x, y);
                    //grid2[x][y] = 1;
                    grid[x][y] = 1;
                  //}

	            	}
	         	//}
	      	}
	        diff = 0;
	   	}


	   	else if (pos > (ipos-360)) {
	    	anaOutVolts(2, 1);
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
	         	//if (sens > .75) {
	            	angle = 180 - (-diff)*360/1440;
	            	if(sens < 40 && sens > .75) {
                     x = (int)currX +(cos((angle)/180*3.1415)*sens)/10.16;
                    	y = (int)currY + (sin((angle)/180*3.1415)*sens)/10.16;
                    	//printf("Distance: %f, Angle: %f\n", sens, angle);
                    	//printf("object at (%d,%d)\n", x, y);
                    	//grid2[x][y] = 1;
                     grid[x][y] = 1;
                  }
	         	//}
	      	}
	        diff = 0;
	   	}
	   	m++;
	   	//printf("Angle: %f\n", angle);
  	}
   anaOutVolts(2, 0);
  anaOutStrobe();
  /*8
   for (i=29; i>=0; i--){
    for (f=0; f<30; f++){
      printf("%d ",grid2[i][f]);
      //grid[i][f] = grid2[i][f];
    }
    printf("\n");
  }
   printf("\n"); */
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