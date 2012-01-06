#include "bitlash.h"

numvar readcode(void) { 
       sp("Ready to read IR code"); speol();

       return TCNT1;  // return the value of Timer 1
}

void setup(void) {

	// initialize bitlash and set primary serial port baud
	// print startup banner and run the startup macro
	initBitlash(57600);

        addBitlashFunction("readcode", (bitlash_function) readcode);
        addBitlashFunction("rc", (bitlash_function) readcode);
}

void loop(void) {
	runBitlash();
}


