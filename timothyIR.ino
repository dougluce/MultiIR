#include <EEPROM.h>
#include <IRremote.h>
#include "bitlash.h"

int RECV_PIN = 11;  // IR detector pin
int STATUS_PIN = 13; // Pin with visible light LED on it.

IRrecv irrecv(RECV_PIN); // IR receiver
IRsend irsend;           // IR sending interface

decode_results results;

//
#define MAX_EEPROM 1024
// Permanent storage locations
int codelocs[] = {0, 170, 340, 510, 680, 850};

// Temporary storage for the recorded code
int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
unsigned int rawCodes[RAWBUF]; // The durations if raw
int codeLen; // The length of the code
int toggle = 0; // The RC5/6 toggle state

// Stores the code for later playback
// Most of this code is just logging
void storeCode(decode_results *results) {
  codeType = results->decode_type;
  int count = results->rawlen;

  if (codeType == UNKNOWN) {
    Serial.println("Received unknown code, saving as raw");
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        Serial.print(" m");
      } else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        Serial.print(" s");
      }
      Serial.print(rawCodes[i - 1], DEC);
    }
    Serial.println("");
  } else {
    if (codeType == NEC) {
      Serial.print("Received NEC: ");
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        Serial.println("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      Serial.print("Received SONY: ");
    } 
    else if (codeType == RC5) {
      Serial.print("Received RC5: ");
    } 
    else if (codeType == RC6) {
      Serial.print("Received RC6: ");
    } 
    else {
      Serial.print("Unexpected codeType ");
      Serial.print(codeType, DEC);
      Serial.println("");
    }
    Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}



numvar storecode(void) { 
  numvar arg;
  arg = getarg(0);
  if (arg != 1) {
    Serial.print("Storecode takes a single argument: which IR blaster to link.");
    return (numvar)1;
  } else {
    arg = getarg(1);

  }
  return (numvar)1;
}



numvar dump(void) { 
  numvar arg;
  int i,j;

  for(i=0;i<MAX_EEPROM;i++) {
    j=EEPROM.read(i);
    if (i%8 == 0) {
      Serial.println("");
      Serial.print(i,HEX);
      Serial.print(": ");
    }
    Serial.print(j,HEX);
    Serial.print(" ");
  }     
  Serial.println("");
}

numvar wipe(void) { 
  numvar arg;
  int i;

  arg = getarg(0);
  if (arg != 1) {
    Serial.println("Wipe takes takes a single argument: a (1) to confirm.");
    return (numvar)1;
  } else {
    Serial.print("wiping...");
    for(i=0;i<MAX_EEPROM;i++)
      EEPROM.write(i,0);
    Serial.println("done.");
  }     
  return (numvar)1;
}

void setup(void) {

	// initialize bitlash and set primary serial port baud
	// print startup banner and run the startup macro
	initBitlash(57600);

        addBitlashFunction("storecode", (bitlash_function) storecode);
        addBitlashFunction("sc", (bitlash_function) storecode);
        addBitlashFunction("wipe", (bitlash_function) wipe);
        addBitlashFunction("dump", (bitlash_function) dump);

	irrecv.enableIRIn(); // Start the receiver
        pinMode(STATUS_PIN, OUTPUT);
}

void loop(void) {
	runBitlash();
        // See a code?  Store it in the temp location!
	if (irrecv.decode(&results)) {
	  digitalWrite(STATUS_PIN, HIGH);
	  storeCode(&results);
	  irrecv.resume(); // resume receiver
	  digitalWrite(STATUS_PIN, LOW);
	}
}


