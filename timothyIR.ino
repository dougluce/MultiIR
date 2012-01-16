#include <IRremote.h>
#include <IRremoteInt.h>
#include "bitlash.h"
#include <EEPROM.h>
#include "eeprom_any.h"

int RECV_PIN = 11;  // IR detector pin
int STATUS_PIN = 13; // Pin with visible light LED on it.

IRrecv irrecv(RECV_PIN); // IR receiver
IRsend irsend;           // IR sending interface

decode_results results;
extern volatile irparams_t irparams;

#define MAX_EEPROM 1024
#define NONE 0
// Permanent storage locations
#define SLOTSIZE 170
int codelocs[] = {SLOTSIZE * 0, 
                  SLOTSIZE * 1,
                  SLOTSIZE * 2,
                  SLOTSIZE * 3,
                  SLOTSIZE * 4,
                  SLOTSIZE * 5};

// Temporary storage for the recorded code
byte codeType = 0; // The type of code
unsigned long codeValue; // The code value if not raw
unsigned int rawCodes[RAWBUF]; // The durations if raw
byte codeLen; // The length of the code
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



numvar store(void) { 
  numvar arg;
  int i;
  arg = getarg(0);
  if (arg != 1) {
    Serial.println("store takes a single argument: the IR blaster to link.");
    return (numvar)1;
  }
  arg = getarg(1);
  if (arg < 1 || arg > 6) {
    Serial.println("Specify a slot, 1-6");
    return (numvar)1;
  }
  if (codeType == NONE) {
    Serial.println("I've no code.  Send me one, would you?");
    return (numvar)1;
  }
  arg--;
  // Have a valid code and slot to store it in.  Save it!
  int Eloc = arg*SLOTSIZE;
  EEPROM.write(Eloc++,codeType);
  EEPROM.write(Eloc++,codeLen);
  if (codeType == UNKNOWN) {
    Serial.print("Storing raw code of size ");
    Serial.print(codeLen, DEC);
    for (i=1; i<codeLen; i++) {
      Eloc += EEPROM_writeAnything(Eloc,rawCodes[i]);
    }
  } else {
    Serial.print("Storing code type ");
    Serial.print(codeType, DEC);
    Eloc += EEPROM_writeAnything(Eloc,codeValue);
  } 
  Serial.println("...done.");
  
  codeType = NONE;
  return (numvar)1;
}

void sendCode() {
  numvar arg;
  int i;
  arg = getarg(0);
  if (arg != 1) {
    Serial.println("send takes a single argument: the code number to blast.");
    return;
  }
  arg = getarg(1);
  if (arg < 1 || arg > 6) {
    Serial.println("Specify a slot, 1-6");
    return;
  }

  int Eloc = arg*SLOTSIZE;
  codeType = EEPROM.read(Eloc++);
  codeLen = EEPROM.read(Eloc++);

  if (codeType == NEC) {
    irsend.sendNEC(codeValue, codeLen);
    Serial.print("Sent NEC ");
    Serial.println(codeValue, HEX);
  } 
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    Serial.print("Sent Sony ");
    Serial.println(codeValue, HEX);
  } 
  else if (codeType == RC5 || codeType == RC6) {
    // Flip the toggle bit for a new button press
    toggle = 1 - toggle;
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (toggle << (codeLen - 1));
    if (codeType == RC5) {
      Serial.print("Sent RC5 ");
      Serial.println(codeValue, HEX);
      irsend.sendRC5(codeValue, codeLen);
    } 
    else {
      irsend.sendRC6(codeValue, codeLen);
      Serial.print("Sent RC6 ");
      Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == UNKNOWN /* i.e. raw */) {
    for (i=1; i<codeLen; i++) {
      Eloc += EEPROM_writeAnything(Eloc,rawCodes[i]);
    }

    // Assume 38 KHz
    irsend.sendRaw(rawCodes, codeLen, 38);
    Serial.println("Sent raw");
  }
}


numvar dump(void) { 
  numvar arg;
  int i,j;

  for(i=0;i<MAX_EEPROM;i++) {
    j=EEPROM.read(i);
    if (i%16 == 0) {
      Serial.println("");
      if (i < 16) Serial.print("0");
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

        addBitlashFunction("store", (bitlash_function) store);
        addBitlashFunction("send", (bitlash_function) sendCode);
        addBitlashFunction("wipe", (bitlash_function) wipe);
        addBitlashFunction("dump", (bitlash_function) dump);

	irrecv.enableIRIn(); // Start the receiver
        pinMode(STATUS_PIN, OUTPUT);
}

#define FLASHER 10000 
#define DOUBLE_FLASHER 60000 

unsigned long t = 0;
void loop(void) {
  if (t++ < FLASHER){
    digitalWrite(STATUS_PIN, HIGH);
    if (t < 2) {
      Serial.print("rawlen: ");
      Serial.println(irparams.rawlen,DEC);
      Serial.print("timer: ");
      Serial.println(irparams.timer,DEC);
      Serial.print("rcvstate: ");
      Serial.println(irparams.rcvstate,DEC);
    }
  } else {
    digitalWrite(STATUS_PIN, LOW);
  }
  if (t > DOUBLE_FLASHER) {
    t = 0;
  }

  runBitlash();
  // See a code?  Store it in the temp location!
  if (irrecv.decode(&results)) {
    digitalWrite(STATUS_PIN, HIGH);
    storeCode(&results);
    irrecv.resume(); // resume receiver
    digitalWrite(STATUS_PIN, LOW);
  }
}


