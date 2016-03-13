#include <Wire.h>

#define IODIR ((byte)0x00)
#define IPOL ((byte)0x01)
#define GPINTEN ((byte)0x02)
#define DEFVAL ((byte)0x03)
#define INTCON ((byte)0x04)
#define IOCON ((byte)0x05)
#define GPPU ((byte)0x06)
#define INTF ((byte)0x07)
#define INTCAP ((byte)0x08)
#define GPIO ((byte)0x09)
#define OLAT ((byte)0x0A)

#define MCP23008ADDR ((byte)0b00100000)
#define MCP23008NCS ((byte)0b00010000)
#define MCP23008UD ((byte)0b00100000)

#define LAMPONOFF 0b00000010
#define LAMPUP 0b00000100
#define LAMPDOWN 0b00001000

#define LEDLAMPINITIAL (((byte)0b00000001) | MCP23008NCS) // LED and lamp off initially; nCS pin high U/nD low

#define RESETLINE ((byte)8)

#define POWERLINE ((byte)13)

static enum LightState {
  Off = 0,
  On = 1
} ledState, lampState;
static byte cRegVal;
static byte rheostatState = 0;

void setup()
{
  Wire.begin(); // Join I2C bus
  Serial.begin(9600);

  pinMode(POWERLINE, OUTPUT);
  pinMode(RESETLINE, OUTPUT);

  send23008IntoReset(); // Start off reset
  powerOn();
    take23008OutOfReset();
      fullInit23008();
      powerDown(96); // Make sure the rheostat starts off in a known state (lowest power)
      delay(1);
    send23008IntoReset(); // Start off in the reset state
  powerOff();
}

void loop()
{
  byte rVal;

  for(;;) {
    powerOn();
    send23008IntoReset();
    take23008OutOfReset();
    shortInit23008();
    readRegs(GPIO, 1, &rVal);
    if((rVal & 0b1110) != 0b1110) { // If any of the switches were triggered
      break; // Then wake up
    }
    send23008IntoReset();
    powerOff();
    delay(1000);
  }
  for(byte iCnt = 20; iCnt != 0; --iCnt) {
    cRegVal ^= 0b0001; // Toggle LED
    rVal = sendOLAT(cRegVal);
    ledState != ledState;
    delay(50);
  }
  lampControl();
}

void lampControl()
{
  unsigned long int tLast = millis();
  boolean update = false;
  byte intRegVal;

  fullInit23008();
  powerDown(96); // Make sure the rheostat starts off in a known state (lowest power)

  do {
    readRegs(INTF, 1, &intRegVal);
    if(intRegVal != 0x0) {
      struct {
        byte intCap;
        byte gpioNow;
      } regVals;
  
      readRegs(INTCAP, 2, (byte*)&regVals);
  
      Serial.println(regVals.intCap, BIN);
      update = processBits(regVals.intCap, &tLast);
  
      if(regVals.intCap != regVals.gpioNow) {
        Serial.println(regVals.gpioNow, BIN);
        update |= processBits(regVals.gpioNow, &tLast);
      }
    }
    if((millis() - tLast) > 1000){ // Large change since last power level change (>1s), this ensures that we don't accidently turn the lamp on or off
      if(lampState == Off && rheostatState > 0) {
        powerDown(96); // Make sure the rheostat starts off in a known state (lowest power)
      } if(lampState == On && rheostatState == 0) {
        powerUp(1); // Make sure we're not in the lowest power state anymore
      }
    }
    if(((millis() - tLast) > 10000) && (lampState==Off)){
      return;      
    }
  
    if(update) {
      cRegVal = MCP23008NCS; // nCS high initially
      cRegVal |= lampState==On ? 0b10000000 : 0b00000000;
      cRegVal |= ledState==On ? 0b0: 0b1;
  
      byte rVal = sendOLAT(cRegVal);
  
      Serial.print("rVal :");
      Serial.print(rVal);
      Serial.println(":");
    }
  } while(true);
}

void powerOn()
{
  digitalWrite(POWERLINE, HIGH);
}

void powerOff()
{
  digitalWrite(POWERLINE, LOW);
}

void send23008IntoReset()
{
  digitalWrite(RESETLINE, LOW);
}

void take23008OutOfReset()
{
  digitalWrite(RESETLINE, HIGH);
}

byte shortInit23008(void)
{
  byte rVal;

  cRegVal = LEDLAMPINITIAL;
  ledState = Off;
  lampState = Off;

  rVal = sendOLAT(cRegVal);
  rVal |= sendRegByte(IODIR, 0b01001110); // Switches to inputs, unused line to input
  rVal |= sendRegByte(GPPU, 0b00001110); // Use built in pullups on switches

  return rVal;
}

void fullInit23008(void)
{
  byte rVal;
  byte intRegVal;

  rVal = sendRegByte(IOCON, 0b00000000);
  rVal |= shortInit23008();

  rVal |= sendRegByte(GPINTEN, 0b00001110); // Enable interrupts
  rVal |= sendRegByte(DEFVAL, 0b00001110); // This is the uninterrupted level
  rVal |= sendRegByte(INTCON, 0b00001110); // These lines honour the DEFVAL register
  rVal |= readRegs(INTCAP, 1, &intRegVal); // Clear the interrupt state
}

boolean processBits(byte pinVals, unsigned long int *tLast)
{
  unsigned long int tNow = millis();

  if((pinVals & LAMPONOFF) == 0) {
    if(lampState == Off && rheostatState > 0) {
      lampState = On;
      powerDown(96); // Make sure the rheostat starts off in a known state (lowest power)
      powerUp(1);
      *tLast = tNow;
      Serial.println("On");
      return true;
    } if(lampState == On && rheostatState == 0) {
      lampState = Off;
      powerDown(96); // Make sure the rheostat starts off in a known state (lowest power)
      *tLast = tNow;
      Serial.println("Off");
      return true;
    }
  } else {
    if((tNow - *tLast) > (rheostatState<<3)) { // Small time since last power level change
      if((pinVals & LAMPUP) == 0) {
        powerUp(1);
        *tLast = tNow;
      } else if((pinVals & LAMPDOWN) == 0){
        powerDown(1);
        *tLast = tNow;
      }
    }
  }

  return false;
}

void powerDown(byte nSteps)
{
  byte rVal;
  byte localReg;

  for(; nSteps != 0; --nSteps) {
    localReg = cRegVal | MCP23008NCS | MCP23008UD;
  
    rVal = sendOLAT(localReg); // nCS and U/nD high
  
    localReg &= ~MCP23008NCS;
  
    rVal |= sendOLAT(localReg); // U/nD high, nCS low
  
    localReg &= ~MCP23008UD;
  
    rVal |= sendOLAT(localReg); // nCS and U/nD low

    localReg |= MCP23008UD;
  
    rVal |= sendOLAT(localReg); // nCS low and U/nD high
  
    rVal |= sendOLAT(cRegVal); // nCS high and U/nD low (default)

    rheostatState -= rheostatState > 0 ? 1 : 0;
  }

  Serial.print("rheostatState :");
  Serial.print(rheostatState);
  Serial.println(":");
}

void powerUp(byte nSteps)
{
  byte rVal;
  byte localReg;

  for(; nSteps != 0; --nSteps) {
    localReg = cRegVal & ~MCP23008NCS;
  
    rVal = sendOLAT(localReg); // nCS low and U/nD low
    
    localReg |= MCP23008UD;
  
    rVal |= sendOLAT(localReg); // nCS low and U/nD high
  
    rVal |= sendOLAT(cRegVal); // nCS high and U/nD low

    rheostatState += rheostatState < 31 ? 1 : 0;
  }

  Serial.print("rheostatState :");
  Serial.print(rheostatState);
  Serial.println(":");
}

byte sendRegByte(byte regAddr, byte regVal)
{
  Wire.beginTransmission(MCP23008ADDR);
  Wire.write(regAddr);
  Wire.write(regVal);
  return Wire.endTransmission(true);
}

byte sendOLAT(byte regVal)
{
  return sendRegByte(OLAT, regVal);
}

byte readRegs(byte regAddr, byte nBytes, byte *bOut)
{
  byte rVal;
  Wire.beginTransmission(MCP23008ADDR);
  Wire.write(regAddr);
  rVal = Wire.endTransmission(true); // At lower clock rates the receiver doesn't like a repeated start here
  Wire.requestFrom(MCP23008ADDR, nBytes, (byte)true);
  for(byte idx = 0; idx < nBytes; idx++) {
    bOut[idx] = Wire.read();
  }
  return rVal;
}

/*
  if(Serial.available()) {
    int inChar = Serial.read();
    inChar = isalpha(inChar)?toupper(inChar):inChar;
    switch(inChar) {
      case 'L':
        powerDown(96);
        lampState = lampState==Off ? On : Off;
        update = true;
        break;
      case 'D':
        ledState = ledState==Off ? On : Off;
        update = true;
        break;
      case '[':
          powerDown(1);
          Serial.println("Down");
        break;
      case ']':
          powerUp(1);
          Serial.println("Up");
        break;
      default:
        break;
    }
  } else {
*/
