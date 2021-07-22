/*IR Modem
 * Author: Scott Mayberry
 */
#include <digitalWriteFast.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define MESSAGE_MAXIMUM_SYMBOL_SIZE 512
#define NUM_PERIODS 20
#define mod_pin 7
#define demod_pin 10
#define PINGS_PER_SYMBOL 15

//////Shared Variables///////
bool hammingActive = true; //hamming (12, 7)
const float frequency = 38000;
const float frequencyMicroSeconds = 1.0 / frequency * 1000000;

//////IR Modulation Variables////////
int messageSymbolSize;
bool bitUpdate[MESSAGE_MAXIMUM_SYMBOL_SIZE];

char outgoingMessagesBytes[MESSAGE_MAXIMUM_SYMBOL_SIZE / 8 + 1];
int outgoingMessagesBytesIndex;

volatile uint16_t periodCounter;
volatile uint16_t outSymbolIndex;
bool modulationActive;

bool outputHigh = false;
bool outputActive = false;

const float callbackUpdateSpeed = frequencyMicroSeconds / 2;

IntervalTimer outgoingSineWaveDACTimer;

//////IR Demodulation Variables////////
char incomingMessageBytes[MESSAGE_MAXIMUM_SYMBOL_SIZE / 8 + 1];

volatile bool incomingSignalDetected = false;

volatile int incomingSymbols[MESSAGE_MAXIMUM_SYMBOL_SIZE];
volatile int inSymbolIndex;

volatile int messageTimeOutCounter;
const int messageTimeOutThreshold = 24;

const float microsPerSymbolIn = frequencyMicroSeconds * NUM_PERIODS;
const float microsUpdatePerPing = microsPerSymbolIn / PINGS_PER_SYMBOL;

const int numOfIndexesForShift = int(PINGS_PER_SYMBOL / 3);

volatile int pingCounter;
volatile int pingReading;

int lowSmallAverage;
int highSmallAverage;
int sumOfPings;

IntervalTimer incomingSignalWavePingTimer;

void setup()
{
  Serial.begin(115200); //begin Serial communication
  while (!Serial)
  {
    ;
  }
  Serial.println("Starting up Modem"); //Send a note saying boot up

  pinSetup();
  modulationReset();
}

void pinSetup()
{
  pinModeFast(mod_pin, OUTPUT);
  pinModeFast(demod_pin, INPUT);
}

void modulationReset()
{
  digitalWriteFast(mod_pin, LOW);
  attachInterruptToPin();
  for (int i = 0; i < outgoingMessagesBytesIndex; i++)
  {
    outgoingMessagesBytes[i] = 0;
  }
  periodCounter = 0;
  outSymbolIndex = 0;
  outgoingMessagesBytesIndex = 0;
  messageSymbolSize = 0;
  outgoingMessagesBytes[outgoingMessagesBytesIndex++] = char('w');
  modulationActive = false;
}

void startModulation()
{
  detachInterrupt(digitalPinToInterrupt(demod_pin));
  modulationActive = true;
  outSymbolIndex = 0;
  periodCounter = 0;
  outgoingSineWaveDACTimer.begin(outgoingSquareWaveCallback, callbackUpdateSpeed);
  digitalWriteFast(mod_pin, LOW);
}

void outgoingSquareWaveCallback()
{
  digitalWriteFast(mod_pin, outputHigh);
  if (outputActive)
  {
    outputHigh = !outputHigh;
  }
  periodCounter++;
  if (periodCounter >= NUM_PERIODS * 2)
  {
    periodCounter = 0;
    outputActive = bitUpdate[++outSymbolIndex];
    if (outSymbolIndex >= messageSymbolSize)
    {
      outgoingSineWaveDACTimer.end();
      modulationReset();
    }
  }
}

int convertIncomingBitToOuputBool(int bit)
{
  if (bit == 0)
  {
    return false;
  }
  return true;
}

void addParityBitsToDACandPeriod(int characterPosition, int tempBits[])
{
  int parityBits[5];
  parityBits[0] = tempBits[6] ^ tempBits[5] ^ tempBits[3] ^ tempBits[2] ^ tempBits[0];
  parityBits[1] = tempBits[6] ^ tempBits[4] ^ tempBits[3] ^ tempBits[1] ^ tempBits[0];
  parityBits[2] = tempBits[5] ^ tempBits[4] ^ tempBits[3];
  parityBits[3] = tempBits[2] ^ tempBits[1] ^ tempBits[0];
  parityBits[4] = tempBits[0] ^ tempBits[1] ^ tempBits[2] ^ tempBits[3] ^ tempBits[4] ^ tempBits[5] ^ tempBits[6] ^ parityBits[0] ^ parityBits[1] ^ parityBits[2] ^ parityBits[3];
  for (int i = 0; i < 5; i++)
  {
    Serial.print(parityBits[i]);
    addBitToBitUpdateArray(characterPosition * 12 + 7 + i, parityBits[i]);
  }
  Serial.print(" ");
}

void addBitToBitUpdateArray(int index, int tempBit)
{
  bitUpdate[index] = convertIncomingBitToOuputBool(tempBit);
}

void readIncomingSerialDataForModulation()
{
  if (Serial.available())
  {
    outgoingMessagesBytes[outgoingMessagesBytesIndex++] = Serial.read();
  }
  if (outgoingMessagesBytes[outgoingMessagesBytesIndex - 1] == '\n')
  {
    Serial.println(outgoingMessagesBytes);
    //Serial.println(outgoingMessagesBytes);
    if (hammingActive)
    {
      messageSymbolSize = 12 * outgoingMessagesBytesIndex;
      for (int k = 0; k < outgoingMessagesBytesIndex; k++)
      {
        int tempBits[7];
        for (int i = 6; i >= 0; i--) //convert new byte into bits and then convert bits into half frequency times for interval timer
        {
          tempBits[(6 - i)] = (outgoingMessagesBytes[k] >> i) & 1;
          Serial.print(tempBits[(6 - i)]);
          addBitToBitUpdateArray(k * 12 + (6 - i), tempBits[(6 - i)]);
        }
        addParityBitsToDACandPeriod(k, tempBits);
      }
    }
    else
    {
      messageSymbolSize = 8 * outgoingMessagesBytesIndex;
      for (int k = 0; k < outgoingMessagesBytesIndex; k++)
      {
        for (int i = 7; i >= 0; i--) //convert new byte into bits and then convert bits into half frequency times for interval timer
        {
          int tempBit = (outgoingMessagesBytes[k] >> i) & 1;
          Serial.print(tempBit);
          addBitToBitUpdateArray(k * 8 + (7 - i), tempBit);
        }
        Serial.print(" ");
      }
    }
    Serial.println();
    Serial.println("Starting Message Modulation");
    startModulation();
  }
}

/////////Demodulation Functions/////////
bool checkForEndedMessageCondition()
{
  if (incomingSymbols[inSymbolIndex] == 0)
  {
    messageTimeOutCounter++;
    if (messageTimeOutCounter >= messageTimeOutThreshold)
    {
      endDemodulation();
      return true;
    }
  }
  else
  {
    messageTimeOutCounter = 0;
  }
  return false;
}

void checkForTimerBeingTooSlow(int average)
{
  if (lowSmallAverage != average)
  {
    shiftTimerForwardSmall();
  }
}

void checkForTimerBeingTooFast(int average)
{
  if (average != highSmallAverage)
  {
    shiftTimerForwardLarge();
  }
}

void shiftTimerForwardSmall()
{
  incomingSignalWavePingTimer.end();
  delayMicroseconds((int(numOfIndexesForShift / 2) - 1) * microsUpdatePerPing);
  incomingSignalWavePingTimer.begin(signalPingTimerCallback, microsUpdatePerPing);
}

void shiftTimerForwardLarge()
{
  incomingSignalWavePingTimer.end();
  incomingSymbols[inSymbolIndex++] = highSmallAverage;
  delayMicroseconds((PINGS_PER_SYMBOL - int(numOfIndexesForShift / 2) - 1) * microsUpdatePerPing);
  incomingSignalWavePingTimer.begin(signalPingTimerCallback, microsUpdatePerPing);
}

void signalPingTimerCallback()
{
  pingReading = digitalReadFast(demod_pin);
  sumOfPings += pingReading;
  pingCounter = (pingCounter + 1) % PINGS_PER_SYMBOL;
  if (pingCounter == numOfIndexesForShift)
  {
    lowSmallAverage = int(round(float(sumOfPings) / numOfIndexesForShift));
  }
  else
  {
    if (pingCounter >= PINGS_PER_SYMBOL - numOfIndexesForShift)
    {
      highSmallAverage += pingReading;
    }
    else
    {
      if (pingCounter == 0)
      {
        highSmallAverage = int(round(float(highSmallAverage) / numOfIndexesForShift));
        incomingSymbols[inSymbolIndex] = int(round(float(sumOfPings) / PINGS_PER_SYMBOL));
        if (checkForEndedMessageCondition())
        {
          return;
        }
        inSymbolIndex++;
        checkForTimerBeingTooSlow(incomingSymbols[inSymbolIndex - 1]);
        checkForTimerBeingTooFast(incomingSymbols[inSymbolIndex - 1]);
        sumOfPings = 0;
        highSmallAverage = 0;
        lowSmallAverage = 0;
      }
    }
  }
}

void correctForShiftedMessage()
{
  if (incomingSymbols[3] == 0)
  {
    return;
  }
  int pos = 0;
  while (incomingSymbols[pos] != 0)
  {
    pos++;
  }
  int correctAmount = abs(pos - 3);
  if (pos - 3 > 0)
  {
    for (int i = 0; i < inSymbolIndex; i++)
    {
      incomingSymbols[i] = incomingSymbols[i + correctAmount];
    }
    inSymbolIndex -= correctAmount;
  }
  else
  {
    for (int i = inSymbolIndex - 1; i >= 0; i--)
    {
      incomingSymbols[i + correctAmount] = incomingSymbols[i];
    }
    for (int i = 0; i < correctAmount; i++)
    {
      incomingSymbols[i] = 1;
    }
    inSymbolIndex += correctAmount;
  }
}

void endDemodulation()
{
  incomingSignalWavePingTimer.end();
  correctForShiftedMessage();
  convertSymbolsIntoMessageBytes();
  publishIncomingData();
  inSymbolIndex = 0;
  pingCounter = 0;
  messageTimeOutCounter = 0;
  for (int i = 0; i < inSymbolIndex; i++)
  {
    incomingSymbols[inSymbolIndex] = 0;
  }
  for (int i = 0; i < inSymbolIndex / 8 + 1; i++)
  {
    incomingMessageBytes[i] = 0;
  }
  attachInterruptToPin();
  incomingSignalDetected = false;
}

void publishBinaryMessage()
{
  for (int i = 0; i < inSymbolIndex; i++)
  {
    if (i % 12 == 0 && i != 0)
    {
      Serial.print(" ");
    }
    Serial.print(incomingSymbols[i]);
  }
  Serial.println();
}

void publishIncomingData() //publishes the available data in bytesIn to the Serial stream
{
  Serial.println(incomingMessageBytes + 1);
}

void startMessage()
{
  if (!incomingSignalDetected)
  {
    incomingSignalWavePingTimer.begin(signalPingTimerCallback, microsUpdatePerPing);
    detachInterrupt(digitalPinToInterrupt(demod_pin));
    incomingSignalDetected = true;
  }
}

void attachInterruptToPin()
{
  attachInterrupt(digitalPinToInterrupt(demod_pin), &startMessage, RISING);
}

void hammingCodeErrorCorrection()
{
  int parityBitChecks[5];
  for (int i = 0; i < inSymbolIndex; i += 12)
  {
    parityBitChecks[0] = (incomingSymbols[i + 6] ^ incomingSymbols[i + 5] ^ incomingSymbols[i + 3] ^ incomingSymbols[i + 2] ^ incomingSymbols[i + 0]) ^ incomingSymbols[i + 7];
    parityBitChecks[1] = (incomingSymbols[i + 6] ^ incomingSymbols[i + 4] ^ incomingSymbols[i + 3] ^ incomingSymbols[i + 1] ^ incomingSymbols[i + 0]) ^ incomingSymbols[i + 8];
    parityBitChecks[2] = (incomingSymbols[i + 5] ^ incomingSymbols[i + 4] ^ incomingSymbols[i + 3]) ^ incomingSymbols[i + 9];
    parityBitChecks[3] = (incomingSymbols[i + 2] ^ incomingSymbols[i + 1] ^ incomingSymbols[i + 0]) ^ incomingSymbols[i + 10];
    parityBitChecks[4] = (incomingSymbols[i + 0] ^ incomingSymbols[i + 1] ^ incomingSymbols[i + 2] ^ incomingSymbols[i + 3] ^ incomingSymbols[i + 4] ^ incomingSymbols[i + 5] ^ incomingSymbols[i + 6] ^ incomingSymbols[i + 7] ^ incomingSymbols[i + 8] ^ incomingSymbols[i + 9] ^ incomingSymbols[i + 10]) ^ incomingSymbols[i + 11];
    if (parityBitChecks[0] + parityBitChecks[1] + parityBitChecks[2] + parityBitChecks[3] > 0)
    { //if there is an error detected, then this if statement catches it
      if (parityBitChecks[4] == 0)
      {
        ; //Possible double bit error
      }
      else
      {
        int errorIndex = errorDetectionIndex(parityBitChecks[0] * 16 + parityBitChecks[1] * 8 + parityBitChecks[2] * 4 + parityBitChecks[3] * 2 + parityBitChecks[4]);
        if (incomingSymbols[i + errorIndex] == 0)
        {
          incomingSymbols[i + errorIndex] = 1;
        }
        else
        {
          incomingSymbols[i + errorIndex] = 0;
        }
      }
    }
  }
}

int errorDetectionIndex(int errorValue)
{
  switch (errorValue)
  {
  case 27:
    return 0;
  case 11:
    return 1;
  case 19:
    return 2;
  case 29:
    return 3;
  case 13:
    return 4;
  case 21:
    return 5;
  case 25:
    return 6;
  case 17:
    return 7;
  case 9:
    return 8;
  case 5:
    return 9;
  case 3:
    return 10;
  case 1:
    return 11;
  default:
    return -1;
  }
}

void convertSymbolsIntoMessageBytes()
{
  if (hammingActive)
  {
    for (int i = 0; i < inSymbolIndex; i += 12)
    {
      incomingMessageBytes[i / 12] = convertSymbolsToByteAtIndexWithHamming(i);
    }
    hammingCodeErrorCorrection();
    for (int i = 0; i < inSymbolIndex; i += 12)
    {
      incomingMessageBytes[i / 12] = convertSymbolsToByteAtIndexWithHamming(i);
    }
  }
  else
  {
    for (int i = 0; i < inSymbolIndex; i += 8)
    {
      incomingMessageBytes[i / 8] = convertSymbolsToByteAtIndex(i);
    }
  }
}

char convertSymbolsToByteAtIndex(int incomingSymbolsTempIndex)
{
  int byteAtIndex = (incomingSymbols[incomingSymbolsTempIndex + 7]) + (incomingSymbols[incomingSymbolsTempIndex + 6] * 2) +
                    (incomingSymbols[incomingSymbolsTempIndex + 5] * 4) + (incomingSymbols[incomingSymbolsTempIndex + 4] * 8) +
                    (incomingSymbols[incomingSymbolsTempIndex + 3] * 16) + (incomingSymbols[incomingSymbolsTempIndex + 2] * 32) +
                    (incomingSymbols[incomingSymbolsTempIndex + 1] * 64) + (incomingSymbols[incomingSymbolsTempIndex + 0] * 128);
  return char(byteAtIndex);
}

char convertSymbolsToByteAtIndexWithHamming(int incomingSymbolsTempIndex)
{
  int byteAtIndex = (incomingSymbols[incomingSymbolsTempIndex + 6]) + (incomingSymbols[incomingSymbolsTempIndex + 5] * 2) +
                    (incomingSymbols[incomingSymbolsTempIndex + 4] * 4) + (incomingSymbols[incomingSymbolsTempIndex + 3] * 8) +
                    (incomingSymbols[incomingSymbolsTempIndex + 2] * 16) + (incomingSymbols[incomingSymbolsTempIndex + 1] * 32) +
                    (incomingSymbols[incomingSymbolsTempIndex + 0] * 64);
  return char(byteAtIndex);
}

////////Shared Functions/////////
void loop()
{
  if (!modulationActive && !incomingSignalDetected)
  {
    readIncomingSerialDataForModulation();
  }
}