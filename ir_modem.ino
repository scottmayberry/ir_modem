/*IR Modulation
 * Author: Scott Mayberry
 */
#include <digitalWriteFast.h>

#define OUTGOING_MESSAGE_MAXIMUM_SYMBOL_SIZE 2048
#define NUM_PERIODS 20
#define mod_pin 7
#define demod_pin 10

int messageSymbolSize;
bool bitUpdate[OUTGOING_MESSAGE_MAXIMUM_SYMBOL_SIZE];

char outgoingMessagesBytes[OUTGOING_MESSAGE_MAXIMUM_SYMBOL_SIZE + 1];
int outgoingMessagesBytesIndex;

volatile uint16_t periodCounter;
volatile uint16_t symbolIndex;
bool modulationActive;

bool outputHigh = false;
bool outputActive = false;

bool hammingActive = true;

const float frequency = 38000;
const float frequencyMicroSeconds = 1.0 / frequency * 1000000;
const float callbackUpdateSpeed = frequencyMicroSeconds / 2;

IntervalTimer outgoingSineWaveDACTimer;

void setup()
{
  Serial.begin(115200); //begin Serial communication
  while (!Serial)
  {
    ;
  }
  Serial.println("Starting up Modulation"); //Send a note saying boot up

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
  for (int i = 0; i < outgoingMessagesBytesIndex; i++)
  {
    outgoingMessagesBytes[i] = 0;
  }
  periodCounter = 0;
  symbolIndex = 0;
  outgoingMessagesBytesIndex = 0;
  messageSymbolSize = 0;
  outgoingMessagesBytes[outgoingMessagesBytesIndex++] = char('w');
  modulationActive = false;
}

void startModulation()
{
  modulationActive = true;
  symbolIndex = 0;
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
    outputActive = bitUpdate[++symbolIndex];
    if (symbolIndex >= messageSymbolSize)
    {
      outgoingSineWaveDACTimer.end();
      modulationReset();
    }
  }
}

void loop()
{
  if (!modulationActive)
  {
    readIncomingSerialDataForModulation();
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
