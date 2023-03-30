#include <OneWire.h>

#define OWPIN 3
#define PIOWr 0xA5

OneWire ds(OWPIN);

bool Fail = false;

void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
 int tReading, negBit=1;
  byte data1, data2;
  byte addr[8];
  float Humidity, Tc_100; 
  ds.reset();
  ds.write(0xCC,1);   // skip rom command
  ds.write(0x44,1);   // start conversion, with parasite power
  delay(850);         // wait for sensors to finished conversion

  while(ds.search(addr)){   // loop through, until all detected sensors are read
    //verify addr is uncorrupted and device is part of the DS18B20 or DS28EA00 family
     Serial.println("STARTED");
    if (OneWire::crc8(addr,7) == addr[7] && (addr[0] == 0x28 || addr[0] == 0x42)){
      ds.reset();
      ds.select(addr);    
      ds.write(0xBE, 1);   // Issue read scratchpad cmd
      data1=ds.read();  // Read only the first 2 bytes which contain temperature data
      data2=ds.read();
  
      tReading = (data2 << 8) + data1;  // combine the 2 temp data bytes, LSB came first, so assemble in reverse order
      if (tReading & 0x8000){           // determine if temp is a negative value
        negBit = -1;
        tReading = (tReading ^ 0xffff) + 1;
      } else {
        negBit = 1;
      }
      // convert reading to Celcius and Fahrenheit to desired decimal places
      Tc_100 = (float)((6 * tReading) + ((float)tReading / 4)) / (100) * negBit; //Adding floats forces the compiler to use them inside the calculations


      byte bin = ds.read();
      byte sensor = ds.read();

      if(addr[0] == 0x42){
        Humidity = getMoisture(addr);
      }
      Serial.print("Bin: ");
      Serial.print(bin);
      Serial.print(" Sensor: ");
      Serial.print(sensor);
      Serial.print(" Temp: ");
      Serial.print(Tc_100);
      Serial.print(" Humidity: ");
      Serial.println(Humidity);
      }
      

    }
  ds.reset_search();    // reset OneWire bus search
}


float getMoisture(byte addr[8]){
  Start(addr); //Starts I2C communication
  Tx(addr, 0x44 << 1); //Selects the sensor by address in write mode
  Tx(addr, 0x2C); //Sends conersion commands first byte
  Tx(addr, 0x06); //Second byte
  Stop(addr); //Stops
  delay(5); //Waits for conversion
  Start(addr); //Starts it again
  Tx(addr, (0x44 << 1) | 0x01); //Selects the sensor this time in read mode
  byte TM = Rx(addr, true); //Read most and least significant bytes of temperature readings
  byte TL = Rx(addr, true);
  Rx(addr, true); //Crc is not used
  byte HM = Rx(addr, true); //Reads most and least significant bytes of humidity reading
  byte HL = Rx(addr, true);
  Rx(addr, true); //CRC is not used
  Stop(addr);
  uint16_t HRd = ((HM << 8) + HL); //Combines the most and least significant bits. Note that it must be an unsigned integer, otherwise the sign will flip at 50% humidity
  float RH = (float)100*((float)HRd / 65535); //Performs calculation
  if(Fail == true){ //Checks if PIO error was detected
    return -97; //It returns -97 if it is a fail
  } else {
    return RH;  
  }

}

byte PIO(byte Bff){ //Writes to the DS28EA00 IO pins
  ds.write(Bff, 1); //Writes the value to the PIO port
  ds.write(Bff ^ 0b11111111, 1); //Writes the inverse as a simple form of error detecting
  ds.read(); //Reads the inverse. After a write the DS28EA00 will repeat what was written to confirm a successful write
  byte state = ds.read();  //Reads the PIO value
  if((Bff & 0b00000001) == ((state & 0b00000010) >> 1) && ((Bff & 0b00000010) == ((state & 0b00001000) >> 2))) { //Makes sure it matches 
    //all good
  } else {
    Fail = true; //Sets the fail flag
  }
  return state;
}


void Start(byte Addr[8]){ //Starts the I2C transmission
  byte bff = 0b11111100; //PIO buffer
  ds.reset(); //Resets bus
  ds.select(Addr); //Selects sensor
  ds.write(PIOWr, 1); //Starts PIO write mode
  bff |= 0b00000001; //Pulls SDA high
  PIO(bff); //Performs PIO Write
  bff |= 0b00000010; //Pulls SCK HIGH
  PIO(bff);
  bff &= 0b11111110; //Pulls SDA LOW
  PIO(bff);
  bff &= 0b11111100; //Pulls SCK LOW
  PIO(bff);
}

void Stop(byte Addr[8]){ //Stops I2C Transmission
  byte bff = 0b11111100; //PIO Buffer
  ds.reset(); //Resets Bus
  ds.select(Addr); //Selects sensor
  ds.write(PIOWr, 1); //Starts PIO write mode
  bff &= 0b11111110; //Pulls Data 
  PIO(bff);
  bff |= 0b00000010;
  PIO(bff);
  bff |= 0b00000001;
  PIO(bff);
}

bool Tx(byte Addr[8], byte dat){ //Transmits byte to I2C
  byte bff = 0b11111100;
  ds.reset(); //Resets, Selects, Starts PIO Write
  ds.select(Addr);
  ds.write(PIOWr, 1);
  for(byte i = 8; i; i--){ //Looks through until all bits are sent
    bff = 0b11111100; //Initiates buffer
    if(dat & 0x80){ //Checks if the MSB is a 1
      bff |= 0b00000001; //Sets SDA high
    }
    dat <<= 1; //Shifts through
    PIO(bff); 
    bff |= 0b00000010; //Writes SCL high
    PIO(bff);
    bff &= 0b11111101; //Writes SCL low
    PIO(bff);
  }
  bff = 0b11111111; //Writes everything high
  PIO(bff);
  bool ack = !(PIO(bff) & 0b00000001); //Checks for ACK pulse
  bff = 0b11111101; 
  PIO(bff);
  return ack;
}

byte Rx(byte Addr[8], bool ack){ //Recieves byte
  byte dat = 0; //Data to be returned
  byte bff = 0b11111101;
  ds.reset(); //Resets, Selects, and sets to PIO write
  ds.select(Addr);
  ds.write(PIOWr, 1);
  PIO(bff);
  for(byte i = 8; i; i--){ //Loops through until all 8 bits are read
    dat <<= 1; //Shifts left
    bff |= 0b00000010; //Sets SCL high
    PIO(bff);
    do{
      
    } while(!(PIO(bff) & 0b00000100)); //Input stretching
    
    if(PIO(bff) & 0b00000001){ //Reads bit
      dat |= 1;
    }
    bff &= 0b11111101; //Pulls SCL low
    PIO(bff);
  }
  if(ack){ //Sends ACK pulse
    bff &= 0b11111110;
    PIO(bff);
  } 
  bff |= 0b00000010;
  PIO(bff);
  bff &= 0b11111101;
  PIO(bff);
  bff |= 0b00000001;
  PIO(bff);
  return dat;
}
