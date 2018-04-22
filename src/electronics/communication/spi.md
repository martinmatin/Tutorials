#Serial Peripheral Interface (SPI)
>*last updated on April 22, 2018*
> 

##Quick theoretical reminders 

The Serial Peripheral Interface bus (SPI) is a synchronous serial communication interface specification used for short distance communication, primarily in embedded systems. 

You can see on the picture below the topology of this communictaion interface and the pinout of the components:  

![](https://upload.wikimedia.org/wikipedia/commons/thumb/f/fc/SPI_three_slaves.svg/363px-SPI_three_slaves.svg.png)

To begin a communication with a *Slave*, the *Master* must fix the Slave Select pin (SS) to **LOW** state. After that, the master can put somme bit into the shift register throught the MOSI  (Master Output Slave Input) pin. There isn't any standard communiction protocol but the communiction frame is often writen for the use. When the communication is done, it's the master who must put back the SS to **HIGH** State.

> **Note** :
> It's a full duplex interface, The slave can push some data on his MISO (Master Input Slave Output) pin during the communication.


![](https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/SPI_8-bit_circular_transfer.svg/500px-SPI_8-bit_circular_transfer.svg.png)


###advantages 
+ Full duplex communication
+ Flexibility of the number of bits to be transmitted as well as the protocol itself
+ no possible collision

###disadvantage
+ \(3 + N*Slaves\) pins are required on the master
+ 4 pins are required on each slaves
+ Master can speak in a vacuum without knowing it
+ Bus can only count one master
 
##Use of our SPI

the communication frame we used to send data to the slave is:

1. Register adress on 8 bits
2. Number of bytes composing the message
3. Message


> **Note** :
> to read some data from a slave's register, the master must just send the register adress. The slave push the data on his MISO (Master Input Slave Output) pin.

###Our Master 
Create a SPI Master is really easy with a atmega328p and the arduino IDE. Indeed, we can find easily some libraries. We used the one provided directly by the arduino IDE in an object that we implemented : [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp)

how to work with [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object will be illustrated at least code use for Cortex's engines control (Eurobot 2018) : [MotorBroker](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/Ecam/examples/MotorControl/MotorBroker/MotorBroker.ino)

#####1. Initialisation of the SPI communication
The [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object must be seen as a communication channel with the slave. It will be necessary to instantiate one [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object by slave. When creating the object, use the number of the SS pin as an argument.

```c
//Define the SS pin for each slave
#define FL_SS  9
#define FR_SS  2
#define BL_SS  10
#define BR_SS  3

//Object Instantiation
SPIManager connFL(FL_SS);
SPIManager connFR(FR_SS);
SPIManager connBL(BL_SS);
SPIManager connBR(BR_SS);
```

Use the *initialize()* method to activate the communication channel. This call should be placed in the *setup()* function of the arduino code.

```c
connFL.initialize();
```

#####2. Write data to a Slave's register
as said above, to write a value in a register of the slave will first send the address of the register, then the size in number of byte and to finish, the message.

The *writedata* function has been written to make its use as intuitive as possible.
below, an example of use where we send to the register **16** (0x10), the message **x.b** whose size is **4 bytes**.

```c
connBR.writeData(0x10, 0x04, x.b);
```

#####3. Read data from a Slave's register
There are two functions to read some data: the first,*readLongData* , will return a value of type **long** and the other ,*readData* , a value of type **float32**

below, an example of use where we get some data from th register **81** (0x51).

```c
encoders_msg.rear_left = connBL.readLongData(0x51);
```


>from here the documentation is still being written


###Our Slaves
When using the Arduino IDE to program atmeg328p, the slave mode is not available. Two registers must be modified to *activate* this mode :

```c
// turn on SPI in slave mode
SPCR |= _BV(SPE);
// turn on interrupts
SPCR |= _BV(SPIE);
```
So that these manipulations are transparent for the user we also create an object for the SPI bus slaves : [SPISlave](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIslave.cpp)

this object has several methods :

1. begin()
2. reset()
3. com()

and some properties :

1. command
2. dataSize
3. msg
4. endtrans

#####1. Initialisation of the SPI communication
```c
SpiSlave mySPI;
//SLK  : pin 13
//MISO : pin 12
//MOSI : pin 11 
#define SS 10
```

```c
mySPI.begin();
```
#####2. Interruption
```c
//Interrupt needed by SPI Communication
ISR (SPI_STC_vect) {
    mySPI.com(SPDR); 
    spiReg();//Function called at the end of the communication.   
}
```
#####3. Registers
```c
void spiReg(){  
switch (mySPI.command) {
  case 0x10:
      if(mySPI.endTrans) {
          for (int i = 0; i < 4; i++){
              motor_speed.b[i] = mySPI.msg[i];
           }
       }
      break;
   case 0x11:
      SPDR = motor_speed.b[mySPI.dataCount - 2];
      break;
   case 0x50:
      motor_encoder.write(0);
      break;    
   case 0x51:
      SPDR = EncoderState.b[mySPI.dataCount - 2];
      break;    
  } 
}
```

    
###Troubleshooting

#####1. Problem between serial port (ROSSERIAL) and SPI bus
#####2. sometimes the slave update the value of a register with the value received previously 
#####3. Changing the values of a register when reading another register


# References
 [https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus)