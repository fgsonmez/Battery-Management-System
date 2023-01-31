#include <SPI.h>
#include<stdint.h>
#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10

#define pot_pin0 A0
#define pot_pin1 A1
#define pot_pin2 A2
#define pot_pin3 A3
#define pot_pin4 A4
#define pot_pin5 A5

char dataBuff[500];
uint16_t deger[6] = {0,0,0,0,0,0};
float gerilim[8] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
char tx_buf[100];
uint16_t tx;
void SPI_SlaveInit(void)
{ 
 #if 0 
  // Initialize SPI pins.
  pinMode(SPI_SCK, INPUT);
  pinMode(SPI_MOSI, INPUT);
  pinMode(SPI_MISO, OUTPUT);
  pinMode(SPI_SS, INPUT);
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
 #endif 
   // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  //make SPI as slave
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}


//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}


//sends one byte of data 
void SPI_SlaveTransmit(char data)
{
  /* Start transmission */
  SPDR = data;
  
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}


void setup() {

  Serial.begin(9600);
  Serial.println("Pot Değer Okuma");

  SPI_SlaveInit();

  Serial.println("Slave Initialized");
}

 uint16_t dataLen = 0;
  uint32_t i = 0;

void loop() {

  deger[0] = analogRead(pot_pin0);
  deger[1] = analogRead(pot_pin1);
  deger[2] = analogRead(pot_pin2);
  deger[3] = analogRead(pot_pin3);
  deger[4] = analogRead(pot_pin4);
  deger[5] = analogRead(pot_pin5);

  for(int i=0; i<6; i++)
  {
      Serial.println(deger[i]);
  }
  Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) );
  for(int i=0; i<6; i++)
  {
    SPI.transfer16(deger[i]);
  }
  

  /* ADC Value to Char 
  deger = analogRead(pot_pin0);
  gerilim[0] = (5.00/1024.00)*deger;
  deger = analogRead(pot_pin1);
  gerilim[1] = (5.00/1024.00)*deger;
  deger = analogRead(pot_pin2);
  gerilim[2] = (5.00/1024.00)*deger;
  deger = analogRead(pot_pin3);
  gerilim[3] = (5.00/1024.00)*deger;
  deger = analogRead(pot_pin4);
  gerilim[4] = (5.00/1024.00)*deger;
  deger = analogRead(pot_pin5);
  gerilim[5] = (5.00/1024.00)*deger;
  
  for(int i=0; i<6; i++)
  {
    Serial.print(gerilim[i]);
    Serial.print(',');
  }
  Serial.println(' ');
  char buf[6];
  int m =0;
  int counter = 0;
  int a =0;
  for(int k=0; k<6; k++)
  { 
    dtostrf(gerilim[m],4,2,buf);
    for(int i=0; i<4; i++)
    {
      if(k>0)
      {
        counter = (k*5)+i;
      }
      else
      {
        counter = (k*4)+i;
      }
      tx_buf[counter] = buf[i];
    }
    if(k == 0){
      tx_buf[k*4+4] = ',';
      
    }
    else if(k == 5)
    {
      
    }
    else{
      tx_buf[k*4+5+a] = ',';
      a++;
    }

    
    m++;
  }
   Serial.println(tx_buf);
/*

   
/*  SPI Recive Code
    Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) );

  i = 0;
  dataLen = SPI_SlaveReceive();
  for(i = 0 ; i < dataLen ; i++ )
  {
    dataBuff[i] =  SPI_SlaveReceive();
  }

    dataBuff[i] = '\0';
  
  Serial.println("Rcvd:");
  Serial.println(dataBuff);
*/
  
  /*SPI Send Code
  
  uint8_t tx_buf_size = sizeof(tx_buf);
  
 Serial.println("Slave waiting for ss to go low to transmite tx size");
 while(digitalRead(SS) );
  SPI_SlaveTransmit(tx_buf_size);
  tx = SPI_SlaveReceive();

  for(int i = 0; i < 6; i++)
  {
    SPI_SlaveTransmit(tx_buf[i]);
    tx_buf[i] = SPI_SlaveReceive();
  }
*/
}
