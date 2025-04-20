
/*******************************************************
This program was created by the CodeWizardAVR V3.32 
Automatic Program Generator
?Copyright 1998-2017 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 2018-01-06
Author  : 
Company : 
Comments: 


Chip type               : ATmega128
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024
*******************************************************/
#define pump_addr 0x07

#include <mega128.h>
#include <delay.h>
#include <stdio.h>
#include <interrupt.h>
#define n_data 9
#define unused_addr 0x0f
// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (1<<ADLAR))

// Read the 8 most significant bits
// of the AD conversion result
unsigned char read_adc(unsigned char adc_input);



// Declare your global variables here
void Putch(unsigned char);
unsigned char Getch(void);  
void Put_string(unsigned char tx_string[], unsigned char n_size) ;
interrupt [USART0_RXC] void UART_RX(void);
void stringtohex(void);
unsigned char chartohex(unsigned char c);
void drive(void);
unsigned char valve_setting(unsigned char i);
void power_drive(unsigned char addr, unsigned char control);

void all_off(void);
void ctrun(void); 
unsigned char change_order(unsigned char t1);
unsigned char hextochar(unsigned char c);
 
          
unsigned char s_rx[15], as_rx[15], ct_status[10];
unsigned char request_ct=0, request_sensor=0;
unsigned char rx,tx;
unsigned char r_state=0, r_ready=0;

int com_error=0;

// Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{

all_off();// Place your code here
com_error=1;
}


void main(void)
{
unsigned char i,j, cont_temp;
unsigned char temp1, temp2;
//int com_error=0;
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=(1<<DDC7) | (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (0<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=(0<<DDF7) | (0<<DDF6) | (0<<DDF5) | (0<<DDF4) | (0<<DDF3) | (0<<DDF2) | (0<<DDF1) | (0<<DDF0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRG=(0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
ASSR=0<<AS0;
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 7.813 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 8.3886 s
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (1<<CS10); //일반모드, 프리스케일 = CK/1024
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer/Counter 3 initialization
// Clock source: System Clock
// Clock value: Timer3 Stopped
// Mode: Normal top=0xFFFF
// OC3A output: Disconnected
// OC3B output: Disconnected
// OC3C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer3 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR3A=(0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) | (0<<COM3C1) | (0<<COM3C0) | (0<<WGM31) | (0<<WGM30);
TCCR3B=(0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (0<<WGM32) | (0<<CS32) | (0<<CS31) | (0<<CS30);
TCNT3H=0x00;
TCNT3L=0x00;
ICR3H=0x00;
ICR3L=0x00;
OCR3AH=0x00;
OCR3AL=0x00;
OCR3BH=0x00;
OCR3BL=0x00;
OCR3CH=0x00;
OCR3CL=0x00;
                                                                         
// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
ETIMSK=(0<<TICIE3) | (0<<OCIE3A) | (0<<OCIE3B) | (0<<TOIE3) | (0<<OCIE3C) | (0<<OCIE1C);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
// INT3: Off
// INT4: Off
// INT5: Off
// INT6: Off
// INT7: Off
EICRA=(0<<ISC31) | (0<<ISC30) | (0<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EICRB=(0<<ISC71) | (0<<ISC70) | (0<<ISC61) | (0<<ISC60) | (0<<ISC51) | (0<<ISC50) | (0<<ISC41) | (0<<ISC40);
EIMSK=(0<<INT7) | (0<<INT6) | (0<<INT5) | (0<<INT4) | (0<<INT3) | (0<<INT2) | (0<<INT1) | (0<<INT0);

// USART0 initialization
// USART0 disabled
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

// USART1 initialization
// USART1 disabled
UCSR1B=(0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

// ADC initialization
// ADC Clock frequency: 1000.000 kHz
// ADC Voltage Reference: AREF pin
// Only the 8 most significant bits of
// the AD conversion result are used
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
SFIOR=(0<<ACME);


// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

 
    DDRC = 0xFF;   
    DDRA=0xFF;    
    DDRD=0x00; // ct runnning state input PORTD0=0 cT;
    
    DDRG=0x00;   //PORTG 0 1   ct 8 9

    
    UCSR0A = 0x0;
    UCSR0B = 0b10011000;  //  TXEN=1, RXEN=1
                          // RXCIE=1
    UCSR0C = 0b10000110;   // receive interrupt enable
    
    UBRR0H = 0;
    UBRR0L = 51;  //8 Mhz 9600 baud rate
      
    SREG=0x80;  //interrupt enable
    TCNT1 = 26473;            // 1/8us * 1024분주 * (65536-26473) = 5sec     
    
    //all_off();

 while(1){
     PORTC = 0xFF;     // LED ON              
     delay_ms(10);
     
    if(com_error=0){
      
     
     if(r_ready==1)
        {
         stringtohex();
         drive();
         r_ready=0;   
         }   
           
     if(request_ct==1)
        {
         Putch('\n');
         Putch('\r');  
    
         ctrun();
    
        
         Put_string(ct_status,6); 
     
         request_ct=0;
        }   
     

  } 
    
  if(request_sensor==1)
    {
       Putch('\n');
       Putch('\r');
       Putch('S');
          for (i=0;i<3;i++){
            temp1=read_adc(i);
            temp2=temp1>>4;
     
            Putch(hextochar(temp2));
            temp2=temp1&0x0f; 
            Putch(hextochar(temp2));  
            if(com_error==1)  Putch('E');  //error report
            else  Putch('K');        // no error ok
           }
     Putch('\n');
     Putch('\r'); 
     request_sensor=0;     
     }
     
    if(com_error=1){
         all_off();
        } 
  
      
     PORTC = 0x00;   //LED OFF
     delay_ms(10);
  
   
   }
 
    }


void Put_string(unsigned char tx_string[], unsigned char n_size)
    {
        int si;
        si=0;
        
        while(si<n_size)
        {
            Putch(tx_string[si]);
            si++;
        }
    
        
    }
void Putch(unsigned char data)
{
    while(!(UCSR0A & 0x20));  //UDRE0=1?    
    UDR0=data;
}

unsigned char Getch(void)
{
  
    while(!(UCSR0A&0x80)); //RXC=1?
    return UDR0;
}



interrupt [USART0_RXC] void UART_RX(void)
{
   TCNT1 = 26473; // 타이머 초기화 
   //count=0;
    //Q
    //Mxxx\r
    //Vxxxxxxxx\r
    rx=Getch();
  //  if(rx == '\r') {UDR0='\n';delay_ms(1);UDR0='\r';}
  //  else  UDR0= rx; 
      
    if(rx=='Q'){ request_ct=1; return;}      
    if(rx=='R'){ com_error=0; return;}    // 'R'을 받으면 com_error=0으로 만듬.
    if(rx=='S'){ request_sensor=1; return;}
    delay_ms(1); 
    if (r_ready==0){
       switch (r_state)
         {
         case 0 :
           if((rx=='V') || (rx=='M'))
            { as_rx[r_state]=rx;
               r_state=1;
            }
           else {r_state=0;
                 //Putch('0')
                 }  
         
           break; 
    
         default: { 
                as_rx[r_state]=rx;
                r_state++;
               // Putch('3');
                if(rx=='\r'){
                   r_ready=1; 
                   r_state=0;
                  // Putch('4');               
                  }           
                         
             
             break;
             }                                              
       }  
    
    }
      
   return; 
}

void stringtohex(void)
  {     unsigned  char i=0;
      unsigned char temp;
       s_rx[0]=as_rx[0]; 
       if(s_rx[0]=='V'){
           for(i=0;i<4;i++)
          { 
           temp=chartohex(as_rx[i+i+1]);
            s_rx[i+1]= temp<<4 | chartohex(as_rx[i+i+2]);}
          } 
       else  if(s_rx[0]=='M'){
         for(i=0;i<3;i++)
            { 
             s_rx[i+1]= chartohex(as_rx[i+1]);
            } 
      
        }
  }

unsigned char chartohex(unsigned char c)
  {
      if (c >= 'A')
          return c - 'A' + 10;
      else
         return c - '0';   
  }
  

void ctrun(void){

   unsigned char ct_temp1, ct_temp2; 
   ct_status[0]='#';
   ct_temp1=~PIND;
   ct_temp2=change_order(ct_temp1); // change order
    // hexa to ascii string
   ct_temp1=ct_temp2>>4;
   ct_status[1]=hextochar(ct_temp1);
   ct_temp1=ct_temp2 &0x0f; 
   ct_status[2]=hextochar(ct_temp1); 
  
   ct_temp1=~PING;
   ct_temp1= ct_temp1&0x03;
  
   ct_temp2=change_order(ct_temp1); // change order
   // hexa to ascii string
   ct_temp1=ct_temp2>>4;
   ct_status[3]=hextochar(ct_temp1);
   ct_temp1=ct_temp2 &0x0f; 
   ct_status[4]=hextochar(ct_temp1);
   ct_status[5]='\r';
  }
  
 
  unsigned char change_order(unsigned char t1){
   unsigned char t2;
   unsigned char i_temp;
   t2=0x00;
     for (i_temp=0;i_temp<8;i_temp++){
         if((t1<<i_temp) & 0x80) t2 = t2 | 1<<i_temp ;
        }
   return t2;
   }  
 
  unsigned char hextochar(unsigned char c){
    
        if (c >= 0x0A)
            return c + 'A' -0x0A;
        else
           return c + '0';   
    }
  
    


void drive(void){
 //Vnn nn nn nn ascii+hexa   
 //Mxxx   Ascii
unsigned char pump_control, valve_control;
unsigned char i=0;
  if(s_rx[0]=='M'){ 
     // Putch('Z');
      pump_control=0x00;
      for (i=0;i<3;i++)
     {
      if(s_rx[i+1]==0x01){ pump_control |= 1<<i;}
     }
   //  pump_control=0x01;
     pump_control=pump_control<<1;//  adjust position   
     
      power_drive(pump_addr,pump_control);      
    }
      
  
    if(s_rx[0]=='V'){
     
     for(i=0;i<6;i++){
     valve_control=valve_setting(i); // rearrange the order
                                    // drive the valves
      power_drive(i, valve_control);
                  
     }
  }    
  

}

void power_drive(unsigned char addr, unsigned char control)
{
 PORTA=addr;
 PORTC=control;  
 delay_us(10);
 PORTA=unused_addr;   /// unused address
}

void all_off(void){
    unsigned char ad1;
    for (ad1=0;ad1<16;ad1++)
    {
  
    if (ad1==pump_addr)power_drive(ad1,0x00);
    else power_drive(ad1,0xff);
    } 

    }

unsigned char valve_setting(unsigned char i)
{  unsigned char valve_temp=0x00;
   unsigned  char valve_con=0x00;
   unsigned char i1=0;
   valve_temp=0x00;
   switch (i)
          {
          case 0:
             valve_temp=s_rx[1];  // 123456xx
           
          break; 
          case 1:
             valve_temp=(s_rx[1]<<6) |(s_rx[2]>>2);  
              
             
          break;
          case 2:
             valve_temp=(s_rx[2]<<4) |(s_rx[3]>>4);
          break;
          case 3:
             valve_temp=(s_rx[3]<<2);
          break;
          case 4:
             valve_temp=(s_rx[4]);
          break;
          } 
          
    //       valve_temp=0x7C;
        
          
    // revwese the binary order     OK Below 
    valve_con=0x00;
    for (i1=0;i1<8;i1++){
        if((valve_temp<<i1) & 0x80) valve_con = valve_con | 1<<i1 ;
      }
           
   valve_con = ~ valve_con;       
   return (valve_con<<1);    //  adjust position c0 is not used
}


unsigned char read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCH;
}

/*
Q // 데이터 없고 대기상태 유지 
Ve0000000 // V Valve on-off + onoff 신호 (밸브 모두를 2진수로  30개 밸브 8자리,1,2,3번 켜기 e   )          
M111 (Pump motor 3개 제어 켜기 )
M111
M111
M111
M111
M111
M000 (Pump motor 3개 제어 끄기 )
V00000000  //  Valve off
V1c000000  0001 1100... 4,5,6 밸브 켜
M111
M111
M111
M111
M111
M111
M000
V00000000
V03800000
M111
M111
M111
M111
M111
M111
M000
V00000000
V00700000
M111
M111
M111
M111
M111
M111
M000
V00000000
V000e0000
M111
M111
M111
M111
M111
M111
M000
V00000000
V0001c000
M111
M111
M111
M111
M111
M111
M000
V00000000
V00003800
M111
M111
M111
M111
M111
M111
M000
V00000000
V00000700
M111
M111
M111
M111
M111
M111
M000
V00000000
V000000e0
M111
M111
M111
M111
M111
M111
M000
V00000000
V0000001c
M111
M111
M111
M111
M111
M111
M000
V00000000
Q
Q
Q
*/
/*
S // sensor data request

==> sensor data 보내기
R //restart
*/