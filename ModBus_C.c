//MODBUS-RTU
//Daniel Koslopp e Talita Tobias Carneiro - 04/2013

#include <18f2550.h>
#fuses HSPLL,PLL4,NOWDT,NOPROTECT,NOLVP
#use delay(CLOCK=20000000)   

#use fast_io(A)
#use fast_io(B)
#use fast_io(C)


//Defines
#byte TXSTA=0xFAC                          
#byte SPBRGH=0xFB0
#byte SPBRG=0xFAF
#byte BAUDCON=0xFB8
#byte PIE1=0xF9D
#byte RCSTA=0xFAB
#byte INTCON=0xFF2
#byte RCON=0xFD0
#byte PIE2=0xFA0
#byte T3CON=0xFB1
#byte RCREG=0xFAE
#byte TXREG=0xFAD
#byte TMR3H=0xFB3
#byte TMR3L=0xFB2
#byte PIR2=0xFA1
#byte PIR1=0xF9E
#byte ADCON0=0xFC2
#byte ADCON1=0xFC1
#byte ADCON2=0xFC0
#byte ADRESH=0xFC4
//Defines LCD
#define RS PIN_C0    //RS no pino C0
#define EN PIN_C1    //EN no pino C1
#define D7 PIN_B7    //D7 LCD
#define D6 PIN_B6    //D6 LCD
#define D5 PIN_B5    //D5 LCD
#define D4 PIN_B4    //D4 LCD
#define lcd_comando(x)  lcd_escreve(0,x)  //Escreve comando LCD
#define lcd_letra(x)    lcd_escreve(1,x)  //Escreve letra LCD
//Estabiliza LCD
#define lcd_clear()\
        lcd_comando(0x01)\    
        delay_ms(2)
//Move cursor LCD                        
#define lcd_gotoxy(l,c)\   
        lcd_comando(0x80+0x40*l+c)
//Desliga cursor LCD
#define lcd_cursor_off()\
        lcd_comando(0b00001100)
//Liga cursor LCD
#define lcd_cursor_on()\
        lcd_comando(0b00001110)
//Pisca cursor LCD
#define lcd_cursor_pisca()\
        lcd_comando(0b00001111)
        
#define RESISTENCIA PIN_C2    //RESISTENCIA PIN_C2

//Bit Defines
#bit BRGH=TXSTA.2 
#bit SYNC=TXSTA.4
#bit TXEN=TXSTA.5 
#bit TX9=TXSTA.6
#bit TRMT=TXSTA.1
#bit SENDB=TXSTA.3
#bit BRG16=BAUDCON.3
#bit RXDTP=BAUDCON.5
#bit TXCKP=BAUDCON.4
#bit TXIE=PIE1.4
#bit RCIE=PIE1.5
#bit CREN=RCSTA.4
#bit RX9=RCSTA.6
#bit SPEN=RCSTA.7
#bit RCIF=RCSTA.5
#bit GIE=INTCON.7
#bit PEIE=INTCON.6
#bit IPEN=RCON.7
#bit TMR3IE=PIE2.1
#bit RD16=T3CON.7
#bit T3CKPS1=T3CON.5
#bit T3CKPS0=T3CON.4
#bit TMR3CS=T3CON.1
#bit TMR3ON=T3CON.0
#bit TMR3IF=PIR2.1
#bit TXIF=PIR1.4
#bit GO_DONE=ADCON0.1

//Variaveis Globais
int1 minha_msg=1; //Indica se o endereço se refere ao próprio slave           
int1 frame_ok=0;     //Indica se o frame recebido está completo
int1 crc_ok=0;    //Indica se mensagem está correta
int8 meu_endereco=1; //Endereço do slave na rede
int8 byte_msg=0;     //Indica indice do byte no vetor de transmissão ou recepção
int8 pos=0;   //Indica posição do registrador
int8 CRCL,CRCH; //Variaveis alt(H) e baixa(L) do CRC
int8 TEMP_MIN=50; //Temperatura em que o controlador irá desligar a saída
int8 TEMP_MAX=120;   //Temperatura em que o controlador irá ligar a saída
int16 CRC1=0xFFFF;  //Variavel para calcular CRC
char T[255];    //Buffer de transmissão
char R[255];    //Buffer de recepção
char ch0='0';   //Variaveis LCD
char ch1='0';   //||
char ch2='0';   //||

//Aux Functions

void lcd_escreve(boolean BITRS, int8 ch)  //Escreve no lcd em 4 bits
{
   output_bit(RS,BITRS);            //Comando BITRS=0 ou dados BITRS=1)
   delay_us(100);                   //Estabiliza pino LCD
   output_bit(D7,bit_test(ch,7));   //Envia nibble (high) ao barramento LCD  
   output_bit(D6,bit_test(ch,6));
   output_bit(D5,bit_test(ch,5));
   output_bit(D4,bit_test(ch,4));
   output_high(EN);                 //Enable vai para 1
   delay_us(1);                     //Establiza Enable
   output_low(EN);                  //Enable vai para 0 (envia comando)
   delay_us(100);                   //Estabiliza LCD
   output_bit(D7,bit_test(ch,3));   //Envia nibble (low) ao barramento LCD 
   output_bit(D6,bit_test(ch,2));
   output_bit(D5,bit_test(ch,1));
   output_bit(D4,bit_test(ch,0));
   output_high(EN);                 //Enable vai para 1
   delay_us(1);                     //Establiza Enable
   output_low(EN);                  //Enable vai para 0 (envia comando)
   delay_us(100);                   //Estabiliza LCD e aguarda LCD
}

void lcd_init()   // Inicializacao LCD
{
   delay_ms(30);
   lcd_escreve(0,0b00110010); //Function set RS-->0 D7AO D0-->00110010
   lcd_escreve(0,0b00101000); //Function set RS-->0 D7AO D0-->00101000
   lcd_escreve(0,0b00101000); //Function set RS-->0 D7AO D0-->00101000
   lcd_escreve(0,0b00001111); //Display on/off control RS-->0 D7AO D0-->00001111
   lcd_escreve(0,0b00000001); //Display clear  RS-->0 D7AO D0-->00000001
   delay_ms(2);               //Aguarda tempo do LCD para display clear
   lcd_escreve(0,0b00000110); //Entry mode set  RS-->0 D7AO D0-->00000110
}

void lcd_string(char ch)      //Escreve string no LCD
{
   switch(ch){
      case'\n':lcd_escreve(0,0xc0);   //Pula linha
               break;
      case'\f':lcd_escreve(0,0x01);   //Limpa
               delay_ms(2);
               break;   
      default:lcd_escreve(1,ch);
   }   
}

void  lcd_bintodec(int8 val)   //Escreve numero binario na forma decimal no LCD
{
   ch0='0';
   ch1='0';
   ch2='0';


   while(val>99)
   {
      val=val-100;
      ch2++;
   }
   while(val>9)
   {
      val=val-10;
      ch1++;
   }
   ch0=val+'0';
}

void calc_crc(int8 b)   //Calcula CRC
{
   int1 flag;
   CRC1=CRC1^b;
   for(int8 i=0;i<8;i++)
   {
      flag=CRC1&0x0001;
      CRC1=CRC1>>1;
      if(flag==1) CRC1=CRC1^0xA001;
   }
}

int1 verifica_msg()  //Verifica integridade da mensagem (somente para function 3)
{
   CRC1=0xFFFF;
   for(int i=0;i<6;i++)
   {
      calc_crc(R[i]);
   }
   CRCL=CRC1&0x00FF;
   CRC1=CRC1>>8;
   CRCH=CRC1&0x00FF;
   if(CRCL==R[6] && CRCH==R[7]) return 1;
   else return 0;
}
void monta_crc()
{
   CRC1=0xFFFF;
   for(int i=0;i<(T[2]+3);i++)
   {
      calc_crc(T[i]);
   }
   T[3+T[2]]=CRC1&0x00FF; //CRCL
   CRC1=CRC1>>8;
   T[4+T[2]]=CRC1&0x00FF; //CRCH
}

void envia_serial()
{

   for(int8 i=0;i<T[2]+5;i++)
   {
      TXREG=T[i];
      while(!(TRMT));
   }
}

int8 verifica_reg() //Verifica registrador de acordo com variável pos.
                    //Utilizado no trata_msg  ()
{
   if (pos == 0) return ADRESH; 
   if (pos == 1) return TEMP_MIN;
   if (pos == 2) return TEMP_MAX; 
}

void trata_msg () //Trata mensagem de acordo com função. Caso msg com erro
                  // (crc_ok == 0) faz tratamento do erro
{
      switch (R[1])  //Seleciona função
      {
         case 3:  //Função 3 (Holding Register)*
         {
            pos = R[3];
            for (int8 i = 0; i < 3-R[3]; i++)
               {
                  T[4 + i * 2] = verifica_reg () ;
                  pos++;
               }
         }
         break;
      }
      T[0] = R[0];
      T[1] = R[1];
      T[2] = R[5] * 2; //Indica número de bytes da resposta do slave
                       //Calcula CRC16 e envia T[T[2] + 3] = CRCL e T[T[2] + 4] = CRCH
      T[3] = 0;
      T[5] = 0;
      T[7] = 0;
}

void config_modbus () //Configurações para comunicação EUSART
{
   set_tris_a (0xFF); //Ports como entrada. Quando EUSART
   set_tris_b (0x0F); //habilitada, TRISC é
   set_tris_c (0b111111000); //ajustado automaticamente
   //Velocidade de comunicação
   BRGH=1; //BRGH e BRG16 configurados para:
   BRG16=0; //Baud Rate = 20M / [16 (n + 1) ]
   SPBRGH = 0x00; //n = 129 = 0081h para BD = 9600 bps
   SPBRG = 129;
   //Transmissao
   SYNC=0; //Modo Assincrono
   TXEN=1;//Hab. Transmissão
   TX9=0; //Transmissão a 8 bits
   TXIE=0; //Desab. Intr. Transmissão
   //Recepcao
   RXDTP=0; //Dados recebidos não são invertidos
   TXCKP=0; //Dados transmitidos não são invertidos
   RCIE=1; //Hab. Intr. Recepção
   CREN=1; //Hab. recebimento continuo
   SPEN=1; //Hab. porta serial (RX e TX como portas seriais)
   RX9=0; //Desabilita 9º bit (sem paridade)
   //Interrupcao
   GIE=1; //Hab. Interrupções
   PEIE=1; //Hab. Intr. periféricas
   IPEN=1; //Hab. Intr. de baixa prioridade
   TMR3IE=1; //Hab. Intr. TMR3
   //Timer3
   TMR3ON=0; //TMR3 desligado
   TMR3CS=0; //TMR3 com clock interno (Fosc / 4)
   RD16=1; //TMR3 modo 16 bits
   T3CKPS1=0; //T3CKPS1:T3CKPS0 = 01 - > Prescale = 1:2
   T3CKPS0=1;
}

void config_AD()
{
   ADCON0=0b00000001;   //0000(RA0/AN0)00X1(Hab. AD) 
   ADCON1=0b00001110;   //0000(Sem referencia)1110(AN0 habilitada)
   ADCON2=0b00001010;   //0(Left Justified)X001(2 TAD)010(Fosc/32)
}

#int_rda
void trata_recepcao  ()
{
   RCIF=0;  //Limpa flag de interrupção serial
   R[byte_msg] = RCREG; //Salva dado na posição determinda
   if (minha_msg == 1) //Mensagem para este slave
   {
      if (byte_msg == 0) //Primeira mensagem (endereço)
      {
         //R[0] = RCREG; //Recebe Endereço
            if (R[0] == 0||R[0] == meu_endereco) //Se é este endereço ou broadcast
            {
               minha_msg = 1; //É este endereço
               TMR3H = 0xBC; //Confg. TMR3 para 1, 5char = 1, 5 * 1, 145ms = 1, 72ms
               TMR3L = 0xCF; //Ncyc = (1, 72ms * 20e6) / 2 = 17200 = 4330h
                             //FFFF - 4330 = BCCF
                             //Com estouro de TMR3, tem - se fim da mensagem ou erro, de acordo
                             //com o resultado obtido no CRC      
               TMR3ON = 1;   //Habilita timer3                      
            }
            else
            {
               minha_msg = 0; //Endereço incorreto, então mensagem não é do slave
               TMR3H = 0x63; //Confg. TMR3 para 3, 5char = 3, 5 * 1, 145ms = 4, 01ms
               TMR3L = 0x5B; //Ncyc = (4, 01ms * 20e6) / 2 = 40100 = 9CA4h
                             //FFFF - 9CA4 = 635B
                             //Com estouro de TMR3, tem - se fim da mensagem para outro nó e
                             //este slave fica pronto para varificar a próxima mensagem
               TMR3ON = 1;   //Habilita timer3
            }
      }
      byte_msg++; //Incrementa posição do próximo dado
      TMR3H = 0xBC; //Confg. TMR3 para 1, 5char = 1, 5 * 1, 145ms = 1, 72ms
      TMR3L = 0xCF; //Ncyc = (1, 72ms * 20e6) / 2 = 17200 = 4330h
                    //FFFF - 4330 = BCCF
                    //Com estouro de TMR3, tem - se fim da mensagem ou erro, de acordo
                    //com o resultado obtido no CRC
      TMR3ON=1;     //Habilita timer3
   }
   else
   {
      minha_msg = 0; //Endereço incorreto, então mensagem não é do slave
      TMR3H = 0x63; //Confg. TMR3 para 3, 5char = 3, 5 * 1, 145ms = 4, 01ms
      TMR3L = 0x5B; //Ncyc = (4, 01ms * 20e6) / 2 = 40100 = 9CA4h
                    //FFFF - 9CA4 = 635B
                    //Com estouro de TMR3, tem - se fim da mensagem para outro nó e
                    //este slave fica pronto para varificar a próxima mensagem
      TMR3ON=1;     //Habilita timer3
   }
}

#int_timer3
void trata_timer3  ()
{
   TMR3IF = 0; //Limpa Flag Intr. TMR3
   TMR3ON = 0; //Desliga TMR3
   if (minha_msg == 1) //Mensagem recebida completa
   {

      crc_ok=verifica_msg(); //Verifica CRC da mensagem recebida - > crc_ok = 1
                             //CRC Nok - > crc_ok = 0
      if (crc_ok == 1)
      {
         trata_msg (); //Faz tratamento da mensagem recebida para enviar resposta T[x]
                    //envia respota

         monta_crc();  //Monta CRCL e CRCH para envio
         envia_serial();
      }
      else  //Mensagem não está ok, faz tratamento de erro.
      {

      }
      minha_msg = 1;
      byte_msg = 0;
      pos=0;
   }
   if (minha_msg == 0)
   {
      minha_msg = 1;
      byte_msg = 0;
      pos=0;
   }
}

void main()
{
   config_modbus();
   config_AD();
   lcd_init(); //Inicia LCD
   lcd_cursor_on(); //Desliga cursor LCD
   while (1)
   {
      GO_DONE=1;
      while(GO_DONE);
//!      lcd_gotoxy(0,0);
//!      for(int8 i=0;i<8;i++)
//!      {
//!         if(i==4) lcd_gotoxy(1,0);
//!         lcd_bintodec(R[i]);
//!         lcd_string(ch2);
//!         lcd_string(ch1);
//!         lcd_string(ch0);
//!      }
      if(ADRESH<TEMP_MIN) output_high(RESISTENCIA);
      if(ADRESH>TEMP_MAX) output_low(RESISTENCIA);
   }
}

