
//Valter Rogério da Silva Júnior
//Microcontroladores
//822
//Jogo da Memória
#define F_CPU 16000000UL  //define a frequencia do microcontrolador - 16MHz
#include <avr/io.h>       //definições do componente especificado
#include <util/delay.h>   //biblioteca para o uso das rotinas de _delay_ms e _delay_us()
#include <avr/pgmspace.h>   //para o uso do PROGMEM, gravação de dados na memória flash

//Definições de macros para o trabalho com bits
#define set_bit(y,bit)  (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit)  (y&=~(1<<bit))  //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit)  (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit)  (y&(1<<bit))  //retorna 0 ou 1 conforme leitura do bit

//Definições para facilitar a troca dos pinos do hardware e facilitar a re-programação
#define DADOS_LCD     PORTD   //4 bits de dados do LCD no PORTD 
#define nibble_dados  1   //0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5, Px2-D6, Px3-D7) 
                //1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5, Px6-D6, Px7-D7) 
#define CONTR_LCD     PORTB   //PORT com os pinos de controle do LCD (pino R/W em 0).
#define E         PB1     //pino de habilitação do LCD (enable)
#define RS        PB0     //pino para informar se o dado é uma instrução ou caractere
#define tam_vetor 5 //número de digitos individuais para a conversão por ident_num()   
#define conv_ascii  48  //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

//sinal de habilitação para o LCD
#define pulso_enable()  _delay_us(1); set_bit(CONTR_LCD,E); _delay_us(1); clr_bit(CONTR_LCD,E); _delay_us(45)

//protótipo das funções
void cmd_LCD(unsigned char c, char cd);
void inic_LCD_4bits();    
void escreve_LCD(char *c);
void escreve_LCD_Flash(const char *c);
void ident_num(unsigned int valor, unsigned char *disp);


//---------------------------------------------------------------------------------------------
// Biblioteca time.h
//---------------------------------------------------------------------------------------------

/*
  time.h - low level time and date functions
*/

/*
  July 3 2011 - fixed elapsedSecsThisWeek macro (thanks Vincent Valdy for this)
              - fixed  daysToTime_t macro (thanks maniacbug)
*/     
 
#ifndef _Time_h
#ifdef __cplusplus
#define _Time_h

#include <inttypes.h>
#ifndef __AVR__
#include <sys/types.h> // for __time_t_defined, but avr libc lacks sys/types.h
#endif


#if !defined(__time_t_defined) // avoid conflict with newlib or other posix libc
typedef unsigned long time_t;
#endif


// This ugly hack allows us to define C++ overloaded functions, when included
// from within an extern "C", as newlib's sys/stat.h does.  Actually it is
// intended to include "time.h" from the C library (on ARM, but AVR does not
// have that file at all).  On Mac and Windows, the compiler will find this
// "Time.h" instead of the C library "time.h", so we may cause other weird
// and unpredictable effects by conflicting with the C library header "time.h",
// but at least this hack lets us define C++ functions as intended.  Hopefully
// nothing too terrible will result from overriding the C library header?!
extern "C++" {
typedef enum {timeNotSet, timeNeedsSync, timeSet
}  timeStatus_t ;

typedef enum {
    dowInvalid, dowSunday, dowMonday, dowTuesday, dowWednesday, dowThursday, dowFriday, dowSaturday
} timeDayOfWeek_t;

typedef enum {
    tmSecond, tmMinute, tmHour, tmWday, tmDay,tmMonth, tmYear, tmNbrFields
} tmByteFields;    

typedef struct  { 
  uint8_t Second; 
  uint8_t Minute; 
  uint8_t Hour; 
  uint8_t Wday;   // day of week, sunday is day 1
  uint8_t Day;
  uint8_t Month; 
  uint8_t Year;   // offset from 1970; 
}   tmElements_t, TimeElements, *tmElementsPtr_t;

//convenience macros to convert to and from tm years 
#define  tmYearToCalendar(Y) ((Y) + 1970)  // full four digit year 
#define  CalendarYrToTm(Y)   ((Y) - 1970)
#define  tmYearToY2k(Y)      ((Y) - 30)    // offset is from 2000
#define  y2kYearToTm(Y)      ((Y) + 30)   

typedef time_t(*getExternalTime)();
//typedef void  (*setExternalTime)(const time_t); // not used in this version


/*==============================================================================*/
/* Useful Constants */
#define SECS_PER_MIN  ((time_t)(60UL))
#define SECS_PER_HOUR ((time_t)(3600UL))
#define SECS_PER_DAY  ((time_t)(SECS_PER_HOUR * 24UL))
#define DAYS_PER_WEEK ((time_t)(7UL))
#define SECS_PER_WEEK ((time_t)(SECS_PER_DAY * DAYS_PER_WEEK))
#define SECS_PER_YEAR ((time_t)(SECS_PER_DAY * 365UL)) // TODO: ought to handle leap years
#define SECS_YR_2000  ((time_t)(946684800UL)) // the time at the start of y2k
 
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) ((_time_) % SECS_PER_MIN)  
#define numberOfMinutes(_time_) (((_time_) / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (((_time_) % SECS_PER_DAY) / SECS_PER_HOUR)
#define dayOfWeek(_time_) ((((_time_) / SECS_PER_DAY + 4)  % DAYS_PER_WEEK)+1) // 1 = Sunday
#define elapsedDays(_time_) ((_time_) / SECS_PER_DAY)  // this is number of days since Jan 1 1970
#define elapsedSecsToday(_time_) ((_time_) % SECS_PER_DAY)   // the number of seconds since last midnight 
// The following macros are used in calculating alarms and assume the clock is set to a date later than Jan 1 1971
// Always set the correct time before setting alarms
#define previousMidnight(_time_) (((_time_) / SECS_PER_DAY) * SECS_PER_DAY)  // time at the start of the given day
#define nextMidnight(_time_) (previousMidnight(_time_)  + SECS_PER_DAY)   // time at the end of the given day 
#define elapsedSecsThisWeek(_time_) (elapsedSecsToday(_time_) +  ((dayOfWeek(_time_)-1) * SECS_PER_DAY))   // note that week starts on day 1
#define previousSunday(_time_) ((_time_) - elapsedSecsThisWeek(_time_))      // time at the start of the week for the given time
#define nextSunday(_time_) (previousSunday(_time_)+SECS_PER_WEEK)          // time at the end of the week for the given time


/* Useful Macros for converting elapsed time to a time_t */
#define minutesToTime_t ((M)) ( (M) * SECS_PER_MIN)  
#define hoursToTime_t   ((H)) ( (H) * SECS_PER_HOUR)  
#define daysToTime_t    ((D)) ( (D) * SECS_PER_DAY) // fixed on Jul 22 2011
#define weeksToTime_t   ((W)) ( (W) * SECS_PER_WEEK)   

/*============================================================================*/
/*  time and date functions   */
int     hour();            // the hour now 
int     hour(time_t t);    // the hour for the given time
int     hourFormat12();    // the hour now in 12 hour format
int     hourFormat12(time_t t); // the hour for the given time in 12 hour format
uint8_t isAM();            // returns true if time now is AM
uint8_t isAM(time_t t);    // returns true the given time is AM
uint8_t isPM();            // returns true if time now is PM
uint8_t isPM(time_t t);    // returns true the given time is PM
int     minute();          // the minute now 
int     minute(time_t t);  // the minute for the given time
int     second();          // the second now 
int     second(time_t t);  // the second for the given time
int     day();             // the day now 
int     day(time_t t);     // the day for the given time
int     weekday();         // the weekday now (Sunday is day 1) 
int     weekday(time_t t); // the weekday for the given time 
int     month();           // the month now  (Jan is month 1)
int     month(time_t t);   // the month for the given time
int     year();            // the full four digit year: (2009, 2010 etc) 
int     year(time_t t);    // the year for the given time

time_t now();              // return the current time as seconds since Jan 1 1970 
void    setTime(time_t t);
void    setTime(int hr,int min,int sec,int day, int month, int yr);
void    adjustTime(long adjustment);

/* date strings */ 
#define dt_MAX_STRING_LEN 9 // length of longest date string (excluding terminating null)
char* monthStr(uint8_t month);
char* dayStr(uint8_t day);
char* monthShortStr(uint8_t month);
char* dayShortStr(uint8_t day);
  
/* time sync functions  */
timeStatus_t timeStatus(); // indicates if time has been set and recently synchronized
void    setSyncProvider( getExternalTime getTimeFunction); // identify the external time provider
void    setSyncInterval(time_t interval); // set the number of seconds between re-sync

/* low level functions to convert to and from system time                     */
void breakTime(time_t time, tmElements_t &tm);  // break time_t into elements
time_t makeTime(const tmElements_t &tm);  // convert time elements into time_t

} // extern "C++"
#endif // __cplusplus
#endif /* _Time_h */

//---------------------------------------------------------------------------------------------
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void cmd_LCD(unsigned char c, char cd)        //c é o dado  e cd indica se é instrução ou caractere
{
  if(cd==0)
    clr_bit(CONTR_LCD,RS);
  else
    set_bit(CONTR_LCD,RS);

  //primeiro nibble de dados - 4 MSB
  #if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);    
  #else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0)|(c>>4);  
  #endif

  pulso_enable();

  //segundo nibble de dados - 4 LSB
  #if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c<<4));   
  #else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
  #endif
  
  pulso_enable();

  if((cd==0) && (c<4))        //se for instrução de retorno ou limpeza espera LCD estar pronto
    _delay_ms(2);
}

//---------------------------------------------------------------------------------------------
//Sub-rotina para inicialização do LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void inic_LCD_4bits()   //sequência ditada pelo fabricando do circuito integrado HD44780
{             //o LCD será só escrito. Então, R/W é sempre zero.
  clr_bit(CONTR_LCD,RS);  //RS em zero indicando que o dado para o LCD será uma instrução 
  clr_bit(CONTR_LCD,E); //pino de habilitação em zero

  _delay_ms(20);      //tempo para estabilizar a tensão do LCD, após VCC ultrapassar 4.5 V (na prática pode
              //ser maior). 
          //interface de 8 bits           

  #if (nibble_dados)

    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;    

  #else   

    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;    

  #endif            

              

  pulso_enable();     //habilitação respeitando os tempos de resposta do LCD
  _delay_ms(5);   
  pulso_enable();
  _delay_us(200);
  pulso_enable(); /*até aqui ainda é uma interface de 8 bits.

          Muitos programadores desprezam os comandos acima, respeitando apenas o tempo de

          estabilização da tensão (geralmente funciona). Se o LCD não for inicializado primeiro no 

          modo de 8 bits, haverá problemas se o microcontrolador for inicializado e o display já o tiver sido.*/

  

  //interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)

  #if (nibble_dados) 

    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;    

  #else   

    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;

  #endif

  pulso_enable();   
    cmd_LCD(0x28,0);    //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas) 
              //são enviados os 2 nibbles (0x2 e 0x8)
    cmd_LCD(0x08,0);    //desliga o display
    cmd_LCD(0x01,0);    //limpa todo o display
    cmd_LCD(0x0C,0);    //mensagem aparente cursor inativo não piscando   
    cmd_LCD(0x80,0);    //inicializa cursor na primeira posição a esquerda - 1a linha
}

//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD -  dados armazenados na RAM
//---------------------------------------------------------------------------------------------
void escreve_LCD(char *c)
{
   for (; *c!=0;c++) cmd_LCD(*c,1);
}

//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na FLASH
//---------------------------------------------------------------------------------------------
void escreve_LCD_Flash(const char *c)
{
  for (;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}

//---------------------------------------------------------------------------------------------
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  for(n=0; n<tam_vetor; n++)
  disp[n] = 0 + conv_ascii;   //limpa vetor para armazenagem do digitos

  do
  {
    *disp = (valor%10) + conv_ascii;  //pega o resto da divisao por 10

    valor /=10;           //pega o inteiro da divisão por 10

    disp++;

  }while (valor!=0);
}
//---------------------------------------------------------------------------------------------

#define UP PC0   //PC0
#define LEFT PC1   //PC1 
#define SELECT PC2   //PC2 
#define DOWN PC3   // PC3 
#define RIGHT PC4   //PC4 

//----------------------------------------------------------------------------------------------------------------------------

char display[2][16] = {0}; //Conteúdo do display
char current_x = 0; //Conteúdo atual do x
char current_y = 0; //Conteúdo atual do y
char current_c_x = 0; //Coordenada x atual
char current_c_y = 0; //Coordenada y atual
char old_content_cursor = ' '; //Armazena o conteúdo anterior

char get_position(char row, char colunm) { //Posição x - y no displyt

  return 0x80 + row + (0x40 * colunm);
}

void display_position(char x, char y) { //Escolhe o lugar no display
   cmd_LCD(get_position(x, y), 0); //Posição escolhida
   current_x = x; //Atualiza as coordenadas nas variável x
   current_y = y; //Atualiza as coordenadas nas variável y
}

void clean_display() { //Função para limpar o display, também atualiza as coordenadas
    cmd_LCD(1,0); //Limpa display
    display_position(0, 0);
    old_content_cursor = ' '; //Limpa a variável em que o cursor acabou de passar
    current_x = 0; //Reinicia a posição
    current_y = 0; 
    for(int j = 0; j < 2; j++) //Não deixa display com espaços vazios
      for(int i = 0; i < 16; i++){
       display[j][i] = ' ';
    }
}


		void display_print_symbol(char c) { //Função que printa um caractere no display
    		if(current_x >= 16) { //Se a posição ultrapassar o visor do display na horizontal
     		 current_x = 0; //A coordenada x é levada ao in ício da próxima linha no display
     		   current_y++;
     		   if(current_y > 2){
     		     current_y = 0;
    		    }
  		      display_position(current_x, current_y); //Atualiza as coordenadas
   		 }
  	  cmd_LCD(c, 1); //Printa o caractere desejado
   			 display[current_y][current_x] = c; //Armazena o conteúdo colocado no display na matriz de conteúdos
   					 current_x++; //A coordenada x é deslocada  para a direita depois da escrita
}

void display_print_text(char *c) { //Função que printa uma string no display
    int i = 0;
    while(c[i] != NULL) { 
        display_print_symbol(c[i]);
        i++;
    }
}

void cursorSet(char x, char y) { //Função que printa o V como cursor na posição desejada 
    if(x < 0) { 
      x = 15;
    }
    if(x > 15) { 
      x = 0;
    }
    if(y < 0) { 
      y = 1;
    }
    if(y > 1) { 
      y = 0;
    }
  
    display_position(current_c_x, current_c_y); //Escolhe a posição atual do cursor
    display_print_symbol(old_content_cursor); //Mostra o simbolo escolhido
  
    old_content_cursor = display[y][x]; //Mantem o conteudo da matriz
    
    current_c_x = x; //Atualiza as coordenadas nas variáveis
    current_c_y = y; 
  
    display_position(current_c_x, current_c_y); //Escolhe a posição atual do cursor
    display_print_symbol('V'); //Põe o V de Valter como cursor
}

int main() {
    DDRC = 0b00000000; //Configura os pinos entradas
    PORTC= 0b11111111; //Pull-up nos botões
    DDRD = 0xFF; // PORTD como saída 
    DDRB = 0xFF; // PORTB como saída 
    
    clean_display(); //Limpa o display
    char symbols[2][16] = { {'!','@','#','$','%','&','*','(',')','+','{','}','/','>','<',']'}, //Matriz com os símbolos que aparecerão no display, são os simbolos do jogo
                            {'!','@','#','$','%','&','*','(',')','+','{','}','/','>','<',']'}}; 
    
    bool new_game = true; //Variável para novo jogo
    int incorrect = 0; //Variável para os erros
    int correct = 0; //Variável para os acertos
    unsigned char score[2]; //Armazena o valor da pontuação final
    
    inic_LCD_4bits(); //Inicia o display
    srand(time_t(0)); //Chama srand, gera um valor aleatório ao usar o rand()
    
    while(1) { //Laço infinito
        char first_select_position[2] = {0,0}; //Guarda os valores das posições
        
        if(new_game) { 
            correct = 0; //zera a pontuação 
            incorrect = 0;
            score[2] = 0;
            
            for(int i = 0; i < 16; i++) { //Aleatoriedade na segunda linha
                int r = rand() % 16;
                int temp = symbols[1][i];
                symbols[1][i] = symbols[1][r];
                symbols[1][r] = temp;
            }
              
            for(int j = 0; j < 2; j++) //Mostra os símbolos do jogo no display
                for(int i = 0; i < 16; i++){
                display_print_symbol(symbols[j][i]);
            }
            _delay_ms(2500); //Mostra por 2 segundos e meio
            clean_display(); //Limpa display
        }
        cursorSet(2,2); //Coloca o cursor X em um espaço no display (melhoria pro futuro seria fazer isto ser aleatório)
        new_game = false; //Verifica que já foi iniciado um novo jogo
        
      /*Escolha do primeiro simbolo*/
        while(tst_bit(PINC,SELECT)) { 
			            if(!tst_bit(PINC,RIGHT)) { //Move o cursor para a direita
			                cursorSet(current_c_x + 1, current_c_y);
			                while(!tst_bit(PINC,RIGHT)); //garante sair do loop somento após botão parar de ser pressionado
			            }
			            if(!tst_bit(PINC,LEFT)) { //Move o cursor para a esquerda
			                cursorSet(current_c_x - 1, current_c_y);
			                while(!tst_bit(PINC,LEFT)); //garante sair do loop somento após botão parar de ser pressionado
			            }
			            if(!tst_bit(PINC,UP)) { //Move o cursor para cima
			                cursorSet(current_c_x, current_c_y - 1);
			                while(!tst_bit(PINC,UP)); //garante sair do loop somento após botão parar de ser pressionado
			            }
			            if(!tst_bit(PINC,DOWN)) { //Move o cursor para baixo
			                cursorSet(current_c_x, current_c_y + 1);
			                while(!tst_bit(PINC,DOWN)); //garante sair do loop somento após botão parar de ser pressionado
			            }
			        }
        
        first_select_position[0] = current_c_x; //Armazena a coordenada x escolhida do primeiro símbolo
        first_select_position[1] = current_c_y; //Armazena a coordenada y escolhida do primeiro símbolo
    
        display_position(current_c_x, current_c_y); //Atualiza a posição do cursor
        old_content_cursor = symbols[current_c_y][current_c_x]; //Armazena o símbolo que deve aparecer no local do cursor
        display_print_symbol(symbols[current_c_y][current_c_x]); //Printa simbolo selecionado
        while(!tst_bit(PINC,SELECT)); //Só inicia após botão parar de ser pressionado
        
        /*Escolha do segundo simbolo*/
        while(tst_bit(PINC,SELECT)) { 
		            if(!tst_bit(PINC,RIGHT)) { //Move o cursor para a direita
		                cursorSet(current_c_x + 1, current_c_y);
		                while(!tst_bit(PINC,RIGHT)); //garante sair do loop somento após botão parar de ser pressionado
		            }
		            if(!tst_bit(PINC,LEFT)) { //Move o cursor para a esquerda
		                cursorSet(current_c_x - 1, current_c_y);
		                while(!tst_bit(PINC,LEFT)); //garante sair do loop somento após botão parar de ser pressionado
		            }
		            if(!tst_bit(PINC,UP)) { //Move o cursor para cima
		                cursorSet(current_c_x, current_c_y - 1);
		                while(!tst_bit(PINC,UP)); //garante sair do loop somento após botão parar de ser pressionado
		            }
		            if(!tst_bit(PINC,DOWN)) { //Move o cursor para baixo
		                cursorSet(current_c_x, current_c_y + 1);
		                while(!tst_bit(PINC,DOWN)); //garante sair do loop somento após botão parar de ser pressionado
		            }   
		        }
        
        display_position(current_c_x, current_c_y); //Atualiza a posição do cursor
        old_content_cursor = symbols[current_c_y][current_c_x]; //Armazena o símbolo que deve aparecer no local do cursor
        display_print_symbol(symbols[current_c_y][current_c_x]); //Printa simbolo selecionado
        _delay_ms(2000); //Tempo para mostrar o segundo símbolo selecionado
        
        if(symbols[current_c_y][current_c_x] == symbols[first_select_position[1]][first_select_position[0]]) { //Se os símbolos escolhidos na primeira escolha e na segunda forem iguais
            correct++; //Contabiliza o acerto
        }
       		 else { //Se os símbolos escolhidos não forem iguais
		            incorrect++; //Contabiliza o erro
		            display_position(current_c_x,current_c_y); //Atualiza a posição do cursor
		            display_print_symbol(' '); //Preenche com espaço
		            display_position(first_select_position[0], first_select_position[1]); //Atualiza a posição do cursor
		            display_print_symbol(' '); //Preenche com espaço
		            old_content_cursor = ' '; //Armazena o símbolo que deve aparecer no local do cursor
        }
		        if(incorrect == 4) { //Se errar mais de 4 vezes 
		            clean_display(); //Limpa display
		            display_position(0,0); //Move o cursor para o início do display
		            display_print_text("Game over!"); //Print de "game over"
		            display_position(0, 1); //Move a posição para essa função na segunda linha
		            display_print_text("Score: "); //Escreve "Score: "
		            ident_num(correct, score); //Pontuação em caracteres
		            new_game = true; //Novo jogo, simbolos em posições diferentes
		            display_print_symbol(score[1]); //Mostra o score de cada jogador separadamente
		            display_print_symbol(score[0]);
		            _delay_ms(2000); //2 Segundos para mostrar 
		        }
				        else if(correct == 16) { //Se acertar todos os símbolos
				            clean_display(); //Limpa display
				            display_position(0,0); //Move o cursor para o início do display
				            display_print_text("Victory!"); //Escreve "Victory!" no display
				            display_position(0,1); //Move a posição para essa função na segunda linha
				            display_print_text("Score: "); //Escreve "Score: "
				            ident_num(correct, score); //Pontuação em caracteres
				            new_game = true; //Novo jogo, simbolos em posições diferentes
				            display_print_symbol(score[1]); //Mostra o score de cada jogador separadamente
				            display_print_symbol(score[0]);
				            _delay_ms(2000); //2 Segundos para mostrar a imagem
        }
    }
}











