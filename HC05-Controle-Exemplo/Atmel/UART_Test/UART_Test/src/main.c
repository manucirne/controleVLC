/************************************************************************
 * 5 semestre - Eng. da Computao - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Material:
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 *
 * Objetivo:
 *  - Demonstrar interrupção do PIO
 *
 * Periféricos:
 *  - PIO
 *  - PMC
 *
 * Log:
 *  - 10/2018: Criação
 ************************************************************************/

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão volume menos
#define BUT_PIOVm      PIOA
#define BUT_PIO_IDVm   ID_PIOA
#define BUT_IDXVm  24
#define BUT_IDX_MASKVm (1 << BUT_IDXVm)

// Botão volume mais
#define BUT_PIOVM      PIOA
#define BUT_PIO_IDVM   ID_PIOA
#define BUT_IDXVM  4
#define BUT_IDX_MASKVM (1 << BUT_IDXVM)

// Botão PLAY/PAUSE
#define BUT_PIOPP      PIOB
#define BUT_PIO_IDPP   ID_PIOB
#define BUT_IDXPP  4
#define BUT_IDX_MASKPP (1 << BUT_IDXPP)

// flags e variáveis voléteis
volatile int flagVmais = 0;
volatile int flagVmenos = 0;
volatile int flagPP = 0;
volatile int but_flag = 0;

volatile long g_systimer = 0;
volatile char str;

void SysTick_Handler() {
	g_systimer++;
}

// funções para funcionamento do bluetooth
void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 115200;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEControleVLC", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 1000);
	usart_log("hc05_server_init", buffer_rx);
}


/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 Callback dos botões 
 */
void vol_mais(void)
{
  flagVmais = 1;
  but_flag = 1;
  str = 'u';
}

void vol_menos(void)
{
	flagVmenos = 1;
	 but_flag = 1;
	 str = 'd';
}

void play_pause(void)
{
	//usart_put_string(USART1, "Inicializando...\r\n");
	str = 'p';
	flagPP = 1;
	 but_flag = 1;
}

void send(){
	if (flagPP){
		str = 'p';
		//usart_put_string(USART1, "Inicializando...\r\n");
		
	}
	if ((flagVmais) && (str != 'p')){
		str = 'u';
		
	} else if ((flagVmenos) && (str != 'p')){
		str = 'd';
		
	}
	flagVmenos = 0;
	flagVmais = 0;
	flagPP = 0;
	//usart_put_string(USART1 ,str);
	//but_flag = 1;
}
/************************************************************************/
/* inicialização                                                             */
/***********************************************************************/



// Inicializa botões
void io_init(void)
{

  // Configura led - testes
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT );

    //BOTAO VOLUME MENOS
    pmc_enable_periph_clk(BUT_PIO_IDVm);
    pio_configure(BUT_PIOVm, PIO_INPUT, BUT_IDX_MASKVm, PIO_PULLUP);
    
    pio_enable_interrupt(BUT_PIOVm, BUT_IDX_MASKVm);
 
	
	  pio_handler_set(BUT_PIOVm,
	  BUT_PIO_IDVm,
	  BUT_IDX_MASKVm,
	  PIO_IT_FALL_EDGE,
	  vol_menos);
  
      NVIC_EnableIRQ(BUT_PIO_IDVm);
      NVIC_SetPriority(BUT_PIO_IDVm, 4);
   

  

	
	 // Voluma mais
	 pmc_enable_periph_clk(BUT_PIO_IDVM);
	 pio_configure(BUT_PIOVM, PIO_INPUT, BUT_IDX_MASKVM, PIO_PULLUP);
	 pio_enable_interrupt(BUT_PIOVM, BUT_IDX_MASKVM);
	   pio_handler_set(BUT_PIOVM,
	   BUT_PIO_IDVM,
	   BUT_IDX_MASKVM,
	   PIO_IT_FALL_EDGE,
	   vol_mais);
	   
	   
	   NVIC_EnableIRQ(BUT_PIO_IDVM);
	   NVIC_SetPriority(BUT_PIO_IDVM, 4);

	
	//Botão Play Pause
	  pmc_enable_periph_clk(BUT_PIO_IDPP);
	  pio_configure(BUT_PIOPP, PIO_INPUT, BUT_IDX_MASKPP, PIO_PULLUP);
	  

	  // Ativa interrupção
	  pio_enable_interrupt(BUT_PIOPP, BUT_IDX_MASKPP);

  


  pio_handler_set(BUT_PIOPP,
  BUT_PIO_IDPP,
  BUT_IDX_MASKPP,
  PIO_IT_FALL_EDGE,
  play_pause);
  
	  // Configura NVIC para receber interrupcoes do PIO do botao
	  // com prioridade 4 (quanto mais próximo de 0 maior)
	  NVIC_EnableIRQ(BUT_PIO_IDPP);
	  NVIC_SetPriority(BUT_PIO_IDPP, 4);
	
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
void main(void)
{
	// Inicializa clock
	sysclk_init();

	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

  // configura botao com interrupcao

  sysclk_init();
  board_init();
  io_init();
  
  delay_init();
  SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
  config_console();
  send();
  
  #ifndef DEBUG_SERIAL
  usart_put_string(USART1, "Inicializando...\r\n");
  usart_put_string(USART1, "Config HC05 Server...\r\n");
  hc05_config_server();
  hc05_server_init();
  #endif
  char eof = 'X';
  char buffer[128];
  char buffer_rx[128];
  //str='3';
  flagPP = 0;
  flagVmais = 0;
  flagVmenos = 0;
  but_flag = 0;
	// super loop
	// aplicacoes embarcadas no devem sair do while(1).
	while(1)
  {
	  //usart_send_command(USART0, buffer, 1000, "Teas", 1000);
	  
	  // trata interrupção do botão
	  if(but_flag == 1){
		  //send();
		  while(!usart_is_tx_ready(USART0));//UART_COMM));
		  //usart_put_string(USART1, str);
		  //usart_write(USART0, str);
		  buffer[0] = str;
		  buffer[1] = eof;
		  buffer[2] = '\0';
		  usart_send_command(USART0, buffer_rx, 1000, buffer, 1000);
		  //while(!usart_is_tx_ready(USART0));//UART_COMM));
		  //usart_put_string(USART1, eof);
		  
		  //usart_send_command(USART0, buffer, 1000, eof, 1000);
		  //flagPP = 0;
		  //str = 'j';
		  but_flag = 0;
	  }
	  
	  //pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	  
	}
}
