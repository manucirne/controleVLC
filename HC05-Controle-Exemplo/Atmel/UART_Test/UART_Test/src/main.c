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

volatile int flagVmais = 0;
volatile int flagVmenos = 0;
volatile int flagPP = 0;
volatile int but_flag = 0;

volatile long g_systimer = 0;

void SysTick_Handler() {
	g_systimer++;
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
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
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 * Exemplo de callback para o botao, sempre que acontecer
 * ira piscar o led por 5 vezes
 *
 * !! Isso é um exemplo ruim, nao deve ser feito na pratica, !!
 * !! pois nao se deve usar delays dentro de interrupcoes    !!
 */
void vol_mais(void)
{
  flagVmais = 1;
}

void vol_menos(void)
{
	flagVmenos = 1;
}

void play_pause(void)
{
	flagPP = 1;
}



void volume(){
	if (flagVmais){
		but_flag = 1;
		
	}
	
	if (flagVmenos){
		for (int i=0;i<7;i++)
		{
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(500);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(500);
			flagVmais = 0;
		}
	}
}

void play(){
	//precisa saber se o filme está ou não rodando pra colocar a condição
	char str;
	if(flagPP){
		str = 'p';
	}
	if (flagVmais){
		str = 'u';
	}
	if (flagVmenos){
		str = 'd';
	}
	usart_put_string(USART1 ,str, strlen(str));
}
/************************************************************************/
/* funções                                                              */
/***

*********************************************************************/

void sendComand(char h, char t, char v){
	usart_putchar(USAR0, h);
	usart
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{

  // Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT );

  // Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_IDVM);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUT_PIOVM, PIO_INPUT, BUT_IDX_MASKVM, PIO_PULLUP);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUT_PIO_IDVM,
                  BUT_PIO_IDVM,
                  BUT_IDX_MASKVM,
                  PIO_IT_FALL_EDGE,
                  vol_mais);

  // Ativa interrupção
  pio_enable_interrupt(BUT_PIOVm, BUT_IDX_MASKVM);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUT_PIO_IDVM);
  NVIC_SetPriority(BUT_PIO_IDVM, 4); // Prioridade 4
  
  //BOTAO VOLUME MENOS
    // Inicializa clock do periférico PIO responsavel pelo botao
    pmc_enable_periph_clk(BUT_PIO_IDVm);

    // Configura PIO para lidar com o pino do botão como entrada
    // com pull-up
    pio_configure(BUT_PIOVm, PIO_INPUT, BUT_IDX_MASKVm, PIO_PULLUP);

    // Configura interrupção no pino referente ao botao e associa
    // função de callback caso uma interrupção for gerada
    // a função de callback é a: but_callback()
    pio_handler_set(BUT_PIO_IDVm,
    BUT_PIO_IDVm,
    BUT_IDX_MASKVm,
    PIO_IT_FALL_EDGE,
    vol_menos);

    // Ativa interrupção
    pio_enable_interrupt(BUT_PIOVm, BUT_IDX_MASKVm);

    // Configura NVIC para receber interrupcoes do PIO do botao
    // com prioridade 4 (quanto mais próximo de 0 maior)
    NVIC_EnableIRQ(BUT_PIO_IDVm);
    NVIC_SetPriority(BUT_PIO_IDVm, 4); // Prioridade 4
	
	//Botão Play Pause
	
	  // Inicializa clock do periférico PIO responsavel pelo botao
	  pmc_enable_periph_clk(BUT_PIO_IDPP);

	  // Configura PIO para lidar com o pino do botão como entrada
	  // com pull-up
	  pio_configure(BUT_PIOPP, PIO_INPUT, BUT_IDX_MASKPP, PIO_PULLUP);

	  // Configura interrupção no pino referente ao botao e associa
	  // função de callback caso uma interrupção for gerada
	  // a função de callback é a: but_callback()
	  pio_handler_set(BUT_PIO_IDPP,
	  BUT_PIO_IDPP,
	  BUT_IDX_MASKPP,
	  PIO_IT_FALL_EDGE,
	  play_pause);

	  // Ativa interrupção
	  pio_enable_interrupt(BUT_PIOPP, BUT_IDX_MASKPP);

	  // Configura NVIC para receber interrupcoes do PIO do botao
	  // com prioridade 4 (quanto mais próximo de 0 maior)
	  NVIC_EnableIRQ(BUT_PIO_IDPP);
	  NVIC_SetPriority(BUT_PIO_IDPP, 0); // Prioridade 4
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
  io_init();
  
  board_init();
  sysclk_init();
  delay_init();
  SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
  config_console();
  
  #ifndef DEBUG_SERIAL
  usart_put_string(USART1, "Inicializando...\r\n");
  usart_put_string(USART1, "Config HC05 Server...\r\n");
  hc05_config_server();
  hc05_server_init();
  #endif
  
  char button1 = '0';
  char eof = 'X';
  char buffer[1024];

	// super loop
	// aplicacoes embarcadas no devem sair do while(1).
	while(1)
  {
	  
	  // trata interrupção do botão
	  if(but_flag){
		  while(!usart_is_tx_ready(UART_COMM));
		  usart_write(UART_COMM, button1);
		  while(!usart_is_tx_ready(UART_COMM));
		  usart_write(UART_COMM, eof);
		  but_flag = false;
	  }
	  play();
	  volume();
	  pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	  
	}
}
