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
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "asf.h"

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

// vibra
#define VIB_PIO      PIOC
#define VIB_PIO_ID   ID_PIOC
#define VIB_IDX      13
#define VIB_IDX_MASK (1 << VIB_IDX)

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

// Botão ON/OFF
#define BUT_PIOLD      PIOD
#define BUT_PIO_IDLD   ID_PIOD
#define BUT_IDXLD  26
#define BUT_IDX_MASKLD (1 << BUT_IDXLD)

// LED ON/OFF
#define LED_PIOLD      PIOD
#define LED_PIO_IDLD   ID_PIOD
#define LED_IDXLD  11
#define LED_IDX_MASKLD (1 << LED_IDXLD)


// flags e variáveis voléteis
volatile int flagVmais = 0;
volatile int flagVmenos = 0;
volatile int flagPP = 0;
volatile int but_flag = 0;
volatile int on = 1;

/************************************************************************/
/* Globals  FFRW                                                            */
/************************************************************************/
/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)



/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool flag_foward = false;
volatile bool flag_neutral = false;
volatile bool flag_back = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11

#define AFEC_CHANNEL_POT 5

/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

/**
 * \brief AFEC interrupt callback function.
 */
static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT);
	g_is_conversion_done = true;
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */



static void configure_console(void)
{

	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * converte valor lido do ADC para temperatura em graus celsius
 * input : ADC reg value
 * output: Temperature in celsius
 */
static int32_t convert_adc_to_temp(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;

  /*
   * converte bits -> tensão (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_temp);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void config_ADC_TEMP(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_5,	AFEC_Temp_callback, 1);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Seleciona canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT);
}

/*
void TC1_Handler(void){
	volatile uint32_t ul_dummy; 
	/ ****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	****************************************************************** /
	ul_dummy = tc_get_status(TC0, 1);
	/ * Avoid compiler warning * /
	UNUSED(ul_dummy);
	if(g_is_conversion_done == true) {
		g_is_conversion_done = false;
		//printf("Temp : %d \r\n", convert_adc_to_temp(g_ul_value));
		//afec_start_software_conversion(AFEC0);
		
	}
}*/
void TC2_Handler(void){
	volatile uint32_t ul_dummy; 

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	char str;
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	str = 't';
	if(g_is_conversion_done == true) {
		g_is_conversion_done = false;
	}
	afec_start_software_conversion(AFEC0);
	if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) > 2700){
		flag_foward = true; //não esquecer de dar false nas flags novas no send
		printf("f");
		str = 'f';
		but_flag = true;  
	}
	if((afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) > 1300) && (afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) < 2700)){
		flag_neutral = true;  //não esquecer de dar false nas flags novas no send
		printf("n");
		str = 'n';
	}
	if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) < 1300){
		flag_back = true;  //não esquecer de dar false nas flags novas no send
		printf("b");
		str = 'b';
		but_flag = true;
	}
	//printf("Teste POT:  %d \n", afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT));
		

}



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


void vibra(void){
	pio_set(VIB_PIO, VIB_IDX_MASK);
	delay_ms(100);
	pio_clear(VIB_PIO, VIB_IDX_MASK);
}

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 Callback dos botões 
 */
void vol_mais(void)
{
	if (on){
	  flagVmais = 1;
	  but_flag = 1;
	  str = 'u';
	  vibra();
	}
	//pio_set`                                                                                                                      
}

void vol_menos(void)
{
	if (on){
	flagVmenos = 1;
	 but_flag = 1;
	 str = 'd';
	vibra();	 
	}
	
}

void play_pause(void)
{
	if (on){
	//usart_put_string(USART1, "Inicializando...\r\n");
	str = 'p';
	flagPP = 1;
	 but_flag = 1;
	 vibra();
	 }
	 
}

void onoff(void)
{
	if (on == 1){
		on = 0;
	} else {
		on = 1;
	}
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
	else if (flag_foward && (str != 'p')){
		str = 'f';
	}
	else if (flag_back  && (str != 'p')){
		str = 'b';
	}
	flagVmenos = 0;
	flagVmais = 0;
	flagPP = 0;
	flag_back = 0;
	flag_foward = 0;
	flag_neutral = 0;
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
	
	pmc_enable_periph_clk(LED_PIO_IDLD);
	pio_configure(LED_PIOLD, PIO_OUTPUT_0, LED_IDX_MASKLD, PIO_DEFAULT);
	
	pmc_enable_periph_clk(VIB_PIO_ID);
	pio_configure(VIB_PIO, PIO_OUTPUT_0, VIB_IDX_MASK, PIO_DEFAULT );


	//BOTAO ON OFF
	pmc_enable_periph_clk(BUT_PIO_IDLD);
	pio_configure(BUT_PIOLD, PIO_INPUT, BUT_IDX_MASKLD, PIO_PULLUP);

	pio_enable_interrupt(BUT_PIOLD, BUT_IDX_MASKLD);


	pio_handler_set(BUT_PIOLD,
	BUT_PIO_IDLD,
	BUT_IDX_MASKLD,
	PIO_IT_FALL_EDGE,
	onoff);

	NVIC_EnableIRQ(BUT_PIO_IDLD);
	NVIC_SetPriority(BUT_PIO_IDLD, 0);




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
  ioport_init();
  board_init();
  io_init();
  
  delay_init();
  SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
  config_console();
  //send();
  
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
  TC_init(TC0, ID_TC2, 2, 10);
  
  if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT)> 2000){
	  printf("f");
  }
  
  /* inicializa delay */              
  delay_init(sysclk_get_cpu_hz());

  /* inicializa console (printf) */
  configure_console();

  /* inicializa e configura adc */
  config_ADC_TEMP();

  /* Output example information. */
  puts(STRING_HEADER);

  /* incializa conversão ADC */
  afec_start_software_conversion(AFEC0);
  
	// super loop
	// aplicacoes embarcadas no devem sair do while(1).
	while(1)
	{
		
		if (on){
			pio_clear(LED_PIOLD, LED_IDX_MASKLD);
		
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
		  
		} else{
		  pio_set(LED_PIOLD, LED_IDX_MASKLD);
		  //pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);              
	  }
	  
		  //pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	  
		
	}
}
                                