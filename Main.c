
/*******************************************************************************/
/*		Arquivos Header						     		  					   */
/*******************************************************************************/

#include "DSP2833x_Device.h"
#include "stdio.h"
#include "Define.h"
#include "Structs.h" 		//#
#include "C28x_FPU_FastRTS.h"
#include <math.h>
#include "Const_disc8.h"	//#
#define pontograf 1000 

/*******************************************************************************/
/*		Protótipos das Funções 	 								   	   	  	   */
/*******************************************************************************/

void INIT_SYSTEM(void);
void INIT_FLASH(void);
void INIT_EPWM(void);
void INIT_INTERRUPT(void);
void INIT_PIE_VEC_TABLE(void);
void INIT_PIE_CTRL(void);
void ENABLE_INTERRUPT(void);
void INIT_ADC(void);
void INIT_GPIO(void);

void Gera_seno(void);
void Teclado(void);
void Teclado_Tratamento(void);
void Teclado_Trava(void);
void Calcula_Aparente(void);
void mPPT(void);
void Atraca_PLL(void);
void Rampas(void);
void interrupt MAIN_ISR(void);

extern void DSP28x_usDelay(unsigned long Count);			// Rotina de delay em ms
#define DELAY_MS(A)  DSP28x_usDelay(((((long double) A * 1000000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

/*******************************************************************************/
/*		Variáveis Globais	    			    		  					   */
/*******************************************************************************/

// Variaveis principais

int razao = 0;
float vret = 0;
float vret1 = 0;
float iboost = 0;
int contador_tempo = 0;

int graph_uv[1250];
int graph_razao[1250];
int vret_acm[1250];
long int acm=0;

//float antiwindup=0;
//float qv1 =  1;
//float qv2 = 0;
//float qv3 =  (181.457187390391e-006);
//float qv4 =  (-179.855623512276e-006);
//float qv5 = (0);
//
//float qi1 =  1;
//float qi2 =  0;
//float qi3 =  547.373979195576;
//float qi4 =  (-285.666571709118);
//float qi5 = (0);


float antiwindup=0;
float qv1 =  1;
float qv2 = 0;
float qv3 =  (60.4857291301303e-006);
float qv4 =  (-59.9518745040921e-006);
float qv5 = (0);

// 2kHz
//float qi1 =  1;
//float qi2 =  0;
//float qi3 =  53.1886966550737;
//float qi4 =  (-27.7584123554077);
//float qi5 = (0);

//4kHz
//float qi1 =  1;
//float qi2 =  0;
//float qi3 =  194.284851468175;
//float qi4 =  (-101.394457104982);
//float qi5 = (0);

//3kHz
float qi1 =  1;
float qi2 =  0;
float qi3 =  114.997959437445;
float qi4 =  (-60.0157736294246);
float qi5 = (0);

//Q=10
	float qn_u1 =  1.595070186811703;
	float qn_u2 = -0.944402099341187;
	float qn_e0 =  0.972201049670594;
	float qn_e1 =  -1.595070186811703;
	float qn_e2 =  0.972201049670593;


	float notch_u0 =  0;
	float notch_u1 =  0;
	float notch_u2 = 0;
	float notch_e0 =  0;
	float notch_e1 = 0;
	float notch_e2 = 0;


float uv=0;
float uv1=0;
float uv2=0;
float ev=0;
float ev1=0;
float ev2=0;
float ui=10;
float ui1=0;
float ui2=0;
static float ei=0;
float ei1=0;
float ei2=0;
float vref=0;
float vout=0;
float vin_ret=0;
float comp=0;
float refi=0;
float sobretensao;
float sobrecorrente;
float protecao;
int counter_protecao = 0;

// calculo de rms
#define BUFFER_SIZE 50
static double buffer[BUFFER_SIZE];
static double vrms = 220;
static double vg_sq_sum = 0;
static int index = 0;
static int counter = 0;

// -----------------------------------------------------------

//----------------------------------------------------------------------------/
//	Rotina Principal, com espera da Interrupcao do AD
//----------------------------------------------------------------------------/

void main(void)
{

	INIT_SYSTEM();
	INIT_GPIO();
	INIT_INTERRUPT();
	INIT_PIE_CTRL();
	INIT_PIE_VEC_TABLE();
	INIT_ADC();
	INIT_EPWM();

	#ifdef FLASH

	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM

	INIT_FLASH();

	#endif


	ENABLE_INTERRUPT();

			EALLOW;
			EPwm1Regs.TZCLR.bit.OST = 1;		// Libera os PWMs
			EPwm2Regs.TZCLR.bit.OST = 1;		// Clear one shot triger
			EPwm3Regs.TZCLR.bit.OST = 1;		// 
			EPwm4Regs.TZCLR.bit.OST = 1;		//
			EPwm5Regs.TZCLR.bit.OST = 1;		// 
			EPwm6Regs.TZCLR.bit.OST = 1;		// 
			EDIS;

			sobretensao = 0;
			sobrecorrente = 0;
			protecao = 1;

			int i = 0;
			for(i = 0; i < BUFFER_SIZE; i++){
				buffer[i] = 0;
			}

	while(1) 
	{
	
	} 	// fim de Loop while(1)					


}	// fim de Main(void)

//----------------------------------------------------------------------------/
//	Interrupcao de Conversao do AD, com proteçoes e controle
//----------------------------------------------------------------------------/

interrupt void MAIN_ISR()
{

//	GpioDataRegs.GPADAT.bit.GPIO25 = 1;//teste de consumo de tempo da interrupcao

// Inicio da conversao AD

	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;	// limpa flag de int 
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;		// Reseta Seq1
	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;		// Disparo para início de conversão
	EPwm1Regs.ETCLR.bit.INT = 1;			//Limpa flag permitindo de INT periferica
	PieCtrlRegs.PIEACK.all = PIEACK_G3;  // Reseta o bit de acknowledgement flag

	while(AdcRegs.ADCST.bit.INT_SEQ1==0)	// loop a espera do fim da conversão
	{
	}
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;	// Limpa flag de int

// Fim da conversao AD

// Leitura das variaveis instantaneas com conversão para Amperes/Volts
// retira o nível medio (-2048) e multiplica pelo ganho correspondente

	vout   = (float)((float)(AdcRegs.ADCRESULT0>>4)) / 8.1351;  // 167.79142
	vret   = (float)((float)(AdcRegs.ADCRESULT1>>4)) / 8.4685;  // 161.18557
	iboost = (float)((float)(AdcRegs.ADCRESULT2>>4)) / 502.3396;// 2.717285

// Proteções

	if(counter_protecao<200)
	{
		counter_protecao ++;
	}
	else{


		if(vout > 460)
		{
			sobretensao = 1;
		}
		if(iboost > 4 || iboost < -1.5)
		{
			sobrecorrente = 1;
		}
	}


// Fim das proteções
	
if(sobretensao == 0 && sobrecorrente == 0)// && protecao == 1)
{
	++contador_tempo;

	if(contador_tempo >= 250){

		contador_tempo = 0;

		ev = 400 - vout;

		uv = (float)(qv1 * uv1) + (float)(qv2 * uv2) + (float)(qv3 * ev) + (float)(qv4 * ev1) + (float)(qv5 * ev2);

		if(uv>1.5) uv = 1.5;
		if(uv<0) uv = 0;

		uv2 = uv1;
		uv1 = uv;
		ev2 = ev1;
		ev1 = ev;

//		} //fim comp>600

	} //fim contador_tempo
	counter++;
	refi = (float)(uv * vret/311);
	ei = refi - iboost;
	ui = (float)(qi1 * ui1) + (float)(qi2 * ui2) + (float)(qi3 * ei) + (float)(qi4 * ei1) + (float)(qi5 * ei2);

	if(ui>1000) ui = 1000;
	if(ui<0) ui = 0;

	ui2 = ui1;
	ui1 = ui;
	ei2 = ei1;
	ei1 = ei;

	razao = ui;

	if(razao > 950){
		razao = 950;
	}
	if(razao < 5){
		razao = 5;
	}
	EPwm1Regs.CMPA.half.CMPA = razao;
}
else
{
	protecao = 0;
}
} // End MAIN_ISR()
