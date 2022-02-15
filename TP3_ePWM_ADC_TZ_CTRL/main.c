#include "Peripheral_Setup.h"

//#include <Peripheral_Setup.h>

/**
 * main.c
 */
//Declaração de variáveis
uint32_t count = 0;
uint32_t index= 0;
float va, vb, vc, iLa, iLb, iLc, vDC, Vref;
float plot1[512];
float plot2[512];

float *padc1 =  &va;
//float *padc2 =  &iLa;

//variavel para o sensor de temperatura
unsigned char ERRO_TEMP = 0;         // Desliga o conversor por elevação de temperatura

//variavel para ligar/desligar o boost
unsigned char LigDesL_PWM = 0;         // Liga ou desliga o conversor boost

//Variavel para o trip zone
struct {
    unsigned int pre_carga;
    unsigned int geral;
    unsigned int index;
    unsigned int trip_event;
} contadores;


// Function Prototypes
interrupt void isr_cpu_timer0(void);  //função de interrupção de tempo
__interrupt void isr_adc(void);     //funçao de interrupçao do ADC
__interrupt void isr_epwm_trip(void);  //função do trip zone


int main(void)
    {
            //Funções básicas definidas pela Texas Instrument

                InitSysCtrl();       //Initialize System Control

                DINT;                //Disable CPU interrupts  (Chave global das interrupções, desliga tudo)
                InitPieCtrl();        //Initialize the PIE control registers to their default state
                IER = 0x0000;        //Disable CPU interrupts (Chave )
                IFR = 0x0000;        //Clear all CPU interrupt flags
                InitPieVectTable();  //Initialize the PIE vector table  (preenche a matriz de interrupção)


                Setup_GPIO();          // Função que configura o GPIO
                Setup_ePWM();    //Função que configura o ePWM
                Setup_ADC_A(); //Configura o ADC A
                Setup_ADC_B(); //Configura o ADC B
                Setup_ADC_C(); //Configura o ADC C

                //Funções que tratam das interrupções,
                EALLOW;
                PieVectTable.TIMER0_INT = &isr_cpu_timer0;  //endereço da função que vai ser executada quando ocorrer a interrupção do timer
                PieVectTable.ADCA1_INT =  &isr_adc;  //endereço da função que vai ser executada quando ocorrer a interrupção do adc
                PieVectTable.EPWM4_TZ_INT = &isr_epwm_trip;  // habilita a função que é chamada quando der o erro, ocorrer o trip
                EDIS;

                //interrupção dos periféricos - pag. 96 PIE channel Mapping - Technical reference
                PieCtrlRegs.PIEIER1.bit.INTx7 = 1; //Timer 0 - habilita a coluna 7 da linha 1 que corresponde a interrupçao do timer 0
                PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // ADC A1 (interrupção do ADC A) (linha 1 da coluna 1)
                PieCtrlRegs.PIEIER2.bit.INTx4 = 1;  //Enable PieVector to TZ4 TRIP1 interrupt (linha 2, coluna 4)
                IER |= (M_INT1|M_INT2); // Enable lines of interrupt ,  habilita a linha 1 e a linha 2 nas interrupções globais

                //timer
                InitCpuTimers(); //inicializa o timer
                ConfigCpuTimer(&CpuTimer0, 200, 100000); //configura a função timer (timer, freq do dsp, período[micro segundos]) t= 0.1seg
                CpuTimer0Regs.TCR.all = 0x4001; //habilita a interrupção dentro do timer


                EINT;                 //Enable Global interrupt INTM
                ERTM;                 //Enable Global realtime interrupt DBGM


                //mantem os leds desligados inicialmente
                GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
                GpioDataRegs.GPADAT.bit.GPIO31 = 0;

                //inicializa as variaveis do contador
                  contadores.geral = 0;
                  contadores.index = 0;
                  contadores.pre_carga = 0;
                  contadores.trip_event = 0;

                //inicializa as variavies de tensão e corrente
                  va = 0;
                  vb = 0;
                  vc = 0;
                  iLa = 0;
                  iLb = 0;
                  iLc = 0;
                  vDC = 0;
                  Vref = 0;

             //inicializa o GPIO 26 que liga e desliga o pwm do boost
              GpioDataRegs.GPADAT.bit.GPIO26 = 0;



                //loop infinito
        while(1){
             for(count = 0; count < 0x00FFFFF; count++){

                 }
             //Faz piscar o led azul ligado na GPIO 31
             // GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // ou exclusivo (XOR), alterna o valor entre 0 e 1

             //Faz piscar o led vermelho ligado na GPIO 34  - observar que ele usa o GPB toggle enquanto que o led azul usa o GPA toggle   (pag. 13 pdf overview)
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Or exclusivo (XOR), alterna o valor entre 0 e 1

            //verifica a temperatura do conversor boost
            ERRO_TEMP = GpioDataRegs.GPADAT.bit.GPIO14;

           // if(ERRO_TEMP != 0){
            //Liga ou desliga o conversor boost
            LigDesL_PWM = GpioDataRegs.GPADAT.bit.GPIO26;
           // }

                }
               return 0;

}

interrupt void isr_cpu_timer0(void){
GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //faz piscar o led azul dentro da interrupção

PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // serve para resetar o flag da interrupção da linha
}

__interrupt void isr_adc(void){

 // Over temperature  - Proteção contra elevação de temperatura (lê o GPIO 14 de temp. e desliga o GPIO 26 pwm boost)
 GpioDataRegs.GPADAT.bit.GPIO26 = (!GpioDataRegs.GPADAT.bit.GPIO14) ? 1 : GpioDataRegs.GPADAT.bit.GPIO26;

 //Liga novamente o pwm do conversor boost caso a temperatura tenha reduzido (quando o GPIO 14 volta pra 1)
  if(GpioDataRegs.GPADAT.bit.GPIO14 != 0){
            GpioDataRegs.GPADAT.bit.GPIO26 = 0;
            }

    while(!AdcbRegs.ADCINTFLG.bit.ADCINT1);     // Wait ADCB finished  quando o ADCINT1 =1 , o pulso de interrupção ja foi gerado
// Le os 4 valores do ADC A, 2 valores do ADC B e 2 valores do ADC C
    va =  0.001*((int)AdccResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC C
    vb =  0.001*((int)AdcbResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC B
    vc =  0.001*((int)AdcaResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC A
    vDC = 0.001*((int)AdcaResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC A
    iLa =  0.001*((int)AdccResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC C
    iLb =  0.001*((int)AdcbResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC B
    iLc =  0.001*((int)AdcaResultRegs.ADCRESULT2 - 0x7FF); //soc 2 ADC A
    Vref =  0.001*((int)AdcaResultRegs.ADCRESULT3 - 0x7FF); //soc 3 ADC A


// Over voltage - Proteção de sobretenção no barramento CC, caso a tensão fique maior que 30 V desliga o PWM
GpioDataRegs.GPADAT.bit.GPIO26 = (vDC > 30.0) ? 1 : GpioDataRegs.GPADAT.bit.GPIO26;


    plot1[index] = *padc1;//envia os valores para o vetor plot para verificação via debug
   // plot2[index] = *padc2;

    index = (index == 512) ? 0 : (index+1); //incrementador

    //limpa os flags dos ADCs
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                      //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void isr_epwm_trip(void){

    //dispara o contador quando ocorrer o trip
       //contadores.trip_event = contadores.trip_event + 1;
      contadores.trip_event++;    //dispara o contador quando ocorrer o trip

     // To Re-enable the OST Interrupt, do the following:

      EALLOW;
      //pwm 4
      EPwm4Regs.TZCLR.bit.OST = 1;
      EPwm4Regs.TZCLR.bit.INT = 1;

      //pwm 5
      EPwm5Regs.TZCLR.bit.OST = 1;
      EPwm5Regs.TZCLR.bit.INT = 1;

      //pwm 6
      EPwm6Regs.TZCLR.bit.OST = 1;
      EPwm6Regs.TZCLR.bit.INT = 1;
      EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

