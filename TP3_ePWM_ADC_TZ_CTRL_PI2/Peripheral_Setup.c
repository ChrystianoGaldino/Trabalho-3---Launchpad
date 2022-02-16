/*
 * Peripheral_Setup.c
 *
 */


#include "Peripheral_Setup.h"

void Setup_GPIO(void)
{

    EALLOW;

    //Led 31 A, 2
    //Led 34 B, 1

    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1;

    //    PWM
    //Os GPIOs que controlam os PWMs 4,5 e 6 (A e B) vão do GPIO 6 ao 11

    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;  //as funções de pwm 4,5 e 6 (A,B) estão no grupo de periféricos 0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;  //GPIO A Peripheral Group Mux (GPIO0 to 15)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1; //o multiplexador que controla é o mux 1  ( GPIO A Mux 1 Register (GPIO0 to 15)  )
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;     // GPIO A Pull Up Disable Register (GPIO0 to 31)
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;


    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;   // GPIO A Peripheral Group Mux (GPIO0 to 15)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // GPIO A Mux 1 Register (GPIO0 to 15)
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;     // GPIO A Pull Up Disable Register (GPIO0 to 31)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 3;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 5;


    //GPIO 14 - input     pino configurado como entrada  do sensor de temperatura  (0 - erro, 1 - ok)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;

    //GPIO 26 - output  pino configurado como saída que liga o pwm do boost (0 - erro, 1 - ok)
    // Start PWM
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO26 = GPIO_MUX_CPU1;


    //TRIP ZONE         o roteamento do trip zone foi escolhido para o GPIO 41 (conector J5 pino 49)

     GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0; //enable pull-up on GPIO41 (TZ4)   , pino de logica invertida fica em logica alta
     GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3; //Asynch input GPIO41 (TZ4) , pino de entrada configurado como assincrono, não depende do clock (tem que atuar assim que chegar o sinal de emergencia)

     EALLOW; //por ser um registrador protegido tem que ser acessado por meio das diretivas eallow e edis
       InputXbarRegs.INPUT1SELECT = 41;  //figura 9.1 pag.1152 do manual  (roteamento feito pelo registrador utilizando o GPIO41)
     EDIS;

     //GPIO 22 - output  pino configurado como sinal de sobrecorrente que liga o Trip Zone (0 - erro, 1 - ok)
     GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0;
     GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
     GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;
     GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
     GpioCtrlRegs.GPACSEL3.bit.GPIO22 = GPIO_MUX_CPU1;




    EDIS;

}

void Setup_ePWM(void){
    EALLOW;    //diretiva para permitir o acesso a registradores restritos

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;  //desabilita o clock do sincronismo para configurar, depois é habilitado novamente

    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;  //habilita o clock do pwm , pois todos foram desabilitados na função InitSysCtrl
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;


    EPwm4Regs.TBPRD = 2778;  //set time period   (f = 1/T)
    EPwm5Regs.TBPRD = 2778;
    EPwm6Regs.TBPRD = 2778;


    EPwm4Regs.CMPA.bit.CMPA = EPwm4Regs.TBPRD >> 1; //largura do pulso (Duty Cycle - D)
    EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD >> 1;
    EPwm6Regs.CMPA.bit.CMPA = EPwm6Regs.TBPRD >> 1;

    EPwm4Regs.TBPHS.bit.TBPHS = 0;  //Phase is 0
    EPwm5Regs.TBPHS.bit.TBPHS = 0;  // Phase is 0
    EPwm6Regs.TBPHS.bit.TBPHS = 0;  // Phase is 0


    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;  //o pwm 1 não possui entrada de sincronismo, ele que fornece a referencia
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  //pulso para sincronizar os contadores dos pwms
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;


    EPwm4Regs.TBCTR = 0x0000;     //Clear counter
    EPwm5Regs.TBCTR = 0x0000;
    EPwm6Regs.TBCTR = 0x0000;


    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //Count up/down
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;


    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;  //Disable phase loading
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;    //Enable phase loading
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;   //Enable phase loading


    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; //Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;


    EPwm4Regs.TBCTL.bit.CLKDIV  = TB_DIV1; //Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV  = TB_DIV1;
    EPwm6Regs.TBCTL.bit.CLKDIV  = TB_DIV1;



    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW; // Load registers every ZERO para o modulo A do pwm
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;


    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;


    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;  //Load registers every ZERO para o modulo B do pwm
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;


    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    //set actions for EPWMx A

    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;  //definição de como o pwm vai funcionar
    EPwm5Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;

    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;


    //Dead - band - configura a função complementar para o pwm B

    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   //Active Hi complementary  (habilita  a função invertida, habilita o pulso complementar)
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;

    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //enable Dead-band module     (tempo morto)
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;

    EPwm4Regs.DBFED.bit.DBFED = 100;   //FED = 20 TBCLKs   (comando para desligar o sinal) Falling edge delay value
    EPwm5Regs.DBFED.bit.DBFED = 100;
    EPwm6Regs.DBFED.bit.DBFED = 100;


    EPwm4Regs.DBRED.bit.DBRED = 100;   //RED = 20 TBCLKs   (comando para ligar o sinal)  Rising edge delay value
    EPwm5Regs.DBRED.bit.DBRED = 100;
    EPwm6Regs.DBRED.bit.DBRED = 100;

    ///         ADC          ////

    //Trigger ADC para os pwms , faz os pwms gerarem o valor de trigger
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;                 // Enable SOC on A group
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;   // Dispara ADC no topo e no zero
    EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;            // Trigger on every event   dispara o trigger no primeiro evento que ocorrer


    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; //habilita o clock do sincronismo novamente


    //Enable TZ1 as one shot trip source    , escolhe o modo one shot para o TZ
    EPwm4Regs.TZSEL.bit.OSHT1 = 1;
    EPwm5Regs.TZSEL.bit.OSHT1 = 1;
    EPwm6Regs.TZSEL.bit.OSHT1 = 1;

    //What do we want the TZ1 to do ?      , o TZ coloca as saidas do PWM em nivel lógico baixo
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;

    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

    //Enable TZ interrupt
    EPwm4Regs.TZEINT.bit.OST = 1;   // habilita a interrupção do trip zone



    EDIS;

}


void Setup_ADC_A(void){
    // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;
    // determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;                             // 75ns
    else                                        // resolution is 16-bit
        acqps = 63;                             // 320ns

    EALLOW;

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);


    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC
    DELAY_US(1000);                             // delay for 1ms to allow ADC time to power up

    // ADC A  - 4 medições //

    //medição de Vdc
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 15;         //SOC0 will convert pin ADCIN15 (63)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;                  //sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;  //trigger on ePWM2 SOCA     //o trigger vem do epwm4 !!!

    //medição de Vc
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 5;          //SOC1 will convert pin ADCINA5 (66)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;

    //medição de Ic
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;          //SOC2 will convert pin ADCINA4 (69)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;

    //medição de Vref
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 1;          //SOC3 will convert pin ADCINA1 (70)
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;


    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;      // end of SOC3 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

    EDIS;
}

void Setup_ADC_B(void){
    // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;
    EALLOW;
    //write configurations
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;                      // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;                   // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;                      // power up the ADC
    DELAY_US(1000);                                         // delay for > 1ms to allow ADC time to power up

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns
    }else{
        //resolution is 16-bit
        acqps = 63; //320ns
    }

    // ADC B  - 2 medições


    //medição de Vb
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 5;                     //SOC0 will convert pin ADCINB5 (65)
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;                  //sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;  //trigger

    //medição de Ib
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 4;                      //SOC1 will convert pin ADCINB4 (68)
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;


    //pg 1569 spruhm8i.pdf - Technical reference
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;                  // End of SOC1 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;                    // Disable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                  // Make sure INT1 flag is cleared
    EDIS;
}

void Setup_ADC_C(void){
    // pg 1592 spruhm8i.pdf - Technical reference
    Uint16 acqps;
    EALLOW;
    //write configurations
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;                      // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;                   // Set pulse positions to late
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;                      // power up the ADC
    DELAY_US(1000);                                         // delay for > 1ms to allow ADC time to power up

    //determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdccRegs.ADCCTL2.bit.RESOLUTION){
        acqps = 14;     //75ns
    }else{
        //resolution is 16-bit
        acqps = 63; //320ns
    }

    // ADC C  - 2 medições

    //medição de Va
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 5;                     //SOC0 will convert pin ADCINC5 (64)
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;                  //sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;  //trigger

    //medição de Ia
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 4;                      //SOC1 will convert pin ADCINC4 (67)
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM4_SOCA;


    //pg 1569 spruhm8i.pdf - Technical reference
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1;                  // End of SOC1 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;                    // Disable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                  // Make sure INT1 flag is cleared
    EDIS;
}











