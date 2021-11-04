
#include "includes.h"

#define F_CPU   16000000UL   // CPU frequency = 16 Mhz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define N_TASKS 4;


#define CDS_VALUE 871   // 조도센서 기준 값, 이 값보다 낮으면 state를 OFF로 하고 FND의 출력을 없앤다.

// state의 값, 노래가 나오고 있거나 꺼진 상태
#define ON 1
#define OFF 0

// 음계에 따른 TCNT2 값
#define DTI 3
#define DO 17
#define RE 43
#define MI 66
#define FA 77
#define SOL 97
#define SOLS 105
#define LA 114
#define TIF 122
#define TI 129
#define UDO 137

typedef unsigned char uc;

///////// Task Stacks /////////
OS_STK LedTaskStk[TASK_STK_SIZE];
OS_STK MelodyTaskStk[TASK_STK_SIZE];
OS_STK FndTaskStk[TASK_STK_SIZE];
OS_STK CdsTaskStk[TASK_STK_SIZE];

//////// Tasks ////////
void  LedTask(void* data);
void MelodyTask(void* data);
void FndTask(void* data);
void CdsTask(void* data);


//////// Variables ////////
const uc S = 0xED; const uc J = 0x1F; const uc bar = 0x40;   // 4번 인터럽트 때 넣을 값

const uc digit[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x27, 0x7F, 0x6F };
const uc dot = 0x80;
uc fnd[4];   // FND에 넣을 값
uc fnd_sel[4] = { 0x01, 0x02, 0x04, 0x08 };   // 출력할 FND 위치


int songs = 4;   // 노래 개수
// 0~31 비행기, 32~63 산토끼, 64~87 무엇이 무엇이 똑같을까, 88~151 나비야
const uc song[152] = { MI, RE, DO, RE, MI, MI, MI, MI, RE, RE, RE, RE, MI, SOL, SOL, SOL, MI, RE, DO, RE, MI, MI, MI, MI, RE, RE, MI, RE, DO, DO, DO, DO,
SOL, SOL, MI, MI, SOL, MI, DO, DO, RE, RE, MI, RE, DO, MI, SOL, SOL, UDO, SOL, UDO, SOL, UDO, SOL, MI, MI, SOL, SOL, RE, FA, MI, RE, DO, DO,
DO, MI, SOL, DO, MI, SOL, LA, LA, LA, SOL, SOL, SOL, FA, FA, FA, MI, MI, MI, RE, RE, RE, DO, DO, DO,
UDO, LA, LA, LA, TIF, SOL, SOL, SOL, FA, SOL, LA, TIF, UDO, UDO, UDO, UDO, UDO, LA, LA, LA, TIF, SOL, SOL, SOL, FA, LA, UDO, UDO, LA, LA, LA, LA, SOL, SOL, SOL, SOL, SOL, LA, TIF, TIF, LA, LA, LA, LA, LA, TIF, UDO, UDO, UDO, LA, LA, LA, TIF, SOL, SOL, SOL, FA, LA, UDO, UDO, LA, LA, LA, LA };

volatile int state = ON;   // 상태, 노래가 나오거나 나오지 않는 상태를 나타낸다.

volatile int track_num = 0;   // 몇 번째 노래인지
volatile int mel_idx = 0;   // 노래 인덱스


////// Event //////
OS_FLAG_GRP* led_grp;
OS_FLAG_GRP* mel_grp;

OS_EVENT* Sem;

OS_EVENT* Mbox;

OS_EVENT* Queue;
void* cds_fnd[1];

//////// Interrupt Service Routine ////////

// 4번 외부 인터럽트
// 스위치를 누르는 동안 "-S.J-가 FND에 출력된다.
ISR(INT4_vect) {

   fnd[3] = bar;
   fnd[2] = S;
   fnd[1] = J;
   fnd[0] = bar;

   PORTC = fnd[3];
   PORTG = fnd_sel[3];
   _delay_us(100);


   PORTC = fnd[2];
   PORTG = fnd_sel[2];
   _delay_us(100);


   PORTC = fnd[1];
   PORTG = fnd_sel[1];
   _delay_us(100);


   PORTC = fnd[0];
   PORTG = fnd_sel[0];
   _delay_us(100);

}

// 5번 외부 인터럽트
// 스위치를 누르면 다음 곡으로 넘어간다.
ISR(INT5_vect) {
   track_num = (track_num + 1) % songs;

   if (mel_idx >= 0 && mel_idx <= 31) {
      mel_idx = 32;
   }
   else if (mel_idx >= 32 && mel_idx <= 63) {
      mel_idx = 64;
   }
   else if (mel_idx >= 64 && mel_idx <= 87) {
      mel_idx = 88;
   }
   else if (mel_idx >= 88 && mel_idx <= 151) {
      mel_idx = 0;
   }

   _delay_ms(2);
}

ISR(TIMER2_OVF_vect) {
   if (state == ON) {
      PORTB = 0x00;
      TIMSK = 0x40;
      state = OFF;

   }
   else {
      PORTB = 0x10;
      TIMSK = 0x41;
      state = ON;

   }

   TCNT2 = song[mel_idx];
}

// 포트 초기화
void Port_init(void) {
   DDRA = 0xFF;
   DDRB = 0x10;
   DDRC = 0xFF;
   DDRG = 0x0F;
   DDRE = 0xCF;
}

// 외부 인터럽트 초기화
void Int_init(void) {

   SREG |= 0x80;   // 인터럽트 허가
   EICRB = 0x08;   // INT4: low level, INT5: falling edge
   EIMSK = 0x30;   // 4, 5 인터럽트 허가

}

// 버저 초기화
void Timer_init(void) {
   TIMSK = 0x40;
   TCCR2 = 0x03;
}

// A/D 컨트롤러 초기화
void ADC_init(void) {
   ADMUX = 0x00;
   ADCSRA = 0x87;
}


int main(void)
{

   INT8U err;
   OSInit();

   Port_init();
   Int_init();
   Timer_init();
   ADC_init();


   mel_grp = OSFlagCreate(0x00, &err);
   led_grp = OSFlagCreate(0x00, &err);
   Sem = OSSemCreate(1);
   Mbox = OSMboxCreate(0);
   Queue = OSQCreate(cds_fnd, 1);

   OSTaskCreate(LedTask, (void*)0, (void*)& LedTaskStk[TASK_STK_SIZE - 1], 0);
   OSTaskCreate(MelodyTask, (void*)0, (void*)& MelodyTaskStk[TASK_STK_SIZE - 1], 1);
   OSTaskCreate(FndTask, (void*)0, (void*)& FndTaskStk[TASK_STK_SIZE - 1], 2);
   OSTaskCreate(CdsTask, (void*)0, (void*)& CdsTaskStk[TASK_STK_SIZE - 1], 3);

   OSStart();

   return 0;
}


// 음계에 맞는 LED 출력
void LedTask(void* data)
{
   data = data;
   INT8U err;
   uc mel_result;

   // 가온 도 ~ 높은 도에 따른 LED 값
   const uc melody[8] = { 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01 };


   for (;;) {
      // FndTask로부터 Task가 끝남을 받아온다.
      OSFlagPend(led_grp, 0x01, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &err);

      // MelodyTask로부터 현재 음계의 멜로디를 받는다.
      mel_result = *(uc*)OSMboxPend(Mbox, 0, &err);

      OSSemPend(Sem, 0, &err);

      // 현재 음계에 맞게 LED를 출력한다.
      if (mel_result == DO) {
         PORTA = melody[0];
      }if (mel_result == RE) {
         PORTA = melody[1];
      }if (mel_result == MI) {
         PORTA = melody[2];
      }if (mel_result == FA) {
         PORTA = melody[3];
      }if (mel_result == SOL) {
         PORTA = melody[4];
      }if (mel_result == LA) {
         PORTA = melody[5];
      }if (mel_result == TI || mel_result == TIF) {
         PORTA = melody[6];
      }if (mel_result == UDO) {
         PORTA = melody[7];
      }

      OSSemPost(Sem);

      OSTimeDlyHMSM(0, 0, 0, 2);
   }

}

// ON이면 노래 출력, OFF이면 노래 끔
void MelodyTask(void* data) {
   data = data;
   INT8U err;

   for (;;) {
      // FndTask로부터 Task가 끝남을 받아온다.
      OSFlagPend(mel_grp, 0x01, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &err);

      OSSemPend(Sem, 0, &err);

      // state가 ON이면 노래를 재생한다.
      if (state == ON) {
         PORTB = 0x10;
         TIMSK = 0x41;

         _delay_ms(300);
         mel_idx = (mel_idx + 1) % 152;

         // 노래가 바뀌면 트랙의 번호를 바꾼다.
         if (mel_idx == 0 || mel_idx == 32 || mel_idx == 64 || mel_idx == 88) {
            track_num = (track_num + 1) % songs;
         }

      }

      // state가 OFF이면 노래를 끈다.
      else {
         PORTB = 0x00;
         TIMSK = 0x00;

      }
      OSMboxPost(Mbox, (void*)& song[mel_idx]); // 현재 음계의 멜로디를 LedTask에게 보낸다.
      OSSemPost(Sem);

      OSTimeDlyHMSM(0, 0, 0, 2);
   }
}

// 조도값이 낮으면 OFF로 만들고 FND를 끔
void FndTask(void* data) {
   data = data;
   INT8U err;
   unsigned short cds_result;   // CdsTask로부터 받는 값

   for (;;) {

      cds_result = *(unsigned short*)(OSQPend(Queue, 0, &err));   // CdsTask로부터 조도값을 받는다.

      OSSemPend(Sem, 0, &err);

      // 빛이 일정량보다 적으면 FND를 끄고 state를 OFF
      if (cds_result < CDS_VALUE) {
         state = OFF;
      }

      // 빛이 일정량보다 많으면 state ON, 몇 번째 노래인지 FND에 출력한다.
      else {
         state = ON;

         fnd[3] = digit[track_num + 1] + dot;
         PORTC = fnd[3];
         PORTG = fnd_sel[3];
      }
      OSFlagPost(mel_grp, 0x01, OS_FLAG_SET, &err);   // MelodyTask에게 FndTask가 일을 마쳤음을 알린다.
      OSFlagPost(led_grp, 0x01, OS_FLAG_SET, &err);   // LedTask에게 FndTask가 일을 마쳤음을 알린다.

      OSSemPost(Sem);

      OSTimeDlyHMSM(0, 0, 0, 2);
   }
}

// 빛 감지
void CdsTask(void* data) {
   data = data;
   INT8U err;
   unsigned char adc_low, adc_high;
   unsigned short adc_value;   // 조도값

   for (;;) {

      OSSemPend(Sem, 0, &err);

      ADCSRA |= 0x40;

      while ((ADCSRA & 0x10) != 0x10);

      adc_low = ADCL;
      adc_high = ADCH;

      adc_value = (adc_high << 8) | adc_low;

      OSQPost(Queue, &adc_value);   // FndTask에게 조도값을 보낸다.

      OSSemPost(Sem);

   }
}
