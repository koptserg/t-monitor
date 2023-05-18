
#ifdef HAL_LCD_PWM_PORT0
#include <stdlib.h>

#include "OSAL.h"
#include "beeping.h"
#include "Debug.h"
#include "utils.h"
#include "math.h"
#include "lcdgui.h"
#include "fonts.h"

#include "breakout.h"
#include "songdata.h"

static void beeping_initTimer1(void);
static void beep(uint32 freq, uint32 duration);
static void beeping_seq_play(const uint16* song_buffer, uint16 w, uint8 tempo);
static void DelayMs(uint32 delaytime);

static uint16 c = 0;
static uint8 songOn = 0;

uint8 beeping_TaskId = 0;

void beeping_Init(uint8 task_id) {
    beeping_TaskId = task_id;
    beeping_initTimer1();
//    beep(400, 0);
}

uint16 beeping_event_loop(uint8 task_id, uint16 events) {
    
    if (events & APP_BEEPING_DURATION_EVT) {
       beeping_stop_beep();
       if (songOn !=0 ) {
         osal_start_timerEx(beeping_TaskId, APP_BEEPING_SEQUENCER_EVT , 1);
       }

        return (events ^ APP_BEEPING_DURATION_EVT);
    }
    
    if (events & APP_BEEPING_SEQUENCER_EVT) {
      
      if (songOn == 1) beeping_seq_play(SONG_DATA, 198, 15);
      if (songOn == 2) beeping_seq_play(SONG_DATA_1, 60, 10);

        return (events ^ APP_BEEPING_SEQUENCER_EVT);
    }
    return 0;
}

void beeping_stop_beep(void){
  T1CTL &= ~(BV(1) | BV(0)); // Operation is suspended timer 1 (if it was running)
}


void beeping_beep(uint16 freq, uint32 duration){
  beep(freq, duration);
}

static void beeping_initTimer1(void){
  //  P0DIR |= 0x08; // p0.3 output
  PERCFG &= ~0x40; //select of alternative 1 for timer 1
  P2DIR = (P2DIR & ~0xC0) | 0x80; // priority timer 1 channels 0 1
  P0SEL |= 0x08; // p0.3 periferal
}

// frequency range 32 - 8000000 Hz
static void beep(uint32 freq, uint32 duration){
  uint16 tfreq;
  if (freq <= 32768) {
    tfreq = (uint16)(2000000/freq);
  } else {
    tfreq = (uint16)(16000000/freq);
  }
  uint8  tfreq_l = (uint8)tfreq;
  uint8  tfreq_h = (uint8)(tfreq >> 8);
  uint16 tduty = tfreq/2;
  uint8  tduty_l = (uint8)tduty;
  uint8  tduty_h = (uint8)(tduty >> 8);
  
  T1CTL &= ~(BV(1) | BV(0)); // Operation is suspended timer 1 (if it was running)
  
  T1CC1H = tduty_h;
  T1CC1L = tduty_l; //PWM Duty Cycle 
  
  T1CC0H = tfreq_h;
  T1CC0L = tfreq_l; //PWM signal period
  
  T1CCTL1 = 0x1c; //00: No capture
                  //1: Compare mode
                  //011: Set output on compare-up, clear on compare-down in up-and-down mode. Otherwise set output on compare, clear on 0.
  if (freq <= 32768) {
    T1CTL |= (BV(2) | 0x03); //11: Up-and-down, repeatedly count from 0x0000 to T1CC0 and from T1CC0 down to 0x0000.
                             //01: Tick frequency / 8
  } else {
    T1CTL = (0x03);          //11: Up-and-down, repeatedly count from 0x0000 to T1CC0 and from T1CC0 down to 0x0000.
                             //00: Tick frequency / 1
  }
  osal_start_timerEx(beeping_TaskId, APP_BEEPING_DURATION_EVT, duration);
}

void beeping_beep_delay(uint16 freq, uint32 duration){
  beep(freq, duration);  
  DelayMs(duration);
  beeping_stop_beep();
}

static void DelayMs(uint32 delaytime) {
  while(delaytime--)
  {
    uint16 microSecs = 1000;
    while(microSecs--)
    {
      asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
  }
}

void beeping_seq_start(uint8 song_num) {
  c = 0;
  songOn = song_num;
  osal_start_timerEx(beeping_TaskId, APP_BEEPING_SEQUENCER_EVT , 1000);
}

void beeping_seq_stop(void) {
  c = 0;
  songOn = 0;
  osal_stop_timerEx(beeping_TaskId, APP_BEEPING_SEQUENCER_EVT);
  osal_clear_event(beeping_TaskId, APP_BEEPING_SEQUENCER_EVT);
}

static void beeping_seq_play(const uint16* song_buffer, uint16 w, uint8 tempo)
{
        uint16 fr = song_buffer[c++];
//        LCD_SetArealColorWH(0, 200, 14*3, 19, SCOREBOARD_COLOR);
//        GUI_DisNum(0, 200, c, &Font20, WHITE, RED);
        
        uint16 d = song_buffer[c++];
        beeping_beep (fr, d * tempo );

        if ( c == w ) {
          c = 0;
        }
}

#endif