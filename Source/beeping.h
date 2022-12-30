#ifndef BEEPING_H
#define BEEPING_H

#define APP_BEEPING_DURATION_EVT        0x0001
#define APP_BEEPING_SEQUENCER_EVT       0x0002

extern void beeping_beep(uint16 freq, uint32 duration);
extern void beeping_stop_beep(void);
extern void beeping_seq_start(uint8 song_num);
extern void beeping_seq_stop(void);

extern void beeping_Init(uint8 task_id);
extern uint16 beeping_event_loop(uint8 task_id, uint16 events);

#endif /* BEEPING_H */

/* END OF FILE */