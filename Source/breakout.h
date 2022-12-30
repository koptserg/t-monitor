#ifndef BREAKOUT_H
#define BREAKOUT_H

#include "fonts.h"
#include "tft3in5.h"

//////config
#define ROWS                5
#define COLS                13

#define TILE_W              (uint8)((MAX_X + 1) / COLS)
#define TILE_H              (uint8)(TILE_W / 2.2)

#define BALL_R              2
#define BALL_SPEED_H        1
#define BALL_SPEED_V        1
#define BALL_COLOR          CYAN
#define BALL_MOVE_WAIT      7//7 //4

#define PADDLE_W            16
#define PADDLE_H            4
#define PADDLE_TOLERANCE    1
#define PADDLE_COLOR        LIGHTGREEN
#define PADDLE_MOVE_WAIT    7//7 // 5

#define BOARD_LEFT          (uint8)(2)
#define BOARD_RIGHT         (uint16)(MAX_X - 2)
#define BOARD_TOP           23

#define TILES_LEFT          (uint16)(((MAX_X + 1) - ( COLS * TILE_W ) )/ 2)
#define TILES_TOP           (uint16)(BOARD_TOP + 2.8 * TILE_H)

#define TILE_IS_DRAWN       1
#define TILE_IS_NOT_DRAWN   0

#define SCOREBOARD_COLOR  WHITE //LIGHTGREY
#define BOARD_COLOR  LIGHTGREY
#define BACKGROUND_COLOR  BLACK

//    Scoreboard
#define GAME_LEVEL  1
#define GAME_LIVES  5
#define GAME_SCORE  0

// interface
#define FONT_SPACE 6
#define FONT_X 8
#define MIN_X 0
#define MIN_Y 0
#define MAX_X 319
#define MAX_Y 319

#define APP_BREAKOUT_BALL_MOVE_EVT        0x0001
#define APP_BREAKOUT_CLOCK_EVT            0x0002

extern uint16 breakout_event_loop(uint8 task_id, uint16 events);
extern void breakout_Init(uint8 task_id);
extern void breakout_start(void);
extern void breakout_keyprocessing(void);
#endif