
#ifdef TFT3IN5
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "Debug.h"
#include "breakout.h"
#include "lcdgui.h"
#include "fonts.h"
#include "tft3in5.h"
#include "utils.h"
#include "math.h"
#include "battery.h"
#include "hal_adc.h"
#include "beeping.h"

void create_butt(void);
void gamerun(void);

void clockupdate(void);
void scoreinit(void);
void scoreupdate( int scored );
void tilesLeftupdate(void);
void scorenextLevel(void);
void scoredied(void);
int nDigits(int value);

void newLevel(void);
void tilesdrawAll(void);
uint16 myrand(uint32 min, uint32 max);
void mysrand(uint16 seed);
void serve(void);
void play(void);
int ballcollision(void);
bool collision(void);

void ballclear(void);
void balldraw(void);
void ballsetXY(int x_, int y_);
void ballsetX(int x_);
void ballsetXiYi(int xi_, int yi_);
bool ballescaped(void);
void move(void);
int ballgetX(void);
int ballgetY(void);
int ballgetXi(void);

bool tilesallGone(void);
void clearTile( int x, int y );
bool tilesexists(int x, int y);
int hitTile( int x, int y);

void paddledraw(int m);
void paddledraw_init(void);
void paddlesetXY(int x_, int y_);
int paddlegetX(void);
int paddlegetY(void);

uint8 level = GAME_LEVEL;
int8 lives = GAME_LIVES;
uint32 score = GAME_SCORE;

//const uint16 COLORS[ROWS] = { RED, MAGENTA, OLIVE, YELLOW, GREEN, NAVY };
const uint16 COLORS[ROWS] = { RED, MAGENTA, ORANGE, YELLOW, GREEN};
uint8 tiles[ROWS][COLS];
uint8 tilesLeft = 0;

static void _delay_us(uint16 microSecs);
static void _delay_ms(uint16 milliSecs);
bool zcl_game = 0;
bool s_play = 0;
bool game_over = 0;
bool butt_pause = 0;
uint32 last = 0;
uint16 beep_f;

UTCTimeStruct time;

uint8 breakout_TaskId = 0;

void breakout_Init(uint8 task_id) {
    breakout_TaskId = task_id;
    osal_start_reload_timer(breakout_TaskId, APP_BREAKOUT_BALL_MOVE_EVT, 9);
    osal_start_reload_timer(breakout_TaskId, APP_BREAKOUT_CLOCK_EVT, 60000);
    
    last = osal_GetSystemClock();
}

uint16 breakout_event_loop(uint8 task_id, uint16 events) {
    if (events & APP_BREAKOUT_BALL_MOVE_EVT) {
        if (zcl_game && s_play ){
          if (!game_over) {            
            play();
          } else {
            s_play = 0;
            beeping_stop_beep();
            gamerun();
            initButton(1, 260, 370, 90, 50,  WHITE, PURPLE,  WHITE, "Fire", 2); 
            drawButton(1, 0);
          }
        } 
        
        return (events ^ APP_BREAKOUT_BALL_MOVE_EVT);
    }
    
    if (events & APP_BREAKOUT_CLOCK_EVT) {
        if (zcl_game){
          clockupdate();
        } 
        
        return (events ^ APP_BREAKOUT_CLOCK_EVT);
    }
    return 0;
}

void breakout_keyprocessing(void) 
{
            int8 butt = pressButton();
            int m = 0;
            if (butt == 0 && !s_play && !butt_pause) {
                s_play = 0;
                zcl_game = 0;
              return;
            }
            if (butt == 2) {
              m=-5;
            }
            if (butt == 3) {
              m=5;
            }
            if (butt == 1) {
              beeping_beep(100, 3);
              if (s_play) {
                s_play = 0;
                butt_pause = 1;
                initButton(1, 260, 370, 90, 50,  WHITE, PURPLE,  WHITE, "Fire", 2); 
                drawButton(1, 0);
//                beeping_seq_start();
              } else {
                s_play = 1;
                butt_pause = 0;
                initButton(1, 260, 370, 90, 50,  WHITE, PURPLE,  WHITE, "Pause", 2); 
                drawButton(1, 0);
                beeping_seq_stop();
              }

            }
            if (!butt_pause) {
              if (butt == 2 || butt == 3){
//                beeping_beep(100, 3);
                paddledraw(m);
                if (!s_play){
                  ballclear();
                  ballsetX(paddlegetX());
                  balldraw();
                }
              }
            }  
}

void clockupdate(void)
{
        uint8 hour = time.hour;
        uint8 minutes = time.minutes;
        osalTimeUpdate();
        osal_ConvertUTCTime(&time, osal_getClock());
        
        char hour_string[] = {'0', '0', '\0'};
        hour_string[0] = time.hour / 10 % 10 + '0';
        hour_string[1] = time.hour % 10 + '0';
        
        char minutes_string[] = {'0', '0', '\0'};
        minutes_string[0] = time.minutes / 10 % 10 + '0';
        minutes_string[1] = time.minutes % 10 + '0';
          
        if (hour != time.hour || time.hour == 0){
          LCD_SetArealColorWH(10*14, 4, 2*14, 19, SCOREBOARD_COLOR);
          GUI_DisString_EN(10*14, 4, hour_string, &Font20, SCOREBOARD_COLOR, BLUE);
        }
        if (minutes != time.minutes){  
          LCD_SetArealColorWH(13*14, 4, 2*14, 19, SCOREBOARD_COLOR);
          GUI_DisString_EN(13*14, 4, minutes_string, &Font20, SCOREBOARD_COLOR, BLUE);
        }
}

void breakout_start(void)
{
  for ( int c = 0; c <  320; c+=40)
  {
    LCD_SetArealColor( c+0, 0, c+0+40, 480, RED);
    for ( int i = 20; i < 480; i+= 20)
    {
      LCD_SetArealColor( c+0, i, c+0+40, i+2, GRAY);
    }

    for ( int i = 0; i < 480; i+= 40)
    {
      LCD_SetArealColor( c+20, i, c+22, i+20, GRAY);
      LCD_SetArealColor( c+38, i+20, c+40, i+20+20, GRAY);
    }
  
  }
  GUI_DisString_EN(100, 110, "BREAKOUT", &Font24, LCD_BACKGROUND, YELLOW);
  
  create_butt();
  gamerun();
  butt_pause = 0;
}

void create_butt(void) {
  char buttonlabels[4][6] = {"Quit", "Fire", "<", ">"};
  uint16 buttoncolors[4] = { DARKGREY, PURPLE, DARKGREEN,ORANGE};
    for (uint8_t row=0; row<2; row++) {
      for (uint8_t col=0; col<2; col++) {
        initButton(col + row*2, 60+col*(80+120), 
                 370+row*(50+20),    // x, y, w, h, outline, fill, text
                  90, 50,  WHITE, buttoncolors[col+row*2],  WHITE,
                  buttonlabels[col + row*2], 2); 
        drawButton(col + row*2, 0);
      }
    }
}

void gamerun(void) {
//    Scoreboard
      game_over = 0;
      level = GAME_LEVEL;
      lives = GAME_LIVES;
      score = GAME_SCORE;
      
      // blank board
      LCD_SetArealColorWH(0, 0, 320, MAX_Y, BACKGROUND_COLOR);
      LCD_SetArealColorWH(0,            0, MAX_X,             BOARD_TOP,            SCOREBOARD_COLOR);
      LCD_SetArealColorWH(0,            0, BOARD_LEFT - 0,    MAX_Y - PADDLE_H - 2, SCOREBOARD_COLOR);
      LCD_SetArealColorWH(BOARD_RIGHT,  0, TILE_W / 2,        MAX_Y - PADDLE_H - 2, SCOREBOARD_COLOR);
      LCD_SetArealColorWH(BOARD_LEFT, BOARD_TOP, BOARD_RIGHT - BOARD_LEFT , MAX_Y - BOARD_TOP - PADDLE_H, BACKGROUND_COLOR);
      LCD_SetArealColorWH(0, MAX_Y, MAX_X, 2, SCOREBOARD_COLOR);

      scoreinit();

      // initialize tile matrix
      newLevel();
      
      ballsetXY(MAX_X / 2, MAX_Y - PADDLE_H - BALL_R * 2 - 2);
      ballsetXiYi(-BALL_SPEED_H, -BALL_SPEED_V);
      
      paddlesetXY((MAX_X -PADDLE_W)/ 2, MAX_Y - PADDLE_H);
      paddledraw_init();
      ballclear();
      ballsetX(paddlegetX());
      balldraw();
}

void newLevel(void) {
      HalAdcSetReference(HAL_ADC_REF_125V);
      mysrand(adcReadSampled(HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_14, HAL_ADC_REF_125V, 1));
      
      tilesdrawAll();
      beep_f = 200;
}

void tilesdrawAll(void) {
      tilesLeft = 0;
      for(int i = 0; i < ROWS; i++)
      {
        for(int j = 0; j < COLS; j++)
        {
          tiles[i][j] = TILE_IS_NOT_DRAWN;
        }
      }

      uint8 tilesLeftToDraw = ROWS * COLS;
    
      while ( tilesLeftToDraw > 0)
      { 
        uint8 c = (uint8)myrand(0, COLS);
        uint8 r = (uint8)myrand(0, ROWS);
        if (tiles[r][c] == TILE_IS_DRAWN)
        {
          tilesLeftToDraw--;
        } else {
        LCD_SetArealColorWH((TILES_LEFT + c * TILE_W), TILES_TOP + r * TILE_H, TILE_W - 2, TILE_H - 2, COLORS[r]);
        tiles[r][c]= TILE_IS_DRAWN;
        tilesLeftToDraw--;
        tilesLeft++;
        beeping_beep( 400 + r* 35 + c * 2, 5 );
        _delay_ms(5);
        }
      }    
       tilesLeftupdate();
    }

static uint32 next  = 1;

uint16 myrand(uint32 min, uint32 max) {
    next = next * 1103515245 + 12345;
    return((uint16)((next%(max-min))+min));
}

void mysrand(uint16 seed) {
    next = seed;
}

/////////////ball

const int SCORE[7] = {0, 1, 1, 3, 5, 5, 7};
int left=BOARD_LEFT;
int right=BOARD_RIGHT-3;
int top=BOARD_TOP;
int bottom=MAX_Y;
  
//float x, y, xi, yi;
int16 x, y, xi, yi;

void ballclear(void) //clear the ball
{
  LCD_SetArealColorWH(x, y, BALL_R * 2, BALL_R * 2, BLACK);
}
  
void balldraw(void) //draw ball
{
  LCD_SetArealColorWH(x, y, BALL_R * 2, BALL_R * 2, BALL_COLOR);
}

void ballsetXY(int x_, int y_) //set the ball to the center of the shovel
    {
      x = x_;
      y = y_;
    }

void ballsetX(int x_) //set the ball on the shovel X
    {
      x = x_;
    }

void ballsetXiYi(int xi_, int yi_) //-1.00 -1.00 indicates when the ball is in the center of the spade
    {
      xi = xi_;
      yi = yi_;
    }

bool ballescaped(void) // the ball is not hit 
{
      return y >= paddlegetY();
}
  
void move(void)  // ?????????? ???????? ???? x,y ??????????
{
      if((xi < 0 && x + xi < left)||(xi > 0 && x +xi >= right))
      {
        xi *= -1;
      }

      if(yi < 0 && y+yi-BALL_R*2 < top)
      {
        yi *= -1;
      }

      x+= xi;
      y+= yi;
}

int ballgetX(void)
{
      return x;
}

int ballgetY(void)
{
      return y;
}

int ballgetXi(void)
{
      return xi;
}

////// tiles

bool tilesallGone(void)
{
      return tilesLeft == 0;
}
    
void clearTile( int x, int y )
{
      tiles[y][x] = TILE_IS_NOT_DRAWN;
      tilesLeft--;
}
    
bool tilesexists(int x, int y)
{
      return tiles[y][x] == TILE_IS_DRAWN;
}

///////paddle

int px, py;
  
  // draws paddle on screen when it moves
  // to avoid spending too much time clearing the screen and drawing
  // only the same amount of pixels that the paddle moves are cleared and drawn
  
void paddledraw(int m)
    {
      if ( m < 0 && px > PADDLE_W )
      {
        // cant move beyond the screen border
        if ( m < -(px - PADDLE_W)) m = -(px - PADDLE_W);
        // remove part of the old paddle on the left
          LCD_SetArealColorWH( px + PADDLE_W + m, py, -m, PADDLE_H, BACKGROUND_COLOR);
      
        // add a bit more to the left of the existing paddle
          LCD_SetArealColorWH( px - PADDLE_W + m, py, -m, PADDLE_H, PADDLE_COLOR);

        px+=m;
      }
      else if ( m > 0 && x < MAX_X - PADDLE_W )
      {
        // cant move beyond the screen border
        if ( m > MAX_X - px - PADDLE_W ) m = MAX_X - px - PADDLE_W;      
        // remove a part of the old paddle on the right
          LCD_SetArealColorWH( px - PADDLE_W, py, m, PADDLE_H, BACKGROUND_COLOR);
      
        // add a bit more  of paddle to the left of the existing paddle
          LCD_SetArealColorWH( px + PADDLE_W, py, m, PADDLE_H, PADDLE_COLOR );
      
        px+=m;
      }
}

    // draws paddle the first time
    // all paddle must be drawn
  
void paddledraw_init(void)
{
  LCD_SetArealColorWH(px- PADDLE_W, py, PADDLE_W * 2, PADDLE_H, PADDLE_COLOR);
}
  
void paddlesetXY(int x_, int y_)
{
      px = x_;
      py = y_;
}

int paddlegetX(void)
{
      return px;
}

int paddlegetY(void)
{
      return py;
}

void scoreinit(void){
      // draw back wall and sidewalls
      GUI_DisString_EN(0, 4, "T   S000    :   L0  B", &Font20, LCD_BACKGROUND, BLACK);
      time.hour = 0;
      time.minutes = 0;
      clockupdate();
      
      LCD_SetArealColorWH((9-nDigits(score))*14, 4, 14*nDigits(score), 19, SCOREBOARD_COLOR);
      GUI_DisNum((9-nDigits(score))*14, 4, score, &Font20, SCOREBOARD_COLOR, RED);

      LCD_SetArealColorWH((19-nDigits(level))*14, 4, 14*nDigits(level), 19, SCOREBOARD_COLOR);
      GUI_DisNum((19-nDigits(level))*14, 4, level, &Font20, SCOREBOARD_COLOR, RED);

      LCD_SetArealColorWH((22-nDigits(lives))*14, 4, 14*nDigits(lives), 19, SCOREBOARD_COLOR);
      GUI_DisNum((22-nDigits(lives))*14, 4, lives, &Font20, SCOREBOARD_COLOR, RED);
}

void scoreupdate( int scored )
{
      score+= scored;

      LCD_SetArealColorWH((9-nDigits(score))*14, 4, 14*nDigits(score), 19, SCOREBOARD_COLOR);
      GUI_DisNum((9-nDigits(score))*14, 4, score, &Font20, SCOREBOARD_COLOR, RED);
      
      tilesLeftupdate();
}

void tilesLeftupdate(void)
{
      LCD_SetArealColorWH(1*14, 4, 2*14, 19, SCOREBOARD_COLOR);
      GUI_DisNum(1*14, 4, tilesLeft, &Font20, SCOREBOARD_COLOR, RED);
}
  
void scorenextLevel(void)
{
      level++;

      LCD_SetArealColorWH((19-nDigits(level))*14, 4, 14*nDigits(level), 19, SCOREBOARD_COLOR);
      GUI_DisNum((19-nDigits(level))*14, 4, level, &Font20, SCOREBOARD_COLOR, RED);

      beeping_seq_start(1);      
}

void scoredied(void)
{
      lives--; // ?????? ?????
      if ( lives == 0) {
        GUI_DisString_EN(100, 110, "GAME OVER", &Font24, LCD_BACKGROUND, YELLOW);
        game_over = 1;
        
        beeping_seq_start(2);
      }
      LCD_SetArealColorWH((22-nDigits(lives))*14, 4, 14*nDigits(lives), 19, SCOREBOARD_COLOR);
      GUI_DisNum((22-nDigits(lives))*14, 4, lives, &Font20, SCOREBOARD_COLOR, RED);
}

int nDigits(int value)
    {
      int digits = 1;
      long compare = 10;
      while (compare <= value)
      {
        compare *= 10;
        digits++;
      }
      return digits;
}

void play(void)
{
        
//        unsigned long waited = millis() - last;
        unsigned long waited = osal_GetSystemClock() - last;
//        LREP("waited=%d\r\n", waited);
//        if ( waited > BALL_MOVE_WAIT )
//        {
//          last = millis();
          last = osal_GetSystemClock();
        
          ballclear();
          move();
          balldraw();

          if (collision()) // rebound from a paddle
          {
            beep_f = 200;
            beeping_beep(400,30);
          }
        
          // check collision w/ tiles        
          int scored = ballcollision();

          if ( scored > 0)
          {
            beep_f *=1.1;
            if ( beep_f > 3000) beep_f = 3000;
            beeping_beep(beep_f,25);
          }
        
          if (scored > 0)
          {
            scoreupdate(scored);

            if ( tilesallGone() )
            {
              s_play = 0;
              ballclear();
              newLevel();
              scorenextLevel();
              ballsetXY(paddlegetX(), MAX_Y - PADDLE_H - BALL_R * 2 - 2);
              ballsetXiYi(-BALL_SPEED_H, -BALL_SPEED_V);
              balldraw();
              initButton(1, 260, 370, 90, 50,  WHITE, PURPLE,  WHITE, "Fire", 2); 
              drawButton(1, 0);
              
              return;
            }
          }

          if (ballescaped()) // the ball is not hit
          {
            scoredied();

            for (int i = 54; i > 30; i--)
            {
              beeping_beep(myrand(i,i*i), 3);
              _delay_ms(6);
            }
          
            s_play = 0;
            ballclear();
            ballsetXY(paddlegetX(), MAX_Y - PADDLE_H - BALL_R * 2 - 2);
            ballsetXiYi(-BALL_SPEED_H, -BALL_SPEED_V);
            balldraw();
            initButton(1, 260, 370, 90, 50,  WHITE, PURPLE,  WHITE, "Fire", 2); 
            drawButton(1, 0);

            return;
          }
//        }

        int m = 0;        
//        if (waited > PADDLE_MOVE_WAIT && m != 0)
//        {
//          last = millis();
          last = osal_GetSystemClock();
          paddledraw(m);
//        }

}

    // check for collision with paddle
    // calculate score and bounce

int ballcollision(void)
{
      const int bx = x+xi;
      const int by = y+yi;
    
      int hit;
    
      if ( hit = hitTile(    bx,              by  ))
      {
        return SCORE[hit];
      }
        
      if ( hit = hitTile(    bx + BALL_R * 2, by + BALL_R * 2  ))
      {
        return SCORE[hit];
      }
    
      if ( hit = hitTile(    bx + BALL_R * 2, by  ))
      {
        return SCORE[hit];
      }
    
      hit = hitTile(         bx,              by + BALL_R * 2  );
    
      return SCORE[hit];
}

    // check for collision with paddle
    // and bounce
bool collision(void)
{
      // only interested if ball is moving down
      if ( yi < 0 ) return false;

      const int ny = y+yi+BALL_R*2;

      if ( ny < paddlegetY() ) return false;
    
      // ball center
      const int bc = x + BALL_R;

      const int diff = bc - paddlegetX();

      if ( diff < -PADDLE_W-PADDLE_TOLERANCE || diff > PADDLE_W + PADDLE_TOLERANCE)
      {
        return false;
      }
      else if (diff > 0)
      {
        const int hit = diff / ((PADDLE_W + PADDLE_TOLERANCE)/2);
//        xi = (float)hit * .5f + .5f;
        xi = (int16)(hit * .5f + .5f);
        yi = - 2 + xi;
      }
      else if (diff < 0)
      {
        const int hit = diff / ((PADDLE_W + PADDLE_TOLERANCE)/2);
//        xi = (float)hit * .5f - .5f;
        xi = (int16)(hit * .5f - .5f);
        yi = - 2 - xi;
      }
      else
      {
        yi *= -1;
      }
    
      return true;
}

int hitTile( int x, int y)
{
      int ty =(y - TILES_TOP)/ TILE_H;
      int tx =(x - TILES_LEFT)/ TILE_W;
    
      const bool hit = ty >= 0 && ty < ROWS && tx >= 0 && tx < COLS && tilesexists(tx,ty);

      if (!hit) return 0;
    
      clearTile( tx, ty );

      const int score = ROWS - ty;
    
      ty*= TILE_H;
      ty+= TILES_TOP;

      LCD_SetArealColorWH(TILES_LEFT + tx * TILE_W, ty, TILE_W - 2, TILE_H - 2, BACKGROUND_COLOR);
        
      const int ty2= ty + TILE_H;

      if ( (yi > 0 && y > ty) || (yi < 0 && y > ty2))
      {
        xi *=-1;
      }
      else
      {
        yi *=-1;
      }

      return score;
}

static void _delay_us(uint16 microSecs)
{
  while(microSecs--)
  {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

static void _delay_ms(uint16 milliSecs)
{
  while(milliSecs--)
  {
    _delay_us(1000);
  }
}
#endif //TFT3IN5