//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Common configuration resources and definitions
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
#include <Arduino.h>
#include "RDX-rp2040.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   TFTgui.cpp

   GUI presentation and management over a TFT display
   Code originally created: May 2022 as display.cpp
   Author: Klaus Fensterseifer
   https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj

   Excerpts: Apr 2021
   Author: Arjan te Marvelde
   May2022: adapted by Klaus Fensterseifer
   https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj

   Adaptation to ADX-rp2040 project by Pedro Colla (LU7DZ) 2022
   Re-factoring by Pedro Colla (LU7DZ) 2022

  =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "SPI.h"
#include "TFTgui.h"

#include <FS.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>

/*==================================================================================================*
    Display definition and standard controls
  ==================================================================================================*/
TFT_eSPI tft = TFT_eSPI();

/*---------------------------------------
   define buttons
*/
ButtonWidget btnTX = ButtonWidget(&tft);
ButtonWidget btnCQ = ButtonWidget(&tft);
ButtonWidget btnAUTO = ButtonWidget(&tft);
ButtonWidget btnBAND = ButtonWidget(&tft);

/*---------------------------------------
   basic TFT std calibration date
   (should include later a tool to define thru actual calibration)
   and store it
*/
uint16_t calData[5] = { 162, 3712, 292, 3458, 7 };
uint16_t xant;
uint16_t yant;
bool prevpressed = false;
char GUIStr[128];
int vmin = 255;
int vmax = 0;
bool GUI_Enabled=false;


uint16_t call_af_frequency;
int8_t call_self_rx_snr;
char call_station_callsign[8];
char call_grid_square[4];

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
//*                          Structures to support primitive GUI constructs
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
/*----------------
   Square
*/
struct square {
  int xStart;
  int yStart;
  int width;
  int height;
  int16_t color;
};
/*----------------
   Rounded square
*/
struct roundedSquare {
  int xStart;
  int yStart;
  int width;
  int height;
  byte cornerRadius;
  int16_t color;
};
/*----------------
   Triangle
*/
struct triangle {
  int point1X;
  int point1Y;
  int point2X;
  int point2Y;
  int point3X;
  int point3Y;
  int16_t color;
  bool inverse;
  bool fickle;
  uint32_t tfickle;
};



/*----------------------------------------
   color conversion support routine
   used to allow calling from other modules, concentrate the use of tft variable locally
  ----------------------------------------*/
uint16_t tft_color565(uint16_t r, uint16_t g, uint16_t b)
{
  return tft.color565(r, g, b);
}

/*-----------------------------
   check if the pen hit a given
   area of the display
*/
bool checkarea(uint16_t x, uint16_t y, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  if (x >= x0 && y >= y0 &&
      x <= x1 && y <= y1) {
    return true;
  }
  return false;
}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
   Class progressBar
   Define a linear bar to show progress
  /*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
class progressBar {
  public:
    int  x;
    int  y;
    int  w;
    int  h;
    int  g;
    int  n;
    int bg;
    byte s = BLUE2BLUE;
    byte progress = 0;

    TFT_eSPI* tft;

    progressBar(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, int _g, int _n, byte _s, int bg);
    void show(int colour);
    void reset();

    /*----------
       Constructor
       object position and visual dimensions
    */
  private:
    uint16_t zambo;
};
progressBar::progressBar(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, int _g, int _n, byte _s, int _bg) {

  tft = _tft;
  x = _x; //Allow space for the label
  y = _y;
  w = _w;
  h = _h;
  g = _g;
  n = _n;
  s = _s;
  bg = _bg;
  progress = 0;
}


void progressBar::show(int colour) {

  if (progress == 0) {
    tft->fillRect(x + XBTN , y, w, 6, TFT_BLACK);
    return;
  }
  if (progress > 15) progress = 15;
  tft->fillRect(x + XBTN , y, (progress) * 18, 6, colour);
}

void progressBar::reset() {
  progress = 0;
  show(TFT_BLACK);
}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
   Class linearMeter
   Define the linear display LED meter object
  /*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
class linearMeter {        // The class
  public:          // Access specifier
    /*---------------------------------------------
       val  =  reading to show (range is 0 to n)
       x, y = position of top left corner
       w, h = width and height of a single bar
       g    = pixel gap to next bar (can be 0)
       n    = number of segments
       s    = colour scheme
      -----------------------------------------------*/
    int  x;
    int  y;
    int  w = 5;
    int  h = 25;
    int  g = 3;
    int  n = 15;
    byte s = BLUE2BLUE;
    int  colour;
    int  bg;
    int  prevval = 0;
    uint8_t mode;
    char lmode[8];
    char mmode[16];
    uint8_t modeprev = 0xff;

    TFT_eSPI* tft;

    linearMeter(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, int _g, int _n, byte _s, int bg);
    void show(int _m, int _v);
    uint16_t rainbowColor(uint8_t spectrum);
    void demo();

  private:
    uint16_t zambo;
};
/*----------
   Constructor
   object position and visual dimensions
*/
linearMeter::linearMeter(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, int _g, int _n, byte _s, int _bg) {

  colour = bg;
  tft = _tft;
  x = _x; //Allow space for the label
  y = _y;
  w = _w;
  h = _h;
  g = _g;
  n = _n;
  s = _s;
  mode = 0xff;
  bg = _bg;
  strcpy(lmode, "");
  strcpy(mmode, "");
  prevval = 0;
}

/*----------------------------------------------------------------------------------------------
   show
   update the display with a given meter indicator and value
   the caller is responsible to scale up the values sent
*/
void linearMeter::show(int _m , int _v)
{

  if (_v < 0 || _v > n) {
    _v = n;
  }
  int val = _v;
  bool mchange = false;
  if (_m != mode) {
    mode = _m;
    mchange = true;
    tft->fillRect((x - X_CHAR1), y, (w + g)*n + X_CHAR1, h + 1.5 * Y_CHAR1 - 17, TFT_BLACK);

    switch (mode) {
      case PWR : sprintf(lmode, "%s", "Pwr");
        s = GREEN2RED;
        break;
      case SWR : sprintf(lmode, "%s", "SWR");
        s = GREEN2RED;
        break;
      case SUNIT : sprintf(lmode, "%s", "S  ");
        s = BLUE2BLUE;
        break;
      case MIC   : sprintf(lmode, "%s", "MIC");
        s = GREEN2RED;

    }
    tft->setTextColor(TFT_CYAN, TFT_BLACK);
    tft->drawString(lmode, x - X_CHAR1,  y + 1.5 * Y_CHAR1, 1); // Print the label
  }

  // Draw n colour blocks
  for (int b = 1; b <= n; b++) {

    /*---
       draw scale supplements according to mode
    */
    if (mchange == true) {

      tft->setTextColor(TFT_CYAN, TFT_BLACK);
      tft->fillRect((x + b * (w + g)), y + h + 3, w, 2, TFT_CYAN);

      if (mode == PWR) {
        if ((b % 3) == 0) {
          sprintf(mmode, "%d", b / 3);
          tft->drawString(mmode, x + b * (w + g), y + 1.5 * Y_CHAR1, 1);
        }
      }

      if (mode == SWR) {
        switch (b) {
          case 1 : sprintf(mmode, "%s", "1"); break;
          case 5 : sprintf(mmode, "%s", "2"); break;
          case 10: sprintf(mmode, "%s", "3"); break;
          case 12: sprintf(mmode, "%s", "5"); break;
          case 15: sprintf(mmode, "%s", "+"); break;
          default: sprintf(mmode, "%s", ""); break;
        }
        tft->drawString(mmode, x + b * (w + g), y + 1.5 * Y_CHAR1, 1);
      }

      if (mode == SUNIT || mode == MIC) {
        if (b <= 9) {
          sprintf(mmode, "%d", b);
          tft->drawString(mmode, x + b * (w + g), y + 1.5 * Y_CHAR1, 1);
        }
        if (b == 10) {
          sprintf(mmode, "%s", "+");
          tft->drawString(mmode, x + b * (w + g), y + 1.5 * Y_CHAR1, 1);
        }
      }
    }
  }
  if (val == prevval) {
  }

  if (val > prevval) {

    for (int b = prevval; b < val; b++) {
      tft->fillRect((x + b * (w + g)), y, w, h, TFT_CYAN);
    }
  } else {

    for (int b = val; b < prevval; b++) {
      tft->fillRect((x + b * (w + g)), y, w, h, bg);
    }
  }
  prevval = val;

}
void linearMeter::demo() {
  for (int mode = 0; mode < 4; mode++) {
    for (int i = 0; i < 6; i++) {
      int pwr = rand() % 5;
      show(PWR, pwr);
      delay(500);
    }
    for (int i = 0; i < 6; i++) {
      int swr = rand() % 12;
      show(SWR, swr);
      delay(500);
    }
    for (int i = 0; i < 6; i++) {
      int s = rand() % 12;
      show(SUNIT, s);
      delay(500);
    }
    for (int i = 0; i < 6; i++) {
      int m = rand() % 12;
      show(MIC, m);
      delay(500);
    }
    delay(1000);
  }
}
/***************************************************************************************
** Function name:           rainbowColor
** Description:             Return a 16 bit rainbow colour
** If 'spectrum' is in the range 0-159 it is converted to a spectrum colour
** from 0 = red through to 127 = blue to 159 = violet
** Extending the range to 0-191 adds a further violet to red band
***************************************************************************************/

uint16_t linearMeter::rainbowColor(uint8_t spectrum) {

  spectrum = spectrum % 192;

  uint8_t red   = 0; // Red is the top 5 bits of a 16 bit colour spectrum
  uint8_t green = 0; // Green is the middle 6 bits, but only top 5 bits used here
  uint8_t blue  = 0; // Blue is the bottom 5 bits

  uint8_t sector = spectrum >> 5;
  uint8_t amplit = spectrum & 0x1F;

  switch (sector)
  {
    case 0:
      red   = 0x1F;
      green = amplit; // Green ramps up
      blue  = 0;
      break;
    case 1:
      red   = 0x1F - amplit; // Red ramps down
      green = 0x1F;
      blue  = 0;
      break;
    case 2:
      red   = 0;
      green = 0x1F;
      blue  = amplit; // Blue ramps up
      break;
    case 3:
      red   = 0;
      green = 0x1F - amplit; // Green ramps down
      blue  = 0x1F;
      break;
    case 4:
      red   = amplit; // Red ramps up
      green = 0;
      blue  = 0x1F;
      break;
    case 5:
      red   = 0x1F;
      green = 0;
      blue  = 0x1F - amplit; // Blue ramps down
      break;
  }

  return red << 11 | green << 6 | blue;
}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        iconPDX class
        manages an specific icon
  =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
class iconPDX {        // The class
  public:          // Access specifier

    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    const short unsigned int* iconOn;
    const short unsigned int* iconOff;
    TFT_eSPI* tft;
    iconPDX(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, const unsigned short* _iconOn, const unsigned short* _iconOff);
    void show(bool f, bool falsify);
    void show(bool f);
    void falsifyIcon();

  private:
    uint16_t zambo;
};
/*----------------------------
   Constructor
*/
iconPDX::iconPDX(TFT_eSPI* _tft, int _x, int _y, int _w, int _h, const unsigned short* _iconOn, const unsigned short* _iconOff) {
  x = _x;
  y = _y;
  w = _w;
  h = _h;

  tft = _tft;

  iconOn = _iconOn;
  iconOff = _iconOff;
}

void iconPDX::falsifyIcon() {

  for (int i = 0; i < 5; i++) {
    tft->drawLine(x + i, y, x + w, y + h - i, TFT_RED);
    tft->drawLine(x, y - i, x + w - i, y + h, TFT_RED);
    tft->drawLine(x + i, y + h, x + w, y - i, TFT_RED);
    tft->drawLine(x, y + h - i, x + w - i, y, TFT_RED);

  }

}
void iconPDX::show(bool f, bool falsify) {
  if (f == true) {
    if (iconOn != NULL) {
      tft->pushImage(x, y, w, h, iconOn);
      if (falsify == true) {
        falsifyIcon();
      }
    } else {
      tft->fillRect(x, y, w, h, TFT_BLACK);
      tft->drawRect(x, y, w, h, TFT_RED);
    }
  } else {
    if (iconOff != NULL) {
      tft->pushImage(x, y, w, h, iconOff);
      if (falsify == true) {
        falsifyIcon();
      }
    } else {
      tft->fillRect(x, y, w, h, TFT_BLACK);
      tft->drawRect(x, y, w, h, TFT_RED);
    }
  }

}
void iconPDX::show(bool f) {
  show(f, false);
}

class buttonPDX {
  public:

    buttonPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg);


  private:

    uint16_t zambo;
};

buttonPDX::buttonPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg) {

}
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        displayPDX class
        Manages an area with several controls
  =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
class displayPDX {        // The class
  public:          // Access specifier
    roundedSquare b;

    uint16_t bg;
    uint16_t tx;
    uint32_t f;
    uint16_t cursor;

    bool enabled;
    bool cq;
    bool autom;

    triangle trianglePDX[2];

    TFT_eSPI* tft;

    struct btn {
      uint16_t x;
      uint16_t y;
      uint16_t w;
      uint16_t h;
      uint16_t color;
      bool     inverse;
      ButtonWidget* btn;
      char label[8];
      bool     fickle;
      uint32_t tfickle;
      int      v;
    };

    btn btnPDX[BUTTON_END];
    bool redraw;

    displayPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg);
    void show(bool fShow);
    void check();
    void addBtn(int btnIndex, ButtonWidget* btn);
    void demo();


    void showFreq();

    void setCursor(uint16_t _c) {
      cursor = _c;
      showCursor();
    }
    
    void setFreq() {
      showFreq();
      showCursor();
    }

    void setBtn(int btnIndex, char* btnLabel, bool inverse, bool fickle);
    void showBtn(int btnIndex);
    void fickleBtn(int btnIndex);
    int checkButton(int x, int y);
    void onclick(int btnIndex);
    void set(int btnIndex,int v);

    void showTriangle(uint16_t t);
    void fickleTriangle(uint16_t t);
    void setTriangle(uint16_t t, bool inverse, bool fickle);
    void init();
    void showCursor();
    bool point(uint16_t x, uint16_t y);
    int  checkTriangle(int x, int y);
    void onclickTriangle(int btnTriangle);

  private:
    uint16_t zambo;
};

/*---------------------
   Constructor and initialization
*/
displayPDX::displayPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg) {

  b.xStart = _x;
  b.yStart = _y;
  b.width = _w;
  b.height = _h;
  b.cornerRadius = _r;
  b.color = _c;

  enabled = true;
  bg = _bg;
  tx = _tx;
  redraw=false;

#define TRIANG_BASE    40
#define TRIANG_HEIGHT  20
#define TRIANG_LEFT_X  20
#define TRIANG_LEFT_Y  40

#define TRIANG_SHIFT_L_X  5
#define TRIANG_SHIFT_L_Y  2

#define TRIANG_SHIFT_R_X  227
#define TRIANG_SHIFT_R_Y  2

  trianglePDX[TRIANGLE_LEFT].point1X = b.xStart + TRIANG_SHIFT_L_X + TRIANG_LEFT_X - TRIANG_HEIGHT;
  trianglePDX[TRIANGLE_LEFT].point1Y = b.yStart + TRIANG_SHIFT_L_Y + TRIANG_LEFT_Y + (TRIANG_BASE / 2);
  trianglePDX[TRIANGLE_LEFT].point2X = b.xStart + TRIANG_SHIFT_L_X + TRIANG_LEFT_X;
  trianglePDX[TRIANGLE_LEFT].point2Y = b.yStart + TRIANG_SHIFT_L_Y + TRIANG_LEFT_Y + TRIANG_BASE;
  trianglePDX[TRIANGLE_LEFT].point3X = b.xStart + TRIANG_SHIFT_L_X + TRIANG_LEFT_X;
  trianglePDX[TRIANGLE_LEFT].point3Y = b.yStart + TRIANG_SHIFT_L_Y + TRIANG_LEFT_Y;
  trianglePDX[TRIANGLE_LEFT].color = TFT_BLUE;

  trianglePDX[TRIANGLE_RIGHT].point1X = b.xStart + TRIANG_SHIFT_R_X + TRIANG_LEFT_X + TRIANG_HEIGHT;
  trianglePDX[TRIANGLE_RIGHT].point1Y = b.yStart + TRIANG_SHIFT_R_Y + TRIANG_LEFT_Y + (TRIANG_BASE / 2);
  trianglePDX[TRIANGLE_RIGHT].point2X = b.xStart + TRIANG_SHIFT_R_X + TRIANG_LEFT_X;
  trianglePDX[TRIANGLE_RIGHT].point2Y = b.yStart + TRIANG_SHIFT_R_Y + TRIANG_LEFT_Y + TRIANG_BASE;
  trianglePDX[TRIANGLE_RIGHT].point3X = b.xStart + TRIANG_SHIFT_R_X + TRIANG_LEFT_X;
  trianglePDX[TRIANGLE_RIGHT].point3Y = b.yStart + TRIANG_SHIFT_R_Y + TRIANG_LEFT_Y;
  trianglePDX[TRIANGLE_RIGHT].color = TFT_BLUE;

  cq = false;
  autom = false;

  for (int i = 0; i < BUTTON_END; i++) {
    strcpy(btnPDX[i].label, "");
    btnPDX[i].v=0;
  }

  tft = _tft;

}

void displayPDX::init() {

  cq = false;
  cursor = S1KHZ;
  
  setTriangle(TRIANGLE_LEFT, false, false);
  setTriangle(TRIANGLE_RIGHT, false, false);

}
void displayPDX::onclickTriangle(int btnTriangle) {
  switch(btnTriangle) {
    case TRIANGLE_LEFT: {
                          Band_slot--;
                          if (Band_slot<0) Band_slot=BANDS;
                          btnPDX[BUTTON_BAND].v=Band_slot;
                          set(BUTTON_BAND,Band_slot-1);
                          break;
    }
    case TRIANGLE_RIGHT: {     
                          Band_slot++;
                          if (Band_slot>BANDS) Band_slot=1;
                          btnPDX[BUTTON_BAND].v=Band_slot;
                          set(BUTTON_BAND,Band_slot-1);
                          break;
    }
  }
  setTriangle(btnTriangle,true,true);

}
int displayPDX::checkTriangle(int x, int y) {
    if (x >=trianglePDX[0].point1X && x <=trianglePDX[0].point2X && y>=trianglePDX[0].point3Y && y<=trianglePDX[0].point2Y) {
       onclickTriangle(TRIANGLE_LEFT);
       return 0;
    }

    if (x >=trianglePDX[1].point2X && x <=trianglePDX[1].point1X && y>=trianglePDX[1].point3Y && y<=trianglePDX[1].point2Y) {
      onclickTriangle(TRIANGLE_RIGHT);
      return 0;
    }
    return -1;

}

int displayPDX::checkButton(int x, int y) {

  for (int i = 0; i < 4; i++) {
    if ( x >= btnPDX[i].x && x <= btnPDX[i].x + btnPDX[i].w && y >= btnPDX[i].y && y <= btnPDX[i].y + btnPDX[i].h ) {
      switch (i) {
        case 0 : {
            onclick(BUTTON_TX);
            break;
          }
        case 1 : {
            onclick(BUTTON_CQ);
            break;
          }
        case BUTTON_AUTO : {
            onclick(BUTTON_AUTO);
            break;
          }
        case 3 : {
            onclick(BUTTON_BAND);
            break;
          }
      }
      return i;
    }
  }
  return -1;
}
/*-------
   Handle Buttons
*/
void displayPDX::fickleBtn(int btnIndex) {
  if (btnPDX[btnIndex].fickle == true) {
    if (millis() - btnPDX[btnIndex].tfickle >= TOUT_FICKLE) {
      btnPDX[btnIndex].fickle = false;
      btnPDX[btnIndex].inverse = !btnPDX[btnIndex].inverse;
      showBtn(btnIndex);
    }
  }
}
void displayPDX::addBtn(int btnIndex, ButtonWidget* b) {

  btnPDX[btnIndex].btn = b;

}

void displayPDX::setBtn(int btnIndex, char* btnLabel, bool inverse, bool fickle) {

  strcpy(btnPDX[btnIndex].label, btnLabel);
  btnPDX[btnIndex].inverse = inverse;

  btnPDX[btnIndex].fickle = fickle;
  btnPDX[btnIndex].tfickle = millis();
}
void displayPDX::set(int btnIndex,int v) {
  switch (btnIndex) {
    case BUTTON_BAND:
                    {
                     
                    char bandstr[6];
                    sprintf(bandstr,"%dm",Bands[v]);
                    setBtn(btnIndex,bandstr,false,false);
                    freq=Slot2Freq(v+1);
                    showFreq();
                    redraw=true;
                    break;
                    }
    case BUTTON_TX:
                    {
                    if (v==0) {
                       setBtn(btnIndex,(char*)"TX",false,false);
                    } else {
                       setBtn(btnIndex,(char*)"TX",true,false);
                    }
                    break;            
                    }
    case BUTTON_CQ:
                    {
                    switch(v) {
                      case 0 : {
                               setBtn(btnIndex,(char*)"CQ",false,false);
                               triggerCQ=false;
                               triggerCALL=false;
                               break;
                               }
                      case 1 : {
                               setBtn(btnIndex,(char*)"CQ",true,false);
                               triggerCQ=true;
                               break;
                               }
                      case 2 : {
                               setBtn(btnIndex,(char*)"Call",true,false);
                               triggerCALL=true;
                               break;
                               }
                    }
                    break;                 
                    }
    case BUTTON_AUTO:
                    {
                    if (v==0) {
                       setBtn(btnIndex,(char*)"Manual",true,true);
                       autosend=false;
                    } else {
                       setBtn(btnIndex,(char*)"Auto",true,true);
                       autosend=true;
                    }
                    break;   
                    }
  }
  btnPDX[btnIndex].v=v;
  showBtn(btnIndex);

}
/*-------------------
 * Handler to onclick buttons
 */
void displayPDX::onclick(int btnIndex) {
  switch (btnIndex) {
    
    case BUTTON_BAND:
                     {
                     Band_slot++;
                     if (Band_slot>BANDS) Band_slot=1;
                     btnPDX[btnIndex].v=Band_slot;
                     set(btnIndex,Band_slot-1);
                     break;
                     }
    case BUTTON_TX:
                     {
                     if (TX_State==0) {
                        startTX();
                     } else {
                        stopTX();
                     }
                     break;
                     }
    case BUTTON_CQ:  {
                     int z=btnPDX[btnIndex].v+1;
                     if (z>=2) z=0;
                     set(btnIndex,z);
                     break;               
                     }
    case BUTTON_AUTO:
                     {
                     if (autosend) {
                        set(btnIndex,0);
                     } else {
                        set(btnIndex,1);
                     }
                     break;
                     }
  }
}

void displayPDX::showBtn(int btnIndex) {

  tft->setTextFont(2);
  tft->setTextSize(1);

  btnPDX[btnIndex].x = b.xStart + XBTN + (btnIndex * (BUTTON_W + BUTTON_G));
  btnPDX[btnIndex].y = b.yStart + YBTN;
  btnPDX[btnIndex].w = BUTTON_W;
  btnPDX[btnIndex].h = BUTTON_H;

  btnPDX[btnIndex].btn->initButtonUL(btnPDX[btnIndex].x, btnPDX[btnIndex].y, btnPDX[btnIndex].w, btnPDX[btnIndex].h, TFT_BLUE, TFT_CYAN, TFT_BLUE, btnPDX[btnIndex].label, 1);
  btnPDX[btnIndex].btn->drawSmoothButton(btnPDX[btnIndex].inverse, 2, TFT_CYAN);
}

/*------
   Handle triangles
*/

void displayPDX::setTriangle(uint16_t t, bool inverse, bool fickle) {

  trianglePDX[t].inverse = inverse;
  trianglePDX[t].fickle = fickle;
  trianglePDX[t].tfickle = millis();
  showTriangle(t);
}

bool displayPDX::point(uint16_t x, uint16_t y) {

  for (int i = 0; i < BUTTON_END; i++) {
    if (checkarea(x, y, btnPDX[i].x, btnPDX[i].y, btnPDX[i].x + btnPDX[i].w, btnPDX[i].y + btnPDX[i].h)) {
      return true;
    }
  }
  if (checkarea(x, y, trianglePDX[TRIANGLE_LEFT].point1X, trianglePDX[TRIANGLE_LEFT].point2Y, trianglePDX[TRIANGLE_LEFT].point2X, trianglePDX[TRIANGLE_LEFT].point2Y)) {
  }
  if (checkarea(x, y, trianglePDX[TRIANGLE_RIGHT].point2X, trianglePDX[TRIANGLE_RIGHT].point2Y, trianglePDX[TRIANGLE_RIGHT].point1X, trianglePDX[TRIANGLE_RIGHT].point2Y)) {
  }

  return false;

}
/*----
    show
*/
void displayPDX::showTriangle(uint16_t t) {

  if (trianglePDX[t].inverse == true) {
    tft->fillTriangle(trianglePDX[t].point1X, trianglePDX[t].point1Y, trianglePDX[t].point2X, trianglePDX[t].point2Y, trianglePDX[t].point3X, trianglePDX[t].point3Y, TFT_BLUE);
  } else {
    tft->fillTriangle(trianglePDX[t].point1X, trianglePDX[t].point1Y, trianglePDX[t].point2X, trianglePDX[t].point2Y, trianglePDX[t].point3X, trianglePDX[t].point3Y, TFT_CYAN);
    tft->drawTriangle(trianglePDX[t].point1X, trianglePDX[t].point1Y, trianglePDX[t].point2X, trianglePDX[t].point2Y, trianglePDX[t].point3X, trianglePDX[t].point3Y, TFT_BLUE);
  }

}
/*--
   fickle
*/
void displayPDX::fickleTriangle(uint16_t t) {


  if (trianglePDX[t].fickle == true) {
    if (millis() - trianglePDX[t].tfickle >= TOUT_FICKLE) {
      trianglePDX[t].fickle = false;
      trianglePDX[t].inverse = !trianglePDX[t].inverse;
      showTriangle(t);
    }
  }

}

/*------
   Check fickleness
*/
void displayPDX::check() {

  fickleTriangle(TRIANGLE_LEFT);
  fickleTriangle(TRIANGLE_RIGHT);
  for (int i = 0; i < BUTTON_END; i++) {
    fickleBtn(i);
  }

}
/*------
   Show main dialog canvas

*/
void displayPDX::show(bool fShow) {

  if (fShow == true) {

    tft->fillRoundRect(b.xStart, b.yStart, b.width, b.height, b.cornerRadius, b.color);
    for (int i = 0; i < BUTTON_END; i++) {
      showBtn(i);
    }
    showFreq();
    showCursor();
    showTriangle(TRIANGLE_LEFT);
    showTriangle(TRIANGLE_RIGHT);

  } else {
    tft->fillRect(b.xStart, b.yStart, b.width, b.height, TFT_CYAN);
  }
}

/*--------
   Show frequency and frequency cursor

*/
void displayPDX::showFreq() {

  int xpos = b.xStart + XFREQ - 5;
  int ypos = b.yStart + YFREQ;
  uint32_t fshow = freq / 1000;
  tft->setTextColor(TFT_CYAN, TFT_CYAN);

  // Font 7 is to show a pseudo 7 segment display.
  // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .

  tft->setTextColor(TFT_CYAN);
  tft->fillRect(b.xStart + 27, b.yStart + 40, 220, Y_CHAR7 + 5, TFT_CYAN);
  tft->setTextColor(TFT_BLUE);

  tft->setTextFont(7);
  tft->setTextSize(1);


  if (fshow < 10000) {
     xpos = xpos + X_CHAR7 + 2;
  }

  tft->setTextFont(7);
  tft->setTextSize(1);

  xpos += tft->drawNumber(fshow, xpos, ypos, 7);

  tft->setTextSize(1);
  tft->setTextFont(7);

  int xcolon = xpos;
  xpos += tft->drawChar('.', xpos, ypos, 7);

  tft->setTextSize(1);
  tft->setTextFont(7);

  uint32_t decHz = (freq - (fshow * 1000)) / 100;
  tft->drawNumber(decHz, xpos, ypos, 7);

}

void displayPDX::showCursor() {


  tft->setTextSize(1);
  uint16_t fh = tft->fontHeight();
  for (uint16_t i = 0; i < 5; i++) {
    tft->drawFastHLine (b.xStart + XFREQ - 5 + (cursor * (X_CHAR7 + 4) + (cursor != 5 ? 0 : (X_CHAR7 / 2) - 1)) , (b.yStart + YFREQ + Y_CHAR7 + 6 ) + i , X_CHAR7, TFT_BLUE);
  }
}


/*********************************************************
  text class
  Handle a scrolling text area in the display
 *********************************************************/
class textPDX {        // The class
  public:          // Access specifier

#define TEXTLINES 10
#define TEXTCOLS  33

#define QSO   0
#define CQ   1
#define CALL 2
#define EXCH 3


    struct scrollText {
      char msg[TEXTCOLS + 1];
      uint16_t bg;
      uint16_t color;
      uint16_t af_frequency;
      int8_t self_rx_snr;
      char station_callsign[8];
      char grid_square[4];
    };

    scrollText t[TEXTLINES];

    roundedSquare b;
    uint16_t fh;
    uint16_t bg;
    uint16_t tx;

    TFT_eSPI* tft;

    textPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg);
    void show();
    int checkPoint(int x, int y);
    void printline(uint16_t _qso,char *s,uint16_t af_frequency,int8_t self_rx_snr,char *station_callsign,char *grid_square);

    void scroll();
    void demo();

  private:

    uint16_t zambo;

};

textPDX::textPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg) {

  b.xStart = _x;
  b.yStart = _y;
  b.width = _w;
  b.height = _h;
  b.cornerRadius = _r;
  b.color = _c;

  bg = _bg;
  tx = _tx;

  tft = _tft;

  for (int i = 0; i < TEXTLINES; i++) {
    strcpy(t[i].msg, "");
    t[i].bg = TFT_NAVY;
    t[i].color = TFT_GREENYELLOW;

  }


  tft->setTextColor(TFT_GREENYELLOW);
  tft->setTextFont(2);
  tft->setTextSize(1);
  fh = tft->fontHeight();

}
void textPDX::show() {

  tft->fillRoundRect(b.xStart, b.yStart, b.width, b.height, b.cornerRadius, b.color);
  tft->setTextColor(TFT_GREENYELLOW);
  tft->setTextFont(2);
  tft->setTextSize(1);

}  
void textPDX::printline(uint16_t _qso,char *s,uint16_t af_frequency,int8_t self_rx_snr,char *station_callsign,char *grid_square) {
  uint16_t _color;
  uint16_t _bg;

  scroll();

  switch (_qso) {
    case 0 : _color=TFT_BLACK;_bg=TFT_WHITE; break;
    case 1 : _color=TFT_BLACK;_bg=TFT_GREEN; break;
    case 2 : _color=TFT_BLACK;_bg=TFT_YELLOW; break;
    case 3 : _color=TFT_WHITE;_bg=TFT_RED; break;
    default : _color=TFT_BLACK;_bg=TFT_WHITE; break;
  }

  t[0].bg = _bg;
  t[0].color = _color;

  memset(t[0].msg, 0, TEXTCOLS);
  sprintf(t[0].msg, "%s", s);
  sprintf(t[0].msg, "%-32s", t[0].msg);

  strcpy(t[0].station_callsign,station_callsign);
  strcpy(t[0].grid_square,grid_square);
  t[0].af_frequency=af_frequency;
  t[0].self_rx_snr=self_rx_snr;
 
  tft->setTextColor(TFT_GREENYELLOW);
  tft->setTextFont(2);
  tft->setTextSize(1);
  show();

  char l[TEXTCOLS + 1];
  for (int i = 0; i < TEXTLINES; i++) {

    int j = TEXTLINES - 1 - i;
    tft->setTextColor(t[i].color, t[i].bg);
    sprintf(l, "%s", t[i].msg);
    tft->drawString(l, b.xStart + 2, b.yStart + 5 + (j * 9), 1); //

  }
  
}
void textPDX::scroll() {

  int i = TEXTLINES - 1;
  while (i > 0) {
    strncpy(t[i].msg, t[i - 1].msg, TEXTCOLS + 1);
    t[i].bg = t[i - 1].bg;
    t[i].color = t[i - 1].color;

    t[i].af_frequency=t[i-1].af_frequency;
    t[i].self_rx_snr=t[i-1].self_rx_snr;
    strncpy(t[i].station_callsign,t[i-1].station_callsign,7);
    strncpy(t[i].grid_square,t[i-1].grid_square,4);   
    i--;
  }
  t[0].bg = TFT_NAVY;
  t[0].color = TFT_GREENYELLOW;
  strcpy(t[0].msg, "");
  strcpy(t[0].station_callsign,"");
  strcpy(t[0].grid_square,"");
  t[0].af_frequency=0;
  t[0].self_rx_snr=0;

  i = 0;

}
int textPDX::checkPoint(int x, int y) {

  if ( (x >= b.xStart) && (x <= b.xStart + b.width) &&
       (y >= b.yStart) && (y <= b.yStart + b.height)) {
    int yPos = y - b.yStart + 5;
    yPos = yPos / 9;
    int yIndex = TEXTLINES - yPos;
    return yIndex;
  }
  return -1;
}

void textPDX::demo() {

}

/*********************************************************
  spectrum class
  This class handles a waterfall (spectrum) where signals
  are shown
 *********************************************************/
class spectrumPDX {        // The class
  public:          // Access specifier

#define BINS  480
#define LINES 125

    roundedSquare b;
    uint16_t fh;
    uint16_t bg;
    uint16_t tx;
    uint16_t time_idx = 0;

    TFT_eSPI* tft;

    spectrumPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg);
    void init();
    void draw(int m[]);
    void linedraw();
    void reset();

  private:

    uint16_t zambo;

};

spectrumPDX::spectrumPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg) {

  b.xStart = _x;
  b.yStart = _y;
  b.width = _w;
  b.height = _h;
  b.cornerRadius = _r;
  b.color = _c;

  bg = _bg;
  tx = _tx;

  tft = _tft;

}
/*----------------
   Initialize waterfall
*/
void spectrumPDX::init() {

  tft->fillRoundRect(b.xStart, b.yStart, b.width, b.height, b.cornerRadius, b.color);
  tft->setTextFont(2);
  tft->setTextSize(1);
  tft->setTextColor(TFT_YELLOW, TFT_RED);

  for (uint16_t i = 0; i < 5; i++) {
    tft->drawFastHLine (b.xStart, b.yStart + 20 + i, b.width, TFT_CYAN);
  }

  fh = tft->fontHeight();
  tft->fillRect(b.xStart, b.yStart + 20 - 5, 5, 5, TFT_CYAN);
  tft->fillRect(b.xStart + 200, b.yStart + 20 - 5, 5, 5, TFT_CYAN);
  tft->fillRect(b.xStart + 400, b.yStart + 20 - 5, 5, 5, TFT_CYAN);

  tft->fillRect(b.xStart + 100, b.yStart + 22 - 5, 3, 3, TFT_CYAN);
  tft->fillRect(b.xStart + 300, b.yStart + 22 - 5, 3, 3, TFT_CYAN);

  char buf[16];

  uint32_t fshow=freq/1000;
  tft->setTextColor(TFT_YELLOW, TFT_BLUE);
  sprintf(buf, "%ld", fshow);
  tft->drawString(buf, b.xStart + 10, b.yStart + 2, 2); // Print the line

  fshow=fshow+1;
  sprintf(buf, "%ld", fshow);
  tft->drawString(buf, b.xStart + 10 + 200, b.yStart + 2, 2); // Print the line

  fshow = fshow + 1;
  sprintf(buf, "%ld", fshow);
  tft->drawString(buf, b.xStart + 10 + 400, b.yStart + 2, 2); // Print the line

}
void spectrumPDX::linedraw() {
  tft->drawFastHLine (b.xStart, b.yStart + time_idx + 0 + 25  , b.width, TFT_CYAN);
  time_idx++;
  if (time_idx > LINES) {
    time_idx = 0;
    tft->fillRect(b.xStart, b.yStart + 25, 478, LINES, TFT_BLUE);
  }

}
void spectrumPDX::reset() {
  tft->fillRect(b.xStart, b.yStart + 25, 478, LINES, TFT_BLUE);
  time_idx=0;
}

/*-----------------
   Draw waterfall directly from energy bins
*/
void spectrumPDX::draw(int m[]) {

  for (int i = 0; i < BINS; i++) {
    int v = m[i];
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    int x = 100 * (v - vmin);
    if ((vmax - vmin) > 0) {
      x = x / (vmax - vmin);
    } else {
      x = 0;
    }


    uint16_t c = TFT_BLUE;
    if (v < 40) c = TFT_CYAN;
    if (x >= 40 && v < 60) c = TFT_YELLOW;
    if (x >= 60 && x < 70) c = TFT_ORANGE;
    if (x > 75) c = TFT_RED;
    if (x >= 25 ) {
      tft->drawPixel(b.xStart + i, b.yStart + time_idx + 0 + 25, c);
    }
    if ((i == 0 || i == 100 || i == 200 || i == 300 || i == 400)) {
      tft->drawPixel(b.xStart + i, b.yStart + time_idx + 0 + 25, TFT_CYAN);
    }
  }
  time_idx++;
  if (time_idx > LINES) {
    time_idx = 0;
    tft->fillRect(b.xStart, b.yStart + 25, 478, LINES, TFT_BLUE);
  }

}

/*********************************************************
  footer class
  Manages the footer of the display with static information
*********************************************************/
class footerPDX {        // The class
  public:          // Access specifier


    roundedSquare b;
    uint16_t fh;
    uint16_t bg;
    uint16_t tx;

    bool clock = true;

    TFT_eSPI* tft;

    footerPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg);
    void init();
    void show();
    void showtime();
    void update();

  private:

    uint16_t zambo;

};
footerPDX::footerPDX(TFT_eSPI* _tft, uint16_t _x, uint16_t _y, uint16_t _w, uint16_t _h, uint8_t _r, uint16_t _c, uint16_t _tx, uint16_t _bg) {

  b.xStart = _x;
  b.yStart = _y;
  b.width = _w - 2;
  b.height = _h;
  b.cornerRadius = _r;
  b.color = _c;

  bg = _bg;
  tx = _tx;
  tft = _tft;
}
void footerPDX::init() {

  tft->fillRoundRect(b.xStart, b.yStart, b.width, b.height, b.cornerRadius, b.color);
  tft->setTextFont(2);
  tft->setTextSize(1);
  tft->setTextColor(TFT_YELLOW, TFT_RED);
  fh = tft->fontHeight();

  time_t now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeinfo);

}
void footerPDX::show() {

  tft->setTextColor(TFT_GREENYELLOW);
  tft->setTextFont(2);
  tft->setTextSize(1);
  tft->setTextColor(tx, bg);


#define XFOOT 6+16*9
#define YFOOT 6

  sprintf(hi, "%s version %s(%s)", programname, version, build);
  tft->drawString(hi, b.xStart + 6, b.yStart + YFOOT, 1); // Print the line
  tft->drawFastVLine (b.xStart + 6 + 16 * 8, b.yStart, 15, TFT_RED);
  char buf[8];

  sprintf(hi, "%s [%s]", my_callsign, my_grid);
  tft->drawString(hi, b.xStart + XFOOT, b.yStart + YFOOT, 1); // Print the line

  tft->drawFastVLine (b.xStart + XFOOT + 9 + 12 + 13 * 6, b.yStart, 18, TFT_RED);
  tft->drawFastVLine (b.xStart + XFOOT + 14 + 12 + 5 +  6 * 8 + 13 * 6, b.yStart, 18, TFT_RED);
  if (strcmp(ip, "") != 0) {
    sprintf(hi, "%s", ip);
    tft->drawString(hi, b.xStart + XFOOT + 15 + 12 + 10 + 6 * 8 + 13 * 6, b.yStart + YFOOT, 1); // Print the line
  }
  showtime();
}
void footerPDX::showtime() {

  if (clock == false) {
    return;
  }

  tft->setTextFont(2);
  tft->setTextSize(1);
  tft->setTextColor(TFT_RED, TFT_WHITE);
  uint16_t fh = tft->fontHeight();

  sprintf(hi, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  tft->drawString(hi, b.xStart + XCLOCK, b.yStart + YCLOCK, 1); // Print the line

}

void footerPDX::update() {

  time_t now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeinfo);

  if ((timeinfo.tm_hour != timeprev.tm_hour) ||
      (timeinfo.tm_min  != timeprev.tm_min)  ||
      (timeinfo.tm_sec  != timeprev.tm_sec)) {
    showtime();
    timeprev = timeinfo;
  }
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
//*                          Objects living on the GUI display                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
iconPDX wifiIcon = iconPDX(&tft, 2, 2, wifiSignWidth, wifiSignHeight, &wifiSign[0], NULL);
iconPDX termIcon = iconPDX(&tft, 36, 2, terminalWidth, terminalHeight, &terminal[0], NULL);
iconPDX calIcon  = iconPDX(&tft, 70, 2, infoWidth, infoHeight, &infoX[0], NULL);
iconPDX catIcon  = iconPDX(&tft, 104, 2, termWidth, termHeight, &termSign[0], NULL);
iconPDX cntIcon  = iconPDX(&tft, 138, 2, readoutWidth, readoutHeight, &readoutSign[0], NULL);
iconPDX wsjtIcon = iconPDX(&tft, 172, 2, wsjtXWidth, wsjtXHeight, &wsjtX[0], NULL);
iconPDX quadIcon = iconPDX(&tft, 206, 2, alertWidth, alertHeight, &alert[0], NULL);
iconPDX muteIcon = iconPDX(&tft, 240, 2, micWidth, micHeight, &mic[0], NULL);
iconPDX spkrIcon = iconPDX(&tft, 274, 2, speakerWidth, speakerHeight, &speaker[0], NULL);
linearMeter m = linearMeter(&tft, 350, 2, 5, 25, 3, 15, BLUE2BLUE, TFT_BLACK);
displayPDX d = displayPDX(&tft, 2, 50, 272, 99, 10, TFT_CYAN, TFT_BLUE, TFT_BLACK);
progressBar p = progressBar(&tft, 2, 44, 272, 4, 10, TFT_CYAN, TFT_BLUE, TFT_BLACK);
textPDX text = textPDX(&tft, 2 + 272 + 2, 50, 204, 99, 10, TFT_WHITE, TFT_BLACK, TFT_WHITE);
spectrumPDX s = spectrumPDX(&tft, 2, 50 + 100 + 2, 480, 150, 10, TFT_BLUE, TFT_CYAN, TFT_WHITE); //previously 160-155
footerPDX foot = footerPDX(&tft, 2, 50 + 100 + 2 + 150, 480, 18, 10, TFT_WHITE, TFT_RED, TFT_WHITE); //previously 160-155
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=[End of definitions]*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
//*                          Operation of GUI and support structures                                         *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
void GUI_init() {


  tft.init();
  tft.setRotation(ROTATION_SETUP);   //ROTATION_SETUP
  tft.fillScreen(TFT_BLACK);     //TFT_CYAN
  tft.fillScreen(TFT_BLACK);

  // Swap the colour byte order when rendering
  tft.setSwapBytes(true);

  wifiIcon.show(true,true);
  termIcon.show(true,true);
  calIcon.show(true,true);
  catIcon.show(true,true);
  cntIcon.show(true, true);
  quadIcon.show(true, true);
  wsjtIcon.show(true, true);
  muteIcon.show(true,true);
  spkrIcon.show(true,true);

  m.show(SUNIT, 0);

  d.addBtn(BUTTON_TX, &btnTX);
  d.addBtn(BUTTON_CQ, &btnCQ);
  d.addBtn(BUTTON_AUTO, &btnAUTO);
  d.addBtn(BUTTON_BAND, &btnBAND);

  d.set(BUTTON_AUTO,0);
  d.set(BUTTON_TX,0);
  d.set(BUTTON_BAND,Band_slot-1);
  d.set(BUTTON_CQ,0);
  
  d.init();
  d.show(true);

  p.reset();

  text.show();

  s.init();


  foot.init();
  delay(1000);
  foot.show();

  GUI_Enabled=true;

}
/*--------------------
 * Handler to GUI buttons
 */
void tft_set(int btnIndex,int v) {
  d.set(btnIndex,v);
}

/*********************************************************
  setup
  initializes the dialog
*********************************************************/
void tft_setup() {


  // Use this calibration code in setup():
  tft.setTouch(calData);

  // Initialize the GUI
  GUI_init();

} // main

uint32_t t1 = time_us_32();
uint32_t t2 = 0;
uint32_t t3 = 0;
uint16_t cycle = 0;
int hh = 16;
int mm = 00;
int ss = 00;
/*-------------------------------------------------------------------
   Verify if the touchscreen has been touched and the coordinates
*/
void tft_checktouch() {

  foot.update();

  uint16_t x = 0, y = 0; // To store the touch coordinates

  // Pressed will be set true is there is a valid touch on the screen

  bool pressed = tft.getTouch(&x, &y);
  if (pressed == false) {
    prevpressed = false;
    return ;
  }
  if (prevpressed == true) {
    if ((x >= xant - 3) &&
        (x <  xant + 3) &&
        (y <= yant + 3) &&
        (y >  yant - 3)) {   //This condition is assumed as a bad pulse and not triggering
      pressed = false;
      return;
    }
  } else {
    prevpressed = true;
    xant = x;
    yant = y;
  }


  // Draw a white spot at the detected coordinates
  if (pressed) {
    bool hit = d.point(x, y);
    tft.fillCircle(x, y, 2, TFT_WHITE);
    d.checkButton(x, y);
    d.checkTriangle(x,y);
    if (d.redraw) {
       d.redraw=false;
       s.reset();
       s.init();
       
       updateEEPROM();
    }
    int i = text.checkPoint(x, y);
    if (i == -1) return;
    d.set(BUTTON_CQ,2);
    
    call_af_frequency=text.t[i].af_frequency;
    call_self_rx_snr=text.t[i].self_rx_snr;
    strcpy(call_station_callsign,text.t[i].station_callsign);
    strcpy(call_grid_square,text.t[i].grid_square);
  }

  foot.update();

}
/*-----------------------------
   update waterfall
*/
void tft_updatewaterfall(int mag[]) {
  if (time_us_32() - t1 >= 1000000) {
    t1 = time_us_32();
    s.draw(mag);
    int magmax = 0;

    for (int i = 0; i < 960; i++) {
      if (mag[i] == 0) {
        continue;
      }
      if (mag[i] > magmax) {
        magmax = mag[i];
      }
    }
    int sunit = (int)((0.125 * magmax) - 18.25);

    m.show(SUNIT, sunit);
    memset(mag, 0, 960);

    p.progress++;
    p.show(TFT_GREEN);

    //m.show(SUNIT,v);
  }
}
void tft_resetBar() {
  p.reset();
}
void tft_setBar(int colour) {
  p.show(colour);
}
void tft_setBarTx() {
  p.progress++;
  p.show(TFT_RED);
}
void tft_endoftime() {
  s.linedraw();
  foot.update();

}
/*---------------------------------
 * Store a message with supplemental
 * data.
 */
void tft_storeQSO(uint16_t _qso,char *s,uint16_t af_frequency,int8_t self_rx_snr,char *station_callsign,char *grid_square) {
   text.printline(_qso,s,af_frequency,self_rx_snr,station_callsign,grid_square);
}

/*---------------------
   This is the operational handling of the TFT
*/
void tft_run() {

  tft_checktouch();
  foot.update();
  d.check();

}
void tft_updateBand() {
     
   if (!GUI_Enabled) return; 
   
   d.btnPDX[BUTTON_BAND].v=Band_slot;
   d.set(BUTTON_BAND,Band_slot-1);
   if (d.redraw) {
       d.redraw=false;
       s.reset();
       s.init();
   }    
}
