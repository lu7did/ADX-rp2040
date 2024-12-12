//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//                                              ddsPIO.h                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
// Pedro (Pedro Colla) - LU7DZ - 2024
// Version 1.0
// This is a rp2040 implementation of a digital clock in the range of 1 to 30 MHz
//*---------------------------------------------------------------------------------------------------------
//
// This implementation is largely based on the pico-hf-oscillator project by
//
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https://www.qrz.com/db/r2bdy
//*---------------------------------------------------------------------------------------------------------
#define PROGNAME "ADX_rp2040"
#define AUTHOR "Pedro E. Colla (LU7DZ)"
#define VERSION  "1.1"
#define BUILD     "00"

#define DEBUG       1
//*---------------------------------------------------------------------------------------------------------
//* Debug macros and tools
//*---------------------------------------------------------------------------------------------------------
#define _SERIAL Serial
#ifdef DEBUG
#define _INFOLIST(...) \
  do { \
    strcpy(hi,"@"); \
    sprintf(hi+1,__VA_ARGS__); \
    _SERIAL.write(hi); \
    _SERIAL.flush(); \
  } while (false)
#else //!DEBUG
#define _INFOLIST(...) (void)0
#endif //_INFOLIST macro definition as NOP when not in debug mode, will consume one byte of nothingness


//*---------------------------------------------------------------------------------------------------------
enum PioDcoMode
{
    eDCOMODE_IDLE = 0,          /* No output. */
    eDCOMODE_GPS_COMPENSATED= 2 /* Internally compensated, if GPS available. */
};

typedef struct
{
    enum PioDcoMode _mode;      /* Running mode. */
    PIO _pio;                   /* Worker PIO on this DCO. */
    int _gpio;                  /* Pico' GPIO for DCO output. */
    pio_sm_config _pio_sm;      /* Worker PIO parameter. */
    int _ism;                   /* Index of state maschine. */
    int _offset;                /* Worker PIO u-program offset. */
    int32_t _frq_cycles_per_pi; /* CPU CLK cycles per PI. */
    uint32_t _ui32_pioreg[8];   /* Shift register to PIO. */
    uint32_t _clkfreq_hz;       /* CPU CLK freq, Hz. */
    GPStimeContext *_pGPStime;  /* Ptr to GPS time context. No GPS correction implemented */
    uint32_t _ui32_frq_hz;      /* Working freq, Hz. */
    int32_t _ui32_frq_millihz;  /* Working freq additive shift, mHz. */
    int _is_enabled;

} PioDco;

//*---------------------------------------------------------------------------------------------------------
//* Interfaces to PIO DDS functions
//*---------------------------------------------------------------------------------------------------------
int PioDCOInit(PioDco *pdco, int gpio, int cpuclkhz);
int PioDCOSetFreq(PioDco *pdco, uint32_t u32_frq_hz, int32_t u32_frq_millihz);
int32_t PioDCOGetFreqShiftMilliHertz(const PioDco *pdco, uint64_t u64_desired_frq_millihz);
void PioDCOStart(PioDco *pdco);
void PioDCOStop(PioDco *pdco);
void PioDCOSetMode(PioDco *pdco, enum PioDcoMode emode);
void RAM (PioDCOWorker2)(PioDco *pDCO);
