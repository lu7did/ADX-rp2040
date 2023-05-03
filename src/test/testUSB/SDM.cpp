#include "SDM.h"

SDM::SDM() {

}

 // oversample X 32(Direct form 2)
uint32_t SDM::o4_os32_df2(int16_t sig) {

    uint32_t out = 0;
    int32_t d = vmin_04 - sig; // vmin_o4  = -32767*3, therefore output is 3.3v/3 = 1.1v pk-pk
    int32_t test = vmin_04*1024;
    
    // depends on software whether LSB or MSB should be determined first
    #if defined ARDUINO_ARCH_MBED_RP2040 || defined ARDUINO_ARCH_RP2040 
    for (int j = 0; j < 32; j++) {
    #else
    for (int j = 31; j > -1; j--) {
    #endif
    
      int32_t wn =d+4*(w[0]+w[2])-6*w[1]-w[3];
      // direct form 2 feedback
      int32_t etmp = -3035 * w[0] + 3477 * w[1] - 1809 * w[2] + 359 * w[3]+1024*wn;
      // update previous values
      w[3] = w[2];
      w[2] = w[1];
      w[1] = w[0];
      w[0]= wn;
      
      // checks if current error minises sum of squares error
      if((etmp)<test){
       w[0]+= pos_error_04;
       out += (1 << (j));
      }
    }
   
    return out;
  }

void SDM::attenuate(uint16_t scale){
 vmin_04 = -32767*scale;
 pos_error_04 = 65534*scale; 
}
       
 
uint32_t SDM::o1_os32(int16_t sig) {

    uint32_t out = 0;
    int32_t d = -32767 - sig;   
    int32_t etmp;
    for (int j = 0; j < 32; j++) {
     etmp = d  +buff[0];
     buff[0] = etmp;
      

      
      // checks if current error minises sum of squares error
      // if not it changes the deltas and errors.
      if (etmp < 0) {
       buff[0] += 65534;
       out += (1 << j);
      }
      
    }
   
    return out;
  }

 uint32_t SDM::o2_os32(int16_t sig) {

    uint32_t out = 0;
    int32_t d = -32767 - sig;   
    int32_t etmp;
    for (int j = 0; j < 32; j++) {
     etmp = d  +2*buff[0]-buff[1];
	 buff[1] = buff[0];
     buff[0] = etmp;
      

      
      // checks if current error minises sum of squares error
      // if not it changes the deltas and errors.
      if (etmp < 0) {
       buff[0] += 65534;
       out += (1 << j);
      }
      
    }
   
    return out;
  }

    // oversample X 32(Direct form 1)
  uint32_t SDM::o4_os32(int16_t sig) {

    uint32_t out = 0;
    int32_t d = -98304 - sig;   
    int32_t d128 = d*128;
    int32_t etmp;
    for (int j = 0; j < 32; j++) {
      // the feedback is composed of deltas(digital output - digital input) and errors
      // this could probably be more efficient and is the bottleneck
    
     etmp = d128 -409 * buff[4] + 498 * buff[5]- 273 * buff[6]  + 57 * buff[7]+(buff[0] + buff[2]) * 4 - 6 * buff[1] - buff[3];
     
     
      buff[7] = buff[6];
      buff[6] = buff[5];
      buff[5] = buff[4];
      buff[4] = d;
      buff[3] = buff[2];
      buff[2] = buff[1];
      buff[1] = buff[0];
      buff[0] = etmp;
      

      
      // checks if current error minises sum of squares error
      // if not it changes the deltas and errors.
      if (etmp < 0) {
       buff[4] += 196608;
       buff[0] += 25165824;
       out += (1 << j);
      }
      
    }
   
    return out;
  }
