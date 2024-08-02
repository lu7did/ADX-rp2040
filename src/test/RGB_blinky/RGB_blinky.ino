/*
    COMO UTILIZAR EL LED RGB INTEGRADO DE LA PLACA DEL RP2040 ZERO
    CODIGO ORIGINAL DE Alan Yorinks
    MODIFICADO POR : JOSE MARIA MENDEZ RUIZ
    DICIEMBRE 2022
    CODIGO LIBRE PARA UTILIZAR COMO QUIERAS

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU GENERAL PUBLIC LICENSE
    Version 3 as published by the Free Software Foundation; either
    or (at your option) any later version.
    This library is distributed in the hope that it will be useful,f
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.
 
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Requires https://www.arduinolibraries.info/libraries/neo-pixel-connect
*/

#include <NeoPixelConnect.h>

NeoPixelConnect led(16, 1, pio0, 0);  // creamos la instacia para utilizar el led, con el nombre de led

void setup(){

}
void paintRGB(uint8_t r, uint8_t g,uint8_t b,uint8_t RGBmode) {

    led.neoPixelFill(r & RGBmode, g & RGBmode, b & RGBmode, true); // le pasamos el valor de RGB del color que queremos
    delay(1000);
    led.neoPixelClear(true); // limpiamos el led
    delay(1000);
}


void loop(){

paintRGB(255,0,0,0xff);    //RED
paintRGB(0,255,0,0xff);    //GREEN
paintRGB(0,0,255,0xff);    //BLUE


paintRGB(255,255,255,0xff); //WHITE
paintRGB(255,255,0,0xff);   //YELLOW
paintRGB(255,0,255,0xff);  //MAGENTA
paintRGB(0,255,255,0xff);   //CYAN


/*----- Low Energy. ----*/

paintRGB(255,0,0,0x7f);    //RED
paintRGB(0,255,0,0x7f);    //GREEN
paintRGB(0,0,255,0x7f);    //BLUE


paintRGB(255,255,255,0x7f); //WHITE
paintRGB(255,255,0,0x7f);   //YELLOW
paintRGB(255,0,255,0x7f);  //MAGENTA
paintRGB(0,255,255,0x7f);   //CYAN

/*----- Low Energy. ----*/

paintRGB(255,0,0,0x3f);    //RED
paintRGB(0,255,0,0x3f);    //GREEN
paintRGB(0,0,255,0x3f);    //BLUE


paintRGB(255,255,255,0x3f); //WHITE
paintRGB(255,255,0,0x3f);   //YELLOW
paintRGB(255,0,255,0x3f);  //MAGENTA
paintRGB(0,255,255,0x3f);   //CYAN


//    /********************   ROJO *************************************/        
/*
    led.neoPixelFill(255, 0, 0, true); // le pasamos el valor de RGB del color que queremos
    delay(1000); 
    led.neoPixelClear(true); // limpiamos el led
    delay(1000);
*/
    /********************   VERDE  *************************************/        
/*
    led.neoPixelFill(0, 255, 0, true);// le pasamos el valor de RGB del color que queremos
    delay(1000);
    led.neoPixelClear(true);// limpiamos el led
    delay(1000);
*/
    /********************   AZUL*************************************/        
/*
    led.neoPixelFill(0, 0, 255, true); // le pasamos el valor de RGB del color que queremos
    delay(1000);
    led.neoPixelClear(true); // limpiamos el led
    delay(1000);
*/    
}