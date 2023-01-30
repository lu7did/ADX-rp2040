#include <Arduino.h>
#include "RDX-rp2040.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * cli.cpp                                                                                     *
 * Simple command line based configuration tool                                                *
 *---------------------------------------------------------------------------------------------*
 *  Copyright  RDX project by Dr. Pedro E. Colla (LU7DZ) 2022. All rights reserved             *
 * This implementation provides a simplified command processor that can ge used to either      *
 * browse or modify configuration data stored in EEPROM, so the program configuration can be   *
 * changed without rebuild from sources                                                        *
 *---------------------------------------------------------------------------------------------*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
 
#include <stddef.h>

char outBuffer[1024];
char inBuffer[128];
int  ptrBuffer=0;

/*---------------------------------------------
 * supporting structures
 */
typedef bool (*CMD)(int idx,char *_cmd,char *_arg,char *_out);
struct cmdSet {
  char token[12];
  char help[32];
  char typearg;
  bool toUpper;
  bool toLower;
  int  min;
  int  max;
  CMD  handlerCmd;
};
/*------------------------------------------------
 * Command token definition
 */
#define MAXTOKEN     25
cmdSet langSet[MAXTOKEN] =
                      {{"help","help for all commands",0x00,false,false,0,0,NULL},
                      {"list","list EEPROM content",0x00,false,false,0,0,NULL},
                      {"load","load EEPROM content",0x00,false,false,0,0,NULL},
                      {"save","save EEPROM content",0x00,false,false,0,0,NULL},
                      {"reset","reset EEPROM content to default",0x00,false,false,0,0,NULL},
                      
                      {"?","list all commands",0x00,false,false,0,0,NULL},
                      
                      {"call","station callsign",'a',true,false,0,0,NULL},
                      {"grid","station grid locator",'a',true,false,0,0,NULL},
                      {"adif","logbook name",'a',false,true,0,0,NULL},
                      {"ssid","WiFi AP SSID",'a',false,false,0,0,NULL},
                      {"psk","WiFi AP password",'a',false,false,0,0,NULL},
                      {"log","USB exported logbook",'a',false,false,0,0,NULL},
                      {"msg","FT8 ADIF message",'a',true,false,0,0,NULL},
                      {"host","host name",'a',false,false,0,0,NULL},
                      
                      {"writelog","enable ADIF log write",'b',false,false,0,0,NULL},
                      {"autosend","enable FT8 QSO auto",'b',false,false,0,0,NULL},
                      {"tx","turn TX on/off",'b',false,false,0,0,NULL},

                      {"tcpport","Telnet Port",'i',false,false,23,10000,NULL},
                      {"http","FS HTTP Port",'i',false,false,80,10000,NULL},
                      {"ft8try","FT8 Max tries",'n',false,false,1,12,NULL},
                      {"ft8tx","FT8 Max tx",'n',false,false,1,12,NULL},
                      {"tz","TZ offset from UTC",'i',false,false,-12,+12,NULL},
                      
                      {"quit","quit terminal mode",0x00,false,false,0,0,NULL},
                      {"","",0x00,false,false,0,0,NULL}};

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*                                     Generic handlers                                        */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
bool handleStr(int idx, char *_var, char *_arg,char *_out) {
  
     if (strcmp(_arg,"") == 0) {
        sprintf(_out+strlen(_out),"%s\n",_var);
        return false;
     }
     if (langSet[idx].toUpper == true) toupperStr(_arg);
     if (langSet[idx].toLower == true) tolowerStr(_arg);
     strcpy(_var,_arg);
     sprintf(_out+strlen(_out),"%s\n",_var);
     return false;
}

bool handleBool(int idx, bool* _bool, char *_arg,char *_out) {

     if (strcmp(_arg,"") == 0) {
        sprintf(_out+strlen(_out),"%d\n",*_bool);
        return false;
     }
     if (!isNumeric(_arg)){
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
        return false;
     }

     char n[16]="";
     parse(_arg,n);   
     int v=atoi(n);

     if (v==0) {
        *_bool=false;
     } else {
        *_bool=true;
     }
     sprintf(_out+strlen(_out),"%d\n",*_bool);
     return false;
}

bool handleNum(int idx, int* _num, char *_arg,char *_out) {

     if (strcmp(_arg,"") == 0) {
        sprintf(_out+strlen(_out),"%d\n",*_num);
        return false;
     }

     if (!isNumeric(_arg)){
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
        return false;
     }

     char n[16]="";
     parse(_arg,n);   
     int v=atoi(n);

     if (v==0) {
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
        return false;
     }  
     
     if (v>=langSet[idx].min && v<=langSet[idx].max) {
        *_num=v;
        sprintf(_out+strlen(_out),"%d\n",*_num);
     } else {
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
     }
     return false;
}

bool handleByte(int idx, uint8_t* _num, char *_arg,char *_out) {

     if (strcmp(_arg,"") == 0) {
        sprintf(_out+strlen(_out),"%d\n",*_num);
        return false;
     }

     if (!isNumeric(_arg)){
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
        return false;
     }

     char n[16]="";
     parse(_arg,n);   
     int v=atoi(n);

     if (v==0) {
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
        return false;
     }  
     
     if (v>=langSet[idx].min && v<=langSet[idx].max) {
        *_num=v;
        sprintf(_out+strlen(_out),"%d\n",*_num);
     } else {
        sprintf(_out+strlen(_out),"invalid parameter (%s)\n",_arg);   
     }
     return false;

}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*                                Parameter specific handlers                                  */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
bool tcpportCmd(int idx, char *_cmd,char *_arg,char *_out) {
#ifdef RP2040_W  
     bool rc=handleNum(idx,&tcp_port,_arg,_out);
     return rc;
#else
     sprintf(_out+strlen(_out),"Not supported on non-wireless Raspberry Pico version\n");     
     return false;
#endif     
}
bool httpportCmd(int idx, char *_cmd,char *_arg,char *_out) {
#ifdef RP2040_W  
     bool rc=handleNum(idx,&http_port,_arg,_out);
     return rc;
#else
     sprintf(_out+strlen(_out),"Not supported on non-wireless Raspberry Pico version\n");     
     return false;
#endif     
     
}
bool ft8tryCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool rc=handleByte(idx,&maxTry,_arg,_out);
     return rc;
}
bool ft8txCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool rc=handleByte(idx,&maxTx,_arg,_out);
     return rc;
}
bool tzCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool rc=handleNum(idx,&timezone,_arg,_out);
     _INFOLIST("%s timezone is %d\n",__func__,timezone);
     return rc;
}

bool writelogCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool rc=handleBool(idx,&logADIF,_arg,_out);
     return rc;
}
bool autosendCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool rc=handleBool(idx,&autosend,_arg,_out);
     return rc;
}
bool txCmd(int idx, char *_cmd,char *_arg,char *_out) {
     bool tx=false;
     bool rc=handleBool(idx,&tx,_arg,_out);
     if (tx) {
        startTX();
        sprintf(_out+strlen(_out),"TX+\n");
     } else {
        stopTX();
        sprintf(_out+strlen(_out),"TX-\n");
     }
     return rc;
}

bool callCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
     bool rc=handleStr(idx,my_callsign,_arg,_out);
     return rc;
}
bool gridCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
     return handleStr(idx,my_grid,_arg,_out);
}
bool adifCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
     return handleStr(idx,adiffile,_arg,_out);
}
bool ssidCmd(int idx, char *_cmd,char *_arg,char *_out) {
#ifdef RP2040_W  
     char v[32];
     return handleStr(idx,wifi_ssid,_arg,_out);
#else
     sprintf(_out+strlen(_out),"Not supported on non-wireless Raspberry Pico version\n");     
     return false;
#endif     
     
}
bool pskCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
#ifdef RP2040_W
     return handleStr(idx,wifi_psk,_arg,_out);
#else
     sprintf(_out+strlen(_out),"Not supported on non-wireless Raspberry Pico version\n");     
     return false;
#endif         
}
bool logCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
     return handleStr(idx,logbook,_arg,_out);
}
bool msgCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
     return handleStr(idx,qso_message,_arg,_out);
}
bool hostCmd(int idx, char *_cmd,char *_arg,char *_out) {
     char v[32];
#ifdef RP2040_W     
     return handleStr(idx,hostname,_arg,_out);
#else
     sprintf(_out+strlen(_out),"Not supported on non-wireless Raspberry Pico version\n");     
     return false;
#endif     
     
}

bool listCmd(int idx, char *_cmd,char *_arg,char *_out) {
    strcpy(_out,"\n");
    int i = EEPROM_ADDR_CAL;
    while (!getEEPROM(&i,_out));
    sprintf(_out+strlen(_out),"\n>");
    return false;
}
bool quitCmd(int idx,char *_cmd,char *_arg,char *_out) {
  return true;
}
bool loadCmd(int idx, char *_cmd,char *_arg,char *_out) {

   readEEPROM();
   sprintf(_out+strlen(_out),"EEPROM values restored\n>");
   return false;
}
bool saveCmd(int idx, char *_cmd,char *_arg,char *_out) {

   updateEEPROM();
   return false;
}

bool resetCmd(int idx, char *_cmd,char *_arg,char *_out) {

   resetEEPROM();
   sprintf(_out+strlen(_out),"EEPROM values reset to default values\n>");
   return false;
}

bool helpCmd(int idx, char *_cmd,char *_arg,char *_out) {
    strcpy(_out,"\n");
    for (int i=0;i<MAXTOKEN;i++) {
      if (strcmp(langSet[i].token,"") == 0) {
         return false;
      }
      sprintf(_out+strlen(_out),"(%s) - %s\n",langSet[i].token,langSet[i].help);
    }
    return false;
}
bool shortCmd(int idx, char *_cmd,char *_arg,char *_out) {
    strcpy(_out,"\n");
    for (int i=0;i<MAXTOKEN;i++) {
      if (strcmp(langSet[i].token,"") == 0) {
         sprintf(_out+strlen(_out),"\n");
         return false;
      }
      sprintf(_out+strlen(_out),"%s ",langSet[i].token);
    }
    sprintf(_out+strlen(_out),"\n");
    return false;
}
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*                                Command processor                                            */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*----------------------------------
 * Main command dispatcher
 * the parsed command is contained in the area pointed by c
 * whilst the rest of the arguments in the area pointed by a
 * depending on the command process the argument in different
 * ways
 */
bool cli_execute(char *buffer, char *outbuffer) {

char cmd[128]="";
char argv[128]="";
bool exitcode=false;

     parse(buffer,cmd);
     strcpy(argv,buffer);
     tolowerStr(cmd);
     
     for (int i=0;i<MAXTOKEN;i++) {
         if (strcmp(cmd,langSet[i].token)==0) {
            exitcode=langSet[i].handlerCmd(i,cmd,argv,outbuffer);         
            break;
         }
     }
     cli_prompt(outbuffer);
     return exitcode;
}
/*--------------------------------------------
 * Generate the prompt with current time
 */
void cli_prompt(char *_out) {
  
  time_t now = time(nullptr) - t_ofs;
  gmtime_r(&now, &timeinfo);

  int tzhour=timeinfo.tm_hour;
  if (timezone != 0) {
     tzhour=tzhour+timezone;
     if (tzhour>23) tzhour=tzhour-24;
     if (tzhour<0)  tzhour=tzhour+24;
  }
  sprintf(_out+strlen(_out), "[%02d:%02d:%02d] >", tzhour, timeinfo.tm_min, timeinfo.tm_sec);
}
/*----------------------------------------------
 * Define pointers to the specific variable
 * handlers (need to improve) this is far from
 * elegant
 */
void cli_setHandlers() {

  langSet[0].handlerCmd=helpCmd;
  langSet[1].handlerCmd=listCmd;
  langSet[2].handlerCmd=loadCmd;
  langSet[3].handlerCmd=saveCmd;
  langSet[4].handlerCmd=resetCmd;
  langSet[5].handlerCmd=shortCmd;
  langSet[6].handlerCmd=callCmd;
  langSet[7].handlerCmd=gridCmd;
  langSet[8].handlerCmd=adifCmd;
  langSet[9].handlerCmd=ssidCmd;
  langSet[10].handlerCmd=pskCmd;
  langSet[11].handlerCmd=logCmd;
  langSet[12].handlerCmd=msgCmd;
  langSet[13].handlerCmd=hostCmd; 

  langSet[14].handlerCmd=writelogCmd; 
  langSet[15].handlerCmd=autosendCmd; 
  langSet[16].handlerCmd=txCmd; 

  langSet[17].handlerCmd=tcpportCmd;
  langSet[18].handlerCmd=httpportCmd;
  langSet[19].handlerCmd=ft8tryCmd;
  langSet[20].handlerCmd=ft8txCmd;
  langSet[21].handlerCmd=tzCmd; 
  
  langSet[22].handlerCmd=quitCmd;
  
}
/*--------------------------
 * sub-system initialization
 */
void cli_init(char *_out) {
  
  cli_setHandlers();
  sprintf(_out+strlen(_out), "\n\rRDX %s build(%s) command interpreter\n\r", VERSION, BUILD);
  strcpy(inBuffer,"");
  ptrBuffer=0;
  cli_prompt(_out);

  
}
/*----------------------------------
 * Serial_processor
 * Collect the command from the serial input
 * on LF parse the command line and tokenize into a command
 * followed by arguments (one to many)
 */

bool Serial_processor() {
  
  tft_checktouch();
  bool eoc=false;
  while (_SERIAL.available()) {
     char c = _SERIAL.read();
     if ((byte)c=='\r') {
     }
     if ((byte)c=='\n') {
        _SERIAL.println(inBuffer);
        eoc=cli_execute(inBuffer,outBuffer);
        ptrBuffer=0;
        strcpy(inBuffer,"");
        _SERIAL.print(outBuffer);
        strcpy(outBuffer,"");
        if (eoc) break;
     }
     if ((byte)c!='\n' && (byte)c!='\r') {
        inBuffer[ptrBuffer++]=c;               
        inBuffer[ptrBuffer]=0x00;
     }   
  }
  return eoc;
 
}
/*-------------------------------------------
 * cli_command
 * loop to collect input from serial terminal
 * and execute
 */
void cli_command() {

  cli_init(outBuffer);
  _SERIAL.print(outBuffer);
  strcpy(outBuffer,"");
  
  while (!Serial_processor());
    
}
