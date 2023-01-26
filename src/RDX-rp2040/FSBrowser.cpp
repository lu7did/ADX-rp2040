#include <Arduino.h>
#include "RDX-rp2040.h"
#ifdef RP2040_W          //This sub-system makes no sense without TCP/IP connectivity
#ifdef FSBROWSER
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * FSBrowser.cpp                                                                               *
 * A web-based FileSystem Browser for Pico filesystems                                         *
 *---------------------------------------------------------------------------------------------*
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.                     
  This file is part of the Pico WebServer library for Arduino environment.

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
 *---------------------------------------------------------------------------------------------*
  Adaptation and integration with RDX project by Dr. Pedro E. Colla (LU7DZ) 2022               *
 *---------------------------------------------------------------------------------------------*
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * Select the FileSystem to use by uncommenting one of the lines below
 * This particular setup is for the LittleFS filesystem (Flash memory)
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#define USE_SPIFFS
#define USE_LITTLEFS
//#define USE_SDFS

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
// Uncomment the following line to embed a version of the web page in the code
// (program code will be larger, but no file will have to be written to the 
// filesystem).
// Note: the source file "extras/index_htm.h" must have been generated 
// by running "extras/reduce_index.sh"
 *=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define INCLUDE_FALLBACK_INDEX_HTM
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <LEAmDNS.h>
#include <SPI.h>
#include <EEPROM.h>

#ifdef INCLUDE_FALLBACK_INDEX_HTM
#include "extras/index_htm.h"
#endif

#if defined USE_SPIFFS
#include <FS.h>
const char* fsName = "SPIFFS";
FS* fileSystem = &SPIFFS;
SPIFFSConfig fileSystemConfig = SPIFFSConfig();

#elif defined USE_LITTLEFS

#include <LittleFS.h>
const char* fsName = "LittleFS";
FS* fileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();

#elif defined USE_SDFS

#include <SDFS.h>
const char* fsName = "SDFS";
FS* fileSystem = &SDFS;
SDFSConfig fileSystemConfig = SDFSConfig();

#else
#error Please select a filesystem first by uncommenting one of the "#define USE_xxx" lines at the beginning of the sketch.
#endif


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*                       Define Web Server parameters                            */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
const char* host = HOSTNAME;
WebServer server(80);

static bool fsOK;
String unsupportedFiles = String();

File uploadFile;

static const char TEXT_PLAIN[] PROGMEM = "text/plain";
static const char FS_INIT_ERROR[] PROGMEM = "FS INIT ERROR";
static const char FILE_NOT_FOUND[] PROGMEM = "FileNotFound";

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
// Utils to return HTTP codes, and determine content-type
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void replyOK() {
  server.send(200, FPSTR(TEXT_PLAIN), "");
}

void replyOKWithMsg(String msg) {
  server.send(200, FPSTR(TEXT_PLAIN), msg);
}

void replyNotFound(String msg) {
  server.send(404, FPSTR(TEXT_PLAIN), msg);
}

void replyBadRequest(String msg) {
  _INFOLIST("%s %s\n",__func__,msg);
  server.send(400, FPSTR(TEXT_PLAIN), msg + "\r\n");
}

void replyServerError(String msg) {
  _INFOLIST("%s %s\n",__func__,msg);
  server.send(500, FPSTR(TEXT_PLAIN), msg + "\r\n");
}

#ifdef USE_SPIFFS
/*----------------------------------------------------------
   Checks filename for character combinations that are not supported by FSBrowser
  (alhtough valid on SPIFFS).
   Returns an empty String if supported, or detail of error(s) if unsupported
*/
String checkForUnsupportedPath(String filename) {
  String error = String();
  if (!filename.startsWith("/")) {
    error += F("!NO_LEADING_SLASH! ");
  }
  if (filename.indexOf("//") != -1) {
    error += F("!DOUBLE_SLASH! ");
  }
  if (filename.endsWith("/")) {
    error += F("!TRAILING_SLASH! ");
  }
  return error;
}
#endif

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 *                                  Request handlers                             + 
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*
   Return the FS type, status and size info
*/
void handleStatus() {
  _INFOLIST("%s\n",__func__);
  FSInfo fs_info;
  String json;
  json.reserve(128);

  json = "{\"type\":\"";
  json += fsName;
  json += "\", \"isOk\":";
  if (fsOK) {
    fileSystem->info(fs_info);
    json += F("\"true\", \"totalBytes\":\"");
    json += fs_info.totalBytes;
    json += F("\", \"usedBytes\":\"");
    json += fs_info.usedBytes;
    json += "\"";
  } else {
    json += "\"false\"";
  }
  json += F(",\"unsupportedFiles\":\"");
  json += unsupportedFiles;
  json += "\"}";

  server.send(200, "application/json", json);
}

/*------------------------------------------------------
   Return the list of files in the directory specified by the "dir" query string parameter.
   Also demonstrates the use of chunked responses.
*/
void handleFileList() {
  _INFOLIST("%s\n",__func__);
  if (!fsOK) {
    return replyServerError(FPSTR(FS_INIT_ERROR));
  }

  if (!server.hasArg("dir")) {
    return replyBadRequest(F("DIR ARG MISSING"));
  }

  String path = server.arg("dir");
  if (path != "/" && !fileSystem->exists(path)) {
    return replyBadRequest("BAD PATH");
  }

  _INFOLIST("%s path:%s\n",__func__,path.c_str());
  Dir dir = fileSystem->openDir(path);
  path = "";

  // use HTTP/1.1 Chunked response to avoid building a huge temporary string
  if (!server.chunkedResponseModeStart(200, "text/json")) {
    server.send(505, F("text/html"), F("HTTP1.1 required"));
    Serial.println("Something wrong occurs");
    return;
  }

  // use the same string for every line
  String output;
  output.reserve(64);
  while (dir.next()) {

#ifdef USE_SPIFFS
    String error = checkForUnsupportedPath(dir.fileName());
    if (error.length() > 0) {
      _INFOLIST("%s Ignoring: %s\n",__func__,(error+dir.fileName()).c_str());
      continue;
    }
#endif

    if (output.length()) {
      // send string from previous iteration
      // as an HTTP chunk
      _INFOLIST("%s output=%s\n",__func__,output.c_str());
      server.sendContent(output);
      output = ',';
    } else {
      output = '[';
    }

    output += "{\"type\":\"";
    if (dir.isDirectory()) {
      output += "dir";
    } else {
      output += F("file\",\"size\":\"");
      output += dir.fileSize();
    }

    output += F("\",\"name\":\"");
    // Always return names without leading "/"
    if (dir.fileName()[0] == '/') {
      output += &(dir.fileName()[1]);
    } else {
      output += dir.fileName();
    }

    output += "\"}";
  }

  // send last string

  output += "]";
  server.sendContent(output);
  server.chunkedResponseFinalize();
}

/*------------------------------------------------------
   Read the given file from the filesystem and stream it back to the client
*/
bool handleFileRead(String path) {
  
  _INFOLIST("%s path:%s\n",__func__,path.c_str());
  if (!fsOK) {
    replyServerError(FPSTR(FS_INIT_ERROR));
    _INFOLIST("%s !fsOK\n",__func__);
    return true;
  }

  if (path.endsWith("/")) {
    path += "index.htm";
    _INFOLIST("%s %s\n",__func__,path.c_str());
    
  }

  String contentType;
  if (server.hasArg("download")) {
    contentType = F("application/octet-stream");
    _INFOLIST("%s contentType=%s\n",__func__,contentType.c_str());

  } else {
    contentType = mime::getContentType(path);
    _INFOLIST("%s contentType=%s\n",__func__,contentType.c_str());   
  }

  if (!fileSystem->exists(path)) { // File not found, try gzip version
    path = path + ".gz";
    _INFOLIST("%s path not found=%s\n",__func__,path.c_str());

  }
  if (fileSystem->exists(path)) {
    _INFOLIST("%s read file=%s\n",__func__,path.c_str());

    File file = fileSystem->open(path, "r");
    int cnt=server.streamFile(file, contentType);
    _INFOLIST("%s file size %d\n",__func__,cnt);
    
    if (cnt != file.size()) {
       _INFOLIST("%s Sent less data than expected: %d\n",__func__,cnt);
    }
    file.close();
    return true;
  }
  return false;
}
/*
   As some FS (e.g. LittleFS) delete the parent folder when the last child has been removed,
   return the path of the closest parent still existing
*/
String lastExistingParent(String path) {
  while (path != "" && !fileSystem->exists(path)) {
    if (path.lastIndexOf('/') > 0) {
      path = path.substring(0, path.lastIndexOf('/'));
    } else {
      path = String();  // No slash => the top folder does not exist
    }
  }
  _INFOLIST("%s Last existing parent: %s\n",__func__,path.c_str());
  return path;
}
/*-----------------------------------------------------------------------
 * Creates an EEPROM dump (hex) into a file called dump.txt 
 */
void handleDump() {
  _INFOLIST("%s EEPROM opened\n",__func__);

  String path="/dump.txt";
  File file = fileSystem->open(path, "w");
  if (file) {
      //file.write((const char*)0);
      //file.close();
  } else {
    return;
  }
  
  _INFOLIST("%s Dump file created!\n",__func__);
  sprintf(hi,"--- %s EEPROM dump ---\n",host);
  file.write((char*)hi);
  
  int i = 10;
  while (i < 250) {
    
    sprintf(hi, "%05d -- ", i);
    file.write((char*)hi);
    Serial.print(hi);
    
    for (int j = 0; j < 10; j++) {
      uint8_t b = EEPROM.read(i + j);
      sprintf(hi, "%02x ", b);
      Serial.print(hi);
      file.write((char*)hi);

    }
    sprintf(hi,"\n");
    Serial.println(hi);
    file.write((char*)hi);
    i = i + 10;
  }
  sprintf(hi,"--- end of EEPROM dump ---\n");
  file.write((char*)hi);
  file.close();
  Serial.print(hi);

  _INFOLIST("%s Sending to display file (%s)\n",__func__,path.c_str());
  handleFileRead(path);
  _INFOLIST("%s Returned from handleFileRead(%s)\n",__func__,path.c_str());
}
/*-------------------------------------------
   Handle the creation/rename of a new file
   Operation      | req.responseText
   ---------------+--------------------------------------------------------------
   Create file    | parent of created file
   Create folder  | parent of created folder
   Rename file    | parent of source file
   Move file      | parent of source file, or remaining ancestor
   Rename folder  | parent of source folder
   Move folder    | parent of source folder, or remaining ancestor
*/
void handleFileCreate() {
  _INFOLIST("%s\n",__func__);

  if (!fsOK) {
    return replyServerError(FPSTR(FS_INIT_ERROR));
  }

  String path = server.arg("path");
  if (path == "") {
    return replyBadRequest(F("PATH ARG MISSING"));
  }

#ifdef USE_SPIFFS
  if (checkForUnsupportedPath(path).length() > 0) {
    return replyServerError(F("INVALID FILENAME"));
  }
#endif

  if (path == "/") {
    return replyBadRequest("BAD PATH");
  }
  if (fileSystem->exists(path)) {
    return replyBadRequest(F("PATH FILE EXISTS"));
  }

  String src = server.arg("src");
  if (src == "") {
    // No source specified: creation
    _INFOLIST("%s %s\n",__func__,path.c_str());
    if (path.endsWith("/")) {
      // Create a folder
      path.remove(path.length() - 1);
      if (!fileSystem->mkdir(path)) {
        return replyServerError(F("MKDIR FAILED"));
      }
    } else {
      // Create a file
      File file = fileSystem->open(path, "w");
      if (file) {
        file.write((const char*)0);
        file.close();
      } else {
        return replyServerError(F("CREATE FAILED"));
      }
    }
    if (path.lastIndexOf('/') > -1) {
      path = path.substring(0, path.lastIndexOf('/'));
    }
    replyOKWithMsg(path);
  } else {
    // Source specified: rename
    if (src == "/") {
      return replyBadRequest("BAD SRC");
    }
    if (!fileSystem->exists(src)) {
      return replyBadRequest(F("SRC FILE NOT FOUND"));
    }
    
    _INFOLIST("%s %s from %s\n",__func__,path.c_str(),src.c_str());

    if (path.endsWith("/")) {
      path.remove(path.length() - 1);
    }
    if (src.endsWith("/")) {
      src.remove(src.length() - 1);
    }
    if (!fileSystem->rename(src, path)) {
      return replyServerError(F("RENAME FAILED"));
    }
    replyOKWithMsg(lastExistingParent(src));
  }
}

/*---------------------------------------------------------------
   Delete the file or folder designed by the given path.
   If it's a file, delete it.
   If it's a folder, delete all nested contents first then the folder itself

   IMPORTANT NOTE: using recursion is generally not recommended on embedded devices and can lead to crashes (stack overflow errors).
   This use is just for demonstration purpose, and FSBrowser might crash in case of deeply nested filesystems.
   Please don't do this on a production system.
*/
void deleteRecursive(String path) {

  _INFOLIST("%s path=%s\n",__func__,path.c_str());

  File file = fileSystem->open(path, "r");
  bool isDir = file.isDirectory();
  file.close();

  // If it's a plain file, delete it
  if (!isDir) {
    fileSystem->remove(path);
    return;
  }

  // Otherwise delete its contents first
  Dir dir = fileSystem->openDir(path);

  while (dir.next()) {
    deleteRecursive(path + '/' + dir.fileName());
  }

  // Then delete the folder itself
  fileSystem->rmdir(path);
}

/*--------------------------------------------------
   Handle a file deletion request
   Operation      | req.responseText
   ---------------+--------------------------------------------------------------
   Delete file    | parent of deleted file, or remaining ancestor
   Delete folder  | parent of deleted folder, or remaining ancestor
*/
void handleFileDelete() {
  _INFOLIST("%s\n",__func__);

  if (!fsOK) {
    return replyServerError(FPSTR(FS_INIT_ERROR));
  }

  String path = server.arg(0);
  if (path == "" || path == "/") {
    return replyBadRequest("BAD PATH");
  }

  _INFOLIST("%s path:%s\n",__func__,path.c_str());
  if (!fileSystem->exists(path)) {
    return replyNotFound(FPSTR(FILE_NOT_FOUND));
  }
  deleteRecursive(path);

  replyOKWithMsg(lastExistingParent(path));
}

/*
   Handle a file upload request
*/
void handleFileUpload() {
  _INFOLIST("%s\n",__func__);

  if (!fsOK) {
    return replyServerError(FPSTR(FS_INIT_ERROR));
  }
  if (server.uri() != "/edit") {
    return;
  }
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;

    // Make sure paths always start with "/"
    if (!filename.startsWith("/")) {
      filename = "/" + filename;
    }
    _INFOLIST("%s File to upload: %s\n",__func__,filename.c_str());
    uploadFile = fileSystem->open(filename, "w");
    if (!uploadFile) {
      return replyServerError(F("CREATE FAILED"));
    }
    _INFOLIST("%s Upload: START(%s)\n",__func__,filename);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      size_t bytesWritten = uploadFile.write(upload.buf, upload.currentSize);
      if (bytesWritten != upload.currentSize) {
        return replyServerError(F("WRITE FAILED"));
      }
    }
    _INFOLIST("%s Upload: WRITE, Bytes: %s\n",__func__,upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }
    _INFOLIST("%s Upload: END, Size: %s\n",__func__,upload.totalSize);
  }
}
/*
   The "Not Found" handler catches all URI not explicitly declared in code
   First try to find and return the requested file from the filesystem,
   and if it fails, return a 404 page with debug information
*/
void handleNotFound() {
  _INFOLIST("%s\n",__func__);

  if (!fsOK) {
    return replyServerError(FPSTR(FS_INIT_ERROR));
  }

  String uri = WebServer::urlDecode(server.uri());  // required to read paths with blanks

  if (handleFileRead(uri)) {
    return;
  }

  // Dump debug data
  String message;
  message.reserve(100);
  message = F("Error: File not found\n\nURI: ");
  message += uri;
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += F("\nArguments: ");
  message += server.args();
  message += '\n';
  for (uint8_t i = 0; i < server.args(); i++) {
    message += F(" NAME:");
    message += server.argName(i);
    message += F("\n VALUE:");
    message += server.arg(i);
    message += '\n';
  }
  message += "path=";
  message += server.arg("path");
  message += '\n';
  _INFOLIST("%s message:%s\n",__func__,message.c_str());

  return replyNotFound(message);
}

/*-----------------------------------------------------------
   This specific handler returns the index.htm (or a gzipped version) from the /edit folder.
   If the file is not present but the flag INCLUDE_FALLBACK_INDEX_HTM has been set, falls back to the version
   embedded in the program code.
   Otherwise, fails with a 404 page with debug information
*/
void handleGetEdit() {
  _INFOLIST("%s\n",__func__);
  
  if (handleFileRead(F("/edit/index.htm"))) {
    _INFOLIST("%s /edit/index.htm",__func__); 
    return;
  }

#ifdef INCLUDE_FALLBACK_INDEX_HTM
  server.sendHeader(F("Content-Encoding"), "gzip");
  server.send(200, "text/html", index_htm_gz, index_htm_gz_len);
#else
  replyNotFound(FPSTR(FILE_NOT_FOUND));
#endif
}
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
void setup_FSBrowser() {

  /*-------------------------
   * Initialize file system 
   */
  fileSystemConfig.setAutoFormat(false);
  fileSystem->setConfig(fileSystemConfig);
  fsOK = fileSystem->begin();
  _INFOLIST("%s %s\n",__func__,(fsOK ? F("Filesystem initialized.") : F("Filesystem init failed!")));

#ifdef USE_SPIFFS
  // Debug: dump on console contents of filesystem with no filter and check filenames validity
  Dir dir = fileSystem->openDir("");
  _INFOLIST("%s List of files at root of filesystem:\n",__func__);
  while (dir.next()) {
    String error = checkForUnsupportedPath(dir.fileName());
    String fileInfo = dir.fileName() + (dir.isDirectory() ? " [DIR]" : String(" (") + dir.fileSize() + "b)");
    _INFOLIST("%s %s\n",__func__,(error + fileInfo).c_str());
    if (error.length() > 0) {
      unsupportedFiles += error + fileInfo + '\n';
    }
  }

  // Keep the "unsupportedFiles" variable to show it, but clean it up
  unsupportedFiles.replace("\n", "<br/>");
  unsupportedFiles = unsupportedFiles.substring(0, unsupportedFiles.length() - 5);
#endif

  /*--------------------------------
   * Start mDNS to locate the board
   * -------------------------------*/
  if (MDNS.begin(host)) {
    MDNS.addService("http", "tcp", 80);
    _INFOLIST("%s Open http://%s.local/edit to open the FileSystem Browser\n",__func__,hostname);
  }

  /*---------------------------------
   * init WebServer
   *---------------------------------*/
   
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/list", HTTP_GET, handleFileList);
  server.on("/dump", HTTP_GET, handleDump);
  server.on("/edit", HTTP_GET, handleGetEdit);
  server.on("/edit", HTTP_PUT, handleFileCreate);
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  server.on("/edit", HTTP_POST, replyOK, handleFileUpload);
  server.onNotFound(handleNotFound);

  server.begin();
  _INFOLIST("%s HTTP server started\n",__func__);
}

/*---------------------------------------
 * Loop
 * Handle Web Server requests and 
 * provide for mDNS query resolution 
 */
void loop_FSBrowser() {
  server.handleClient();
  MDNS.update();
}
#endif //FSBROWSER
#endif //RP2040_W
