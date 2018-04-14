/* Gsender class helps send e-mails from Gmail account
 *  using Arduino core for ESP32 WiFi chip
 *  by jubaid hossain
 *  4.9.2018
 *  
 *  You need to supply your gmail login and password pre encoded in base64
 *    Use this site to do your conversion -->    https://www.base64decode.org/
*/
#ifndef G_SENDER
#define G_SENDER

//#define GS_SERIAL_LOG_1         //  Print to Serial only server responce
//#define GS_SERIAL_LOG_2         //  Print to Serial client commands and server responce

#include <WiFiClientSecure.h>

class Gsender
{
   protected:
       Gsender();
   private:
       const int SMTP_PORT = 465;
       const char* SMTP_SERVER = "smtp.gmail.com";
       const char* EMAILBASE64_LOGIN      = "xxxxxxxxxxxxxxxxxxxxxxx";     // 64 bit encoded email address name
       const char* EMAILBASE64_PASSWORD   = "xxxxxxxxxxx";                 // 64 bit encoded email password
       const char* FROM                   = "xxxxxxxxxxxx";                // email address
       const char* _error = nullptr;
       char* _subject = nullptr;
       String _serverResponce;
       static Gsender* _instance;
       bool AwaitSMTPResponse(WiFiClientSecure &client, const String &resp = "", uint16_t timeOut = 60000);

   public:
       static Gsender* Instance();
       Gsender* Subject(const char* subject);
       Gsender* Subject(const String &subject);
       bool Send(const String &to, const String &message);
       String getLastResponce();
       const char* getError();
};

#endif // G_SENDER

