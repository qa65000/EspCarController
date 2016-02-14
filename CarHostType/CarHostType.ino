///Ver 1.02
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Ticker.h>

//////////////////////
// Port Definitions //
//////////////////////
#define  NO_130MOTOR    1
#define  OLED_RED    16
#define  OLED_GREEN  5
#define  OLED_BLUE   4
#define  OLEFT_MTRP 14
#define  OLEFT_MTRM 12
#define  ORIGHT_MTRP 15
#define  ORIGHT_MTRM 13
//////////////////////
// Timer Definition //
//////////////////////
#define  TIMER100MS  (10)
#define  TIMER5000MS (500)
//////////////////////
// WiFi Definitions //
//////////////////////
const char APPass[] = "12345678";      // 8 charctor
const word localPort = 65000;
/////////////////////
// Class           //
/////////////////////
WiFiUDP Udp;
Ticker Timer10Ms;
/////////////////////
// Struct          //
/////////////////////
struct LedCommad {
  byte Command;
  byte Red;
  byte Green;
  byte Blue;
  byte Dummy;
};

struct MotorCommand {
  byte Command;
  byte LeftP;
  byte LeftM;
  byte RightP;
  byte RightM;
};
/////////////////////
// Data            //
/////////////////////

byte PacketBuff[256]; //buffer to hold incoming and outgoing packets
byte Status;
word BaseTimer;
word Blink;
word MotorTimer;
word InitTimer;
byte RedLed;
byte BlueLed;
byte GreenLed;
word LeftMotorP;
word LeftMotorM;
word RightMotorP;
word RightMotorM;
word SLeftMotorP;
word SLeftMotorM;
word SRightMotorP;
word SRightMotorM;


/////////////////////////////////////////
/////// 10ms Base Timer //////////////////
/////////////////////////////////////////
void Int10ms( void )
{
  if (--BaseTimer == 0)
  { /*** 500ms Blink Timer & Flag ****/
    BaseTimer = 50;
    Blink = ~Blink;
  }
  if(MotorTimer) 
           MotorTimer--;
  if(InitTimer)
          InitTimer--;
}
/////////////////////////////////////////
/////// Nible2Pwm  //////////////////////
/////////////////////////////////////////
word ToPwm( byte Val )
{
    return (((word)Val) << 5);    //max fff-min000
}

/////////////////////////////////////////
/////// Led Main            /////////////
/////////////////////////////////////////
void LedMain(void)
{
  if(InitTimer)
  {
     GreenLed= BlueLed =RedLed = Blink;
  }
  if (RedLed )  digitalWrite(OLED_RED, HIGH);
  else            digitalWrite(OLED_RED, LOW );

  if (BlueLed ) digitalWrite(OLED_BLUE, HIGH);
  else            digitalWrite(OLED_BLUE, LOW );

  if (GreenLed)  digitalWrite(OLED_GREEN, HIGH);
  else            digitalWrite(OLED_GREEN, LOW );

}
/////////////////////////////////////////
/////// Target ... up/down  /////////////
/////////////////////////////////////////
void TargetUpDown( word* Target , word* Now )
{
#if  NO_130MOTOR
   *Now = *Target;
#else
   if( *Target > *Now ) 
  {
    if( (*Target - *Now) >= 0x60 ) *Now += 0x60;  
    else                           *Now = *Target; 
   }else{
     if( (*Now - *Target) >= 0x60 ) *Now -= 0x60;
     else                           *Now = *Target;
   }
#endif
}
/////////////////////////////////////////
/////// Motor Main          /////////////
/////////////////////////////////////////
void MotorMain(void)
{
  if(!MotorTimer)
  {
     LeftMotorP   = 0;
     LeftMotorM   = 0;
     RightMotorP  = 0;
     RightMotorM  = 0;
  }
  if ( SLeftMotorP != LeftMotorP )
  {
    TargetUpDown( &LeftMotorP , &SLeftMotorP );
    analogWrite(OLEFT_MTRP , SLeftMotorP );
  }
  if ( SLeftMotorM != LeftMotorM )
  {
    TargetUpDown( &LeftMotorM , &SLeftMotorM );
    analogWrite(OLEFT_MTRM , SLeftMotorM );
  }
  if ( SRightMotorP != RightMotorP )
  {
    TargetUpDown( &RightMotorP , &SRightMotorP );
    analogWrite(ORIGHT_MTRP, SRightMotorP );
  }
  if ( SRightMotorM != RightMotorM )
  {
    TargetUpDown( &RightMotorM , &SRightMotorM );
    analogWrite(ORIGHT_MTRM, SRightMotorM );
  }
}


/////////////////////////////////////////
/////// Command Analyze Main /////////////
/////////////////////////////////////////
void UdpCommandAnalyze(void)
{
  unsigned char  ReadCount;
  delay(10);
  ReadCount = Udp.parsePacket();
  if ( ReadCount >= 5 )
  {
    Udp.read(PacketBuff, ReadCount); // read the packet into the buffer
    switch ( PacketBuff[0] )
    {
        case  'L' :
          RedLed   = ((LedCommad*)PacketBuff)->Red;
          GreenLed = ((LedCommad*)PacketBuff)->Green;
          BlueLed  = ((LedCommad*)PacketBuff)->Blue;
          break;
        case  'M' :
          LeftMotorP  =  ToPwm( ((MotorCommand*)PacketBuff)->LeftP );
          LeftMotorM  =  ToPwm( ((MotorCommand*)PacketBuff)->LeftM );
          RightMotorP =  ToPwm( ((MotorCommand*)PacketBuff)->RightP );
          RightMotorM =  ToPwm( ((MotorCommand*)PacketBuff)->RightM );
          MotorTimer  = TIMER100MS;
        default:
          break;
   }
  }
}

/////////////////////////////////////////
/////// SetUp ///////////////////////////
/////////////////////////////////////////
void setup() 
{
//////////////////////////////////////
  BaseTimer = 49;
  InitTimer = TIMER5000MS;
  uint8_t MacAdr[WL_MAC_ADDR_LENGTH];
  String StrMac = "";
  int i;
//////////////////////////////////////
  pinMode(OLED_RED   , OUTPUT);
  pinMode(OLED_GREEN , OUTPUT);
  pinMode(OLED_BLUE  , OUTPUT);
  pinMode(OLEFT_MTRP  , OUTPUT);
  pinMode(OLEFT_MTRM  , OUTPUT);
  pinMode(ORIGHT_MTRP , OUTPUT);
  pinMode(ORIGHT_MTRM , OUTPUT);

  RedLed  = 0;
  BlueLed = 0;
  GreenLed = 0;
  LeftMotorP = 0;
  LeftMotorM = 0;
  RightMotorP = 0;
  RightMotorM = 0;
  SLeftMotorP = 1;
  SLeftMotorM = 1;
  SRightMotorP = 1;
  SRightMotorM = 1;
  Blink = 0;
  MotorMain();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);       //access point Start!
 
  WiFi.softAPmacAddress(MacAdr);
  ////////////// MacAdr=>String//////////////////
   for ( i = 0; i < sizeof(MacAdr) ; i++)
  {
     StrMac = StrMac + String(MacAdr[i], HEX);
    if(i != (sizeof(MacAdr)-1)) StrMac = StrMac + ":"; 
  }
  Serial.println("\nStart");
  Serial.println(StrMac);

  String APName = "ecar8266-"
               +String(MacAdr[sizeof(MacAdr)-3],HEX)+'-'
                 +String(MacAdr[sizeof(MacAdr)-2],HEX)+'-'
                  +String(MacAdr[sizeof(MacAdr)-1],HEX);    //Null is Auto Set
             
  WiFi.softAPConfig(IPAddress(192, 168, 0, 1),              // ip
                          IPAddress(192, 168, 0, 1),        // gateway
                           IPAddress(255, 255, 255, 0)  );  //sub net mask
                 
  WiFi.softAP((char *)&APName[0] , APPass );
  Serial.println("Name:"+APName );

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
/////////////////////////////////////////////////////
  if(Udp.begin(localPort))
  {
    Serial.println("Udp OPen Success");
  }
  else
  {
    Serial.println("Udp Error");
  }

  Serial.print("Local port: ");  Serial.println(Udp.localPort());
  Timer10Ms.attach(0.010f, Int10ms);
}

/////////////////////////////////////////
/////// Main Loop ///////////////////////
/////////////////////////////////////////
void loop(void) 
{
  MotorMain();
  LedMain();
  UdpCommandAnalyze();
}  
