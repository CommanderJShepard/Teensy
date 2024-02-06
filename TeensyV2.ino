#define Radio     Serial8 //Xbee Radio RX8 on Pin 34, TX8 on Pin 35 
#define GPS       Serial4 //GPS RX4 Pin 16, TX4 Pin 17
#define IMU       Serial3 // IMU RX3 Pin 30, TX3 Pin 31 
#define Raspberry Serial7 // Rpi RX7 Pin 28, TX7 Pin 29 
#define Master    Serial2 // Master Controller RX/Tx2

int  radio_incoming_data=0;
char radio_message[50]; 
int  radio_message_receiving=0,radio_message_received=0,radio_i=0;
char radio_c;

int gps_incoming_data=0;
int gps_i=0,gps_1=0,gps_2=0,gps_3=0,gps_4=0,gps_5=0,gps_6=0,gps_c[36];
long gps_lon=0,gps_lat=0,gps_height=0;
unsigned long gps_time;

const char gps_INIT[] PROGMEM=
{
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
      
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT     off
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH  on
 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0xB2, //NAV-POSECEF off

 //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
 //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

 0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
 0x00,0x4B,0x00,0x00,0x20,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x5F,0x13, //PRT:UART1,RTCM3,none,19200
};

int imu_incoming_data=0;
int imu_i=0,imu_1=0,imu_2=0,imu_3=0,imu_4=0,imu_5=0,imu_6=0,imu_7=0,imu_c[21];
long imu_s_roll,imu_e_roll,imu_m_roll;
long imu_s_pitch,imu_e_pitch,imu_m_pitch;
long imu_s_yaw,imu_e_yaw,imu_m_yaw;
float imu_roll,imu_pitch,imu_yaw;
unsigned long imu_time;

const char imu_INIT[] PROGMEM=
{
  0xFA,0xFF,0x30,0x00,0xD1,                      //GoToConfig
  //0xFA,0xFF,0xC0,0x04,0x20,0x30,0x00,0x01,0xEC,  //Euler 1Hz
  0xFA,0xFF,0xC0,0x04,0x20,0x30,0x00,0x0A,0xE3,  //Euler 10Hz
  0xFA,0xFF,0x10,0x00,0xF1,                      //GoToMeasurement
};

int store_gps_data=0,store_gps_i=0;
long store_gps_lon[500],store_gps_lat[500],store_gps_height[500];
unsigned long store_gps_time[500];
int store_imu_data=0,store_imu_i=0;
float store_imu_roll[500],store_imu_pitch[500],store_imu_yaw[500];
unsigned long store_imu_time[500];

unsigned long int raspberry_c=0,raspberry_i=0;

void setup()
{
 Serial.begin(9600);
 Radio.begin(57600);
 GPS.begin(19200);while(GPS.available()){GPS.read();}
 IMU.begin(115200);while(IMU.available()){IMU.read();}
 Raspberry.begin(115200);while(Raspberry.available()){Raspberry.read();}
   
 for(int i=0;i<(int)sizeof(gps_INIT);i++)
  GPS.write(pgm_read_byte(gps_INIT+i));

 for(int i=0;i<(int)sizeof(imu_INIT);i++)
  IMU.write(pgm_read_byte(imu_INIT+i));
}

void loop()
{
 ////////////////radio/////////////////////////
 radio_incoming_data=Radio.available();
 if (radio_incoming_data>0)
 {
  radio_c=Radio.read();
  Serial.print("radio_c: ");Serial.println(radio_c);
  if ((radio_c=='W') && (radio_message_receiving==0))
  {
   radio_message_receiving=1;
   radio_message_received=0; 
   radio_i=0;
   }
  else if ((radio_c=='Z') && (radio_message_receiving==1))
  {
   Serial.print("Message:");
   for (int i=1;i<radio_i;i++)
    Serial.print(radio_message[i]);
   Serial.println();   
   radio_message_receiving=0;
   radio_message_received=1;
  }
  if (radio_message_receiving==1)
  {
   radio_message[radio_i]=radio_c;
   radio_i++;
  }
 }

 ////////////////radio message action//////////
 if (radio_message_received==1)
 {
  if ((radio_message[1]=='d') && (radio_message[2]=='d'))
  {//just a test
   Radio.write("\n\r");
   Radio.print(imu_roll);Radio.print("\t");
   Radio.print(imu_pitch);Radio.print("\t");
   Radio.println(imu_yaw);
  
   radio_message_received=0;
  }
    else if ((radio_message[1]=='k') && (radio_message[2]=='k'))
  {//start storing gps and imu measured data
   Master.write('*'); 
   Master.write('1'); 
   Master.write('.'); 
   Master.write('0'); 
   Master.write('1'); 
   radio_message_received=0;
  }
  else if ((radio_message[1]=='s') && (radio_message[2]=='0'))
  {//start storing gps and imu measured data
   store_gps_data=1;
   store_gps_i=0;
   store_imu_data=1;
   store_imu_i=0;   
   radio_message_received=0;
  }
  else if ((radio_message[1]=='s') && (radio_message[2]=='1'))
  {//stop storing gps and imu measured data
   store_gps_data=0;
   store_imu_data=0;   
   radio_message_received=0;
  }
  else if ((radio_message[1]=='s') && (radio_message[2]=='2'))
  {//send stored data to hyperterminal
   for (int i=0;i<store_gps_i;i++)
   {
    Radio.write("\n\r");
    Radio.print(store_gps_lon[i]);Radio.print("\t");delay(100);
    Radio.print(store_gps_lat[i]);Radio.print("\t");delay(100);
    Radio.print(store_gps_height[i]);Radio.print("\t");delay(100);
    Radio.print(store_gps_time[i]);delay(100);  
   }
   for (int i=0;i<store_imu_i;i++)
   {
    Radio.write("\n\r");
    Radio.print(store_imu_roll[i]);Radio.print("\t");delay(100);
    Radio.print(store_imu_pitch[i]);Radio.print("\t");delay(100);
    Radio.print(store_imu_yaw[i]);Radio.print("\t");delay(100);
    Radio.print(store_imu_time[i]);delay(100);    
   }
   radio_message_received=0;
   while(GPS.available()){GPS.read();}
   while(IMU.available()){IMU.read();}
   while(GPS.available()){GPS.read();}
   while(IMU.available()){IMU.read();}
  }
  else if ((radio_message[1]=='r') && (radio_message[2]=='0'))
  {//start storing raspberry data
   Serial.println("Sending SR");
   Raspberry.write("SR");Raspberry.write("\n");
   radio_message_received=0;
  }
  else if ((radio_message[1]=='r') && (radio_message[2]=='1'))
  {//start sending raspberry data
   Serial.println("Sending SS");    
   Raspberry.write("SS");Raspberry.write("\n");
   while(raspberry_i<76800) //240*320
   {
    if (Raspberry.available()>0)
    {
     raspberry_c=Raspberry.read();
     Serial.print("rasp:");Serial.println(raspberry_c);
     raspberry_i++;
    }
   }
   radio_message_received=0;
  }
  else if ((radio_message[1]=='r') && (radio_message[2]=='2'))
  {//get video in raspberry
   Raspberry.write("SV");Raspberry.write("\n");
   radio_message_received=0;   
  }  
  else if ((radio_message[1]=='r') && (radio_message[2]=='3'))
  {//break while loop in raspberry
   Raspberry.write("BR");Raspberry.write("\n");
   radio_message_received=0;   
  }
 }

 ////////////////gps///////////////////////////
 gps_incoming_data=GPS.available();
 if (gps_incoming_data>0)
 {
  gps_1=gps_2;
  gps_2=gps_3;
  gps_3=gps_4;
  gps_4=gps_5;
  gps_5=gps_6;
  gps_6=GPS.read();
  if ((gps_1==0xB5) && (gps_2==0x62) && (gps_3==0x01) && (gps_4==0x02) && (gps_5==0x1C) && (gps_6==0x00))
   gps_i=0;
  else if ((gps_1!=0xB5) || (gps_2!=0x62) || (gps_3!=0x01) || (gps_4!=0x02) || (gps_5!=0x1C) || (gps_6!=0x00))
  {
   gps_c[gps_i]=gps_6;
   gps_i++;
  }
  if (gps_i==30)
  {
   gps_lon=gps_c[7]*pow(2,24)+gps_c[6]*pow(2,16)+gps_c[5]*pow(2,8)+gps_c[4]-pow(2,32);
   gps_lat=gps_c[11]*pow(2,24)+gps_c[10]*pow(2,16)+gps_c[9]*pow(2,8)+gps_c[8];
   gps_height=gps_c[15]*pow(2,24)+gps_c[14]*pow(2,16)+gps_c[13]*pow(2,8)+gps_c[12];//-pow(2,32);
   gps_time=millis();
   if (store_gps_data==1)
   {
    store_gps_lon[store_gps_i]=gps_lon;
    store_gps_lat[store_gps_i]=gps_lat;
    store_gps_height[store_gps_i]=gps_height;
    store_gps_time[store_gps_i]=gps_time;
    store_gps_i++;
   }
   Serial.print("lon=");  Serial.print(gps_lon);
   Serial.print("\tlat=");Serial.print(gps_lat);    
   Serial.print("\theight=");Serial.print(gps_height);
   Serial.print("\ttime=");Serial.println(gps_time);
  }
 }

 ////////////////imu///////////////////////////
  imu_incoming_data=IMU.available();
 if (imu_incoming_data>0)
 {
  imu_1=imu_2;
  imu_2=imu_3;
  imu_3=imu_4;
  imu_4=imu_5;
  imu_5=imu_6;  
  imu_6=imu_7;
  imu_7=IMU.read();
  if ((imu_1==0xFA) && (imu_2==0xFF) && (imu_3==0x36) && (imu_4==0x0F) && (imu_5==0x20) && (imu_6==0x30) && (imu_7==0x0C))
   imu_i=0;
  else if ((imu_1!=0xFA) || (imu_2!=0xFF) || (imu_3!=0x36) || (imu_4!=0x0F) || (imu_5!=0x20) || (imu_6!=0x30) || (imu_7!=0x0C))
  { 
   imu_c[imu_i]=imu_7;//Serial.print(imu_i);Serial.print("->");Serial.println(imu_c[imu_i],HEX); //TEST
   imu_i++;
  }

  if (imu_i==14)
  {
   imu_s_roll=bitRead(imu_c[0],7);
   imu_e_roll=bitRead(imu_c[0],6) *pow(2,7)+
              bitRead(imu_c[0],5) *pow(2,6)+
              bitRead(imu_c[0],4) *pow(2,5)+
              bitRead(imu_c[0],3) *pow(2,4)+
              bitRead(imu_c[0],2) *pow(2,3)+
              bitRead(imu_c[0],1) *pow(2,2)+
              bitRead(imu_c[0],0) *pow(2,1)+
              bitRead(imu_c[1],7) *pow(2,0);
   imu_m_roll=bitRead(imu_c[1],6) *pow(2,22)+
              bitRead(imu_c[1],5) *pow(2,21)+
              bitRead(imu_c[1],4) *pow(2,20)+
              bitRead(imu_c[1],3) *pow(2,19)+
              bitRead(imu_c[1],2) *pow(2,18)+
              bitRead(imu_c[1],1) *pow(2,17)+
              bitRead(imu_c[1],0) *pow(2,16)+
              bitRead(imu_c[2],7) *pow(2,15)+
              bitRead(imu_c[2],6) *pow(2,14)+
              bitRead(imu_c[2],5) *pow(2,13)+
              bitRead(imu_c[2],4) *pow(2,12)+
              bitRead(imu_c[2],3) *pow(2,11)+
              bitRead(imu_c[2],2) *pow(2,10)+
              bitRead(imu_c[2],1) *pow(2,9)+
              bitRead(imu_c[2],0) *pow(2,8)+
              bitRead(imu_c[3],7)*pow(2,7)+
              bitRead(imu_c[3],6)*pow(2,6)+
              bitRead(imu_c[3],5)*pow(2,5)+
              bitRead(imu_c[3],4)*pow(2,4)+
              bitRead(imu_c[3],3)*pow(2,3)+
              bitRead(imu_c[3],2)*pow(2,2)+
              bitRead(imu_c[3],1)*pow(2,1)+
              bitRead(imu_c[3],0)*pow(2,0);
   imu_roll=pow(-1,imu_s_roll)*(1+imu_m_roll/pow(2,23))*pow(2,imu_e_roll-127);
   imu_s_pitch=bitRead(imu_c[4],7);
   imu_e_pitch=bitRead(imu_c[4],6) *pow(2,7)+
               bitRead(imu_c[4],5) *pow(2,6)+
               bitRead(imu_c[4],4) *pow(2,5)+
               bitRead(imu_c[4],3) *pow(2,4)+
               bitRead(imu_c[4],2) *pow(2,3)+
               bitRead(imu_c[4],1) *pow(2,2)+
               bitRead(imu_c[4],0) *pow(2,1)+
               bitRead(imu_c[5],7) *pow(2,0);
   imu_m_pitch=bitRead(imu_c[5],6) *pow(2,22)+
               bitRead(imu_c[5],5) *pow(2,21)+
               bitRead(imu_c[5],4) *pow(2,20)+
               bitRead(imu_c[5],3) *pow(2,19)+
               bitRead(imu_c[5],2) *pow(2,18)+
               bitRead(imu_c[5],1) *pow(2,17)+
               bitRead(imu_c[5],0) *pow(2,16)+
               bitRead(imu_c[6],7) *pow(2,15)+
               bitRead(imu_c[6],6) *pow(2,14)+
               bitRead(imu_c[6],5) *pow(2,13)+
               bitRead(imu_c[6],4) *pow(2,12)+
               bitRead(imu_c[6],3) *pow(2,11)+
               bitRead(imu_c[6],2) *pow(2,10)+
               bitRead(imu_c[6],1) *pow(2,9)+
               bitRead(imu_c[6],0) *pow(2,8)+
               bitRead(imu_c[7],7)*pow(2,7)+
               bitRead(imu_c[7],6)*pow(2,6)+
               bitRead(imu_c[7],5)*pow(2,5)+
               bitRead(imu_c[7],4)*pow(2,4)+
               bitRead(imu_c[7],3)*pow(2,3)+
               bitRead(imu_c[7],2)*pow(2,2)+
               bitRead(imu_c[7],1)*pow(2,1)+
               bitRead(imu_c[7],0)*pow(2,0);
   imu_pitch=-(pow(-1,imu_s_pitch)*(1+imu_m_pitch/pow(2,23))*pow(2,imu_e_pitch-127));  
   imu_s_yaw=bitRead(imu_c[8],7);
   imu_e_yaw=bitRead(imu_c[8],6) *pow(2,7)+
             bitRead(imu_c[8],5) *pow(2,6)+
             bitRead(imu_c[8],4) *pow(2,5)+
             bitRead(imu_c[8],3) *pow(2,4)+
             bitRead(imu_c[8],2) *pow(2,3)+
             bitRead(imu_c[8],1) *pow(2,2)+
             bitRead(imu_c[8],0) *pow(2,1)+
             bitRead(imu_c[9],7) *pow(2,0);
   imu_m_yaw=bitRead(imu_c[9],6) *pow(2,22)+
             bitRead(imu_c[9],5) *pow(2,21)+
             bitRead(imu_c[9],4) *pow(2,20)+
             bitRead(imu_c[9],3) *pow(2,19)+
             bitRead(imu_c[9],2) *pow(2,18)+
             bitRead(imu_c[9],1) *pow(2,17)+
             bitRead(imu_c[9],0) *pow(2,16)+
             bitRead(imu_c[10],7) *pow(2,15)+
             bitRead(imu_c[10],6) *pow(2,14)+
             bitRead(imu_c[10],5) *pow(2,13)+
             bitRead(imu_c[10],4) *pow(2,12)+
             bitRead(imu_c[10],3) *pow(2,11)+
             bitRead(imu_c[10],2) *pow(2,10)+
             bitRead(imu_c[10],1) *pow(2,9)+
             bitRead(imu_c[10],0) *pow(2,8)+
             bitRead(imu_c[11],7)*pow(2,7)+
             bitRead(imu_c[11],6)*pow(2,6)+
             bitRead(imu_c[11],5)*pow(2,5)+
             bitRead(imu_c[11],4)*pow(2,4)+
             bitRead(imu_c[11],3)*pow(2,3)+
             bitRead(imu_c[11],2)*pow(2,2)+
             bitRead(imu_c[11],1)*pow(2,1)+
             bitRead(imu_c[11],0)*pow(2,0);
   imu_yaw=-(pow(-1,imu_s_yaw)*(1+imu_m_yaw/pow(2,23))*pow(2,imu_e_yaw-127));
   imu_time=millis();
   if (store_imu_data==1)
   {
    store_imu_roll[store_imu_i]=imu_roll;
    store_imu_pitch[store_imu_i]=imu_pitch;
    store_imu_yaw[store_imu_i]=imu_yaw;
    store_imu_time[store_imu_i]=imu_time;
    store_imu_i++;
   }
   Serial.print("roll=");Serial.print(imu_roll);
   Serial.print("\tpitch=");Serial.print(imu_pitch);
   Serial.print("\tyaw=");Serial.print(imu_yaw);
   Serial.print("\ttime=");Serial.println(imu_time);   
  } 
 } 
}
