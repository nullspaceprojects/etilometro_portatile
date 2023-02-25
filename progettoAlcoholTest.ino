#include <Wire.h>
//in Adafruit_HTU21DF.cpp I lowered the delay from 50 to 20 in both readTemperature and readHumidity fcns
#include "Adafruit_HTU21DF.h" 
#include <U8g2lib.h>
#include <NullSpaceLib.h>


/********* GLOBAL VARS ************/

//humdity-temperature sensor
Adafruit_HTU21DF HTU21 = Adafruit_HTU21DF();

/********* ISTANZA CLASSE DISPLAY OLED I2C SH1106 128X64 ************/
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//to save RAM with the NANO Board use this constructor with buffer size=1 
//(Only one page of the display memory is stored in the microcontroller RAM)
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define MQ3_AI_PIN 0 //A0
#define PUSH_BOTTON_PIN 5//D5
const float gVoltageConversionConstant = 5.f/1023;

TimerC timer_sensors_sampling;
const uint16_t sensors_sampling_time_ms=50;

R_TRIG fronte_discesa_button(true);
R_TRIG fronte_salita_button(true);

TON button_filter(false,200);

bool gTakeMeasure = false;

//Convert the 1-1023 value from analog read to a voltage.
template <class T>
float inline toVoltage(T reading){
  //constant derived from 5/1023 = 0.0048875
  return reading * gVoltageConversionConstant;
}

class DataHolder
{
  public:
    DataHolder(bool use_temp_comp=true)
    {
      this->enable_temperature_compensation=use_temp_comp;
      this->raw_mq3=0;
      this->temperature=0; // in °C
      this->relative_humidity=0; //in %
      this->g_liter=0;
      this->resetMinMax();
    }
    float temperature; // in °C
    float temperature_min;
    float temperature_max;
    
    float relative_humidity; //in %
    float relative_humidity_min;
    float relative_humidity_max;
    
    bool enable_temperature_compensation;
    float raw_mq3;
    float raw_mq3_min;
    float raw_mq3_max;
    double g_liter;
  private:
    String TempStr;
    String RelHumStr;
    String RawMq3Str;
    String VoltsMq3Str;
    String RawgLStr;

  public:
    void resetMinMax()
    {
      this->raw_mq3_min=50000;
      this->raw_mq3_max=0;
      this->temperature_min=50000;
      this->temperature_max=-500;
      this->relative_humidity_min=50000;
      this->relative_humidity_max=0;     
    }

    void setRelHumidityAndTemperature(float rh, float temp)
    {
      this->temperature=temp;
      if (this->enable_temperature_compensation)
      {
          //Temperature coefficient from Manual
          //Tcc = -0.15 [%/°C] 
          this->relative_humidity = rh + (25.f-this->temperature)*(-0.15f);       
      }
      else
      {
          this->relative_humidity = rh;
      }

      //max-min temperature
      if(this->temperature>this->temperature_max)
      {
        this->temperature_max=this->temperature;
      }
      if(this->temperature<this->temperature_min)
      {
        this->temperature_min=this->temperature;
      }
      //max-min relative humidity
      if(this->relative_humidity>this->relative_humidity_max)
      {
        this->relative_humidity_max=this->relative_humidity;
      }
      if(this->relative_humidity<this->relative_humidity_min)
      {
        this->relative_humidity_min=this->relative_humidity;
      }

      //String Info update
      /*
      this->getTempStr();
      this->getRelHumStr();
      this->getRawMq3Str();
      this->getVoltsMq3Str();
      */
    }

    String getTempStr(String txt="T: ")
    {      
      TempStr = txt + String(this->temperature,1) + String(" [") + String(this->temperature_min,1) + String(":") + String(this->temperature_max,1)+String("]C");
      return TempStr;
    }
    String getRelHumStr(String txt="RH: ")
    {     
      RelHumStr = txt + String(this->relative_humidity,0) + String(" [") + String(this->relative_humidity_min,0) + String(":") + String(this->relative_humidity_max,0)+String("]%");
      return RelHumStr;
    }
    String getRawMq3Str(String txt="")
    {     
      RawMq3Str = txt + String(this->raw_mq3,0) + String(" [") + String(this->raw_mq3_min,0) + String(":") + String(this->raw_mq3_max,0)+String("]");
      return RawMq3Str;
    }
    String getVoltsMq3Str(String txt="V: ")
    {
      VoltsMq3Str = txt + String(toVoltage<float>(this->raw_mq3),2) + String(" [") + String(toVoltage<float>(this->raw_mq3_min),2) + String(":") + String(toVoltage<float>(this->raw_mq3_max),2)+String("]");
      return VoltsMq3Str;
    }
    String get_gL_Str(String txt="Blood: ")
    {     
      RawgLStr = txt + String(this->g_liter,2) + String(" g/L");
      return RawgLStr;
    }
};
DataHolder gData;


/********* GLOBAL FUNCTIONS ************/

float read_mq3_sensor(int num_samples=10)
{
  uint16_t adcValue=0;
  for(int i=0;i<num_samples;i++)
  {
    adcValue+=analogRead(MQ3_AI_PIN);
    delay(3);
  }
  //compute mean-value
  float mean_value = adcValue*(1.f/num_samples);
  if(mean_value<gData.raw_mq3_min)
  {
    gData.raw_mq3_min = mean_value;
  }
  if(mean_value>gData.raw_mq3_max)
  {
    gData.raw_mq3_max = mean_value;
  }
  return mean_value; 
}

double raw2gLiter(double raw)
{
  /**
  raw       -> g/L
  232 (257) -> 0
  616 (637) -> 0.5
  627 (657) -> 0.5
  620 (639) -> 0.5
  mean 621 (644.3)

  Points:
  (x1: 232 y1: 0), (x2: 621, y2: 0.5)
  **/
  //LINEAR RELATION
  //double mgliter = 1.28534704370180d*raw + (-298.20051413881754d);
  //Quadratic Relation with third point (467, 160 mg/L)
  //0.0039252986 x^2 + -2.0629326370 x^1 + 267.3251015243
  double mgliter = 0.0039252986d*raw*raw + (-2.0629326370d)*raw + 267.3251015243d;
  if (mgliter<0)
  {
    mgliter=0;
  }
  return mgliter*0.001; // g/liter
}


void writeOnOled(bool write_iters=false, int n_iter=0)
{
    //u8g2.clearBuffer();
    u8g2.firstPage();
    do
    {
      uint8_t ypos=12;
      //Titolo
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.drawStr(0,ypos,"Bei Poco Fra");
      //Fine titolo

      //Set Font          
      //u8g2.setFont(u8g2_font_ncenB08_tr);
      //uint8_t CapitalASize = 8; //pixels 

      u8g2.setFont(u8g2_font_courB08_tr); //A6
      uint8_t CapitalASize = 6; //pixels 
      
      ypos+=CapitalASize+5;
      if(write_iters)
      {
        String full = String("n. ") + String(n_iter) + String(" ") + gData.getTempStr();
        u8g2.drawStr(2,ypos,full.c_str());      
      } 
      else
      {
        u8g2.drawStr(2,ypos,gData.getTempStr().c_str());
      }    
      ypos+=CapitalASize+5;
      u8g2.drawStr(2,ypos,gData.getRelHumStr().c_str());
      ypos+=CapitalASize+5;
      u8g2.drawStr(2,ypos,gData.getRawMq3Str().c_str());
      ypos+=CapitalASize+5;
      //u8g2.drawStr(2,ypos,gData.getVoltsMq3Str().c_str());
      u8g2.drawStr(2,ypos,gData.get_gL_Str().c_str());
    }while(u8g2.nextPage());
    //u8g2.sendBuffer();// transfer internal memory to the display
}

bool checkIfReadingIsStable()
{
  for (int i=0; i <= 20 ; i++)
  {
      //READ ANALOG MQ3
      gData.raw_mq3 = analogRead(MQ3_AI_PIN);
      //READ TEMP AND HUM
      gData.setRelHumidityAndTemperature(HTU21.readHumidity(), HTU21.readTemperature());
       //set first sample as baseline
      if (i==0){
        gData.raw_mq3_min = gData.raw_mq3;
        gData.raw_mq3_max = gData.raw_mq3;
      }
      //Determine if last sensor reading is higher then previous high
      if (gData.raw_mq3 > gData.raw_mq3_max){
        gData.raw_mq3_max = gData.raw_mq3;
      }//Determine if sensor reading is lower then previous high
      if (gData.raw_mq3 < gData.raw_mq3_min){
        gData.raw_mq3_min = gData.raw_mq3;
      }
      writeOnOled(true,i);
      delay(1000);
      //readings must be very close for the entire 20sec. otherwise start over
      if (gData.raw_mq3_max - gData.raw_mq3_min > 30)
      {
        return false;
      }   
  }//END FOR

  // Check if resistance has not changed more then 5 steps during 10 sec = unstabile sensor
  if ((gData.raw_mq3_max - gData.raw_mq3_min) <= 30 )//|| gData.raw_mq3_max <= 135)
  {
    return true;   
  } 
  else return false;
  
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Alcool Test Warming Up..."));
  Serial.println(F("V3.5"));

  //LOW WHEN BUTTON IS PRESSED
  pinMode(PUSH_BOTTON_PIN,INPUT_PULLUP); 

  u8g2.begin();
  u8g2.setBitmapMode(1); //set the background to transparant
 
  //splash screen
  u8g2.firstPage();
  do
  {
    u8g2.drawXBMP(32,
                0, 
                logo_ninf_width, 
                logo_ninf_height, 
                logo_ninf_bits);
  }while(u8g2.nextPage());

  if (!HTU21.begin()) {
    Serial.println(F("Sensor not found!"));
    while (1);
  }
  Wire.setClock(400000UL);
  delay(4000);

  Serial.println(F("Wait For Stable Reading..."));
  while(!checkIfReadingIsStable())
  {
    delay(10);
  }


  timer_sensors_sampling.start();
  Serial.println(F("Setup OK !"));

}

void loop() {

  if(timer_sensors_sampling.getET()>sensors_sampling_time_ms)
  {
    timer_sensors_sampling.reset();

    //isPressedPb=1->PB pressed
    //isPressedPb=0->PB released
    bool isPressedPb = !digitalRead(PUSH_BOTTON_PIN);
    button_filter.run(isPressedPb);
    //fronte_discesa_button.call(!isPressedPb);
    fronte_salita_button.call(button_filter.q);
    if(fronte_salita_button.getQ())
    {
      //start reading
      //Serial.println(F("PB Start Reading"));
      gData.resetMinMax();
      gTakeMeasure=true;
    }
    if(!isPressedPb)
    {
      //end reading
      gTakeMeasure=false;
    }
    /*
    if(fronte_discesa_button.getQ())
    {
      //end reading
      Serial.println(F("PB Stop Reading"));
    }
    */   
    if (gTakeMeasure)
    {
      gData.setRelHumidityAndTemperature(HTU21.readHumidity(), HTU21.readTemperature());
      gData.raw_mq3 = read_mq3_sensor(10);
      gData.g_liter = raw2gLiter((double)gData.raw_mq3);
      /*
      Serial.println(gData.getTempStr());
      Serial.println(gData.getRelHumStr());
      Serial.println(gData.getRawMq3Str());
      Serial.println(gData.getVoltsMq3Str());
      */
      writeOnOled();      
    }
    
  }

}


