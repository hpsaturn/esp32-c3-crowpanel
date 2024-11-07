#define LGFX_USE_V1
#include <WiFi.h>
#include "Arduino.h"
#include <lvgl.h>
#include "demos/lv_demos.h"
#include <LovyanGFX.hpp>
#include <Ticker.h>
#include "CST816D.h"
#include "do_mian.h"
#include "ui.h"
#include <Preferences.h>
#include "I2C_BM8563.h"
#define I2C_SDA 4
#define I2C_SCL 5
#define TP_INT 0
#define TP_RST -1

//encoder
#define ENCODER_A_PIN 19
#define ENCODER_B_PIN 18
#define SWITCH_PIN    8

//Custom key pins
#define Custom_PIN    1

long position = 0;
long position_tmp = 0;
bool switchPressed = false;

#define PI4IO_I2C_ADDR 0x43
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
I2C_BM8563_DateTypeDef dateStruct;
I2C_BM8563_TimeTypeDef timeStruct;

#define off_pin 35
#define buf_size 120
//Alarm switch sign
int fal = 0;
//Indicates whether the alarm has gone off
int fal1 = 0;
uint32_t hourValue = 0;
uint32_t minuteValue = 0;

void Preferences_read();
void Preferences_init();

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_GC9A01 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        cfg.spi_host = SPI2_HOST; 
        cfg.spi_mode = 0;                  
        cfg.freq_write = 80000000;         
        cfg.freq_read = 20000000;          
        cfg.spi_3wire = true;              
        cfg.use_lock = true;               
        cfg.dma_channel = SPI_DMA_CH_AUTO; 
        cfg.pin_sclk = 6;  
        cfg.pin_mosi = 7;  
        cfg.pin_miso = -1; 
        cfg.pin_dc = 2;    
        _bus_instance.config(cfg);              
        _panel_instance.setBus(&_bus_instance); 
      }
      { 
        auto cfg = _panel_instance.config();
        cfg.pin_cs = 10;   
        cfg.pin_rst = -1;  
        cfg.pin_busy = -1; 
        cfg.memory_width = 240;   
        cfg.memory_height = 240;  
        cfg.panel_width = 240;    
        cfg.panel_height = 240;   
        cfg.offset_x = 0;         
        cfg.offset_y = 0;         
        cfg.offset_rotation = 0;  
        cfg.dummy_read_pixel = 8; 
        cfg.dummy_read_bits = 1;  
        cfg.readable = false;     
        cfg.invert = true;        
        cfg.rgb_order = false;   
        cfg.dlen_16bit = false;   
        cfg.bus_shared = false;   
        _panel_instance.config(cfg);
      }
      setPanel(&_panel_instance); 
    }
};

LGFX tft;
CST816D touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

/*Change to your screen resolution*/
static const uint32_t screenWidth = 240;
static const uint32_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[2][screenWidth * buf_size];

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *fn_name, const char *dsc)
{
  Serial.printf("%s(%s)@%d->%s\r\n", file, fn_name, line, dsc);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (tft.getStartCount() == 0)
  {
    tft.endWrite();
  }

  tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::swap565_t *)&color_p->full);

  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  bool touched;
  uint8_t gesture;
  uint16_t touchX, touchY;
  touched = touch.getTouch(&touchX, &touchY, &gesture);
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

Ticker ticker;

//Extended IO function
void init_IO_extender() {
  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x01); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  Serial.print("Device ID: ");
  Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x03); // IO direction register
  Wire.write((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)); // set pins 0, 1, 2 as outputs
  Wire.endTransmission();
  
  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x07); // Output Hi-Z register
  Wire.write(~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4))); // set pins 0, 1, 2 low
  Wire.endTransmission();
}

void set_pin_io(uint8_t pin_number, bool value) {

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  Serial.print("Before the change: ");
  Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // Output register

  if (!value)
    Wire.write((~(1 << pin_number)) & rxdata); // set pin low
  else
    Wire.write((1 << pin_number) | rxdata); // set pin high
  Wire.endTransmission();
  
  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  rxdata = Wire.read();
  Serial.print("after the change: ");
  Serial.println(rxdata, HEX);
}

//RTC function
void RTC_init()
{
  rtc.begin();
  // Set custom time
  //  I2C_BM8563_TimeTypeDef timeStruct;
  //  timeStruct.hours   = 11;    // Hour (0 - 23)
  //  timeStruct.minutes = 59;   // Minute (0 - 59)
  //  timeStruct.seconds = 0;    // Second (0 - 59)
  //  rtc.setTime(&timeStruct);
  //
  //  I2C_BM8563_DateTypeDef dateStruct;
  //  dateStruct.weekDay = 3;    // Weekday (0 - 6, where 0 is Sunday)
  //  dateStruct.month   = 1;    // Month (1 - 12)
  //  dateStruct.date    = 24;   // Day of the month (1 - 31)
  //  dateStruct.year    = 2024; // Year
  //  rtc.setDate(&dateStruct);
}

//Encoder function
void updateEncoder() {
  static int previousState = 0;
  static int flag_A = 0;
  static int flag_C = 0;
  int currentState = (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);


  if ((currentState == 0b00 && previousState == 0b01) || (currentState == 0b01 && previousState == 0b11) ||
      (currentState == 0b11 && previousState == 0b10) || (currentState == 0b10 && previousState == 0b00)) {
    // foreward

    //    if (switchPressed) {
    flag_A++;
    if (flag_A == 50)
    {
      flag_A = 0;
      flag_C = 0;
      //        position++;
      //        position_tmp=position;
      position_tmp = 1;
    }
    //          flag_C=0;
    //    }
  } else if ((currentState == 0b01 && previousState == 0b00) || (currentState == 0b11 && previousState == 0b01) ||
             (currentState == 0b10 && previousState == 0b11) || (currentState == 0b00 && previousState == 0b10)) {
    // reversal
    //    if (switchPressed) {
    flag_C++;

    if (flag_C == 50)
    {
      //        position--;
      flag_C = 0;
      flag_A = 0;
      //        position_tmp=position;
      position_tmp = 0;
    }
    //        flag_A=0;
    //    }
  }

  previousState = currentState;
}

void switchPressedInterrupt() {
  switchPressed = !switchPressed;
}

void setup()
{

  Serial.begin(115200); /* prepare for possible serial debug */
  Serial.println("I am LVGL_Arduino");

  Wire.begin(4, 5);
  init_IO_extender();
  delay(100);
  set_pin_io(3, true);
  set_pin_io(4, true);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(Custom_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switchPressedInterrupt, FALLING);
  // ticker.attach(1, tcr1s);
  tft.init();
  tft.initDMA();
  tft.startWrite();
  tft.setColor(0, 0, 0);

  tft.fillScreen(TFT_BLACK);

  delay(200);
  if (is_touch == 1)
  {
    touch.begin();
  }

  lv_init();

#if LV_USE_LOG != 0
  //lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  lv_disp_draw_buf_init(&draw_buf, buf[0], buf[1], screenWidth * buf_size);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  if (is_touch == 1)
  {
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
  }

#if 0
  /* Create simple label */
  lv_obj_t *label = lv_label_create( lv_scr_act() );
  lv_label_set_text( label, "Hello Arduino! (V8.0.X)" );
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
#else
  /* Try an example from the lv_examples Arduino library
      make sure to include it as written above.
    lv_example_btn_1();
  */

  // uncomment one of these demos
  // lv_demo_widgets(); // OK
  // lv_demo_benchmark(); // OK
  //  lv_demo_keypad_encoder();
  //  works, but I haven't an encoder
  //  lv_demo_music();              // NOK
  //  lv_demo_printer();
  //  lv_demo_stress();             // seems to be OK
  ui_mian(); // watch
#endif
  Serial.println("Setup done");
  //
  delay(200);
  set_pin_io(2, true);



  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  //  pinMode(0, INPUT);
  // Get RTC
  RTC_init();

  rtc.getDate(&dateStruct);
  rtc.getTime(&timeStruct);

  h = timeStruct.hours;
  m = timeStruct.minutes;
  s = timeStruct.seconds * 60 % 3600;
  Serial.printf("current time： %02d:%02d:%02d\n",
                h,
                m,
                s);

  //Initializing the alarm page from the RTC
  lv_img_set_angle(ui_shand,  s);
  lv_img_set_angle(ui_shand2,  s);
  lv_img_set_angle(ui_mhand2, m * 60);
  lv_img_set_angle(ui_hhand2, h * 300 + m / 12 * 60);
  lv_img_set_angle(ui_mhand, m * 60);
  lv_img_set_angle(ui_hhand, h * 300 + m / 12 * 60);
  Preferences_read();

  //  xTaskCreatePinnedToCore(Watch_Function, "Watch", 20240 , NULL, 1, NULL,1); //Create Task 1
  // Start the scheduler
  //Initialize time
  if (h >= 12 &&  h <= 23)
  {
    lv_label_set_text(ui_AmPm, "PM");
    lv_label_set_text(ui_AmPm2, "PM");
  } else
  {
    lv_label_set_text(ui_AmPm, "AM");
    lv_label_set_text(ui_AmPm2, "AM");
  }
}
float  angle = 0;
void Alarm_task()
{
  int i = 0;
  long frequency = 2700;
  int toneCount = 0;
  unsigned long previousMillis = 0;
  const long interval = 500;  //The time interval for each drop is 0.5 seconds
  unsigned long delayStart = 0;
  bool delayActive = false;
  while (1)
  {
    unsigned long currentMillis = millis();  
    if (currentMillis - previousMillis >= interval && !delayActive)
    {
      previousMillis = currentMillis;
      i = !i;
      if (i)
      {
        tone(3, frequency); 
      }
      else
      {
        tone(3, 0); 
        toneCount++; 
      }
    }

    if (toneCount == 3 && !delayActive)
    {
      delayStart = currentMillis;  
      delayActive = true; 
    }

    if (delayActive && currentMillis - delayStart >= 1000)
    {
      delayActive = false;  
      toneCount = 0;  
    }
    lv_timer_handler();
    if (!switchState)
    {
      tone(3, 0); 
      digitalWrite(3, LOW);
      break;
    }
  }
}
//void Watch_Function(void *param)
void loop()
{
  //  size_t freeMem;
  while (1)
  {
    //   freeMem = esp_get_free_heap_size();
    // Print the current amount of remaining memory
    //  Serial.print("Free memory: ");
    //  Serial.print(freeMem);
    //  Serial.println(" bytes");
    int Scr = 2;
    lv_timer_handler(); /* let the GUI do its work */
    //  delay(5);
    //  angle = lv_img_get_angle(ui_shand);
    //  Serial.print("s angle:");
    //  Serial.println(angle );
    // Customize the key function
    if (!digitalRead(Custom_PIN))
    {
      delay(500);
      if (digitalRead(Custom_PIN))
      {
        if (Screen_flag != 4)
        {
          _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen4_screen_init);
          Screen_flag = 4;
        }
      }
    }

    // Alarm clock function
    if (switchState && !fal)
    {
      // Turn on the alarm
      fal = 1;
      fal1 = 0;
      Serial.println(switchState ? "On" : "Off");
      Preferences_init();
    } else if (!switchState && fal)
    {
      // Turn off the alarm
      fal = 0;
      fal1 = 0;
      Serial.println(switchState ? "On" : "Off");
      set_pin_io(0, false);

      Serial.println("The alarm clock is off.");
      // Update time

      //    rtc.getDate(&dateStruct);
      //    rtc.getTime(&timeStruct);
      //
      //    h=timeStruct.hours;
      //    m=timeStruct.minutes;
      //
      //    lv_img_set_angle(ui_mhand,m * 60);
      //    lv_img_set_angle(ui_hhand, h * 300+m/12*60);
      //    lv_img_set_angle(ui_mhand2,m * 60);
      //    lv_img_set_angle(ui_hhand2, h * 300+m/12*60);
    }
    //Determine if the alarm is about to go off
    if (h == hourValue && m == minuteValue && switchState && !fal1)
    {
      Serial.println("The alarm goes off.");
      set_pin_io(0, true);
      //Switching interface 5
      _ui_screen_change( &ui_Screen5, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen5_screen_init);
      fal1 = 1;
    }

    if (fal1)
    {
      //The alarm clock keeps ringing
      Alarm_task();
    }
    //Encoder function: adjust time \ cut screen
    if (switchPressed) {
      while (switchPressed)
      {
        lv_timer_handler();
        delay(10);
        //foreward
        if (position_tmp == 1) //position_tmp<position
        {
          m++;
          lv_img_set_angle(ui_mhand, m * 60);
          lv_img_set_angle(ui_hhand, h * 300 + m / 12 * 60);
          lv_img_set_angle(ui_mhand2, m * 60);
          lv_img_set_angle(ui_hhand2, h * 300 + m / 12 * 60);
          if (m == 60  )
          {
            m = 0;
            if (h == 23)
              h = 0;
            else
              h++;
          }
          //            position_tmp=position;
          position_tmp = 2;
        }
        //reversal
        if (position_tmp == 0) //position_tmp>position
        {
          if (m == 0)
          {
            m = 60;
            if (h == 0)
              h = 23;
            else
              h--;
          }
          m--;
          lv_img_set_angle(ui_mhand, m * 60);
          lv_img_set_angle(ui_hhand, h * 300 + m / 12 * 60);
          lv_img_set_angle(ui_mhand2, m * 60);
          lv_img_set_angle(ui_hhand2, h * 300 + m / 12 * 60);

          //position_tmp=position;
          position_tmp = 2;
        }

      }
      //Update time to RTC
      angle = lv_img_get_angle(ui_shand);
      //    s = (angle/3600*60)-1;
      //    if((angle/3600*60)-1>=56)
      //      m++;
      int i = ((int)(angle / 3600 * 60) - 1 + 4) % 60;
      if (m == 60)
        m == 0;
      timeStruct.hours   = h;    // Hour (0 - 23)
      timeStruct.minutes = m;   // Minute (0 - 59)
      timeStruct.seconds = i;    // Second (0 - 59)

      Serial.printf("The update time is： %02d:%02d:%02d\n",
                    timeStruct.hours,
                    timeStruct.minutes,
                    timeStruct.seconds
                   );
      rtc.setTime(&timeStruct);

      rtc.getTime(&timeStruct);
      //
      //    Serial.printf("The update time is： %02d:%02d:%02d\n",
      //                timeStruct.hours,
      //                timeStruct.minutes,
      //                timeStruct.seconds
      //               );
      if (h >= 12)
      {
        lv_label_set_text(ui_AmPm, "PM");
        lv_label_set_text(ui_AmPm2, "PM");
      } else
      {
        lv_label_set_text(ui_AmPm, "AM");
        lv_label_set_text(ui_AmPm2, "AM");
      }

    }

    //Encoder function: switch interface
    //  delay(10);
    //foreward
    if (position_tmp == 1) //position_tmp<position
    {
      Serial.print("++++ ");
      //    Serial.println(position, DEC);
      lv_obj_t *current_screen = lv_scr_act();
      if (current_screen == ui_Screen2)
      {
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen3_screen_init);
        Screen_flag = 3;

      }

      if (current_screen == ui_Screen3)
      {
        _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen4_screen_init);
        Screen_flag = 4;
      }

      if (current_screen == ui_Screen4)
      {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen2_screen_init);
        Screen_flag = 2;
      }
      //position_tmp=position;
      position_tmp = 2;
    }
    //reversal
    if (position_tmp == 0) //position_tmp>position
    {
      Serial.print("----");
      lv_obj_t *current_screen = lv_scr_act();
      if (current_screen == ui_Screen2)
      {
        _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen4_screen_init);
        Screen_flag = 4;
      }

      if (current_screen == ui_Screen3)
      {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen2_screen_init);
        Screen_flag = 2;
      }

      if (current_screen == ui_Screen4)
      {
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen3_screen_init);
        Screen_flag = 3;

      }

      // position_tmp=position;
      position_tmp = 2;
    }


    //Update Time
    if (tmp1 == 1)
    {
      tmp1 = 0;
      rtc.getDate(&dateStruct);
      rtc.getTime(&timeStruct);

      h = timeStruct.hours;
      m = timeStruct.minutes;

      lv_img_set_angle(ui_mhand, m * 60);
      lv_img_set_angle(ui_hhand, h * 300 + m / 12 * 60);
      lv_img_set_angle(ui_mhand2, m * 60);
      lv_img_set_angle(ui_hhand2, h * 300 + m / 12 * 60);

      Serial.printf("Now time is： %02d:%02d:%02d\n",
                    timeStruct.hours,
                    timeStruct.minutes,
                    timeStruct.seconds
                   );
      if (h >= 12 &&  h <= 23)
      {
        lv_label_set_text(ui_AmPm, "PM");
        lv_label_set_text(ui_AmPm2, "PM");
      } else
      {
        lv_label_set_text(ui_AmPm, "AM");
        lv_label_set_text(ui_AmPm2, "AM");
      }
    }
  }

}


void Preferences_read()
{
  Preferences prefs; // Declaring Preferences objects
  prefs.begin("alarm_clock"); // Open the namespace mynamespace
  hourValue = prefs.getUInt("hourValue", 0); // Gets the value of the key named "hourValue" in the current namespace
  // If this element is not present, the default value 0 is returned
  minuteValue = prefs.getUInt("minuteValue", 0); // Gets the value of the key name "minuteValue" in the current namespace
  // If this element is not present, the default value 0 is returned
  lv_dropdown_set_selected(ui_Dropdown1, hourValue + 1);
  lv_dropdown_set_selected(ui_Dropdown2, minuteValue + 1);
  prefs.end(); // Close the current namespace
}

void Preferences_init()
{
  //Initializes the read alarm
  Preferences prefs; // Declaring Preferences objects
  prefs.begin("alarm_clock"); // Open the namespace mynamespace
  hourValue = prefs.getUInt("hourValue", 0); // Gets the value of the key named "hourValue" in the current namespace
  // If this element is not present, the default value 0 is returned
  minuteValue = prefs.getUInt("minuteValue", 0); // Gets the value of the key name "minuteValue" in the current namespace
  // If this element is not present, the default value 0 is returned
  hourValue = atoi(hour); // Record the alarm time
  minuteValue = atoi(mimu);
  Serial.printf("Set an alarm： %u:%u \n", hourValue, minuteValue);
  prefs.putUInt("hourValue", hourValue); // Saves the data to the "hourValue" key of the current namespace
  prefs.putUInt("minuteValue", minuteValue); // Save the data to the "minuteValue" key of the current namespace键中
  prefs.end(); // Close the current namespace
}

// #include "Arduino.h"
// #include "CST816D.h"

// #define I2C_SDA 4
// #define I2C_SCL 5
// #define TP_INT 0
// #define TP_RST 1

// CST816D touch(I2C_SDA, I2C_SCL, TP_RST, TP_INT);

// void setup()
// {
//   Serial.begin(115200);
//   touch.begin();
// }

// bool FingerNum;
// uint8_t gesture;
// uint16_t touchX, touchY;
// void loop()
// {
//   FingerNum = touch.getTouch(&touchX, &touchY, &gesture);
//   if (FingerNum)
//   {
//     Serial.printf("X:%d,Y:%d,gesture:%x\n", touchX, touchY, gesture);
//   }

//   delay(100);
// }
