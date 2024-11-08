// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
void play_Animation(lv_obj_t * TargetObject, int delay);
void play2_Animation(lv_obj_t * TargetObject, int delay);




// SCREEN: ui_back
void ui_back_screen_init(void);
void ui_event_back(lv_event_t * e);
lv_obj_t * ui_back;


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1(lv_event_t * e);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_yuang;
lv_obj_t * ui_logo;
lv_obj_t * ui_yuang1;
void anim_end_cb(lv_anim_t * anim);

// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
void ui_event_Screen2(lv_event_t * e);
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Watchback;
lv_obj_t *ui_elecrow1;
lv_obj_t *ui_AmPm;
void ui_event_shand( lv_event_t * e);
lv_obj_t * ui_shand;
lv_obj_t * ui_mhand;
void ui_event_hhand(lv_event_t * e);
lv_obj_t * ui_hhand;


// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
void ui_event_Screen3(lv_event_t * e);
lv_obj_t * ui_Screen3;
lv_obj_t * ui_Watchback2;
lv_obj_t *ui_elecrow2;
lv_obj_t *ui_AmPm2;
lv_obj_t * ui_shand2;
lv_obj_t * ui_mhand2;
lv_obj_t * ui_hhand2;


// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
void ui_event_Screen4(lv_event_t * e);
lv_obj_t * ui_Screen4;
lv_obj_t * ui_Image1;
lv_obj_t * ui_Switch1;
lv_obj_t * ui_Dropdown1;
lv_obj_t * ui_Dropdown2;
lv_obj_t * ui_minuteLabel;
lv_obj_t * ui_hourLabel;
lv_obj_t * ui_Label4;
lv_obj_t * ui_Label6;
lv_obj_t * ui____initial_actions0;

// SCREEN: ui_Screen5
void ui_Screen5_screen_init(void);
lv_obj_t *ui_Screen5;
void ui_event_Screen5( lv_event_t * e);
lv_obj_t *ui_Image1;
lv_obj_t *ui_Label1;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
#error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
#error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////
void time_s_Animation( lv_obj_t *TargetObject, int delay)
{
  ui_anim_user_data_t *PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
  PropertyAnimation_0_user_data->target = TargetObject;
  PropertyAnimation_0_user_data->val = -1;
  lv_anim_t PropertyAnimation_0;
  lv_anim_init(&PropertyAnimation_0);
  lv_anim_set_time(&PropertyAnimation_0, 60000);
  lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
  lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_angle );
  lv_anim_set_values(&PropertyAnimation_0, 0, 3600 );
  lv_anim_set_path_cb( &PropertyAnimation_0, lv_anim_path_linear);
  lv_anim_set_delay( &PropertyAnimation_0, delay + 0 );
  //lv_anim_set_deleted_cb( &PropertyAnimation_0, _ui_anim_callback_free_user_data );
  lv_anim_set_playback_time(&PropertyAnimation_0, 0);
  lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
  lv_anim_set_early_apply( &PropertyAnimation_0, false );
  lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_image_angle );
  lv_anim_set_repeat_count(&PropertyAnimation_0, LV_ANIM_REPEAT_INFINITE);
  lv_anim_start(&PropertyAnimation_0);

}
void play_Animation(lv_obj_t * TargetObject, int delay)
{
  ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
  PropertyAnimation_0_user_data->target = TargetObject;
  PropertyAnimation_0_user_data->val = -1;
  lv_anim_t PropertyAnimation_0;
  lv_anim_init(&PropertyAnimation_0);
  lv_anim_set_time(&PropertyAnimation_0, 3000);
  lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
  lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_angle);
  lv_anim_set_values(&PropertyAnimation_0, 0, -3600);
  lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_linear);
  lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
  //lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
  lv_anim_set_playback_time(&PropertyAnimation_0, 0);
  lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
  lv_anim_set_early_apply(&PropertyAnimation_0, false);
  lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_image_angle);
  lv_anim_set_ready_cb(&PropertyAnimation_0, anim_end_cb);
  lv_anim_start(&PropertyAnimation_0);

}
void play2_Animation(lv_obj_t * TargetObject, int delay)
{
  ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
  PropertyAnimation_0_user_data->target = TargetObject;
  PropertyAnimation_0_user_data->val = -1;
  lv_anim_t PropertyAnimation_0;
  lv_anim_init(&PropertyAnimation_0);
  lv_anim_set_time(&PropertyAnimation_0, 2000);
  lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
  lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_image_zoom);
  lv_anim_set_values(&PropertyAnimation_0, 0, 150);
  lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_linear);
  lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
  //lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
  lv_anim_set_playback_time(&PropertyAnimation_0, 0);
  lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
  lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
  lv_anim_set_early_apply(&PropertyAnimation_0, false);
  lv_anim_set_get_value_cb(&PropertyAnimation_0, &_ui_anim_callback_get_image_zoom);
  lv_anim_start(&PropertyAnimation_0);
  ui_anim_user_data_t * PropertyAnimation_1_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
  PropertyAnimation_1_user_data->target = TargetObject;
  PropertyAnimation_1_user_data->val = -1;
  lv_anim_t PropertyAnimation_1;
  lv_anim_init(&PropertyAnimation_1);
  lv_anim_set_time(&PropertyAnimation_1, 3000);
  lv_anim_set_user_data(&PropertyAnimation_1, PropertyAnimation_1_user_data);
  lv_anim_set_custom_exec_cb(&PropertyAnimation_1, _ui_anim_callback_set_image_angle);
  lv_anim_set_values(&PropertyAnimation_1, 0, 3600);
  lv_anim_set_path_cb(&PropertyAnimation_1, lv_anim_path_linear);
  lv_anim_set_delay(&PropertyAnimation_1, delay + 0);
  //lv_anim_set_deleted_cb(&PropertyAnimation_1, _ui_anim_callback_free_user_data);
  lv_anim_set_playback_time(&PropertyAnimation_1, 0);
  lv_anim_set_playback_delay(&PropertyAnimation_1, 0);
  lv_anim_set_repeat_count(&PropertyAnimation_1, 0);
  lv_anim_set_repeat_delay(&PropertyAnimation_1, 0);
  lv_anim_set_early_apply(&PropertyAnimation_1, false);
  lv_anim_set_get_value_cb(&PropertyAnimation_1, &_ui_anim_callback_get_image_angle);

  lv_anim_start(&PropertyAnimation_1);

}
//开机动画结束自动切屏函数
void anim_end_cb(lv_anim_t * anim)
{
  //    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 400, 10, &ui_Screen2_screen_init);
  //    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 400, 10, &ui_Screen2_screen_init);
  _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 400, 0, &ui_Screen3_screen_init);
  Screen_flag = 3;


  //    lv_obj_del(ui_Screen1);
  //    lv_obj_del(lv_anim_get_var(anim));
}
///////////////////// FUNCTIONS ////////////////////
void ui_event_back(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_SCREEN_LOADED) {
    _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 1000, 0, &ui_Screen1_screen_init);
  }
}
void ui_event_Screen1(lv_event_t * e)
{

  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_SCREEN_LOADED) {
    lv_obj_del(ui_back);
    play2_Animation(ui_logo, 0);
    play_Animation(ui_yuang1, 0);
  }//LV_EVENT_CLICKED
  //    if(event_code == LV_EVENT_ANIM_END) {
  //        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 10, 0, &ui_Screen3_screen_init);
  //        Screen_flag=2;
  //        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 400, 0, &ui_Screen2_screen_init);
  //    }
}
void ui_event_Screen2(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen3_screen_init);
    Screen_flag = 3;

  }
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen4_screen_init);
    Screen_flag = 4;
  }
  if ( event_code == LV_EVENT_SCREEN_LOADED) {
    //      time_s_Animation(ui_shand, 0);
  }
}

void ui_event_hhand(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen3_screen_init);
    Screen_flag = 3;

  }
}
void ui_event_Screen3(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen4_screen_init);
    Screen_flag = 4;
  }
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen2_screen_init);
    Screen_flag = 2;
  }
  if ( event_code == LV_EVENT_SCREEN_LOADED) {
    //      time_s_Animation(ui_shand2, 0);
  }
}
void ui_event_Screen4(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen2_screen_init);
    Screen_flag = 2;
  }
  if (event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen3_screen_init);
    Screen_flag = 3;

  }
}

uint32_t  s = 30;
uint32_t  m = 45;
uint32_t  h = 1;
uint32_t  tmp1 = 0;
uint32_t  tmp = 0;
//当前屏幕标志
uint32_t Screen_flag = 0;
//刷新时针位置
void time_refresh(lv_event_t* event) {

  //    tmp=lv_img_get_angle(ui_shand);
  s += 3;
  lv_img_set_angle(ui_shand, s % 3600);
  lv_img_set_angle(ui_shand2, s % 3600);
  tmp = lv_img_get_angle(ui_shand);
  if (s == 3600)
  {
    s = 0;
    tmp1 = 1;
    //          m++;
    //          //更新时间
    //          lv_img_set_angle(ui_mhand,m * 60);
    //          lv_img_set_angle(ui_hhand, h * 300+m/12*60);
    //          lv_img_set_angle(ui_mhand2,m * 60);
    //          lv_img_set_angle(ui_hhand2, h * 300+m/12*60);
    //区分上下午ui_AmPm

    
    //          if(m==60)
    //          {
    //            m=0;
    //            h++;
    //            if(h==23)
    //              h=0;
    //          }
  }



}

void send_event(void) {
  lv_event_send(ui_Screen2, LV_EVENT_REFRESH, NULL);
}

void ui_event_Screen5( lv_event_t * e)  {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT  ) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 400, 0, &ui_Screen2_screen_init);
    Screen_flag = 2;
    switchState = false;
    lv_obj_clear_state(ui_Switch1, LV_STATE_CHECKED);;
  }
  if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT  ) {
    lv_indev_wait_release(lv_indev_get_act());
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen2_screen_init);
    Screen_flag = 2;
    switchState = false;
    lv_obj_clear_state(ui_Switch1, LV_STATE_CHECKED);;
  }
  if (event_code == LV_EVENT_CLICKED && lv_indev_get_type(lv_indev_get_act()) == LV_INDEV_TYPE_POINTER) {
    _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, &ui_Screen2_screen_init);
    Screen_flag = 2;
    switchState = false;
    lv_obj_clear_state(ui_Switch1, LV_STATE_CHECKED);
  }
}
///////////////////// SCREENS ////////////////////

void ui_init(void)
{
  lv_disp_t * dispp = lv_disp_get_default();
  lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                       false, LV_FONT_DEFAULT);
  lv_disp_set_theme(dispp, theme);
  ui_back_screen_init();
  ui_Screen1_screen_init();
  ui_Screen2_screen_init();
  ui_Screen3_screen_init();
  ui_Screen4_screen_init();
  ui_Screen5_screen_init();
  ui____initial_actions0 = lv_obj_create(NULL);
  lv_disp_load_scr(ui_back);

}
