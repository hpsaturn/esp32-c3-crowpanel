// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project
#include "Arduino.h"
#include "ui.h"
bool switchState = false;
char  hour[32];
char  mimu[32];
static void switch_event_handler(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    
    if (code == LV_EVENT_VALUE_CHANGED){
        // LV_UNUSED(obj);  
        switchState = lv_obj_has_state(obj, LV_STATE_CHECKED);
        lv_dropdown_get_selected_str(ui_Dropdown1,hour,sizeof(hour));
        lv_dropdown_get_selected_str(ui_Dropdown2,mimu,sizeof(mimu));
        
    }
}
void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image1 = lv_img_create(ui_Screen4);
    lv_img_set_src(ui_Image1, &ui_img_1760865983);
    lv_obj_set_width(ui_Image1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Image1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Image1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image1, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Switch1 = lv_switch_create(ui_Screen4);
    lv_obj_set_width(ui_Switch1, 50);
    lv_obj_set_height(ui_Switch1, 25);
    lv_obj_set_x(ui_Switch1, 0);
    lv_obj_set_y(ui_Switch1, 43);
    lv_obj_set_align(ui_Switch1, LV_ALIGN_CENTER);
     // 注册 switch 的事件回调函数
    lv_obj_add_event_cb(ui_Switch1, switch_event_handler, LV_EVENT_ALL, NULL);

    ui_Dropdown1 = lv_dropdown_create(ui_Screen4);
    lv_dropdown_set_dir(ui_Dropdown1, LV_DIR_RIGHT);
    lv_dropdown_set_options(ui_Dropdown1,
                            "\n0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23");
    lv_dropdown_set_selected_highlight(ui_Dropdown1, false);
    lv_obj_set_width(ui_Dropdown1, 50);
    lv_obj_set_height(ui_Dropdown1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown1, -60);
    lv_obj_set_y(ui_Dropdown1, -6);
    lv_obj_set_align(ui_Dropdown1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown2 = lv_dropdown_create(ui_Screen4);
    lv_dropdown_set_dir(ui_Dropdown2, LV_DIR_LEFT);
    lv_dropdown_set_options(ui_Dropdown2,
                            "\n0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n50\n51\n52\n53\n54\n55\n56\n57\n58\n59");
    lv_obj_set_width(ui_Dropdown2, 50);
    lv_obj_set_height(ui_Dropdown2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown2, 60);
    lv_obj_set_y(ui_Dropdown2, -6);
    lv_obj_set_align(ui_Dropdown2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_minuteLabel = lv_label_create(ui_Screen4);
    lv_obj_set_height(ui_minuteLabel, 20);
    lv_obj_set_width(ui_minuteLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_minuteLabel, 60);
    lv_obj_set_y(ui_minuteLabel, -37);
    lv_obj_set_align(ui_minuteLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_minuteLabel, "minute");

    ui_hourLabel = lv_label_create(ui_Screen4);
    lv_obj_set_height(ui_hourLabel, 20);
    lv_obj_set_width(ui_hourLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_hourLabel, -60);
    lv_obj_set_y(ui_hourLabel, -37);
    lv_obj_set_align(ui_hourLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_hourLabel, "hour\n");

    ui_Label4 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label4, 0);
    lv_obj_set_y(ui_Label4, 71);
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "off-on");

    ui_Label6 = lv_label_create(ui_Screen4);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label6, 0);
    lv_obj_set_y(ui_Label6, -81);
    lv_obj_set_align(ui_Label6, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label6, "alarm clock");
    lv_obj_set_style_text_font(ui_Label6, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Screen4, ui_event_Screen4, LV_EVENT_ALL, NULL);

}