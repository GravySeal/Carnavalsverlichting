// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 9.1.0
// Project name: DMX_Transmitter

#include "ui.h"

void ui_ConfigEbyte_screen_init(void)
{
    ui_ConfigEbyte = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ConfigEbyte, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Titel4 = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_Titel4, lv_pct(96));
    lv_obj_set_height(ui_Titel4, lv_pct(10));
    lv_obj_set_align(ui_Titel4, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Titel4, "Configure EByte E32");
    lv_obj_set_style_text_align(ui_Titel4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Titel4, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Titel4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Titel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Titel4, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Titel4, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_UARTLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_UARTLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_UARTLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UARTLabel, lv_pct(4));
    lv_obj_set_y(ui_UARTLabel, lv_pct(16));
    lv_label_set_text(ui_UARTLabel, "UART");

    ui_ParityLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_ParityLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ParityLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ParityLabel, lv_pct(4));
    lv_obj_set_y(ui_ParityLabel, lv_pct(30));
    lv_label_set_text(ui_ParityLabel, "Parity");

    ui_AirLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_AirLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AirLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AirLabel, lv_pct(4));
    lv_obj_set_y(ui_AirLabel, lv_pct(44));
    lv_label_set_text(ui_AirLabel, "AirRate");

    ui_PowerLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_PowerLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_PowerLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PowerLabel, lv_pct(4));
    lv_obj_set_y(ui_PowerLabel, lv_pct(58));
    lv_label_set_text(ui_PowerLabel, "Power");

    ui_FECLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_FECLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_FECLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_FECLabel, lv_pct(4));
    lv_obj_set_y(ui_FECLabel, lv_pct(72));
    lv_label_set_text(ui_FECLabel, "FEC");

    ui_FixedLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_FixedLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_FixedLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_FixedLabel, lv_pct(4));
    lv_obj_set_y(ui_FixedLabel, lv_pct(86));
    lv_label_set_text(ui_FixedLabel, "Fixed");

    ui_WORLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_WORLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_WORLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_WORLabel, lv_pct(54));
    lv_obj_set_y(ui_WORLabel, lv_pct(16));
    lv_label_set_text(ui_WORLabel, "WOR");

    ui_IOLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_IOLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_IOLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_IOLabel, lv_pct(54));
    lv_obj_set_y(ui_IOLabel, lv_pct(30));
    lv_label_set_text(ui_IOLabel, "IO Mode");

    ui_ChannelLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_ChannelLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ChannelLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ChannelLabel, lv_pct(54));
    lv_obj_set_y(ui_ChannelLabel, lv_pct(44));
    lv_label_set_text(ui_ChannelLabel, "Channel");

    ui_ChannelValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_ChannelValue,
                            "Ch 0\nCh 1\nCh 2\nCh 3\nCh 4\nCh 5\nCh 6\nCh 7\nCh 8\nCh 9\nCh 10\nCh 11\nCh 12\nCh 13\nCh 14\nCh 15\nCh 16\nCh 17\nCh 18\nCh 19\nCh 20\nCh 21\nCh 22\nCh 23\nCh 24\nCh 25\nCh 26\nCh 27\nCh 28\nCh 29\nCh 30\nCh 31");
    lv_obj_set_width(ui_ChannelValue, lv_pct(25));
    lv_obj_set_height(ui_ChannelValue, LV_SIZE_CONTENT);    /// 20
    lv_obj_set_x(ui_ChannelValue, lv_pct(70));
    lv_obj_set_y(ui_ChannelValue, lv_pct(41));
    lv_obj_add_flag(ui_ChannelValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_ChannelValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_submitConfigButton = lv_button_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_submitConfigButton, 100);
    lv_obj_set_height(ui_submitConfigButton, 40);
    lv_obj_set_x(ui_submitConfigButton, lv_pct(75));
    lv_obj_set_y(ui_submitConfigButton, lv_pct(81));
    lv_obj_add_flag(ui_submitConfigButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_submitConfigButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_submitConfigButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_submitConfigButton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_submitConfigButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_submitConfigButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_submitConfigButton, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_submitConfigText = lv_label_create(ui_submitConfigButton);
    lv_obj_set_width(ui_submitConfigText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_submitConfigText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_submitConfigText, LV_ALIGN_CENTER);
    lv_label_set_text(ui_submitConfigText, "Set config");

    ui_returnButton = lv_button_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_returnButton, 100);
    lv_obj_set_height(ui_returnButton, 40);
    lv_obj_set_x(ui_returnButton, lv_pct(50));
    lv_obj_set_y(ui_returnButton, lv_pct(81));
    lv_obj_add_flag(ui_returnButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_returnButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_returnButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_returnButton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_returnButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_returnButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_returnButton, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_returnButtonText = lv_label_create(ui_returnButton);
    lv_obj_set_width(ui_returnButtonText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_returnButtonText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_returnButtonText, LV_ALIGN_CENTER);
    lv_label_set_text(ui_returnButtonText, "Return");

    ui_WarningLabel = lv_label_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_WarningLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_WarningLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_WarningLabel, lv_pct(49));
    lv_obj_set_y(ui_WarningLabel, lv_pct(62));
    lv_label_set_text(ui_WarningLabel, "M0 & M1 OFF \nWHILE\nCONFIGURING");

    ui_UARTValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_UARTValue, "1200\n2400\n4800\n9600\n19200\n38400\n57600\n115200");
    lv_obj_set_width(ui_UARTValue, lv_pct(25));
    lv_obj_set_height(ui_UARTValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_UARTValue, lv_pct(20));
    lv_obj_set_y(ui_UARTValue, lv_pct(13));
    lv_obj_add_flag(ui_UARTValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_UARTValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_text_font(lv_dropdown_get_list(ui_UARTValue), &lv_font_montserrat_12,
                               LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_ParityValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_ParityValue, "NONE\nODD\nEVEN");
    lv_obj_set_width(ui_ParityValue, lv_pct(25));
    lv_obj_set_height(ui_ParityValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_ParityValue, lv_pct(20));
    lv_obj_set_y(ui_ParityValue, lv_pct(27));
    lv_obj_add_flag(ui_ParityValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_ParityValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_AirrateValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_AirrateValue, "2400\n2400_1\n2400_2\n4800\n9600\n19200\n19200_1\n19200_2");
    lv_obj_set_width(ui_AirrateValue, lv_pct(25));
    lv_obj_set_height(ui_AirrateValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AirrateValue, lv_pct(20));
    lv_obj_set_y(ui_AirrateValue, lv_pct(41));
    lv_obj_add_flag(ui_AirrateValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_AirrateValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_PowerValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_PowerValue, "30dBm\n27dBm\n24dBm\n21dBm");
    lv_obj_set_width(ui_PowerValue, lv_pct(25));
    lv_obj_set_height(ui_PowerValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_PowerValue, lv_pct(20));
    lv_obj_set_y(ui_PowerValue, lv_pct(55));
    lv_obj_add_flag(ui_PowerValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_PowerValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_FECValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_FECValue, "OFF\nON\n");
    lv_obj_set_width(ui_FECValue, lv_pct(25));
    lv_obj_set_height(ui_FECValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_FECValue, lv_pct(20));
    lv_obj_set_y(ui_FECValue, lv_pct(69));
    lv_obj_add_flag(ui_FECValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_FECValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_TransparencyValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_TransparencyValue, "TRANSPARENT\nFIXED\n");
    lv_obj_set_width(ui_TransparencyValue, lv_pct(25));
    lv_obj_set_height(ui_TransparencyValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TransparencyValue, lv_pct(20));
    lv_obj_set_y(ui_TransparencyValue, lv_pct(83));
    lv_obj_add_flag(ui_TransparencyValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_TransparencyValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_WORValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_WORValue, "250MS\n500MS\n750MS\n1000MS\n1250MS\n1500MS\n1750MS\n2000MS");
    lv_obj_set_width(ui_WORValue, lv_pct(25));
    lv_obj_set_height(ui_WORValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_WORValue, lv_pct(70));
    lv_obj_set_y(ui_WORValue, lv_pct(13));
    lv_obj_add_flag(ui_WORValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_WORValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_IOModeValue = lv_dropdown_create(ui_ConfigEbyte);
    lv_dropdown_set_options(ui_IOModeValue, "OPEN_COLLECTOR\nPUSH_PULL");
    lv_obj_set_width(ui_IOModeValue, lv_pct(25));
    lv_obj_set_height(ui_IOModeValue, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_IOModeValue, lv_pct(70));
    lv_obj_set_y(ui_IOModeValue, lv_pct(27));
    lv_obj_add_flag(ui_IOModeValue, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_IOModeValue, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_retrieveConfigButton = lv_button_create(ui_ConfigEbyte);
    lv_obj_set_width(ui_retrieveConfigButton, 100);
    lv_obj_set_height(ui_retrieveConfigButton, 40);
    lv_obj_set_x(ui_retrieveConfigButton, lv_pct(75));
    lv_obj_set_y(ui_retrieveConfigButton, lv_pct(63));
    lv_obj_add_flag(ui_retrieveConfigButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_retrieveConfigButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_retrieveConfigButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_retrieveConfigButton, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_retrieveConfigButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_retrieveConfigButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_retrieveConfigButton, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_retrieveConfigText = lv_label_create(ui_retrieveConfigButton);
    lv_obj_set_width(ui_retrieveConfigText, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_retrieveConfigText, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_retrieveConfigText, LV_ALIGN_CENTER);
    lv_label_set_text(ui_retrieveConfigText, "Get config");

    lv_obj_add_event_cb(ui_submitConfigButton, ui_event_submitConfigButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_returnButton, ui_event_returnButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_retrieveConfigButton, ui_event_retrieveConfigButton, LV_EVENT_ALL, NULL);

}