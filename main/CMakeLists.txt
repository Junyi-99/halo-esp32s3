idf_component_register(
    SRCS "main.c" "radio.c" "board_led.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_phy nvs_flash esp_timer esp_driver_usb_serial_jtag esp_driver_rmt esp_hw_support)

# Reserve the modem SRAM banks and intercept two private PHY calls (see radio.c).
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/sram_guard.ld"
    "-Wl,--wrap=chip_v7_set_chan_ana"
    "-Wl,--wrap=set_rx_gain_cal_dc")
