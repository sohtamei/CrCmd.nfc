idf_component_register(
  SRCS "CrCmd.nfc.cpp" "PN532.cpp" "PN532_I2C.cpp"
  INCLUDE_DIRS "."
  REQUIRES usb esp_driver_gpio esp_driver_uart driver
)
