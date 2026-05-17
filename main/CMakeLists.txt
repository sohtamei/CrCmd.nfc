idf_component_register(
  SRCS "CrCmd.nfc.cpp"
  INCLUDE_DIRS "."
  REQUIRES usb esp_driver_gpio esp_driver_uart driver m5unit_nfc_idf
)
