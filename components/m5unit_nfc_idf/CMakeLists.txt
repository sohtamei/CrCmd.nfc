idf_component_register(SRCS "src/m5unit_nfc_idf.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
