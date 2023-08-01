# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

set(SOURCES ./src/main.cpp)

idf_component_register(
    SRCS ${SOURCES}         # list the source files of this component
    INCLUDE_DIRS  . inc      # optional, add here public include directories
    REQUIRES  esp_adc driver freertos esp_common esp_wifi esp_timer  nvs_flash esp_netif esp_event lwip json esp_http_client esp_https_ota app_update efuse  # optional, list the public requirements (component names)
)

