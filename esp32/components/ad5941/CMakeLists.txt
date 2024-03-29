idf_component_register(SRCS "ElectrodermalActivity.c" "AD5940Main.c" "ad5941.c" "AD5941toESP32port.c" "ad5940.c"
                    INCLUDE_DIRS "include")
add_compile_definitions(CHIPSEL_594X)