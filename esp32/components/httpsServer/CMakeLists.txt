idf_component_register(SRCS "websocketClient.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_websocket_client json led)
