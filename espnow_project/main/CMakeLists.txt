set(SOURCES espnow_project_main.c)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    )
