idf_component_register(
    SRCS "tmp102.c" "modules/tmp102_i2c.c"     
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES "driver"
)