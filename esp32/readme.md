### Do not use it for medical purposes unless you know what you are doing.

Modify the `CMakeLists.txt` file in `components\wavelib\CMakeLists.txt` to
```cmake
set(src   "src/conv.c"
          "src/cwt.c"
          "src/cwtmath.c"
          "src/hsfft.c"
          "src/real.c"
          "src/wavefilt.c"
          "src/wavefunc.c"
          "src/wavelib.c"
          "src/wtmath.c"
          )

idf_component_register(
    SRCS src
    INCLUDE_DIRS "header" "src"
)
```