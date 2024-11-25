add_library(DataCapture
  # list of cpp source files:
  data_capture.cpp
  )

target_include_directories(DataCapture PUBLIC
  # list of directories:
  .
  )