add_library(Reconstruction
  # list of cpp source files:
  reconstruct.cpp
  )

target_include_directories(Reconstruction PUBLIC
  # list of directories:
  .
  )