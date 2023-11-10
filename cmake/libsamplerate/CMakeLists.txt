include(FetchContent)

# set(LIBSAMPLERATE_EXAMPLES OFF)
set(BUILD_TESTING OFF)
FetchContent_Declare(
  libsamplerate
  GIT_REPOSITORY git@github.com:libsndfile/libsamplerate.git
  GIT_TAG 0.2.2
  GIT_SHALLOW ON
)

FetchContent_MakeAvailable(libsamplerate)
