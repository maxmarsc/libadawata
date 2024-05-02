include(FetchContent)

FetchContent_Declare(
  xsimd
  GIT_REPOSITORY git@github.com:xtensor-stack/xsimd.git
  GIT_TAG 13.0.0
  GIT_SHALLOW ON
)

FetchContent_MakeAvailable(xsimd)
