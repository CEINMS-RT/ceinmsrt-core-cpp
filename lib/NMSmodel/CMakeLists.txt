add_library(NMSmodel STATIC
        NMSmodel.cpp
        MuscleParameters.cpp
        Activation/ExponentialActivation.cpp
        Activation/PiecewiseActivation.cpp
        Activation/ExponentialActivationRT.cpp)

target_link_libraries(NMSmodel
        XSD::XSD
        Boost::timer)

target_compile_features(NMSmodel PUBLIC cxx_std_17)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_libraries(NMSmodel stdc++fs)
endif()

