cmake_minimum_required(VERSION 3.22)

project(HaldenVT4 VERSION 1.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
    set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "" FORCE)
endif()
if(APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "" FORCE)
endif()

# --- JUCE --------------------------------------------------------------------
set(JUCE_DIR "" CACHE PATH "Path to a local JUCE checkout (leave empty to download)")
if(JUCE_DIR)
    add_subdirectory(${JUCE_DIR} JUCE)
else()
    include(FetchContent)
    FetchContent_Declare(JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG        8.0.4
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(JUCE)
endif()

# --- Plugin ------------------------------------------------------------------
if(APPLE)
    set(VT4_DEFAULT_FORMATS AU VST3 Standalone)
else()
    set(VT4_DEFAULT_FORMATS VST3 Standalone)
endif()
set(VT4_FORMATS "${VT4_DEFAULT_FORMATS}" CACHE STRING "Plugin formats to build")
option(VT4_COPY_AFTER_BUILD "Install plugins into the user plugin folders after building" OFF)

juce_add_plugin(HaldenVT4
    COMPANY_NAME                "Halden Audio"
    BUNDLE_ID                   com.haldenaudio.vt4
    PLUGIN_MANUFACTURER_CODE    Hldn
    PLUGIN_CODE                 Hvt4
    FORMATS                     ${VT4_FORMATS}
    PRODUCT_NAME                "Halden VT-4"
    IS_SYNTH                    FALSE
    NEEDS_MIDI_INPUT            FALSE
    NEEDS_MIDI_OUTPUT           FALSE
    IS_MIDI_EFFECT              FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD     ${VT4_COPY_AFTER_BUILD}
    VST3_CATEGORIES             Fx EQ
    AU_MAIN_TYPE                kAudioUnitType_Effect)

juce_add_binary_data(HaldenVT4Data
    HEADER_NAME VT4BinaryData.h
    NAMESPACE   VT4BinaryData
    SOURCES
        Resources/fonts/Inter-Medium.otf
        Resources/fonts/Inter-SemiBold.otf
        Resources/fonts/InterDisplay-Light.otf
        Resources/fonts/InterDisplay-Medium.otf)

target_sources(HaldenVT4 PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
    Source/ui/LookAndFeel.cpp
    Source/ui/Faceplate.cpp
    Source/ui/Meters.cpp
    Source/ui/Screen.cpp
    Source/ui/RearPanel.cpp)

target_include_directories(HaldenVT4 PRIVATE Source)

target_compile_definitions(HaldenVT4 PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
    JUCE_DISPLAY_SPLASH_SCREEN=0)

# Exact floating-point comparisons (x == 0.0, cl == s) and int → size_t
# indexing are deliberate throughout the DSP code. Linked after JUCE's
# warning set so these flags come last on the command line.
add_library(vt4_warning_overrides INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vt4_warning_overrides INTERFACE -Wno-float-equal -Wno-sign-conversion)
endif()

target_link_libraries(HaldenVT4
    PRIVATE
        HaldenVT4Data
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
        vt4_warning_overrides)

# --- Measurement benches -----------------------------------------------------
# vt4_bench:         the DSP alone (no JUCE), every figure quoted in the README
# vt4_plugin_bench:  the real AudioProcessor (oversampling, SOLO, bypass, state)
option(VT4_BUILD_TESTS "Build the measurement benches" ON)
if(VT4_BUILD_TESTS)
    add_executable(vt4_bench tests/bench.cpp)
    target_include_directories(vt4_bench PRIVATE Source)
    target_compile_features(vt4_bench PRIVATE cxx_std_17)
    if(NOT MSVC)
        target_compile_options(vt4_bench PRIVATE -O3)
        find_package(Threads REQUIRED)
        target_link_libraries(vt4_bench PRIVATE Threads::Threads)
    endif()

    juce_add_console_app(vt4_plugin_bench PRODUCT_NAME "vt4_plugin_bench")
    target_sources(vt4_plugin_bench PRIVATE
        tests/plugin_bench.cpp
        Source/PluginProcessor.cpp
        Source/PluginEditor.cpp
        Source/ui/LookAndFeel.cpp
        Source/ui/Faceplate.cpp
        Source/ui/Meters.cpp
        Source/ui/Screen.cpp
        Source/ui/RearPanel.cpp)
    target_include_directories(vt4_plugin_bench PRIVATE Source)
    target_compile_definitions(vt4_plugin_bench PRIVATE
        JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0 JUCE_STANDALONE_APPLICATION=1 JUCE_MODAL_LOOPS_PERMITTED=1
        JucePlugin_Name="Halden VT-4")
    target_link_libraries(vt4_plugin_bench PRIVATE
        HaldenVT4Data juce::juce_audio_utils juce::juce_dsp
        juce::juce_recommended_config_flags)
endif()
