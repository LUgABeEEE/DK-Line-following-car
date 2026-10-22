cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME linefllowing_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Drivers/BSD/Src/PWM.c
        Drivers/BSD/Inc/PWM.h
        Drivers/BSD/Src/Encoder.c
        Drivers/BSD/Inc/Encoder.h
        Drivers/BSD/Src/SpeedPID.c
        Drivers/BSD/Inc/SpeedPID.h
        Drivers/BSD/Src/Gray.c
        Drivers/BSD/Inc/Gray.h
        Drivers/BSD/Src/LinePD.c
        Drivers/BSD/Inc/LinePD.h
        Drivers/BSD/Src/Led.c
        Drivers/BSD/Inc/Led.h
        Drivers/BSD/Src/Key.c
        Drivers/BSD/Inc/Key.h
        Drivers/BSD/Src/Ble.c
        Drivers/BSD/Inc/Ble.h
        Drivers/BSD/Src/Oled.c
        Drivers/BSD/Inc/Oled.h


)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Drivers/BSD/Src/Ble.c
        Drivers/BSD/Src/Oled.c
        #Freertos kernel sources
        FreeRTOS/Src/croutine.c
        FreeRTOS/Src/event_groups.c
        FreeRTOS/Src/list.c
        FreeRTOS/Src/port.c
        FreeRTOS/Src/queue.c
        FreeRTOS/Src/stream_buffer.c
        FreeRTOS/Src/tasks.c
        FreeRTOS/Src/timers.c
        FreeRTOS/Src/heap_4.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Drivers/BSD/Inc
        # --- FreeRTOS kernel headers ---
        FreeRTOS/Inc
        # FreeRTOSConfig.h is in Core/Inc (already provided via stm32cubemx)
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
