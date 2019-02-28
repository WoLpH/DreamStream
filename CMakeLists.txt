cmake_minimum_required(VERSION 3.13)
project(dreamstream_nix)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(dreamstream_nix
        src/Command.h
        src/Commands.h
        src/CommandsList.h
        src/Defines.h
        src/Device.cpp
        src/Device.h
        src/DeviceDreamScreen.cpp
        src/DeviceDreamScreen.h
        src/DeviceEmulator.cpp
        src/DeviceEmulator.h
        src/DeviceManager.cpp
        src/DeviceManager.h
        src/DeviceSideKick.cpp
        src/DeviceSideKick.h
        src/DeviceType.h
        src/Enums.h
        src/main.cpp
        src/Packet.cpp
        src/Packet.h
        src/PacketHelpers.h
        src/StaticString.h
        src/Timer.h
        src/UDPClient.cpp
        src/UDPClient.h
        src/Utils.cpp
        src/Utils.h)
