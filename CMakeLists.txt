cmake_minimum_required(VERSION 3.20)
project(FlexRevive VERSION 1.1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static runtime: the DLL is loaded by a script extender into a process that may not have a
# Visual C++ redistributable installed.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Compute shaders are compiled to C headers at build time and embedded in the DLL.
#
# Nothing is compiled at runtime, so the plugin never needs d3dcompiler_47.dll on a user's
# machine, and a shader that does not build breaks the build rather than the game. fxc ships
# with the Windows SDK, which anyone building this already has; without it the GPU backend is
# left out and the CPU path is the whole plugin.
find_program(FXC_EXECUTABLE fxc
    HINTS ${CMAKE_VS_WINDOWS_TARGET_PLATFORM_VERSION}
    PATHS "$ENV{WindowsSdkVerBinPath}x64" "$ENV{WindowsSdkDir}bin/x64"
          "C:/Program Files (x86)/Windows Kits/10/bin/10.0.22621.0/x64"
          "C:/Program Files (x86)/Windows Kits/10/bin/10.0.19041.0/x64"
    DOC "HLSL compiler from the Windows SDK")

set(FLEXREVIVE_SHADER_HEADERS "")
if(FXC_EXECUTABLE)
    set(SHADER_OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders)
    file(MAKE_DIRECTORY ${SHADER_OUT})
    # Each name yields shaders/<name>.hlsl -> <name>_cs.h, declaring g_<name>CS. The generated
    # header is what declares the blob, so nothing has to restate its length by hand.
    set(FLEXREVIVE_SHADERS integrate)
    foreach(name ${FLEXREVIVE_SHADERS})
        add_custom_command(
            OUTPUT ${SHADER_OUT}/${name}_cs.h
            # /WX because a shader warning is a physics bug waiting to happen, and there are
            # few enough of these that none of them may be noisy.
            COMMAND ${FXC_EXECUTABLE} /nologo /T cs_5_0 /E main /O3 /WX
                    /Fh ${SHADER_OUT}/${name}_cs.h /Vn g_${name}CS
                    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.hlsl
            DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${name}.hlsl
            COMMENT "Compiling shaders/${name}.hlsl"
            VERBATIM)
        list(APPEND FLEXREVIVE_SHADER_HEADERS ${SHADER_OUT}/${name}_cs.h)
    endforeach()
else()
    message(WARNING "fxc not found: building without the GPU backend, ComputeBackend=gpu "
                    "will report it is unavailable and the CPU path will be used")
endif()

add_library(FlexRevive SHARED
    src/main.cpp
    src/Collision.cpp
    src/Config.cpp
    src/Contact.cpp
    src/f4kit/CrashLog.cpp
    src/f4kit/EngineSetting.cpp
    src/Fragment.cpp
    src/f4kit/ImportHook.cpp
    src/f4kit/Ini.cpp
    src/Math3D.cpp
    src/DebrisSolver.cpp
    src/f4kit/Log.cpp
    src/PairContact.cpp
    src/PieceGrid.cpp
    src/ParticleMap.cpp
    src/f4kit/PeImage.cpp
    src/Response.cpp
    src/Shock.cpp
    src/SlotTracker.cpp
    src/f4kit/ThreadPool.cpp
    src/gpu/Device.cpp
    src/gpu/GpuSolver.cpp
    src/version.rc
    ${FLEXREVIVE_SHADER_HEADERS}
)

target_include_directories(FlexRevive PRIVATE src)

# Windows.h defines min/max as macros, which breaks std::min/std::max in the solver.
target_compile_definitions(FlexRevive PRIVATE NOMINMAX)

if(FXC_EXECUTABLE)
    target_include_directories(FlexRevive PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/shaders)
    target_compile_definitions(FlexRevive PRIVATE FLEXREVIVE_HAVE_GPU=1)
endif()

# d3d11 and dxgi are loaded rather than linked, so the plugin still loads on a machine where
# they are missing or broken; only the import libraries for the types are needed.
target_link_libraries(FlexRevive PRIVATE shell32 ole32 dbghelp)

set_target_properties(FlexRevive PROPERTIES
    OUTPUT_NAME "FlexRevive"
    PREFIX ""
)

if(MSVC)
    # /Zi with /DEBUG emits a PDB without changing codegen; /OPT:REF /OPT:ICF restore the
    # linker folding /DEBUG would otherwise disable. The PDB stays with the build rather than
    # shipping, so a crash report resolves plugin frames only against a matching local copy.
    target_compile_options(FlexRevive PRIVATE /W4 /permissive- /Zi
                           $<$<CONFIG:Release>:/GL>)
    target_link_options(FlexRevive PRIVATE /DEBUG /OPT:REF /OPT:ICF
                        $<$<CONFIG:Release>:/LTCG>)
endif()

# Stage a ready-to-zip mod layout. The archive root is the game's Data folder, which is what
# Vortex and Mod Organizer 2 expect from an F4SE plugin.
set(STAGE ${CMAKE_CURRENT_SOURCE_DIR}/dist/F4SE/Plugins)
add_custom_command(TARGET FlexRevive POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${STAGE}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:FlexRevive> ${STAGE}/FlexRevive.dll
    COMMENT "Staging FlexRevive.dll into dist/F4SE/Plugins"
)

# Tests. The plugin's physics core is deliberately free of Windows and engine dependencies,
# so it compiles into a plain executable and is tested directly. Each suite is its own binary
# and runs on every build.
option(FLEXREVIVE_TESTS "Build and run the unit tests" ON)
if(FLEXREVIVE_TESTS)
    function(flexrevive_test name)
        add_executable(${name} tests/${name}.cpp tests/LogStub.cpp ${ARGN})
        target_include_directories(${name} PRIVATE src tests)
        target_compile_definitions(${name} PRIVATE NOMINMAX)
        if(MSVC)
            target_compile_options(${name} PRIVATE /W4 /permissive-)
        endif()
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${name}
            COMMENT "Running ${name}")
    endfunction()

    flexrevive_test(test_math3d   src/Math3D.cpp)
    flexrevive_test(test_collision src/Collision.cpp)
    flexrevive_test(test_contact   src/Contact.cpp src/Math3D.cpp src/Fragment.cpp)
    flexrevive_test(test_shock     src/Shock.cpp src/Response.cpp)
    flexrevive_test(test_response  src/Response.cpp)
    flexrevive_test(test_particlemap src/ParticleMap.cpp)
    flexrevive_test(test_paircontact src/PairContact.cpp src/Response.cpp)
    flexrevive_test(test_slottracker src/SlotTracker.cpp src/Math3D.cpp)
    flexrevive_test(test_piecegrid src/PieceGrid.cpp)
    flexrevive_test(test_imports  src/f4kit/ImportHook.cpp)
    flexrevive_test(test_threadpool src/f4kit/ThreadPool.cpp)

    # The GPU suite talks to a real device, so it needs the shader headers and skips itself
    # cleanly on a machine that has no capable adapter.
    if(FXC_EXECUTABLE)
        flexrevive_test(test_gpu src/gpu/Device.cpp src/gpu/GpuSolver.cpp src/Response.cpp
                        ${FLEXREVIVE_SHADER_HEADERS})
        target_include_directories(test_gpu PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/shaders)
        target_compile_definitions(test_gpu PRIVATE FLEXREVIVE_HAVE_GPU=1)
    endif()
    flexrevive_test(test_fragment src/Fragment.cpp)
    flexrevive_test(test_scene src/PairContact.cpp src/Response.cpp)
    flexrevive_test(test_terrain src/Collision.cpp src/Math3D.cpp)

    # Benchmarks. Built but not run automatically: they measure, they do not assert, and a
    # timing figure has no business failing someone's build.
    add_executable(bench tests/bench.cpp tests/LogStub.cpp
                   src/Collision.cpp src/Contact.cpp src/Fragment.cpp src/Math3D.cpp
                   src/Response.cpp src/Shock.cpp)
    target_include_directories(bench PRIVATE src tests)
    target_compile_definitions(bench PRIVATE NOMINMAX)
    if(MSVC)
        target_compile_options(bench PRIVATE /W4 /permissive-)
    endif()
endif()
