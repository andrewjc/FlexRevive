cmake_minimum_required(VERSION 3.20)
project(FlexRevive VERSION 1.1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static runtime: the DLL is loaded by a script extender into a process that may not have a
# Visual C++ redistributable installed.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

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
    src/version.rc
)

target_include_directories(FlexRevive PRIVATE src)

# Windows.h defines min/max as macros, which breaks std::min/std::max in the solver.
target_compile_definitions(FlexRevive PRIVATE NOMINMAX)

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
