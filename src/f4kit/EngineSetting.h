#pragma once

#include <cstdint>

// Locates the engine's static Setting objects by name, so the plugin can read and write
// them at runtime.
//
// Setting layout: vptr @0x00, data @0x08, name @0x10. An object is found by locating its
// name string, then finding the static object whose name pointer references it; a candidate
// is accepted only if its vptr lands in .rdata.
namespace f4kit::engine {

union Data {
    uint32_t u32;
    int32_t s32;
    float f32;
    uint8_t u8; // bool
    char* s;
};

struct Binding {
    const char* fullName; // e.g. "bNVFlexEnable:NVFlex"
    Data* data;           // filled in by Resolve; null if not found
    // How many objects in the image carry this name. More than one means the image holds
    // duplicates and the first was taken, which may not be the one the engine reads.
    int candidates;
};

// Resolves every binding in one pass over the image. Returns how many were found.
int Resolve(Binding* bindings, int count);

bool GetBool(const Binding& b, bool& out);
bool SetBool(Binding& b, bool value);
bool GetInt(const Binding& b, int32_t& out);
bool SetInt(Binding& b, int32_t value);
bool GetFloat(const Binding& b, float& out);
bool SetFloat(Binding& b, float value);

}
