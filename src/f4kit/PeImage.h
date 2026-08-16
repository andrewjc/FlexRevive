#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

// PE section walker for the running game executable. Every engine address the plugin uses is
// found by scanning these sections at runtime.
namespace f4kit::pe {

struct Section {
    const uint8_t* begin = nullptr;
    size_t size = 0;
    bool executable = false;
    bool writable = false;
    char name[9] = {};
};

// Walks the game executable's PE headers once. Safe to call repeatedly.
bool Init();

uintptr_t ImageBase();
size_t ImageSize();
const std::vector<Section>& Sections();

bool PointsIntoImage(uintptr_t p);

// True when p lands in a mapped, non-executable, non-writable section (.rdata), where MSVC
// places vtables. Used to validate candidate engine objects.
bool PointsIntoReadOnlyData(uintptr_t p);

}
