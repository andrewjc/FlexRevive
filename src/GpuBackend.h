#pragma once

#include <cstddef>

// Choosing between the CPU solver and a GPU one, and describing the device that decision was
// made about.
//
// A compute dispatch and the readback that follows it cost roughly the same whether they carry
// ten pieces or ten thousand, and the engine needs every transform back on the CPU before the
// frame can be drawn. So a GPU backend only pays for itself once there is enough work to
// amortise that round trip, and below it the CPU solver is simply faster.
//
// Every refusal carries a reason. A backend that quietly fails to engage is the hardest kind of
// problem to diagnose from a bug report.
namespace flexrevive::gpu {

// PCI vendor identifiers, as reported by the graphics adapter.
constexpr unsigned kVendorNvidia = 0x10DE;
constexpr unsigned kVendorAmd = 0x1002;
constexpr unsigned kVendorIntel = 0x8086;

// Below this many pieces the round trip costs more than the solve saves.
constexpr int kMinPiecesForGpu = 2048;

// The least device memory worth attempting a working set in.
constexpr size_t kMinDeviceMemory = 512ull * 1024 * 1024;

// What was learned about the graphics device.
struct Capability {
    bool present = false;          // an adapter was found at all
    bool computeCapable = false;   // supports compute at the level the solver needs
    unsigned vendorId = 0;
    unsigned deviceId = 0;
    size_t dedicatedMemory = 0;
    char description[128] = {};
};

bool IsNvidia(const Capability& cap);

// What the user asked for.
enum Mode {
    kModeOff = 0,     // never
    kModeAuto = 1,    // when the hardware and the workload both suit it
    kModeForce = 2,   // whenever a compute-capable device exists, whatever its vendor
};

// Reads the INI value. Anything unrecognised is off, since this comes from a file edited by
// hand and the safe answer is the one that changes nothing.
Mode ModeFromSetting(int value);

enum Backend {
    kBackendCpu = 0,
    kBackendGpu = 1,
};

enum Reason {
    kReasonDisabled = 0,
    kReasonNoDevice,
    kReasonNoCompute,
    kReasonNotEnoughMemory,
    kReasonUntestedVendor,
    kReasonTooFewPieces,
    kReasonSelected,
    kReasonForced,
};

// A sentence naming why, for the log. Never null, including for an unknown value.
const char* ReasonText(Reason reason);

struct Choice {
    Backend backend = kBackendCpu;
    Reason reason = kReasonDisabled;
};

// Decides which backend runs this step.
//
// Forcing skips the vendor and workload tests, since the user has asked deliberately, but it
// cannot conjure hardware: without a usable device the choice still falls back to the CPU.
Choice Select(const Capability& cap, Mode mode, int pieceCount, int minPiecesForGpu);

// Queries the graphics adapters present and returns the most capable one. Safe to call when no
// graphics device exists; the result simply reports nothing present.
Capability Probe();

}
