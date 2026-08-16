// Deciding which of a module's imports to redirect, and remembering what they pointed at.
//
// The walk over the executable's import tables cannot be exercised without a live process, but
// the decisions it makes can: which name maps to which replacement, which names are deliberately
// left alone, and how a shim later finds the original it needs to forward to.

#include "f4kit/ImportHook.h"
#include "TestHarness.h"

using namespace f4kit;

#include <cstring>

using namespace f4kit::imports;

namespace {

int Dummy1() { return 1; }
int Dummy2() { return 2; }
int Dummy3() { return 3; }

const Redirect kTable[] = {
    {"someInit", reinterpret_cast<void*>(&Dummy1)},
    {"someUpdate", reinterpret_cast<void*>(&Dummy2)},
    {"someShutdown", reinterpret_cast<void*>(&Dummy3)},
};
constexpr int kCount = int(sizeof(kTable) / sizeof(kTable[0]));

} // namespace

static void TestFindingReplacements()
{
    test::Suite("matching an imported name");

    CHECK(Find("someUpdate", kTable, kCount, nullptr, 0) == &kTable[1]);
    CHECK(Find("someInit", kTable, kCount, nullptr, 0) == &kTable[0]);

    // A name the table does not carry is left alone rather than matched loosely.
    CHECK(Find("somethingElse", kTable, kCount, nullptr, 0) == nullptr);
    CHECK(Find("someUpdat", kTable, kCount, nullptr, 0) == nullptr);
    CHECK(Find("someUpdateExtra", kTable, kCount, nullptr, 0) == nullptr);

    // Matching is case sensitive, as exported names are.
    CHECK(Find("SomeUpdate", kTable, kCount, nullptr, 0) == nullptr);

    // Nothing to match against, and nothing to match.
    CHECK(Find("someInit", nullptr, 0, nullptr, 0) == nullptr);
    CHECK(Find(nullptr, kTable, kCount, nullptr, 0) == nullptr);
}

static void TestSkipList()
{
    test::Suite("imports deliberately left alone");

    // An interception can be opted out of, in which case the thunk keeps pointing at the
    // original and that call is untouched.
    const char* skip[] = {"someUpdate"};
    CHECK(Find("someUpdate", kTable, kCount, skip, 1) == nullptr);
    CHECK(Find("someInit", kTable, kCount, skip, 1) == &kTable[0]);

    // Skipping something the table never carried changes nothing.
    const char* absent[] = {"neverHeardOf"};
    CHECK(Find("someInit", kTable, kCount, absent, 1) == &kTable[0]);

    // Every entry can be skipped at once.
    const char* all[] = {"someInit", "someUpdate", "someShutdown"};
    for (int i = 0; i < kCount; ++i)
        CHECK(Find(kTable[i].name, kTable, kCount, all, 3) == nullptr);
}

static void TestOriginals()
{
    test::Suite("remembering the original targets");

    Originals originals;
    CHECK_EQ(originals.Count(), 0);
    CHECK(originals.For("someInit") == nullptr);

    void* a = reinterpret_cast<void*>(&Dummy1);
    void* b = reinterpret_cast<void*>(&Dummy2);

    CHECK(originals.Add("someInit", a));
    CHECK(originals.Add("someUpdate", b));
    CHECK_EQ(originals.Count(), 2);
    CHECK(originals.For("someInit") == a);
    CHECK(originals.For("someUpdate") == b);
    CHECK(originals.For("someShutdown") == nullptr);

    // A shim that only watches a call needs the address the thunk held, so a null target is
    // not worth recording and neither is a nameless one.
    CHECK(!originals.Add(nullptr, a));
    CHECK(!originals.Add("someShutdown", nullptr));
    CHECK_EQ(originals.Count(), 2);

    // A name already recorded is refused rather than overwritten: the first value is the one
    // the thunk genuinely held, before anything rewrote it.
    CHECK(!originals.Add("someInit", b));
    CHECK(originals.For("someInit") == a);
    CHECK_EQ(originals.Count(), 2);

    originals.Clear();
    CHECK_EQ(originals.Count(), 0);
    CHECK(originals.For("someInit") == nullptr);
}

static void TestOriginalsCapacity()
{
    test::Suite("the table has a bound");

    Originals originals;
    char names[128][16];
    int added = 0;
    for (int i = 0; i < 128; ++i) {
        snprintf(names[i], sizeof(names[i]), "fn%d", i);
        if (originals.Add(names[i], reinterpret_cast<void*>(&Dummy1)))
            ++added;
    }

    CHECK_EQ(added, originals.Count());
    CHECK(originals.Count() <= kMaxOriginals);
    CHECK(originals.For("fn0") != nullptr);          // the earliest survive
    CHECK(originals.For("fn127") == nullptr);        // the overflow is refused, not wrapped
}

static void TestModuleMatching()
{
    test::Suite("choosing which modules to touch");

    // The prefix is matched case-insensitively, since import descriptors carry whatever case
    // the linker recorded.
    CHECK(ModuleMatches("flexRelease_x64.dll", "flex"));
    CHECK(ModuleMatches("FLEXEXTRELEASE_X64.DLL", "flex"));
    CHECK(ModuleMatches("Flex.dll", "FLEX"));

    CHECK(!ModuleMatches("kernel32.dll", "flex"));
    CHECK(!ModuleMatches("fle.dll", "flex"));

    // An empty prefix would match every import in the process, so it matches nothing instead.
    CHECK(!ModuleMatches("anything.dll", ""));
    CHECK(!ModuleMatches("anything.dll", nullptr));
    CHECK(!ModuleMatches(nullptr, "flex"));
}

int main()
{
    printf("ImportHook\n");
    TestFindingReplacements();
    TestSkipList();
    TestOriginals();
    TestOriginalsCapacity();
    TestModuleMatching();
    return test::Report("ImportHook");
}
