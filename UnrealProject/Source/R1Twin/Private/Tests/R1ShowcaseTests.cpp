// Automation tests for the showcase helpers (Session Frontend > Automation, or
// UnrealEditor-Cmd ... -ExecCmds="Automation RunTests R1Twin; Quit" WITHOUT -nullrhi, see AGENTS.md section 3).
#include "Misc/AutomationTest.h"
#include "MjUeConvert.h"
#include "R1IdleClock.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FR1IdleClockIgnoresLongFramesTest, "R1Twin.Showcase.IdleClockIgnoresLongFrames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::ProductFilter)

bool FR1IdleClockIgnoresLongFramesTest::RunTest(const FString& Parameters)
{
	FR1IdleClock Clock;
	// 29.9 s of normal 60 fps frames
	for (int32 i = 0; i < 1794; ++i) Clock.Advance(1.f / 60.f);
	TestFalse(TEXT("29.9 s of frames is not yet 30 s idle"), Clock.Reached(30.f));
	// one 5 s hitch (shader warm-up / level load) must not count
	TestFalse(TEXT("a 5 s frame is ignored"), Clock.Advance(5.f));
	TestFalse(TEXT("a 0.26 s frame is ignored"), Clock.Advance(0.26f));
	TestFalse(TEXT("still below 30 s after long frames"), Clock.Reached(30.f));
	TestTrue(TEXT("a 0.25 s frame counts (boundary)"), Clock.Advance(0.25f));
	TestTrue(TEXT("30 s reached with normal frames only"), Clock.Reached(30.f));
	Clock.Reset();
	TestEqual(TEXT("input resets the clock"), Clock.Seconds, 0.f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FR1MjUeConvertSelfTest, "R1Twin.Convert.MjUeConvertSelfTest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::ProductFilter)

bool FR1MjUeConvertSelfTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("MuJoCo <-> Unreal frame conversion round-trips"), MjUe::RunSelfTest());
	return true;
}

#endif
