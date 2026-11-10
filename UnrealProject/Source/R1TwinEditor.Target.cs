// R1Twin editor target. Written for UE 5.7; NOT YET COMPILED on this machine (see docs/ue5_setup.md).

using UnrealBuildTool;
using System.Collections.Generic;

public class R1TwinEditorTarget : TargetRules
{
	public R1TwinEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;

		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;

		ExtraModuleNames.AddRange(new string[] { "R1Twin" });
	}
}
