// R1Twin runtime module rules. Written for UE 5.7; NOT YET COMPILED on this machine (see docs/ue5_setup.md).

using System.IO;
using UnrealBuildTool;

public class R1Twin : ModuleRules
{
	public R1Twin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"EnhancedInput",
			// NNE is an engine runtime module (Engine/Source/Runtime/NNE) since UE 5.3.
			// The ORT backend ("NNERuntimeORTCpu") comes from the NNERuntimeORT *plugin*
			// enabled in R1Twin.uproject; it is discovered at runtime by name, so no link dependency.
			"NNE",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
		});

		// ---------------------------------------------------------------------------------------
		// UnrealRoboticsLab (URLab) plugin
		//
		// The plugin lives at UnrealProject/Plugins/UnrealRoboticsLab (git submodule, git-ignored).
		// TODO verify: the module name below must match the "Modules" list in
		//   Plugins/UnrealRoboticsLab/UnrealRoboticsLab.uplugin
		// (expected "UnrealRoboticsLab"; the plugin may also split editor-only modules, e.g.
		//  "UnrealRoboticsLabEditor", which must NOT be added to a Runtime module).
		//
		// The dependency is added only when the plugin directory is present so the project still
		// compiles (with the mock articulation bridge) before the submodule has been cloned.
		// ---------------------------------------------------------------------------------------
		string ProjectDir = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
		string UrlabPluginDir = Path.Combine(ProjectDir, "Plugins", "UnrealRoboticsLab");
		string UrlabPluginDescriptor = Path.Combine(UrlabPluginDir, "UnrealRoboticsLab.uplugin");

		bool bHaveUrlab = File.Exists(UrlabPluginDescriptor);
		if (bHaveUrlab)
		{
			PrivateDependencyModuleNames.Add("URLab"); // verified: UnrealRoboticsLab.uplugin Modules = URLab (Runtime), URLabEditor (Editor)
			PublicDefinitions.Add("R1_WITH_URLAB=1");
		}
		else
		{
			PublicDefinitions.Add("R1_WITH_URLAB=0");
		}

		// Flip to 1 ONLY after the accessor names used in Private/R1ArticulationBridge.cpp
		// (FR1UrlabBridge) have been checked against the real AMjArticulation / UMjComponent /
		// UMjPDController headers. Until then FR1UrlabBridge compiles but reports "not connected"
		// and UR1PolicyComponent falls back to FR1MockBridge.
		PublicDefinitions.Add(bHaveUrlab ? "R1_URLAB_API_VERIFIED=1" : "R1_URLAB_API_VERIFIED=0"); // verified: URLab Aug 2026, UE 5.8.4
	}
}
