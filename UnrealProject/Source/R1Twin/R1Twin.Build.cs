// R1Twin runtime module rules (UE 5.8).

using System;
using System.Diagnostics;
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

		// Behaviour panel (AR1BehaviourPanel) talks to RobotControl's local behaviour service over HTTP/JSON.
		PrivateDependencyModuleNames.AddRange(new string[] { "HTTP", "Json", "UMG", "Slate", "SlateCore", "DeveloperSettings", "ImageWrapper" });  // UMG: hide URLab's debug widget; Slate: click input; ImageWrapper: twin head camera PNG

		// Voice commands (FR1VoiceSapi): Windows SAPI speech recognition + text to speech, no ATL / sphelper.h.
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.AddRange(new string[] { "sapi.lib", "ole32.lib" });
		}

		// One version stamp (Po Game Standard): UTC yyMMdd of the HEAD commit (web/build.py uses the same rule), else today (UTC).
		string Stamp = DateTime.UtcNow.ToString("yyMMdd");
		try
		{
			var Git = Process.Start(new ProcessStartInfo("git", "log -1 --format=%ct")
				{ WorkingDirectory = ModuleDirectory, RedirectStandardOutput = true, UseShellExecute = false, CreateNoWindow = true });
			string Out = Git.StandardOutput.ReadToEnd().Trim();
			Git.WaitForExit();
			long Secs;
			if (Git.ExitCode == 0 && long.TryParse(Out, out Secs)) Stamp = DateTimeOffset.FromUnixTimeSeconds(Secs).UtcDateTime.ToString("yyMMdd");
		}
		catch (Exception) {}
		PublicDefinitions.Add("R1_BUILD_VERSION=\"" + Stamp + "\"");

		// ---------------------------------------------------------------------------------------
		// UnrealRoboticsLab (URLab) plugin
		//
		// The plugin is vendored at UnrealProject/Plugins/UnrealRoboticsLab (see its README_VENDORED.md).
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
