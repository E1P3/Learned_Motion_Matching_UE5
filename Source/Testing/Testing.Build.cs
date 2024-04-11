// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class Testing : ModuleRules
{
	public Testing(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		AddEngineThirdPartyPrivateStaticDependencies(Target,
			"Eigen",
			"nanoflann"
		);

		PublicDependencyModuleNames.AddRange(
			new string[] 
			{ 
				"Core", 
				"CoreUObject", 
				"Engine", 
				"AnimationCore",
				"InputCore", 
				"EnhancedInput", 
				"AnimGraphRuntime",
				"StructUtils",
				"NNE",
				"PoseSearch",
				"MotionTrajectory",
				"BlueprintGraph"
			}
		);

		if (Target.bCompileAgainstEditor)
		{
			PrivateDependencyModuleNames.AddRange(
				new string[] {
					"DerivedDataCache",
					"UnrealEd"
				}
			);
		}
	}
}
