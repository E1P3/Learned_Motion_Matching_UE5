// Copyright Epic Games, Inc. All Rights Reserved.

#include "FeatureExtractionCommands.h"

#define LOCTEXT_NAMESPACE "FFeatureExtractionModule"

void FFeatureExtractionCommands::RegisterCommands()
{
	UI_COMMAND(OpenPluginWindow, "FeatureExtraction", "Bring up FeatureExtraction window", EUserInterfaceActionType::Button, FInputChord());
}

#undef LOCTEXT_NAMESPACE
