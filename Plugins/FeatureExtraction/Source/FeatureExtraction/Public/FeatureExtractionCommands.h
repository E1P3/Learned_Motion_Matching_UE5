// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"
#include "FeatureExtractionStyle.h"

class FFeatureExtractionCommands : public TCommands<FFeatureExtractionCommands>
{
public:

	FFeatureExtractionCommands()
		: TCommands<FFeatureExtractionCommands>(TEXT("FeatureExtraction"), NSLOCTEXT("Contexts", "FeatureExtraction", "FeatureExtraction Plugin"), NAME_None, FFeatureExtractionStyle::GetStyleSetName())
	{
	}

	// TCommands<> interface
	virtual void RegisterCommands() override;

public:
	TSharedPtr< FUICommandInfo > OpenPluginWindow;
};