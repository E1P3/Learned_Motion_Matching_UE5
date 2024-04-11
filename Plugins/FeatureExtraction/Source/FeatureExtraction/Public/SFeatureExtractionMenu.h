// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "PoseSearch/PoseSearchDatabase.h"

/**
 * 
 */
class FEATUREEXTRACTION_API SFeatureExtractionMenu : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFeatureExtractionMenu)
	{}
	SLATE_END_ARGS()

	/** Constructs this widget with InArgs */
	void Construct(const FArguments& InArgs);
	FReply OnTestButtonClicked();
protected:
	FText Info;
	int32 TestInt = 0;
};
