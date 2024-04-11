// Fill out your copyright notice in the Description page of Project Settings.


#include "SFeatureExtractionMenu.h"
#include "SlateOptMacros.h"

BEGIN_SLATE_FUNCTION_BUILD_OPTIMIZATION
void SFeatureExtractionMenu::Construct(const FArguments& InArgs)
{
	ChildSlot
	[
		SNew(SHorizontalBox)
		+SHorizontalBox::Slot()
		.VAlign(VAlign_Top)
		[
			SNew(STextBlock)
			.Text(FText::FromString(FString::Printf(TEXT("Test Button Pressed: %d"), TestInt)))
		]
		+SHorizontalBox::Slot()
		.VAlign(VAlign_Top)
		[
			SNew(SButton)
			.Text(FText::FromString("Press Me"))
			.OnClicked(FOnClicked::CreateSP(this, &SFeatureExtractionMenu::OnTestButtonClicked))
		]
	];
}

FReply SFeatureExtractionMenu::OnTestButtonClicked()
{
	TestInt++;
	UE_LOG(LogTemp, Warning, TEXT("Test Button Pressed: %d"), TestInt);
	return FReply::Handled();
}

END_SLATE_FUNCTION_BUILD_OPTIMIZATION
