#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "AnimGraphNode_Base.h"
#include "AnimNode_Learned_MM.h"
#include "AnimGraphNode_Learned_MM.generated.h"

UCLASS(BlueprintType)
class TESTING_API UAnimGraphNode_Learned_MM : public UAnimGraphNode_Base
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category=Settings)
	FAnimNode_Learned_MM Node;

    virtual FLinearColor GetNodeTitleColor() const override;
	virtual FText GetTooltipText() const override;
	virtual FText GetMenuCategory() const override;
    virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	
};