#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "EdGraph/EdGraphNodeUtils.h"
#include "AnimNode_LMM.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "AnimGraphNode_LMM.generated.h"

UCLASS()
class TESTING_API UAnimGraphNode_LMM : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_UCLASS_BODY()
	
	UPROPERTY(EditAnywhere, Category=Settings)
	FAnimNode_LMM Node;

public:
	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	// End of UEdGraphNode interface

protected:

	// UAnimGraphNode_SkeletalControlBase interface
	virtual void ValidateAnimNodeDuringCompilation(USkeleton* ForSkeleton, FCompilerResultsLog& MessageLog) override;
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface
	
};
