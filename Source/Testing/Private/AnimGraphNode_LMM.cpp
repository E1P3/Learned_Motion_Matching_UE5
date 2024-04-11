#include "AnimGraphNode_LMM.h"
#include "AnimNodeEditModes.h"
#include "Kismet2/CompilerResultsLog.h"
#include "UnrealWidgetFwd.h"

#define LOCTEXT_NAMESPACE "A3Nodes"

UAnimGraphNode_LMM::UAnimGraphNode_LMM(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

FText UAnimGraphNode_LMM::GetControllerDescription() const
{
    return LOCTEXT("LMM implementation for Unreal", "Learned Motion Matching");
}

FText UAnimGraphNode_LMM::GetTooltipText() const
{
    return LOCTEXT("AnimGraphNode_LMM_Tooltip", "Feeds current positions into a neural network and applies the output to the skeleton");
}

FText UAnimGraphNode_LMM::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
    return GetControllerDescription();
}

void UAnimGraphNode_LMM::ValidateAnimNodeDuringCompilation(USkeleton* ForSkeleton, FCompilerResultsLog& MessageLog)
{
    if (Node.BoneReferences.Num() == 0)
    {
        MessageLog.Warning(*LOCTEXT("NoBonesSelected", "@@ - You must pick at least one bone to modify").ToString(), this);
    }

    if (!ForSkeleton)
    {
        MessageLog.Warning(TEXT("Skeleton is not valid or fully loaded"), this);
        return;
    }

    if (ForSkeleton && !ForSkeleton->HasAnyFlags(RF_NeedPostLoad))
	{
        for(FBoneReference& BoneRef : Node.BoneReferences)
        {
            if (ForSkeleton->GetReferenceSkeleton().FindBoneIndex(BoneRef.BoneName) == INDEX_NONE)
		    {
                if (BoneRef.BoneName == NAME_None)
                {
                    MessageLog.Warning(*LOCTEXT("BoneRefNone", "@@ - You must pick a bone to modify").ToString(), this);
                }
            }
            else{
                FFormatNamedArguments Args;
				Args.Add(TEXT("BoneName"), FText::FromName(BoneRef.BoneName));

				FText Msg = FText::Format(LOCTEXT("NoBoneFoundToModify", "@@ - Bone {BoneName} not found in Skeleton"), Args);

				MessageLog.Warning(*Msg.ToString(), this);
            }

        }
    }

    Super::ValidateAnimNodeDuringCompilation(ForSkeleton, MessageLog);
}

#undef LOCTEXT_NAMESPACE
