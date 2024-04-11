#include "AnimGraphNode_Learned_MM.h"
#include "AnimationGraphSchema.h"

#define LOCTEXT_NAMESPACE "AnimGraphNode_Learned_MM"

FLinearColor UAnimGraphNode_Learned_MM::GetNodeTitleColor() const
{
    return FColor(86, 182, 194);
}

FText UAnimGraphNode_Learned_MM::GetTooltipText() const
{
    return LOCTEXT("NodeToolTip", "Motion matching using learned data");
}

FText UAnimGraphNode_Learned_MM::GetMenuCategory() const
{
    return LOCTEXT("NodeCategory", "Learned Motion Matching");
}

FText UAnimGraphNode_Learned_MM::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
    return LOCTEXT("NodeTitle", "Learned Motion Matching");
}

#undef LOCTEXT_NAMESPACE