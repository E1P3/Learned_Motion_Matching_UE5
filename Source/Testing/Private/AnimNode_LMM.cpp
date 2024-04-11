#pragma once

#include "AnimNode_LMM.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include "Animation/AnimStats.h"
#include "Animation/AnimTrace.h"

FAnimNode_LMM::FAnimNode_LMM()
{
}

void FAnimNode_LMM::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(InitializeBoneReferences)

    const FReferenceSkeleton ReferenceSkeleton = RequiredBones.GetReferenceSkeleton();

    ControlledBoneNames.Add(TEXT("Root"));       
    ControlledBoneNames.Add(TEXT("Hips"));         
    ControlledBoneNames.Add(TEXT("LeftUpLeg"));    
    ControlledBoneNames.Add(TEXT("LeftLeg"));      
    ControlledBoneNames.Add(TEXT("LeftFoot"));     
    ControlledBoneNames.Add(TEXT("LeftToe"));      
    ControlledBoneNames.Add(TEXT("RightUpLeg"));   
    ControlledBoneNames.Add(TEXT("RightLeg"));     
    ControlledBoneNames.Add(TEXT("RightFoot"));    
    ControlledBoneNames.Add(TEXT("RightToe"));     
    ControlledBoneNames.Add(TEXT("Spine"));        
    ControlledBoneNames.Add(TEXT("Spine1"));       
    ControlledBoneNames.Add(TEXT("Spine2"));       
    ControlledBoneNames.Add(TEXT("Neck"));         
    ControlledBoneNames.Add(TEXT("Head"));         
    ControlledBoneNames.Add(TEXT("LeftShoulder")); 
    ControlledBoneNames.Add(TEXT("LeftArm"));      
    ControlledBoneNames.Add(TEXT("LeftForeArm"));  
    ControlledBoneNames.Add(TEXT("LeftHand"));     
    ControlledBoneNames.Add(TEXT("RightShoulder"));
    ControlledBoneNames.Add(TEXT("RightArm"));     
    ControlledBoneNames.Add(TEXT("RightForeArm")); 
    ControlledBoneNames.Add(TEXT("RightHand"));    

    UE_LOG(LogTemp, Warning, TEXT("Initialising Bones"));

    Hip.Initialize(RequiredBones);

    for(FBoneReference& BoneReference : TrackedBones)
    {
        BoneReference.Initialize(RequiredBones);
    }

    BoneReferences = TArray<FBoneReference>();

    for(int16 boneIndex = 0; boneIndex < 23; boneIndex++)
    {
        FBoneReference BoneReference = FBoneReference(ControlledBoneNames[boneIndex]);
        BoneReference.Initialize(RequiredBones);
        UE_LOG(LogTemp, Warning, TEXT("Bone Name: %s"), *BoneReference.BoneName.ToString());
        BoneReferences.Add(BoneReference);
    }

    TWeakInterfacePtr<INNERuntimeCPU> Runtime = UE::NNE::GetRuntime<INNERuntimeCPU>(FString("NNERuntimeORTCpu"));

    if(Runtime.IsValid())
    {
        if(Decompressor)
        {
            DecompressorInstance = new FModelInstance(Decompressor, Runtime, "/Import/decompressor.bin");
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Decompressor is null"));
        }
        if(Stepper)
        {
            StepperInstance = new FModelInstance(Stepper, Runtime, "/Import/stepper.bin");
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Stepper is null"));
        }
        if(Projector)
        {
            ProjectorInstance = new FModelInstance(Projector, Runtime, "/Import/projector.bin");
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Projector is null"));
        }
    } else {
        UE_LOG(LogTemp, Warning, TEXT("Runtime is null"));
    }
    CachedTrackedBonesPositions.Init(0, (TrackedBones.Num() + 1) * 3);
    PoseCurrent.Init(0, DecompressorInstance->OutputTensorShapes[0].Volume());
    FeaturesCurrent.Init(0, ProjectorInstance->InputTensorShapes[0].Volume());
    LatentCurrent.Init(0, StepperInstance->InputTensorShapes[0].Volume() - FeaturesCurrent.Num());
}

void FAnimNode_LMM::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(EvaluateSkeletalControl_AnyThread)
	check(OutBoneTransforms.Num() == 0);

    const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

    FTransform ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();
    TArray<float> features_curr = GetFeatures(BoneContainer, Output, ComponentTransform);

    //printTensorData("Features Current: ", features_curr);

    if(DecompressorInstance != nullptr || StepperInstance != nullptr || ProjectorInstance != nullptr){
        bool transition = false;
        ProjectorEvaluate(FeaturesCurrent, LatentCurrent, features_curr, transition);
        if(transition){
            DecompressorEvaluate(FeaturesCurrent, LatentCurrent, PoseCurrent);
        }

        StepperEvaluate(FeaturesCurrent, LatentCurrent);
        DecompressorEvaluate(FeaturesCurrent, LatentCurrent, PoseCurrent);
    }

    SetBoneTransforms(Output, OutBoneTransforms, BoneContainer, ComponentTransform, PoseCurrent);

}

void FAnimNode_LMM::SetBoneTransforms(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer, FTransform ComponentTransform, TArray<float> Pose){
    TArray<FCompactPoseBoneIndex> CompactPoseBoneIndices;
    TArray<FTransform> NewBoneTMs;

    for(int i = 0; i < BoneReferences.Num(); i++){
        CompactPoseBoneIndices.Add(BoneReferences[i].GetCompactPoseIndex(BoneContainer));
        NewBoneTMs.Add(Output.Pose.GetComponentSpaceTransform(CompactPoseBoneIndices[i]));
    }

    TArray<FVector> positions;
    TArray<FQuat> rotations;
    positions.Init(FVector(0, 0, 0), 23);
    rotations.Init(FQuat(0, 0, 0, 0), 23);

    ExtractPositionsAndRotations(PoseCurrent, positions, rotations);


    FString data = "";
    for(int i = 0; i < BoneReferences.Num() - 1; i++)
    {
        //data += ControlledBoneNames[i+1].ToString();
        //data += FString::Printf(TEXT("Model Position Output: %f, %f, %f\n"), positions[i].X, positions[i].Y, positions[i].Z);
        //data += FString::Printf(TEXT("Model Rotation Output: %f, %f, %f, %f\n"), rotations[i].X, rotations[i].Y, rotations[i].Z, rotations[i].W);
        //FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, NewBoneTMs[i], CompactPoseBoneIndices[i], BCS_ComponentSpace);
        //data += FString::Printf(TEXT("Bone Space: %f, %f, %f\n"), NewBoneTMs[i].GetTranslation().X, NewBoneTMs[i].GetTranslation().Y, NewBoneTMs[i].GetTranslation().Z);
        //NewBoneTMs[i].SetTranslation(positions[i]);
        //NewBoneTMs[i].SetRotation(rotations[i]);
        //FAnimationRuntime::ConvertBoneSpaceTransformToCS(ComponentTransform, Output.Pose, NewBoneTMs[i], CompactPoseBoneIndices[i], BCS_ComponentSpace);
    }
    //UE_LOG(LogTemp, Warning, TEXT("%s"), *data);

    for(int i = 0; i < BoneReferences.Num() - 1; i++)
    {
        OutBoneTransforms.Add(FBoneTransform(CompactPoseBoneIndices[i], NewBoneTMs[i]));
    }

    OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

bool FAnimNode_LMM::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
    for(FBoneReference& BoneReference : BoneReferences)
    {
        if(!BoneReference.IsValidToEvaluate(RequiredBones))
        {
            return false;
        }
    }
    return true;
}

void FAnimNode_LMM::GatherDebugData(FNodeDebugData& DebugData)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(GatherDebugData)
	FString DebugLine = DebugData.GetNodeName(this);

	DebugLine += "(";
	AddDebugNodeData(DebugLine);
    for(FBoneReference& BoneReference : BoneReferences)
    {
        DebugLine += FString::Printf(TEXT(" Target: %s)"), *BoneReference.BoneName.ToString());
    }
	DebugData.AddDebugItem(DebugLine);

	ComponentPose.GatherDebugData(DebugData);
}

TArray<float> FAnimNode_LMM::GetFeatures(const FBoneContainer& BoneContainer, FComponentSpacePoseContext& Output, FTransform ComponentTransform){
    
    TArray<float> features;
    for(FBoneReference& BoneReference : TrackedBones)
    {
        FCompactPoseBoneIndex CompactPoseBoneIndex = BoneReference.GetCompactPoseIndex(BoneContainer);
        FTransform BoneTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseBoneIndex);
        FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, BoneTransform, CompactPoseBoneIndex, BCS_ComponentSpace);
        features.Add((float)BoneTransform.GetTranslation().X / 100.0f);
        features.Add((float)BoneTransform.GetTranslation().Y / 100.0f);
        features.Add((float)BoneTransform.GetTranslation().Z / 100.0f);

    }

    for(int i = 0; i < TrackedBones.Num() * 3; i++)
    {
        features.Add(features[i] - CachedTrackedBonesPositions[i]);
        CachedTrackedBonesPositions[i] = features[i];
    }

    FCompactPoseBoneIndex CompactPoseBoneIndex = Hip.GetCompactPoseIndex(BoneContainer);
    FTransform HipTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseBoneIndex);
    FAnimationRuntime::ConvertCSTransformToBoneSpace(ComponentTransform, Output.Pose, HipTransform, CompactPoseBoneIndex, BCS_ComponentSpace);
    features.Add((float)ComponentTransform.GetTranslation().X - CachedTrackedBonesPositions[TrackedBones.Num() * 3]);
    features.Add((float)ComponentTransform.GetTranslation().Y - CachedTrackedBonesPositions[TrackedBones.Num() * 3 + 1]);
    features.Add((float)HipTransform.GetTranslation().Z - CachedTrackedBonesPositions[TrackedBones.Num() * 3 + 2]);
    CachedTrackedBonesPositions[TrackedBones.Num() * 3] = (float)ComponentTransform.GetTranslation().X;
    CachedTrackedBonesPositions[TrackedBones.Num() * 3 + 1] = (float)ComponentTransform.GetTranslation().Y;
    CachedTrackedBonesPositions[TrackedBones.Num() * 3 + 2] = (float)HipTransform.GetTranslation().Z;

    features.Append(GetTrajectoryData(ComponentTransform));

    for(int i = 0; i < 27; i++){
        if(i < 6){
            features[i] = features[i] * 0.75f;
        } else if( i > 20){
            features[i] = features[i] * 1.5f;
        } 
    }

    // FString dataString = "Features ";
    // for(int i = 0; i < features.Num(); i++)
    // {
    //     if(i == 0){
    //         dataString += FString::Printf(TEXT(" Positions: "));
    //     } else if(i == 6){
    //         dataString += FString::Printf(TEXT(" Velocities: "));
    //     } else if(i == 15){
    //         dataString += FString::Printf(TEXT(" Trajectory Positions: "));
    //     } else if(i == 21){
    //         dataString += FString::Printf(TEXT(" Trajectory Velocities: "));
    //     }
    //     dataString += FString::Printf(TEXT("%f, "), features[i]);
    // }
    // UE_LOG(LogTemp, Warning, TEXT("Features: %s"), *dataString);

    return features;
}

TArray<float> FAnimNode_LMM::GetTrajectoryData(FTransform& ComponentTransform){
    TArray<float> trajectoryData;
    if(Trajectory.Samples.Num() > 3)
    {
        TArray<FVector> trajectoryPositions;
        TArray<FVector> trajectoryDirections;

        trajectoryPositions.Add(getTrajectoryPositions(ComponentTransform, Trajectory.Samples[2], 100.0f));
        trajectoryPositions.Add(getTrajectoryPositions(ComponentTransform, Trajectory.Samples[3], 100.0f));
        trajectoryPositions.Add(getTrajectoryPositions(ComponentTransform, Trajectory.Samples[4], 100.0f));
        trajectoryDirections.Add(getTrajectoryDirection(ComponentTransform, Trajectory.Samples[2]));
        trajectoryDirections.Add(getTrajectoryDirection(ComponentTransform, Trajectory.Samples[3]));
        trajectoryDirections.Add(getTrajectoryDirection(ComponentTransform, Trajectory.Samples[4]));

        for(int i = 0; i < trajectoryPositions.Num(); i++)
        {
            trajectoryData.Add(trajectoryPositions[i].X);
            trajectoryData.Add(trajectoryPositions[i].Y);
        }

        for(int i = 0; i < trajectoryDirections.Num(); i++)
        {
            trajectoryData.Add(trajectoryDirections[i].X);
            trajectoryData.Add(trajectoryDirections[i].Y);
        }


        // Movement 400cm/s , 4 samples per second
        // trajectoryData.Add((Trajectory.Samples[2].Position.X - ComponentTransform.GetLocation().X) / 100.0f); 
        // trajectoryData.Add((Trajectory.Samples[2].Position.Y - ComponentTransform.GetLocation().Y) / 100.0f);
        // trajectoryData.Add((Trajectory.Samples[3].Position.X - ComponentTransform.GetLocation().X) / 100.0f);
        // trajectoryData.Add((Trajectory.Samples[3].Position.Y - ComponentTransform.GetLocation().Y) / 100.0f);
        // trajectoryData.Add((Trajectory.Samples[4].Position.X - ComponentTransform.GetLocation().X) / 100.0f);
        // trajectoryData.Add((Trajectory.Samples[4].Position.Y - ComponentTransform.GetLocation().Y) / 100.0f);

        // TArray<FVector> facing;
        // facing.Add(Trajectory.Samples[2].Facing.RotateVector(FVector(1, 0, 0)));
        // facing.Add(Trajectory.Samples[3].Facing.RotateVector(FVector(1, 0, 0)));
        // facing.Add(Trajectory.Samples[4].Facing.RotateVector(FVector(1, 0, 0)));

        // trajectoryData.Add(facing[0].X);
        // trajectoryData.Add(facing[0].Y);
        // trajectoryData.Add(facing[1].X);
        // trajectoryData.Add(facing[1].Y);
        // trajectoryData.Add(facing[2].X);
        // trajectoryData.Add(facing[2].Y);

    } else {
        UE_LOG(LogTemp, Warning, TEXT("Trajectory Samples are less than 5"));
        TArray<float> zeros;
        zeros.SetNumZeroed(12);
        trajectoryData.Append(zeros);
    }

    // FString dataString = "Trajectory";
    // for(int i = 0; i < trajectoryData.Num(); i++)
    // {
    //     dataString += FString::Printf(TEXT(" %f ") , trajectoryData[i]);
    // }
    // UE_LOG(LogTemp, Warning, TEXT("%s"), *dataString);

    return trajectoryData;
}

FVector FAnimNode_LMM::getTrajectoryPositions(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& TrajectorySample, float scalingFactor){
    FQuat rootRotation = ComponentTransform.GetRotation();
    FVector rootPosition = ComponentTransform.GetLocation();

    FVector trajectoryPosition = rootRotation.Inverse().RotateVector((TrajectorySample.Position - rootPosition) / scalingFactor);

    return trajectoryPosition;
}

FVector FAnimNode_LMM::getTrajectoryDirection(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& TrajectorySample){
    FQuat rootRotation = ComponentTransform.GetRotation();
    FVector rootPosition = ComponentTransform.GetLocation();

    FVector trajectoryDirection = rootRotation.Inverse().RotateVector(TrajectorySample.GetTransform().GetRotation().RotateVector(FVector(1, 0, 0)));

    return trajectoryDirection;
}

void FAnimNode_LMM::ProjectorEvaluate(TArray<float>& Features, TArray<float>& Latent, TArray<float>& ComputedFeatures, bool& Transition)
{
    //UE_LOG(LogTemp, Warning, TEXT("Projector Evaluate"));
    TArray<float> input;
    input.Append(ComputedFeatures);

    TArray<float> output = ProjectorInstance->RunModel(input);

    TArray<float> current;
    current.Append(Features);
    current.Append(Latent);

    float distance = GetDistance(output, current);

    if(distance > 0.1)
    {
        for (int32 i = 0; i < Features.Num() - 1; ++i)
        {
            Features[i] = output[i];
        }

        for (int32 i = Features.Num(); i < Features.Num() + Latent.Num() - 1; ++i)
        {
            Latent[i - Features.Num()] = output[i];
        }
    } else {
        Transition = false;
    }
}

void FAnimNode_LMM::StepperEvaluate(TArray<float>& Features, TArray<float>& Latent)
{
    //UE_LOG(LogTemp, Warning, TEXT("Stepper Evaluate"));
    TArray<float> input;
    input.Append(Features);
    input.Append(Latent);

    TArray<float> output = StepperInstance->RunModel(input);

    const float dt = 1/TimeStep;

    for (int32 i = 0; i < Features.Num() - 1; ++i)
    {
        Features[i] = output[i] * dt;
    }

    for (int32 i = Features.Num(); i < Features.Num() + Latent.Num() - 1; ++i)
    {
        Latent[i - Features.Num()] = output[i] * dt;
    }
}

void FAnimNode_LMM::DecompressorEvaluate(TArray<float>& Features, TArray<float>& Latent, TArray<float>& Pose)
{
    //UE_LOG(LogTemp, Warning, TEXT("Decompressor Evaluate"));
    TArray<float> input;
    input.Append(Features);
    input.Append(Latent);

    Pose = DecompressorInstance->RunModel(input);
}

float FAnimNode_LMM::GetDistance(TArray<float> a, TArray<float> b)
{
    float distance = 0;
    for(int i = 0; i < a.Num(); i++)
    {
        distance += FMath::Pow(a[i] - b[i], 2);
    }
    return FMath::Sqrt(distance);
}



void FAnimNode_LMM::printTensorData(FString prefix , TArray<float> data)
{
    FString dataString = prefix;
    for(int i = 0; i < data.Num() - 1; i++)
    {
        dataString += FString::Printf(TEXT("%f, "), data[i]);
    }
    UE_LOG(LogTemp, Warning, TEXT("Data: %s"), *dataString);

}

void FAnimNode_LMM::ExtractPositionsAndRotations(TArray<float>& Pose, TArray<FVector>& Positions, TArray<FQuat>& Rotations)
{
    int numBones = Positions.Num();

    FString posData = "Positions: \n";
    for(int i = 0; i < numBones - 1; i++)
    {
        Positions[i] = FVector(Pose[i * 3] * 100.0f, Pose[i * 3 + 1] * 100.0f, Pose[i * 3 + 2] * 100.0f);
        posData += FString::Printf(TEXT("Bone %i: %f, %f, %f\n"), i, Positions[i].X, Positions[i].Y, Positions[i].Z);
    }
    //UE_LOG(LogTemp, Warning, TEXT("%s"), *posData);

    for(int i = 0; i < numBones - 1; i++)
    {
        Rotations[i] = GetQuatFromXformXY(
            FVector(Pose[i * 6], Pose[i * 6 + 2], Pose[i * 6 + 4]), 
            FVector(Pose[i * 6 + 1], Pose[i * 6 + 3], Pose[i * 6 + 5]
        ));
    }
}

FQuat FAnimNode_LMM::GetQuatFromXformXY(FVector x, FVector y)
{
    FVector c2 = FVector::CrossProduct(x, y);
    c2.Normalize();
    FVector c1 = FVector::CrossProduct(c2, x);
    c1.Normalize();
    FVector c0 = x;

    FQuat result;

    if (c2.Z < 0.0f)
    {
        if (c0.X > c1.Y)
        {
            result = FQuat(c1.Z -c2.Y, 1.0f + c0.X - c1.Y - c2.Z, c0.Y+c1.X, c2.X+c0.Z);
        }
        else
        {
            result = FQuat(c2.X-c0.Z, c0.Y+c1.X, 1.0f - c0.X + c1.Y - c2.Z, c1.Z+c2.Y);
        }
    }
    else
    {
        if (c0.X < -c1.Y)
        {
            result = FQuat(c0.Y-c1.X, c2.X+c0.Z, c1.Z+c2.Y, 1.0f - c0.X - c1.Y + c2.Z);
        }
        else
        {
            result = FQuat(1.0f + c0.X + c1.Y + c2.Z, c1.Z-c2.Y, c2.X-c0.Z, c0.Y-c1.X);
        }
    }

    result.Normalize();
    return result;
}
