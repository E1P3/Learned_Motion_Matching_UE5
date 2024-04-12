#pragma once

#include "AnimNode_Learned_MM.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include "Animation/AnimStats.h"
#include "Animation/AnimTrace.h"

FAnimNode_Learned_MM::FAnimNode_Learned_MM()
{
}

void FAnimNode_Learned_MM::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(Initialize_AnyThread)
    FAnimNode_Base::Initialize_AnyThread(Context);
    Features.Initialize("/Import/LMM/features.bin");
    Source.Initialize(Context);
    searchTimer = 0.0f;
    forceSearchTimer = 0.0f;
    TWeakInterfacePtr<INNERuntimeCPU> Runtime = UE::NNE::GetRuntime<INNERuntimeCPU>(FString("NNERuntimeORTCpu"));

    if(Runtime.IsValid())
    {
        if(Decompressor)
        {
            DecompressorInstance = new FModelInstance(Decompressor, Runtime);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Decompressor is null"));
        }
        if(Stepper)
        {
            StepperInstance = new FModelInstance(Stepper, Runtime);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Stepper is null"));
        }
        if(Projector)
        {
            ProjectorInstance = new FModelInstance(Projector, Runtime);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Projector is null"));
        }
    } else {
        UE_LOG(LogTemp, Warning, TEXT("Runtime is null"));
    }

    FeaturesCurrent = Features.featuresdb[0];
    LatentCurrent.Init(0, StepperInstance->InputTensorShapes[0].Volume() - FeaturesCurrent.Num());


    //Initialize Bones
    BoneNames.Add(TEXT("Root"));  
    BoneNames.Add(TEXT("Hips"));         
    BoneNames.Add(TEXT("LeftUpLeg"));    
    BoneNames.Add(TEXT("LeftLeg"));      
    BoneNames.Add(TEXT("LeftFoot"));     
    BoneNames.Add(TEXT("LeftToe"));      
    BoneNames.Add(TEXT("RightUpLeg"));   
    BoneNames.Add(TEXT("RightLeg"));     
    BoneNames.Add(TEXT("RightFoot"));    
    BoneNames.Add(TEXT("RightToe"));     
    BoneNames.Add(TEXT("Spine"));        
    BoneNames.Add(TEXT("Spine1"));       
    BoneNames.Add(TEXT("Spine2"));       
    BoneNames.Add(TEXT("Neck"));         
    BoneNames.Add(TEXT("Head"));         
    BoneNames.Add(TEXT("LeftShoulder")); 
    BoneNames.Add(TEXT("LeftArm"));      
    BoneNames.Add(TEXT("LeftForeArm"));  
    BoneNames.Add(TEXT("LeftHand"));     
    BoneNames.Add(TEXT("RightShoulder"));
    BoneNames.Add(TEXT("RightArm"));     
    BoneNames.Add(TEXT("RightForeArm")); 
    BoneNames.Add(TEXT("RightHand"));   

    for (int32 i = 0; i < BoneNames.Num(); ++i)
    {
        FBoneReference BoneReference = FBoneReference(BoneNames[i]);
        BoneReferences.Add(BoneReference);
    }

    PoseCurrent = FPose_LMM(BoneReferences.Num());
    Inertializer = FInertializer(BoneReferences.Num(), InertializerHalflife);

    if(debug){
        animData.ReadAnimDataFromBinary("/Import/LMM/animData.bin");
    }
}

void FAnimNode_Learned_MM::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(CacheBones_AnyThread)
    Source.CacheBones(Context);
    const FBoneContainer& BoneContainer = Context.AnimInstanceProxy->GetRequiredBones();
    for (FBoneReference& BoneReference : Features.PositionBones)
    {
        BoneReference.Initialize(BoneContainer);
    }
    for (FBoneReference& BoneReference : Features.VelocityBones)
    {
        BoneReference.Initialize(BoneContainer);
    }
    for (FBoneReference& BoneReference : BoneReferences)
    {
        BoneReference.Initialize(BoneContainer);
    }
}

void FAnimNode_Learned_MM::Update_AnyThread(const FAnimationUpdateContext& Context)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(Update_AnyThread)
    GetEvaluateGraphExposedInputs().Execute(Context);
    Source.Update(Context);
}

void FAnimNode_Learned_MM::Evaluate_AnyThread(FPoseContext& Output)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(Evaluate_AnyThread)

    const FBoneContainer& BoneContainer = Output.AnimInstanceProxy->GetRequiredBones();

    Source.Evaluate(Output);

    FCompactPose& SourcePose = Output.Pose;

    float deltaTime = Output.AnimInstanceProxy->GetDeltaSeconds();

    bool forceSearch = false;

    FTransform ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();
    TArray<float> updatedFeatures = Features.Update(FeaturesCurrent, ComponentTransform, Trajectory, deltaTime, forceSearch);

    if(forceSearchTimer > 0.0f){
        forceSearchTimer -= deltaTime;
        forceSearch = false;
    }

    // if(debug){
    //     int currentFrame = animData.currentFrame;
    //
    //     for(int i = 0; i < BoneReferences.Num(); i++)
    //     {
    //         const FCompactPoseBoneIndex BoneIndex = BoneReferences[i].GetCompactPoseIndex(BoneContainer);
    //         FTransform& OutTransform = SourcePose[BoneIndex];
    //        
    //         OutTransform.SetLocation(FVector(animData.frames[currentFrame][i][0] * 100.0f, animData.frames[currentFrame][i][1] * 100.0f, animData.frames[currentFrame][i][2] * 100.0f));
    //         OutTransform.SetRotation(FQuat(animData.frames[currentFrame][i][3], animData.frames[currentFrame][i][4], animData.frames[currentFrame][i][5], animData.frames[currentFrame][i][6]));
    //         animData.step();
    //     }
    //
    //     return;
    // }


    if(DecompressorInstance != nullptr || StepperInstance != nullptr || ProjectorInstance != nullptr){

        if(searchTimer <= 0.0f || forceSearch){
            bool transition = false;
            ProjectorEvaluate(updatedFeatures, transition);
            if(transition){
                if(isInertializing){
                    FPose_LMM PoseTransition = DecompressorEvaluate(deltaTime);
                    Inertializer.Transition(PoseCurrent, PoseTransition);
                }
            }
            searchTimer = SearchInterval;
            forceSearchTimer = ForceSearchInterval;
        } else {
            searchTimer -= deltaTime;
        }

        StepperEvaluate(deltaTime); 

        FPose_LMM InputPose = DecompressorEvaluate(deltaTime);
        if(isInertializing){
            Inertializer.Update(InputPose, PoseCurrent, deltaTime);
        } else {
            PoseCurrent = InputPose;
        }

        for(int i = 0; i < BoneReferences.Num(); i++)
        {
            const FCompactPoseBoneIndex BoneIndex = BoneReferences[i].GetCompactPoseIndex(BoneContainer);
            FTransform& OutTransform = SourcePose[BoneIndex];
            OutTransform.SetLocation((PoseCurrent.BonePositions[i] + PoseCurrent.BoneVelocities[i] * deltaTime)* 100.0f);
            FQuat newRotation = PoseCurrent.BoneRotations[i] * Inertializer.QuatFromScaledAngleAxis(PoseCurrent.BoneAngularVelocities[i] * deltaTime);
            newRotation.Normalize();
            OutTransform.SetRotation(newRotation);
        }
    }   

}

void FAnimNode_Learned_MM::GatherDebugData(FNodeDebugData& DebugData)
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(GatherDebugData)
    FString DebugLine = DebugData.GetNodeName(this);
    DebugData.AddDebugItem(DebugLine);
    Source.GatherDebugData(DebugData);
}

void FAnimNode_Learned_MM::ProjectorEvaluate(TArray<float>& updatedFeatures, bool& transition, float transition_cost)
{
    TArray<float> output = ProjectorInstance->RunModel(updatedFeatures);

    TArray<float> projectedFeatures;
    TArray<float> projectedLatent;

    for (int32 i = 0; i < FeaturesCurrent.Num(); ++i)
    {
        projectedFeatures.Add(output[i]);
    }

    for (int32 i = FeaturesCurrent.Num(); i < FeaturesCurrent.Num() + LatentCurrent.Num(); ++i)
    {
        projectedLatent.Add(output[i]);
    }

    float best_cost = 0.0;
    for (int32 i = 0; i < FeaturesCurrent.Num(); ++i)
    {
        best_cost += FMath::Pow(FeaturesCurrent[i] - projectedFeatures[i], 2);
    }
    best_cost = FMath::Sqrt(best_cost);

    float transition_distance = 0.0f;
    for (int32 i = 0; i < FeaturesCurrent.Num(); ++i)
    {
        transition_distance += FMath::Pow(updatedFeatures[i] - projectedLatent[i], 2);
    }
    transition_distance = FMath::Sqrt(transition_distance);

    if (transition_distance > transition_cost)
    {
        transition = true;
        FeaturesCurrent = projectedFeatures;
        LatentCurrent = projectedLatent;
    }
    else
    {
        transition = false;
    }
}

void FAnimNode_Learned_MM::StepperEvaluate(float dt)
{
    TArray<float> input;
    input.Append(FeaturesCurrent);
    input.Append(LatentCurrent);

    TArray<float> output = StepperInstance->RunModel(input);

    for (int32 i = 0; i < FeaturesCurrent.Num(); ++i)
    {
        FeaturesCurrent[i] += output[i] * dt;
    }

    for (int32 i = FeaturesCurrent.Num(); i < FeaturesCurrent.Num() + LatentCurrent.Num(); ++i)
    {
        LatentCurrent[i - FeaturesCurrent.Num()] += output[i] * dt;
    }
}

FPose_LMM FAnimNode_Learned_MM::DecompressorEvaluate(float dt)
{
    TArray<float> input;
    TArray<float> output;
    input.Append(FeaturesCurrent);
    input.Append(LatentCurrent);

    return GetPoseContext(DecompressorInstance->RunModel(input), PoseCurrent.BoneRotations[0], PoseCurrent.boneCount, dt);
}

FPose_LMM FAnimNode_Learned_MM::GetPoseContext(TArray<float> input, FQuat rootRotation, float boneCount, float dt){

    // Check if pose is the correct size
    if (input.Num() < (boneCount - 1) * 3 + (boneCount - 1) * 6 + (boneCount - 1) * 3 + (boneCount - 1) * 3)
    {
        return false;
    }

    TArray<FVector> positions;
    TArray<FQuat> rotations;
    TArray<FVector> velocities;
    TArray<FVector> angularVelocities;
    positions.Init(FVector::ZeroVector, boneCount);
    rotations.Init(FQuat::Identity, boneCount);
    velocities.Init(FVector::ZeroVector, boneCount);
    angularVelocities.Init(FVector::ZeroVector, boneCount);

    // Extract bone positions
    int offset = 0;
    for (int i = 0; i < positions.Num() - 1; i++)
    {
        positions[i + 1] = FVector(
            input[offset+i*3+0],
            input[offset+i*3+1],
            input[offset+i*3+2]);
    }
    offset += (positions.Num() - 1) * 3;
    
    // Extract bone rotations, convert from 2-axis representation
    for (int i = 0; i < rotations.Num() - 1; i++)
    {   
        rotations[i + 1] = GetQuatFromXformXY(
            FVector(input[offset+i*6+0],
                    input[offset+i*6+2],
                    input[offset+i*6+4]),
            FVector(input[offset+i*6+1],
                    input[offset+i*6+3],
                    input[offset+i*6+5]));
    }
    offset += (rotations.Num() - 1) * 6;
     
    // Extract bone velocities
    for (int i = 0; i < velocities.Num() - 1; i++)
    {
        velocities[i + 1] = FVector(
            input[offset+i*3+0],
            input[offset+i*3+1],
            input[offset+i*3+2]);
    }
    offset += (velocities.Num() - 1) * 3;
    
    // Extract bone angular velocities
    for (int i = 0; i < angularVelocities.Num() - 1; i++)
    {
        angularVelocities[i + 1] = FVector(
            input[offset+i*3+0],
            input[offset+i*3+1],
            input[offset+i*3+2]);
    }
    offset += (angularVelocities.Num() - 1) * 3;
    
    // Extract root velocities and put in world space
    
    FVector rootVelocity = rootRotation.RotateVector(FVector(
        input[offset+0],
        input[offset+1],
        input[offset+2]));
        
    FVector rootAngularVelocity = rootRotation.RotateVector(FVector(
        input[offset+3],
        input[offset+4],
        input[offset+5]));
    
    offset += 6;

    // Find new root position/rotation/velocities etc.
    
    positions[0] = dt * rootVelocity;
    rotations[0] = FQuat(rootAngularVelocity, dt);
    velocities[0] = rootVelocity;
    angularVelocities[0] = rootAngularVelocity;    

    return FPose_LMM(positions, rotations, velocities, angularVelocities);
}

float FAnimNode_Learned_MM::GetDistance(TArray<float> a, TArray<float> b)
{
    float distance = 0;
    for(int i = 0; i < a.Num(); i++)
    {
        distance += FMath::Pow(a[i] - b[i], 2);
    }
    return FMath::Sqrt(distance);
}

FQuat FAnimNode_Learned_MM::GetQuatFromXformXY(FVector x, FVector y)
{
    FVector c2 = FVector::CrossProduct(x, y); 
    FVector c1 = FVector::CrossProduct(c2, x); 
    FVector c0 = x; 


    FQuat result = GetQuatFromColumns(c0, c1, c2);
    result.Normalize();
    return result;
}

FQuat FAnimNode_Learned_MM::GetQuatFromColumns(FVector c0, FVector c1, FVector c2){
    if (c2.Z < 0.0f)
    {
        if (c0.X > c1.Y)
        {
            return FQuat(
                1.0f + c0.X - c1.Y - c2.Z, 
                c0.Y+c1.X, 
                c2.X+c0.Z,
                c1.Z-c2.Y);
        }
        else
        {
            return FQuat(
                c0.Y+c1.X, 
                1.0f - c0.X + c1.Y - c2.Z, 
                c1.Z+c2.Y,
                c2.X-c0.Z); 
        }
    }
    else
    {
        if (c0.X < -c1.Y)
        {
            return FQuat(
                c2.X+c0.Z, 
                c1.Z+c2.Y, 
                1.0f - c0.X - c1.Y + c2.Z,
                c0.Y-c1.X); 
        }
        else
        {
            return FQuat(
                c1.Z-c2.Y, 
                c2.X-c0.Z, 
                c0.Y-c1.X,
                1.0f + c0.X + c1.Y + c2.Z);
        }
    }

}