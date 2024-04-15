#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Animation/AnimNodeBase.h"
#include "Animation/InputScaleBias.h"
#include "BoneContainer.h"
#include "PoseSearch/PoseSearchTrajectoryTypes.h"
#include "NNE.h"
#include "NNERuntimeCPU.h"
#include "NNEModelData.h"
#include "AnimNode_Learned_MM.generated.h"

struct FModelInstance{
	TUniquePtr<UE::NNE::IModelInstanceCPU> ModelInstance;
    TArray<float> InputData = TArray<float>();
    TArray<float> OutputData = TArray<float>();
    TArray<UE::NNE::FTensorBindingCPU> InputBindings;
    TArray<UE::NNE::FTensorBindingCPU> OutputBindings;
	TArray<UE::NNE::FTensorShape> InputTensorShapes;
	TArray<UE::NNE::FTensorShape> OutputTensorShapes;

    FModelInstance(const TObjectPtr<UNNEModelData> ModelData, const TWeakInterfacePtr<INNERuntimeCPU> Runtime){
        TUniquePtr<UE::NNE::IModelCPU> Model = Runtime->CreateModel(ModelData);
		if(Model.IsValid()){
			ModelInstance = Model->CreateModelInstance();
			if(ModelInstance.IsValid()){
				TConstArrayView<UE::NNE::FTensorDesc> InputTensorDescs = ModelInstance->GetInputTensorDescs();
				UE::NNE::FSymbolicTensorShape SymbolicInputTensorShape = InputTensorDescs[0].GetShape();
				InputTensorShapes = { UE::NNE::FTensorShape::MakeFromSymbolic(SymbolicInputTensorShape) };

				ModelInstance->SetInputTensorShapes(InputTensorShapes);

				TConstArrayView<UE::NNE::FTensorDesc> OutputTensorDescs = ModelInstance->GetOutputTensorDescs();
				UE::NNE::FSymbolicTensorShape SymbolicOutputTensorShape = OutputTensorDescs[0].GetShape();
				OutputTensorShapes = { UE::NNE::FTensorShape::MakeFromSymbolic(SymbolicOutputTensorShape) };

				// Example for creating in- and outputs
				InputData.SetNumZeroed(InputTensorShapes[0].Volume());
				InputBindings.SetNumZeroed(1);
				InputBindings[0].Data = InputData.GetData();
				InputBindings[0].SizeInBytes = InputData.Num() * sizeof(float);

				OutputData.SetNumZeroed(OutputTensorShapes[0].Volume());
				OutputBindings.SetNumZeroed(1);
				OutputBindings[0].Data = OutputData.GetData();
				OutputBindings[0].SizeInBytes = OutputData.Num() * sizeof(float);

				UE_LOG(LogTemp, Warning, TEXT("Created Model with %d inputs and %d outputs"), InputTensorShapes[0].Volume(), OutputTensorShapes[0].Volume());
			}
		}
	}

	void SetInputData(TArray<float> Data){

		if(InputData.Num() != InputTensorShapes[0].Volume()){
			UE_LOG(LogTemp, Error, TEXT("Input data size does not match model input size (%d != %d)"), InputData.Num(), InputTensorShapes[0].Volume());
			return;
		}

		InputData = Data;
		InputBindings[0].Data = InputData.GetData();
		InputBindings[0].SizeInBytes = InputData.Num() * sizeof(float);
	}

	void ClearOutputData(){
		OutputData.Empty();
		OutputData.SetNumZeroed(OutputTensorShapes[0].Volume());
		OutputBindings[0].Data = OutputData.GetData();
		OutputBindings[0].SizeInBytes = OutputData.Num() * sizeof(float);
	}

	TArray<float> RunModel(TArray<float> _InputData){

		SetInputData(_InputData);
		ClearOutputData();

		if (ModelInstance->RunSync(InputBindings, OutputBindings) != 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to run the model"));
		}

		return OutputData;
	}
};

struct FPose_LMM{
	TArray<FVector> BonePositions;
	TArray<FQuat> BoneRotations;
	TArray<FVector> BoneVelocities;
	TArray<FVector> BoneAngularVelocities;
	float boneCount = 0;

	FPose_LMM(){
	}

	FPose_LMM(float boneCount){
		this->boneCount = boneCount;
		BonePositions.Init(FVector::ZeroVector, boneCount);
		BoneRotations.Init(FQuat::Identity, boneCount);
		BoneVelocities.Init(FVector::ZeroVector, boneCount);
		BoneAngularVelocities.Init(FVector::ZeroVector, boneCount);
	}

	FPose_LMM(TArray<FVector> _BonePositions, TArray<FQuat> _BoneRotations, TArray<FVector> _BoneVelocities, TArray<FVector> _BoneAngularVelocities){
		BonePositions = _BonePositions;
		BoneRotations = _BoneRotations;
		BoneVelocities = _BoneVelocities;
		BoneAngularVelocities = _BoneAngularVelocities;
		boneCount = BonePositions.Num();
	}
};

struct FInertializer{
	TArray<FVector> position_offset;
	TArray<FQuat> rotation_offset;
	TArray<FVector> velocity_offset;
	TArray<FVector> angular_velocity_offset;
	FVector transition_src_position;
	FQuat transition_src_rotation;
	FVector transition_dst_position;
	FQuat transition_dst_rotation;
	bool rootAdjustment = true;
	float halflife = 0.1f;
	float lambda;
	float boneCount;

	FInertializer(){
	}

	FInertializer(float boneCount, float halfLife){
		this->boneCount = boneCount;
		position_offset.Init(FVector::ZeroVector, boneCount);
		rotation_offset.Init(FQuat::Identity, boneCount);
		velocity_offset.Init(FVector::ZeroVector, boneCount);
		angular_velocity_offset.Init(FVector::ZeroVector, boneCount);
		this->halflife = halfLife;
		this->lambda = GetLambda() / 2.0f;
	}

	void print(){
		FString inertializerInfo = "";
		for(int i = 0; i < boneCount; i++){
			if(i == 0 || i == 4 || i == 8 || i == 12 || i == 18 || i == 22){
				inertializerInfo += FString::Printf(TEXT("Bone %d : "), i);
				inertializerInfo += FString::Printf(TEXT("Position Offset : %s "), *position_offset[i].ToString());
				inertializerInfo += FString::Printf(TEXT("Rotation Offset : %s "), *rotation_offset[i].ToString());
				inertializerInfo += FString::Printf(TEXT("Velocity Offset : %s "), *velocity_offset[i].ToString());
				inertializerInfo += FString::Printf(TEXT("Angular Velocity Offset : %s \n"), *angular_velocity_offset[i].ToString());
			}
			
		}
		UE_LOG(LogTemp, Warning, TEXT("%s"), *inertializerInfo);
	}

	void Transition(FPose_LMM source, FPose_LMM destination){
		if(source.boneCount != destination.boneCount){
			UE_LOG(LogTemp, Error, TEXT("Cannot perform the transition. Source and Destination bone counts do not match"));
			return;
		}

		if(rootAdjustment){
			transition_src_position = source.BonePositions[0];
			transition_src_rotation = source.BoneRotations[0].GetNormalized();
			transition_dst_position = destination.BonePositions[0];
			transition_dst_rotation = destination.BoneRotations[0].GetNormalized();

			FVector world_space_dst_velocity = transition_dst_rotation.RotateVector(transition_src_rotation.Inverse().RotateVector(destination.BoneVelocities[0]));
			FVector world_space_dst_angular_velocity = transition_dst_rotation.RotateVector(transition_src_rotation.Inverse().RotateVector(destination.BoneAngularVelocities[0]));

			InertializeTransition(position_offset[0], velocity_offset[0], source.BonePositions[0], source.BoneVelocities[0], destination.BonePositions[0], world_space_dst_velocity);
			InertializeTransition(rotation_offset[0], angular_velocity_offset[0], source.BoneRotations[0], source.BoneAngularVelocities[0], destination.BoneRotations[0], world_space_dst_angular_velocity);
		}

		for(int i = 1; i < boneCount; i++){
			InertializeTransition(position_offset[i], velocity_offset[i], source.BonePositions[i], source.BoneVelocities[i], destination.BonePositions[i], destination.BoneVelocities[i]);
			InertializeTransition(rotation_offset[i], angular_velocity_offset[i], source.BoneRotations[i], source.BoneAngularVelocities[i], destination.BoneRotations[i], destination.BoneAngularVelocities[i]);
		}

	}

	void Update(FPose_LMM input, FPose_LMM& output, float dt){
		if(input.boneCount != output.boneCount){
			UE_LOG(LogTemp, Error, TEXT("Cannot perform update. Input and Output bone counts do not match"));
			return;
		}

		OffsetUpdate(dt);

		if(rootAdjustment){
			FVector WorldSpacePosition = transition_dst_rotation.RotateVector(transition_src_rotation.Inverse().RotateVector(input.BonePositions[0] - transition_src_position)) + transition_dst_position;
			FVector WorldSpaceVelocity = transition_dst_rotation.RotateVector(transition_src_rotation.Inverse().RotateVector(input.BoneVelocities[0]));
			FQuat WorldSpaceRotation = FQuat(transition_dst_rotation * (transition_src_rotation.Inverse() * input.BoneRotations[0])).GetNormalized();
			FVector WorldSpaceAngularVelocity = transition_dst_rotation.RotateVector(transition_src_rotation.Inverse().RotateVector(input.BoneAngularVelocities[0]));

			InertializeUpdate(output.BonePositions[0], output.BoneVelocities[0], position_offset[0], velocity_offset[0], WorldSpacePosition, WorldSpaceVelocity);
			InertializeUpdate(output.BoneRotations[0], output.BoneAngularVelocities[0], rotation_offset[0], angular_velocity_offset[0], WorldSpaceRotation, WorldSpaceAngularVelocity);
		}	

		for(int i = 1 ; i < boneCount; i++){
			InertializeUpdate(output.BonePositions[i], output.BoneVelocities[i], position_offset[i], velocity_offset[i], input.BonePositions[i], input.BoneVelocities[i]);
			InertializeUpdate(output.BoneRotations[i], output.BoneAngularVelocities[i], rotation_offset[i], angular_velocity_offset[i], input.BoneRotations[i], input.BoneAngularVelocities[i]);
		}
	}

	void InertializeTransition(
		FQuat& OffX, 
		FVector& OffV, 
		const FQuat& SrcX,
		const FVector& SrcV,
		const FQuat& DstX,
		const FVector& DstV)
	{
		OffX = (OffX * SrcX * DstX.Inverse()).GetNormalized();
		OffX = OffX.W < 0 ? OffX * -1 : OffX;
		OffV = (OffV + SrcV) - DstV;
	}

	void InertializeTransition(
		FVector& OffX, 
		FVector& OffV, 
		const FVector& SrcX,
		const FVector& SrcV,
		const FVector& DstX,
		const FVector& DstV)
	{
		OffX = (SrcX + OffX) - DstX;
		OffV = (SrcV + OffV) - DstV;
	}

	void InertializeUpdate(
		FQuat& OutX, 
		FVector& OutV,
		FQuat& OffX, 
		FVector& OffV,
		const FQuat& InX, 
		const FVector& InV)
	{
		OutX = OffX * InX;
		OutV = OffV + OffX.RotateVector(InV);
	}

	static inline void InertializeUpdate(
		FVector& OutX, 
		FVector& OutV,
		FVector& OffX, 
		FVector& OffV,
		const FVector& InX, 
		const FVector& InV)
	{
		OutX = InX + OffX;
		OutV = InV + OffV;
	}

	void OffsetUpdate(float dt){
		float exponential = exp(-lambda * dt);

		for(int i = 0; i < boneCount; i++){
			Decay_Spring_Damper(position_offset[i], velocity_offset[i], exponential, dt);
			Decay_Spring_Damper(rotation_offset[i], angular_velocity_offset[i], exponential, dt);
		}
	}

	void Decay_Spring_Damper(FVector& x, FVector& v, const float exponential, float dt){
		FVector j1 = v + lambda * x;
		x = exponential*(x + dt * j1);
		v = exponential*(v - dt * lambda * j1);
	}

	void Decay_Spring_Damper(FQuat& x, FVector& v, const float exponential, float dt){
		FVector j0 = QuatToScaledAngleAxis(x);
		FVector j1 = v + lambda * j0;
		x = QuatFromScaledAngleAxis(exponential*(j0 + dt * j1));
		v = exponential*(v - dt * lambda * j1);
	}

	float GetLambda(){
		return 6.64 * 0.69314718056f / halflife;
	}

	FQuat QuatExp(const FVector& V, float Eps = 1e-8f)
	{
		float HalfAngle = FMath::Sqrt(V.X * V.X + V.Y * V.Y + V.Z * V.Z);

		if (HalfAngle < Eps)
		{
			return FQuat::MakeFromEuler(V).GetNormalized();
		}
		else
		{
			float C = FMath::Cos(HalfAngle);
			float S = FMath::Sin(HalfAngle) / HalfAngle;
			return FQuat(S * V.X, S * V.Y, S * V.Z, C).GetNormalized();
		}
	}

	FVector QuatLog(const FQuat& Q, float Eps = 1e-8f)
	{
		float Length = FMath::Sqrt(Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z);

		if (Length < Eps)
		{
			return FVector(Q.X, Q.Y, Q.Z);
		}
		else
		{
			float HalfAngle = FMath::Acos(FMath::Clamp(Q.W, -1.0f, 1.0f));
			return (HalfAngle / Length) * FVector(Q.X, Q.Y, Q.Z);
		}
	}

	FQuat QuatFromScaledAngleAxis(const FVector& V, float Eps = 1e-8f)
	{
		return QuatExp(V / 2.0f, Eps);
	}

	FVector QuatToScaledAngleAxis(const FQuat& Q, float Eps = 1e-8f)
	{
		return 2.0f * QuatLog(Q, Eps);
	}


};

USTRUCT(BlueprintInternalUseOnly)
struct TESTING_API FFeatures{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, Category = Settings)
    TArray<FBoneReference> PositionBones;

    UPROPERTY(EditAnywhere, Category = Settings)
    TArray<FBoneReference> VelocityBones;

    UPROPERTY(EditAnywhere, Category = Settings)
    bool debug = false;

	FVector previousTrajectory = FVector(0, 0, 0);
	FVector currentTrajectory = FVector(0, 0, 0);
    TArray<FVector> CachedBonePositions;
    TArray<float> feature_offset;
    TArray<float> feature_scale;
	TArray<TArray<float>> featuresdb;
	int dbIndex = 0;
	bool isInitialized = false;

    FFeatures()
    {
    }

    void Initialize(const FString& FilePath){
        for(FBoneReference& BoneReference : VelocityBones){
            CachedBonePositions.Add(FVector(0, 0, 0));
        }

        GetFeatureParameters(FilePath);
    }

    void Normalize(TArray<float>& features){
        for(int i = 0; i < features.Num(); i++){
            features[i] = (features[i] - feature_offset[i]) / feature_scale[i];
        }
    }

	void Denormalize(TArray<float>& features){
		for (int i = 0; i < features.Num(); i++){
			features[i] = features[i] * feature_scale[i] + feature_offset[i];
		}
	}

	TArray<float> Update(TArray<float> FeaturesCurrent, FTransform RootTransform, FPoseSearchQueryTrajectory Trajectory, float dt, bool& forceSearch){
		TArray<float> updatedFeatures = FeaturesCurrent;

		Denormalize(updatedFeatures);

		updatedFeatures.SetNum(updatedFeatures.Num() - 12);
		updatedFeatures.Append(GetTrajectoryData(RootTransform, Trajectory));

		float difference = FVector::Dist(currentTrajectory, previousTrajectory) / dt;

		if(difference > 5){
			forceSearch = true;
		}

		previousTrajectory = currentTrajectory;

		FString boneInfo = " ";
		for(int i = 0; i < updatedFeatures.Num(); i++){
            if(i == 0){
                boneInfo += "Position Features : ";
            } else if(i == PositionBones.Num()*3){
                boneInfo += "Velocity Features : ";
            } else if(i == PositionBones.Num()*3 + VelocityBones.Num()*3){
                boneInfo += "Trajectory Features : ";
            }
            boneInfo += FString::Printf(TEXT(" %f "), updatedFeatures[i]);
        }
        if(debug){
            UE_LOG(LogTemp, Warning, TEXT("%s"), *boneInfo);
        }

		Normalize(updatedFeatures);
		return updatedFeatures;
	}

    TArray<float> GetTrajectoryData(FTransform& RootTransform, FPoseSearchQueryTrajectory& Trajectory){
        TArray<float> trajectoryData;
        if(Trajectory.Samples.Num() > 3)
        {
            TArray<FVector> trajectoryPositions;
            TArray<FVector> trajectoryDirections;

            trajectoryPositions.Add(GetTrajectoryPositions(RootTransform, Trajectory.Samples[2], 0.01f));
            trajectoryPositions.Add(GetTrajectoryPositions(RootTransform, Trajectory.Samples[3], 0.01f));
            trajectoryPositions.Add(GetTrajectoryPositions(RootTransform, Trajectory.Samples[4], 0.01f));
            trajectoryDirections.Add(GetTrajectoryDirection(RootTransform, Trajectory.Samples[2]));
            trajectoryDirections.Add(GetTrajectoryDirection(RootTransform, Trajectory.Samples[3]));
            trajectoryDirections.Add(GetTrajectoryDirection(RootTransform, Trajectory.Samples[4]));

			currentTrajectory = FVector(trajectoryPositions[0].X, trajectoryPositions[0].Y, 0);

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
        } else {
            if(debug){
                UE_LOG(LogTemp, Warning, TEXT("Trajectory Samples are less than 5"));
            }
            TArray<float> zeros;
            zeros.SetNumZeroed(12);
            trajectoryData.Append(zeros);
        }

        return trajectoryData;
    }

    FVector GetTrajectoryPositions(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& TrajectorySample, float scalingFactor = 0.01f){
        FQuat rootRotation = ComponentTransform.GetRotation();
        FVector rootPosition = ComponentTransform.GetLocation();
        FVector trajectoryPosition = rootRotation.Inverse().RotateVector((TrajectorySample.Position - rootPosition) * scalingFactor);
        return trajectoryPosition;
    }

    FVector GetTrajectoryDirection(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& TrajectorySample){
        FQuat rootRotation = ComponentTransform.GetRotation();
        FVector rootPosition = ComponentTransform.GetLocation();
        FVector trajectoryDirection = rootRotation.Inverse().RotateVector(TrajectorySample.GetTransform().GetRotation().RotateVector(FVector(1, 0, 0)));
        return trajectoryDirection;
    }

    void GetFeatureParameters(const FString& FilePath){
		TArray<TArray<float>> ModelData = ReadFloatArraysFromFile(FPaths::Combine(FPaths::ProjectDir(), FilePath));
		if(ModelData.Num() < 2){
			UE_LOG(LogTemp, Error, TEXT("Failed to read feature parameters from file"));
			return;
		}
		feature_offset = ModelData[0];
		feature_scale = ModelData[1];
		isInitialized = true;
	}

	TArray<TArray<float>> ReadFloatArraysFromFile(const FString& FilePath)
	{
		TArray<uint8> FileData;
		if (!FFileHelper::LoadFileToArray(FileData, *FilePath))
		{
			// Failed to load file
			return TArray<TArray<float>>();
		}

		int32 NumElements = 2 * (PositionBones.Num() * 3 + VelocityBones.Num() * 3 + 12) + 2;
		int32 Offset = 0;

		int32 dbsize1 = DecodeToInt(FileData, Offset);
		int32 dbsize2 = DecodeToInt(FileData, Offset);

		for(int i = 0; i < dbsize1; i++){
			TArray<float> floats = GetArrayFromBinary(FileData, Offset, dbsize2);
			featuresdb.Add(floats);
		}

		TArray<TArray<float>> FloatArrays;
		while (Offset != FileData.Num())
		{
			int32 Size = DecodeToInt(FileData, Offset);
			TArray<float> Floats = GetArrayFromBinary(FileData, Offset, Size);
			FloatArrays.Add(Floats);
		}
		return FloatArrays;
	}

	TArray<float> GetArrayFromBinary(TArray<uint8>& FileData, int32& Offset, int32 Size)
	{
		TArray<float> Floats;
		for (int32 j = 0; j < Size; j++)
		{
			Floats.Add(DecodeToFloat(FileData, Offset));
		}
		return Floats;
	}

	float DecodeToFloat(TArray<uint8>& FileData, int32& Offset)
	{
		float Value;
		FMemory::Memcpy(&Value, &FileData[Offset], sizeof(float));
        Offset += 4;
		return Value;
	}

	int32 DecodeToInt(TArray<uint8>& FileData, int32& Offset)
	{
		int32 Value;
		FMemory::Memcpy(&Value, &FileData[Offset], sizeof(int32));
        Offset += 4;
		return Value;
	}

};

USTRUCT(BlueprintInternalUseOnly)
struct TESTING_API FAnimNode_Learned_MM : public FAnimNode_Base
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY(EditAnywhere, Category = Settings)
    FPoseLink Source;

    UPROPERTY(EditAnywhere, Category = Settings)
	FFeatures Features;

    UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Decompressor;

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Stepper;

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Projector;

    UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	FPoseSearchQueryTrajectory Trajectory;

	UPROPERTY(EditAnywhere, Category = Settings)
	float SearchInterval = 0.1f;

	UPROPERTY(EditAnywhere, Category = Settings)
	float ForceSearchInterval = 0.1f;

	UPROPERTY(EditAnywhere, Category = Settings)
	float InertializerDecayRate = 1.0f;

	UPROPERTY(EditAnywhere, Category = Settings)
	bool debug = true;

	UPROPERTY(EditAnywhere, Category = Settings)
	bool isRootAdjustment = true;

	UPROPERTY(EditAnywhere, Category = Settings)
	bool isInertializing = true;

    FAnimNode_Learned_MM();

private:
    // Model Instances
    FModelInstance* DecompressorInstance;
	FModelInstance* StepperInstance;
	FModelInstance* ProjectorInstance;

    // Bone References
    TArray<FBoneReference> BoneReferences;
    TArray<FName> BoneNames;

    // Storage for LMM state
    FPose_LMM PoseCurrent;
	TArray<float> FeaturesCurrent;
	TArray<float> LatentCurrent;
	FInertializer Inertializer;

	// Search Parameters
	float searchTimer = 0.0f;
	float forceSearchTimer = 0.0f;

public:
    // FAnimNode_Base interface
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void CacheBones_AnyThread(const FAnimationCacheBonesContext& Context) override;
	virtual void Update_AnyThread(const FAnimationUpdateContext& Context) override;
	virtual void Evaluate_AnyThread(FPoseContext& Output) override;
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	// End of FAnimNode_Base interface

    FPose_LMM DecompressorEvaluate(float dt);
    void StepperEvaluate(float dt);
    void ProjectorEvaluate(TArray<float>& updatedFeatures, bool& transition, float transition_cost = 1.0f);
	FPose_LMM GetPoseContext(TArray<float> input, FQuat rootRotation, float boneCount, float dt);
	float GetDistance(TArray<float> a, TArray<float> b);
    FQuat GetQuatFromXformXY(FVector x, FVector y);
	FQuat GetQuatFromColumns(FVector c0, FVector c1, FVector c2);

};