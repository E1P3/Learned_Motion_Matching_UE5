#pragma once 

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "NNE.h"
#include "NNERuntimeCPU.h"
#include "NNEModelData.h"
#include "BoneControllers/AnimNode_ModifyBone.h"
#include "PoseSearch/PoseSearchTrajectoryTypes.h"
#include "AnimNode_LMM.generated.h"

class USkeletalMeshComponent;

struct FModelInstance{
	TUniquePtr<UE::NNE::IModelInstanceCPU> ModelInstance;
    TArray<float> InputData = TArray<float>();
	TArray<float> InputStd = TArray<float>();
	TArray<float> InputMean = TArray<float>();
    TArray<float> OutputData = TArray<float>();
	TArray<float> OutputStd = TArray<float>();
	TArray<float> OutputMean = TArray<float>();
    TArray<UE::NNE::FTensorBindingCPU> InputBindings;
    TArray<UE::NNE::FTensorBindingCPU> OutputBindings;
	TArray<UE::NNE::FTensorShape> InputTensorShapes;
	TArray<UE::NNE::FTensorShape> OutputTensorShapes;

    FModelInstance(const TObjectPtr<UNNEModelData> ModelData, const TWeakInterfacePtr<INNERuntimeCPU> Runtime, const FString& FilePath){
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

				GetStdMean(FPaths::Combine(FPaths::ProjectDir(), FilePath));

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


		SetInputData(Normalise(_InputData, InputStd, InputMean));
		ClearOutputData();

		if (ModelInstance->RunSync(InputBindings, OutputBindings) != 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to run the model"));
		}

		return Denormalise(OutputData, OutputStd, OutputMean);

	}

	TArray<float> Normalise(TArray<float> Data, TArray<float> Std, TArray<float> Mean){
		TArray<float> NormalisedData;
		NormalisedData.Init(0, Data.Num());
		for(int i = 0; i < Data.Num(); i++){
			NormalisedData[i] = (Data[i] - Mean[i]) / Std[i];
		}
		return NormalisedData;
	}

	TArray<float> Denormalise(TArray<float> Data, TArray<float> Std, TArray<float> Mean){
		TArray<float> DenormalisedData;
		DenormalisedData.Init(0, Data.Num());
		for(int i = 0; i < Data.Num(); i++){
			DenormalisedData[i] = Data[i] * Std[i] + Mean[i];
		}
		return DenormalisedData;
	}

	void GetStdMean(const FString& FilePath){
		TArray<TArray<float>> ModelData = ReadFloatArraysFromFile(FilePath);
		InputMean = ModelData[0];
		InputStd = ModelData[1];
		OutputMean = ModelData[2];
		OutputStd = ModelData[3];
		check(InputStd.Num() == InputTensorShapes[0].Volume());
		check(InputMean.Num() == InputTensorShapes[0].Volume());
		check(OutputStd.Num() == OutputTensorShapes[0].Volume());
		check(OutputMean.Num() == OutputTensorShapes[0].Volume());
	}

	TArray<TArray<float>> ReadFloatArraysFromFile(const FString& FilePath)
	{
		TArray<uint8> FileData;
		if (!FFileHelper::LoadFileToArray(FileData, *FilePath))
		{
			// Failed to load file
			return TArray<TArray<float>>();
		}

		int32 NumElements = InputTensorShapes[0].Volume() * 2 + OutputTensorShapes[0].Volume() * 2 + 4;
		int32 Offset = 0;

		TArray<TArray<float>> FloatArrays;
		while (Offset != NumElements * 4)
		{
			int32 Size = DecodeToInt(FileData, Offset);
			Offset += 4;
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
			Offset += 4;
		}
		return Floats;
	}

	float DecodeToFloat(TArray<uint8>& FileData, int32& Offset)
	{
		float Value;
		FMemory::Memcpy(&Value, &FileData[Offset], sizeof(float));
		return Value;
	}

	int32 DecodeToInt(TArray<uint8>& FileData, int32& Offset)
	{
		int32 Value;
		FMemory::Memcpy(&Value, &FileData[Offset], sizeof(int32));
		return Value;
	}
};

USTRUCT(BlueprintInternalUseOnly)
struct TESTING_API FAnimNode_LMM : public FAnimNode_SkeletalControlBase
{
    GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Decompressor;

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Stepper;

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	TObjectPtr<UNNEModelData> Projector;

	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinShownByDefault))
	FPoseSearchQueryTrajectory Trajectory;

	UPROPERTY(EditAnywhere, Category = Settings)
	TArray<FBoneReference> TrackedBones;

	UPROPERTY(EditAnywhere, Category = Settings)
	FBoneReference Hip;

	UPROPERTY(EditAnywhere, Category = Settings)
	float TimeStep;

	TArray<FBoneReference> BoneReferences;
	FModelInstance* DecompressorInstance;
	FModelInstance* StepperInstance;
	FModelInstance* ProjectorInstance;

	// FAnimNode_Base interface
    virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

    FAnimNode_LMM();
private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	
	// Model Functions
	void ProjectorEvaluate(TArray<float>& Features, TArray<float>& Latent, TArray<float>& ComputedFeatures, bool& Transition);
	void StepperEvaluate(TArray<float>& Features, TArray<float>& Latent);
	void DecompressorEvaluate(TArray<float>& Features, TArray<float>& Latent, TArray<float>& Pose);

	// End of FAnimNode_SkeletalControlBase interface
	void SetBoneTransforms(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms, const FBoneContainer& BoneContainer, FTransform ComponentTransform, TArray<float> Pose);
	TArray<float> GetFeatures(const FBoneContainer& BoneContainer, FComponentSpacePoseContext& Output, FTransform ComponentTransform);
	float GetDistance(TArray<float> a, TArray<float> b);
	void ExtractPositionsAndRotations(TArray<float>& Pose, TArray<FVector>& Positions, TArray<FQuat>& Rotations);
	FQuat GetQuatFromXformXY(FVector x, FVector y);
	void printTensorData(FString prefix , TArray<float> data);
	TArray<float> GetTrajectoryData(FTransform& ComponentTransform);
	FVector getTrajectoryPositions(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& Trajectory, float scalingFactor);
	FVector getTrajectoryDirection(FTransform& ComponentTransform, FPoseSearchQueryTrajectorySample& Trajectory);
	// Cached bone positions for computing velocity
	TArray<float> CachedTrackedBonesPositions;
	TArray<float> PoseCurrent;
	TArray<float> FeaturesCurrent;
	TArray<float> LatentCurrent;
	TArray<FName> ControlledBoneNames;
};