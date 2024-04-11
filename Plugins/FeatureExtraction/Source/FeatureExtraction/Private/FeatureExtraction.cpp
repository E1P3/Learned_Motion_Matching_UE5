// Copyright Epic Games, Inc. All Rights Reserved.

#include "FeatureExtraction.h"
#include "FeatureExtractionStyle.h"
#include "FeatureExtractionCommands.h"
#include "SFeatureExtractionMenu.h"
#include "LevelEditor.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "ToolMenus.h"

static const FName FeatureExtractionTabName("FeatureExtraction");

#define LOCTEXT_NAMESPACE "FFeatureExtractionModule"

void FFeatureExtractionModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	
	FFeatureExtractionStyle::Initialize();
	FFeatureExtractionStyle::ReloadTextures();

	FFeatureExtractionCommands::Register();
	
	PluginCommands = MakeShareable(new FUICommandList);

	PluginCommands->MapAction(
		FFeatureExtractionCommands::Get().OpenPluginWindow,
		FExecuteAction::CreateRaw(this, &FFeatureExtractionModule::PluginButtonClicked),
		FCanExecuteAction());

	UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FFeatureExtractionModule::RegisterMenus));
	
	FGlobalTabmanager::Get()->RegisterNomadTabSpawner(FeatureExtractionTabName, FOnSpawnTab::CreateRaw(this, &FFeatureExtractionModule::OnSpawnPluginTab))
		.SetDisplayName(LOCTEXT("FFeatureExtractionTabTitle", "FeatureExtraction"))
		.SetMenuType(ETabSpawnerMenuType::Hidden);
}

void FFeatureExtractionModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.

	UToolMenus::UnRegisterStartupCallback(this);

	UToolMenus::UnregisterOwner(this);

	FFeatureExtractionStyle::Shutdown();

	FFeatureExtractionCommands::Unregister();

	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(FeatureExtractionTabName);
}

TSharedRef<SDockTab> FFeatureExtractionModule::OnSpawnPluginTab(const FSpawnTabArgs& SpawnTabArgs)
{
	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			SNew(SFeatureExtractionMenu)
		];
}

void FFeatureExtractionModule::PluginButtonClicked()
{
	FGlobalTabmanager::Get()->TryInvokeTab(FeatureExtractionTabName);
}

void FFeatureExtractionModule::RegisterMenus()
{
	// Owner will be used for cleanup in call to UToolMenus::UnregisterOwner
	FToolMenuOwnerScoped OwnerScoped(this);

	{
		UToolMenu* Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Window");
		{
			FToolMenuSection& Section = Menu->FindOrAddSection("WindowLayout");
			Section.AddMenuEntryWithCommandList(FFeatureExtractionCommands::Get().OpenPluginWindow, PluginCommands);
		}
	}

	{
		UToolMenu* ToolbarMenu = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar");
		{
			FToolMenuSection& Section = ToolbarMenu->FindOrAddSection("Settings");
			{
				FToolMenuEntry& Entry = Section.AddEntry(FToolMenuEntry::InitToolBarButton(FFeatureExtractionCommands::Get().OpenPluginWindow));
				Entry.SetCommandList(PluginCommands);
			}
		}
	}
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FFeatureExtractionModule, FeatureExtraction)