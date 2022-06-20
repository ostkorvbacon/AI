// Fill out your copyright notice in the Description page of Project Settings.


#include "TextReaderComponent.h"

// Sets default values for this component's properties
UTextReaderComponent::UTextReaderComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UTextReaderComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void UTextReaderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

FString UTextReaderComponent::ReadFile(FString filename)
{
	//Read file ini [project]/Content/Data/ 
        //you can change with other location
	FString directory = FPaths::Combine(FPaths::GameSourceDir(), TEXT("Data"));
	FString result;
	IPlatformFile& file = FPlatformFileManager::Get().GetPlatformFile();
	if (file.CreateDirectory(*directory)) {
		FString myFile = directory + "/" + filename;
		FFileHelper::LoadFileToString(result, *myFile);
	}
	return result;
}

