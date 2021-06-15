// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SamBot.generated.h"

UCLASS()
class SAMBOTSIM_API ASamBot : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASamBot();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		AActor* BpActor;

	// The components to update
	UPrimitiveComponent* Lidar;
	UPrimitiveComponent* LeftWheel;
	UPrimitiveComponent* RightWheel;

	// Motor update booleans
	bool LidarOn = true;
	bool LeftMotorOn = false;
	bool RightMotorOn = false;

	// The strength of each motor type
	int LidarForceStrength = 10000;
	int MotorForceStrength = 10;

	int LIDAR_RADIUS = 2.722;
};
