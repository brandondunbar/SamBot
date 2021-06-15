// Fill out your copyright notice in the Description page of Project Settings.


#include "SamBot.h"

// Sets default values
ASamBot::ASamBot()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    // mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
    // RootComponent = mesh;
}

// Called when the game starts or when spawned
void ASamBot::BeginPlay()
{
	Super::BeginPlay();
    TArray<UPrimitiveComponent*> Components;
    BpActor->GetComponents(Components);
    for (UPrimitiveComponent* Component : Components)
    {
        if (Component && Component->IsSimulatingPhysics())
        {
            if (Component->GetName() == "RPLidarTop")
            {
                Lidar = Component;
            }
            else if (Component->GetName() == "LeftWheel")
            {
                LeftWheel = Component;
            }
            else if (Component->GetName() == "RightWheel")
            {
                RightWheel = Component;
            }
        }
    }
}

// Called every frame
void ASamBot::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    
    // GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("%s"), *FString(BpActor->GetName())));
    if (LidarOn)
    {
        //FVector Force = FVector(10, 0, 0) * LidarForceStrength;
        //Lidar->AddForce(Force);
        UStaticMeshComponent* LidarMeshComp = Cast<UStaticMeshComponent>(Lidar);
        if (LidarMeshComp)
        {
            LidarMeshComp->AddRadialImpulse(Lidar->GetComponentLocation(), 40.0f, LidarForceStrength, ERadialImpulseFalloff::RIF_Linear, false);
        }
        
    }
    if (LeftMotorOn)
    {
        FVector Force = FVector(10, 0, 0) * MotorForceStrength;
        LeftWheel->AddForce(Force);
    }
    if (RightMotorOn)
    {
        FVector Force = FVector(10, 0, 0) * MotorForceStrength;
        RightWheel->AddForce(Force);
    }
}

