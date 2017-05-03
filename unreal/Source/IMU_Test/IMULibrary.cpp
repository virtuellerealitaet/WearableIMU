// Fill out your copyright notice in the Description page of Project Settings.

#include "IMU_Test.h"
#include "IMULibrary.h"

#include "CoreMinimal.h"

SerialReceiver UIMULibrary::serialReceiver;
FQuat UIMULibrary::rotation;

UIMULibrary::UIMULibrary(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UIMULibrary::InitializeIMU(int32 comPortNumber)
{
	UE_LOG(LogTemp, Log, TEXT("IMU Library: Initializing IMU..."));
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, TEXT("IMU Library: Initializing IMU"));
	
	std::string portName("/COM");
	portName.append(std::to_string(comPortNumber)); // e.g. /COM4

	if (!serialReceiver.openSerialConnection(portName))// open serial connection
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("IMU Library: Initialization failed !"));
		return false;
	}
	else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("IMU Library: Initialization successful."));
	}

	return true;
}

bool UIMULibrary::DeinitializeIMU()
{
	bool disconnectionSuccess = serialReceiver.closeSerialConnection(); // close connection

	if (!disconnectionSuccess)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("IMU Library: Closing IMU connection failed !"));
		return false;
	}
	else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Green, TEXT("IMU Library: Closing IMU connection successful."));
	}
	return true;
}

FRotator UIMULibrary::UpdateIMU()
{
	if (serialReceiver.isConnected())
	{
		serialReceiver.Update();
		SSock_Quaternion q;
		
		while (serialReceiver.getNextSample(q))
		{
			rotation.X = q.x;
			rotation.Y = q.y;
			rotation.Z = q.z;
			rotation.W = q.w;
		}
	}

	FRotator rot(rotation);
	return rot;

}
