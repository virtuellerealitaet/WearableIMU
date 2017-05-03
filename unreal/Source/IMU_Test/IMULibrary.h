// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "SerialReceiver.h"


#include "Kismet/BlueprintFunctionLibrary.h"
#include "IMULibrary.generated.h"



/**
 * 
 */
UCLASS()
class IMU_TEST_API UIMULibrary : public UBlueprintFunctionLibrary
{
	//GENERATED_BODY()
	
	GENERATED_UCLASS_BODY()

	/** Initialize IMU serial reader */
	UFUNCTION(BlueprintCallable, Category = "IMULibrary")
	static bool InitializeIMU(int32 comPortNumber);

	/** Deinitialize IMU serial reader */
	UFUNCTION(BlueprintCallable, Category = "IMULibrary")
	static bool DeinitializeIMU();

	/** Update data IMU serial reader */
	UFUNCTION(BlueprintCallable, Category = "IMULibrary")
	static FRotator UpdateIMU();
	

private:

	static SerialReceiver serialReceiver;
	
	static FQuat rotation;

};
