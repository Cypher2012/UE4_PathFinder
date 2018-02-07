#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "PathFinder.generated.h"

UCLASS()
class UPathingPoint : public UObject
{
	GENERATED_BODY()

	friend class APathFinder;

	FVector Location;

	bool bVisited;
	float LocalCost;
	float GlobalCost;

	UPathingPoint * ParentPathingPoint;
	TArray<UPathingPoint*> ConnectedPathingPoints;

	void ResetPathing();	

	void SetVisited(const bool bInVisited);	
	void SetLocalCost(const float inLocalCost);	
	void SetGlobalCost(const float inGlobalCost);	

public:		

	void SetParentPathingPoint(UPathingPoint * inParent);
	void SetLocation(const FVector inLocation);

	FVector GetLocation() const;
	bool GetVisited() const;
	float GetLocalCost() const;
	float GetGlobalCost() const;

	UPathingPoint * GetParentPathingPoint() const;

	void AddConnectedPathingPoint(UPathingPoint * inPathingConnectedPathingPoint);
	TArray<UPathingPoint*> GetConnectedPathingPoints() const;
};

UCLASS()
class AIRPORTSIM_API APathFinder : public AActor
{
	GENERATED_BODY()

	/**
	* Calculates a heuristic value between the given points
	* @param a First UPathingPoint to be tested
	* @param b Second UPathingPoint to be tested
	* @return Returns a new heuristic value
	*/
	static float Heuristic(UPathingPoint * a, UPathingPoint * b);


	/**
	* Sorts the passed PathingPoints by their GlobalCost. Implements a bubble sort algorythm
	* @param PathingPoints Array of UPathingPoint to be re-ordered
	*/
	static void SortByGlobalCost(TArray<UPathingPoint*>& PathingPoints);

public:

	/**
	* Runs A star algorithm
	* @param PathingPoints Available pathing points that the pather finder can use
	* @param StartPoint The starting point of the path
	* @param TargetPoint The target point for the end of the path
	* @param OutPathingPoints The resulting path returned as an array of pathing points
	* @return Returns true if a path was found
	*/
	static bool Solve_AStar(TArray<UPathingPoint*> PathingPoints, UPathingPoint * StartPoint, UPathingPoint * TargetPoint, TArray<UPathingPoint*>& OutPathingPoints);

	/**
	* Converts an array of UPathingPoint to an array of FVector
	* @param PathingPoints Array of UPathingPoint to be converted
	* @return An array of FVector converted from the pathing points
	*/
	static TArray<FVector> PathingPointsToVector(const TArray<UPathingPoint*> PathingPoints);
};
