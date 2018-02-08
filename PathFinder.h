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
	void SetParentPathingPoint(UPathingPoint * inParent);

	bool GetVisited() const;
	float GetLocalCost() const;
	float GetGlobalCost() const;

public:		

	void SetLocation(const FVector inLocation);

	FVector GetLocation() const;	
	UPathingPoint * GetParentPathingPoint() const;

	void AddConnectedPathingPoint(UPathingPoint * inPathingConnectedPathingPoint);
	TArray<UPathingPoint*> GetConnectedPathingPoints() const;
};

UCLASS()
class AIRPORTSIM_API APathFinder : public AInfo
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
	* Sorts the passed PathingPoints by their GlobalCost. Implements a bubble sort algorithm
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
	* @param bStopIfPathFound Decides if the algorithm should stop as soon as it has found a path, or continue to completely finish and find the shortest path
	* @return Returns true if a path was found
	*/
	static bool Solve_AStar(TArray<UPathingPoint*> const PathingPoints,  UPathingPoint * const StartPoint, UPathingPoint * const TargetPoint, TArray<UPathingPoint*>& OutPathingPoints, const bool bStopIfPathFound = false);

	/**
	* Converts an array of UPathingPoint pointers to an array of FVector's
	* @param PathingPoints Array of UPathingPoint to be converted
	* @return An array of FVector converted from the pathing points
	*/
	static TArray<FVector> PathingPointsToVector(const TArray<UPathingPoint*> PathingPoints);

	/**
	* Coverts TArray of FVector's to a TArray of UPathingPoint pointers, with the locations set to the incoming FVectors.
	* You will still need to set connected UPathingPoints manually.
	* @param PathingVectors TArray of FVector to be converted
	* @return an array of UPathingPoint pointers
	*/
	static TArray<UPathingPoint *> VectorToPathingPoint(const TArray<FVector> PathingVectors);

	/**
	* Will add the passed UPathingPoint's to each others connected pathing points.
	* This will create a connection in both directions.
	* @param a The first UPathingPoint
	* @param b The second UPathingPoint
	*/
	static void JoinPathingPoints(UPathingPoint * a, UPathingPoint * b);

	/**
	* Creates two way connections between all of the UPathingPoint's in the TArray
	* @param PathingPoints TArray of pathing points to join
	*/
	static void JoinPathingPointArray(TArray<UPathingPoint*> PathingPoints);
};
