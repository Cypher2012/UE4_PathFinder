#include "PathFinder.h"

#include "Algo/Reverse.h"

#include "Engine.h"

void UPathingPoint::ResetPathing()
{
	SetVisited(false);
	SetLocalCost(INFINITY);
	SetGlobalCost(INFINITY);
	SetParentPathingPoint(nullptr);
}

void UPathingPoint::SetVisited(const bool bInVisited)
{
	bVisited = bInVisited;
}

void UPathingPoint::SetLocalCost(const float inLocalCost)
{
	LocalCost = inLocalCost;
}

void UPathingPoint::SetGlobalCost(const float inGlobalCost)
{
	GlobalCost = inGlobalCost;
}

void UPathingPoint::SetParentPathingPoint(UPathingPoint * inParent)
{
	ParentPathingPoint = inParent;
}

bool UPathingPoint::GetVisited() const
{
	return bVisited;
}

float UPathingPoint::GetLocalCost() const
{
	return LocalCost;
}

float UPathingPoint::GetGlobalCost() const
{
	return GlobalCost;
}

void UPathingPoint::SetLocation(const FVector inLocation)
{
	Location = inLocation;
}

FVector UPathingPoint::GetLocation() const
{
	return Location;
}

UPathingPoint * UPathingPoint::GetParentPathingPoint() const
{
	return ParentPathingPoint;
}

void UPathingPoint::AddConnectedPathingPoint(UPathingPoint * inPathingConnectedPathingPoint)
{
	ConnectedPathingPoints.Add(inPathingConnectedPathingPoint);
}

TArray<UPathingPoint*> UPathingPoint::GetConnectedPathingPoints() const
{
	return ConnectedPathingPoints;
}

float APathFinder::Heuristic(UPathingPoint * a, UPathingPoint * b)
{
	return FVector::Dist(a->GetLocation(), b->GetLocation());
}

void APathFinder::SortByGlobalCost(TArray<UPathingPoint*>& PathingPoints)
{
	for (int i = 0; i < PathingPoints.Num() - 1; i++)
	{
		for (int j = 0; j < PathingPoints.Num() - i - 1; j++)
		{
			if (PathingPoints[i]->GetGlobalCost() < PathingPoints[j]->GetGlobalCost())
			{
				UPathingPoint* pTmp = PathingPoints[i];
				PathingPoints[i] = PathingPoints[j];
				PathingPoints[j] = pTmp;
			}
		}
	}
}

bool APathFinder::Solve_AStar(TArray<UPathingPoint*> const PathingPoints, UPathingPoint * const StartPoint,  UPathingPoint * const TargetPoint, TArray<UPathingPoint*>& OutPathingPoints, const bool bStopIfPathFound)
{
	//We must remember to reset the pathing values if we are running the path finding multiple times
	for (UPathingPoint * tmpPathingPoint : PathingPoints)
	{
		tmpPathingPoint->ResetPathing();
	}	

	//Set up the starting pathing point
	UPathingPoint * CurrentPathingPoint = StartPoint;
	StartPoint->SetLocalCost(0);
	StartPoint->SetGlobalCost(Heuristic(StartPoint, TargetPoint));

	//Create points to test array and add starting point to it
	TArray<UPathingPoint*> PointsToTest;
	PointsToTest.Push(CurrentPathingPoint);

	bool bPathFound = false;

	while (PointsToTest.Num() > 0)
	{
		//Sort by global cost
		SortByGlobalCost(PointsToTest);

		//If first point to test has already been visited, remove it and go to next
		while (PointsToTest.Num() > 0 && PointsToTest[0]->GetVisited())
		{
			PointsToTest.RemoveAt(0);
		}

		//If no more points to test, the algorithm has finished!
		if (PointsToTest.Num() <= 0)
		{
			break;
		}

		//If we have found the target, set bPathFound flag to true. This will be returned once the algorithm has finished
		if (CurrentPathingPoint == TargetPoint)
		{
			bPathFound = true;

			//Break from the loop if the path is found and bStopIfPathFound flag is set to true
			if (bStopIfPathFound)
			{
				break;
			}
		}

		//Onces the prior checks of PointsToTest have completed, we can set the current pathing point
		CurrentPathingPoint = PointsToTest[0];
		CurrentPathingPoint->SetVisited(true);

		//Loop through all pathing points that are connected to the current point
		for (UPathingPoint * ConnectedPoint : CurrentPathingPoint->GetConnectedPathingPoints())
		{
			//Only add to PointsToTest if it hasn't already been visited
			if (!ConnectedPoint->GetVisited())
			{
				PointsToTest.Add(ConnectedPoint);
			}

			//Find the best next point
			float PossiblyLowestGoal = CurrentPathingPoint->GetLocalCost() + FVector::Dist(CurrentPathingPoint->GetLocation(), ConnectedPoint->GetLocation());

			if (PossiblyLowestGoal < ConnectedPoint->GetLocalCost())
			{
				ConnectedPoint->SetParentPathingPoint(CurrentPathingPoint);
				ConnectedPoint->SetLocalCost(PossiblyLowestGoal);
				ConnectedPoint->SetGlobalCost(ConnectedPoint->GetLocalCost() + Heuristic(ConnectedPoint, TargetPoint));
			}
		}

	}

	//Generate the final route
	OutPathingPoints.Empty();

	if (TargetPoint != nullptr)
	{
		UPathingPoint *p = TargetPoint;

		while (p->GetParentPathingPoint() != nullptr)
		{
			OutPathingPoints.Add(p);

			p = p->GetParentPathingPoint();
		}
	}

	//We want to reverse the pathing points, so the returned result is in logical order
	Algo::Reverse(OutPathingPoints);

	return bPathFound;
}

TArray<FVector> APathFinder::PathingPointsToVector(const TArray<UPathingPoint*> PathingPoints)
{
	TArray<FVector> ReturnVectors;

	for (UPathingPoint * PathingPoint : PathingPoints)
	{
		ReturnVectors.Add(PathingPoint->GetLocation());
	}

	return ReturnVectors;
}

TArray<UPathingPoint*> APathFinder::VectorToPathingPoint(const TArray<FVector> PathingVectors)
{
	TArray<UPathingPoint*> ReturnPathingPoints;

	for (FVector PathingVector : PathingVectors)
	{
		UPathingPoint* tmpPathingPoint = NewObject<UPathingPoint>();
		tmpPathingPoint->SetLocation(PathingVector);

		ReturnPathingPoints.Add(tmpPathingPoint);
	}

	return ReturnPathingPoints;
}

void APathFinder::JoinPathingPoints(UPathingPoint * a, UPathingPoint * b)
{
	a->AddConnectedPathingPoint(b);
	b->AddConnectedPathingPoint(a);
}

void APathFinder::JoinPathingPointArray(TArray<UPathingPoint*> PathingPoints)
{
	for (int i = 0; i < PathingPoints.Num(); i++)
	{
		for (int j = 0; j < PathingPoints.Num(); j++)
		{
			if (i != j)
			{
				PathingPoints[i]->AddConnectedPathingPoint(PathingPoints[j]);
			}
		}
	}
}


