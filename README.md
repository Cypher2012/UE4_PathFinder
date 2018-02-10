# UE4_PathFinder
Static class that can be added to your UE4 C++ project.

## How to use
Include the .h/.cpp files to your project and include the .h where you need the functionality

The system works around using UPathingPoint objects. These essentialy hold their own location and a referene to any UPathingPoints that it is connected to. It is up to your program to generate these. 

Once you have generated your TArray of UPathingPoint, you can pass this to the Solve_AStar() function. 

The Solve_AStar() function also asks for which UPathingPoint is the target and which is the start. 

Example:
```
TArray<UPathingPoint*> CachedPathingPoints;
//Generate your pathing points

UPathingPoint * StartPathingPoint = .....
UPathingPoint * EndPathingPoint = ....

TArray<UPathingPoint*> OutPathingPoints;

if (APathFinder::Solve_AStar(CachedPathingPoints, StartPathingPoint, EndPathingPoint, OutPathingPoints))
{
  TArray<FVector> RoutePathingVectors = APathFinder::PathingPointsToVector(OutPathingPoints);
  
}
```
It gives you back a filled TArray of pathing points of the found path. 

## Help Functions

There are now a few help functions that you can use! 

* PathingPointsToVector() - Use this  to easily convert TArray of UPathingPoint to a TArray of FVector, with matching location.

* VectorToPathingPoint() - This will take a TArray of FVector and create a TArray of UPathingPoint with matching locations. It must be noted though, that this will not CONNECT the UPathingPoints.You will still need to do this.

* JoinPathingPoints() - This will take two UPathingPoints, and create a two way connection between them.

* JoinPathingPointArray() - This will take TArray of UPathingPoint, and make two way connections between ALL of them.
