# UE4_PathFinder
Static class that can be added to your UE4 C++ project.

## How to use
Include the .h/.cpp files to your project and include the .h where you need the functionality

The system works around using UPathingPoint objects. These essentialy hold their own location and a referene to any UPathingPoints that it is connected to. It is up to your program to generate these. 

Once you have generated your TArray of UPathingPoint, you can pass this to the Solve_AStar() function. 

The Solve_AStar() function also asks for which UPathingPoint is the target and which is the start. 

It gives you back a filled TArray of pathing points of the found path. 

You can use the static function PathingPointsToVector() to easily convert those pathing points to a TArray of FVector. Make it even easier to implement in to an existing project.
