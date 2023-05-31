# FS22_FlyPathfinding

![Debug visualizing a path that has been generated](https://i.gyazo.com/276cf7af27bb4457710b0a92254b1d4a.jpg)

## Description

This is a mod for Farming Simulator 22, which provides access to a 3D octree of the game map. 
Includes an A* pathfinding algorithm to pathfind within the game world.
Also a functionality to make a catmull-rom using CatmullRomSplineCreator object class.
This is not a standalone mod, this does not do any gameplay functionality except provide some classes for use in other mods.

## Features

Currently features octree generation, A* pathfinding class and catmull-rom class. 

## Installation

Drop the zipped file into your FS22 mods folder as usual.

## Usage

You can check if this mod is available with the FS22_FlyPathfinding ~= nil and checking if the octree object exists by FS22_FlyPathfinding.g_GridMap3D ~= nil.

To check if the octree grid has been generated you can either call FS22_FlyPathfinding.g_gridMap3D:isAvailable() which returns a bool.
Can also bind on server to the g_messageCenter:subscribe(MessageType.GRIDMAP3D_GRID_GENERATED, "your function to call here", "function's self ref here")

Can create the AStar pathfinding class by FS22_FlyPathfinding.AStar.new(isServer,isClient) and remembering to call :register(true) on the created object.
Currently the AStar pathfinding can't be queued up with the function call astarobject:find before octree is ready, so need to wait till octree has generated the grid done by like above.

After creating an AStar and making sure the grid is ready, one can start pathfinding using the find function.
AStar:find(startPosition,goalPosition,findNearest,allowSolidStart,allowSolidGoal,callback,smoothPath,customPathLoopAmount,customSearchNodeLimit)
Where startPosition and goalPosition are given as tables {x=,y=,z=},
findNearest a bool to indicate if goal wasn't reach to return the nearest found grid node if possible. 
allowSolidStart and allowSolidGoal bool to allow start and goal being inside a collision.
callback will be called when path is done, and will contain the path like {array of {x=,y=,z=}, bool reached goal}, returns {nil,false} if did not reach goal and findNearest was false or didn't have a best node.
customPathLoopAmount indicates how many loops per frame to pathfind, if not given uses value from config.xml.
customSearchNodeLimit can limit how many closed nodes will search through before cancelling search, also if not given uses value from config.xml.

Can create a catmull-rom by first creating the CatmullRomSplineCreator.new(isServer,isClient) and remembering to call :register(true) on the created object.
Then the CatmullRomSplineCreator object has function:
CatmullRomSplineCreator:createSpline(points,callback,customStartControlPoint,customEndControlPoint,lastDirection,roundSharpAngles,roundSharpAngleLimit,roundConnectionRadius,segment)
points is an array of {x=,y=,z=} minimum of 2 points needed to create a spline.
callback is a function to be called after spline is created and gives the created CatmullRomSpline as argument. 
customStartControlPoint and customEndControlPoint are optional P0 first segment, and P3 last segment's points given as {x=,y=,z=}.
lastDirection is optional direction that a previous spline was going towards on last segment, if combining two splines without using the combineSplinesAtDistance function.
roundSharpAngles is a bool to indicate if sharp angles beyond the roundSharpAngleLimit should be rounded a bit by the roundConnectionRadius

Two splines can be combined with the:
CatmullRomSplineCreator:combineSplinesAtDistance(spline1,spline2,distance,callback,roundSharpAngles,roundSharpAngleLimit,roundConnectionRadius)
spline1 and spline2 are the splines to be combined the spline2 will be used up and should be nilled and not used anymore. 
distance is the distance at the spline to combine the two splines.
callback is the function which will be called when the splines have been combined, returns the spline1 combined.

The created spline CatmullRomSpline has mainly the function:
CatmullRomSpline:getSplineInformationAtDistance(distance)
Which returns the position, forward vector, right vector and upvector on the spline at given distance along the spline.

The mod fully works in multiplayer. 
While in single player the config.xml has the maxOctreePreLoops that can be adjusted for example, this affects the speed of creating the octree grid, after loading screen is done and entering game the pre loops will be run and will lag the game for a few seconds depending on the amount of maxOctreePreLoops. Helps a minute or so of the generation time while game is fully running, and the maxOctreeGenerationLoopsPerUpdate can be also lowered if performance is too low, while generating at the beginning the octree grid. In dedicated servers by default the dedicatedScalingFactor makes sure that the octree grid is fully generated when starting the dedicated server.   

## Issues

Does not work on modded maps with non-original map size, so larger than 2048. 
I think it is an issue with the FS22 LUA API's overlapBox collision check function.
Did not see any possiblity to avoid including very thin meshes near terrain like the roads in the octree grid as solid, could cut some generation time by a good solution.

## Changelog

1.0.0 initial release.




 
 
 
