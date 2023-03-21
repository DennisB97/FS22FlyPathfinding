# FS22_FlyPathfinding

## Description

This is a mod for Farming Simulator 22, which provides access to a 3D octree of the game map. 
Includes an A* pathfinding algorithm to pathfind within the game world.
This is not a standalone mod, this does not do any gameplay functionality except provide some classes for use in other mods.
Still WORK IN PROGRESS.

## Features

Currently features octree generation, A* pathfinding class. 

## Installation

Drop the zipped file into your FS22 mods folder as usual.

## Usage

You can check if this mod is available with the FS22_FlyPathfinding ~= nil and checking if the octree object exists by FS22_FlyPathfinding.g_GridMap3D ~= nil.
You can create the AStar pathfinding class by FS22_FlyPathfinding.AStar.new() and remembering to call :register(true) on the created object.
Currently the AStar pathfinding can't be queued up with the function call astarobject:find(x,y,z,x2,y2,z2,findNearest,allowSolidStart,allowSolidGoal,callback) before octree is ready, so need to wait till octree is done, which can be done by subscribing to 
g_messageCenter:subscribe(MessageType.GRIDMAP3D_GRID_GENERATED, "your function to call here", "function's self ref")
The callback given to the AStar's find function will be called when path is done, and will contain the path in a array with table value of {x = , y = , z = } for path coordinates. Or 0 length array if no path was found/possible.

## Issues

Does not work on modded maps with non-original map size, so larger than 2048. 
I think it is an issue with the FS22 LUA API's overlapBox collision check function.

## Changelog

-- No release yet.

## Roadmap

Still on the work a spline/curve class for the generated AStar path to be able to smooth it.
 
 
 