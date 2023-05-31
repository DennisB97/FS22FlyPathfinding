---@class FlyPathfinding adds mod event listener to this.
-- So that GridMap3D can be created on loadmap and then inited in loadMapData.
FlyPathfinding = {}
FlyPathfinding.modName = g_currentModName;
FlyPathfinding.modDir = g_currentModDirectory;

--Global 3d grid ref
g_GridMap3D = nil

--- loadMap is FS22 function called after loading or creating a new save game.
function FlyPathfinding:loadMap(savegame)

    -- create on server only the navigation grid
    if g_server ~= nil and g_GridMap3D == nil then
        g_GridMap3D = GridMap3D.new()
        -- adds a debugging console command to be able to visualize the octree and A* pathfinding.
        addConsoleCommand( 'GridMap3DOctreeDebug', 'toggle debugging for octree', 'octreeDebugToggle', g_GridMap3D)
        addConsoleCommand( 'AStarFlypathfindingDebug', 'toggle debugging for AStar pathfinding', 'aStarDebugToggle', AStar)
        addConsoleCommand( 'AStarFlypathfindingDebugPathCreate', 'Given two vector positions creates a debug path between those', 'aStarDebugPathCreate', AStarDebug)
        addConsoleCommand( 'CatmullRomDebug', 'toggle debugging for catmullrom', 'catmullRomDebugToggle', CatmullRomSplineCreator)
        addConsoleCommand( 'CatmullRomDebugSplineCreate', 'given at least 2 x y z points creates a catmullrom', 'catmullRomDebugSplineCreate', CatmullRomDebug)
        g_GridMap3D:register(true)
    end

end

--- deleteMap is FS22 function called after exiting played save.
function FlyPathfinding:deleteMap(savegame)
	
    -- delete the 3d grid if it hasn't already
    if g_GridMap3D ~= nil and not g_GridMap3D.isDeleted then
        g_GridMap3D:delete()
    end

    g_GridMap3D = nil
end

--- Hook after the farmlandmanager's loadmapdata, where the g_currentMission and g_currentMission.terrainNode will be at least valid.
-- Handles initing the class for 3d navigation grid.
function FlyPathfinding:loadMapData(xmlFile)
	
    if g_GridMap3D ~= nil then
        if g_GridMap3D:init() == false then
            g_GridMap3D:delete()
            g_GridMap3D = nil
        end
    end

end

FarmlandManager.loadMapData = Utils.appendedFunction(FarmlandManager.loadMapData,FlyPathfinding.loadMapData)
addModEventListener(FlyPathfinding)