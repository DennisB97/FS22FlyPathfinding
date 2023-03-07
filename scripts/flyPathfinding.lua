
FlyPathfinding = {}
FlyPathfinding.modName = g_currentModName;
FlyPathfinding.modDir = g_currentModDirectory;

--Global 3d grid ref
g_GridMap3D = nil


function FlyPathfinding:loadMap(savegame)

    -- for now create on server only the navigation grid
    if g_server ~= nil or g_dedicatedServerInfo ~= nil and g_GridMap3D == nil then
        g_GridMap3D = GridMap3D.new()
        g_GridMap3D:register(true)
    end

end

function FlyPathfinding:deleteMap(savegame)
	
    if g_GridMap3D ~= nil and not g_GridMap3D.isDeleted then
        g_GridMap3D:delete()
    end

    g_GridMap3D = nil
end


--- Hook after the farmlandmanager's loadmapdata, where the g_currentMission and g_currentMission.terrainNode will be at least valid.
-- Handles initing the class for 3d navigation grid if server.
function FlyPathfinding:loadMapData(xmlFile)
	
    if g_GridMap3D ~= nil then
        g_GridMap3D:init()
    end

end

FarmlandManager.loadMapData = Utils.appendedFunction(FarmlandManager.loadMapData,FlyPathfinding.loadMapData)
addModEventListener(FlyPathfinding)