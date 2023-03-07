--[[
This file is part of Fly Pathfinding Mod (https://github.com/DennisB97/FS22FlyPathfinding)
MIT License
Copyright (c) 2023 Dennis B

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This mod is for personal use only and is not affiliated with GIANTS Software or endorsed by the game developer.
Selling or distributing this mod for a fee or any other form of consideration is prohibited by the game developer's terms of use and policies.
Please refer to the game developer's website for more information.
]]

---@class GridMap3DLatentUpdateAction enable a function to be run in sec update tick delay.
GridMap3DLatentUpdateAction = {}
GridMap3DLatentUpdateAction_mt = Class(GridMap3DLatentUpdateAction)
InitObjectClass(GridMap3DLatentUpdateAction, "GridMap3DLatentUpdateAction")

--- new creates a new GridMap3DLatentUpdateAction.
--@param inOwner, if the function will have a self reference this is it given to it.
--@param inFunction the function that will be run when delay has been reached.
--@param updateDelay is the delay in seconds to wait until function activation.
function GridMap3DLatentUpdateAction.new(inOwner,inFunction,updateDelay)
    local self = setmetatable({},GridMap3DLatentUpdateAction_mt)
    self.latentFunction = inFunction
    self.updateDelay = updateDelay
    self.delayCounter = 0
    self.owner = inOwner
    self.bFinished = false
    return self
end

--- run needs to be called when wanted from update to give deltaTime to it.
-- So that it will eventually call the function provided. Sets bFinished to true when it is done.
--@param dt is deltaTime that needs to be given each update to this function.
function GridMap3DLatentUpdateAction:run(dt)
    self.delayCounter = self.delayCounter + dt
    if self.delayCounter >= self.updateDelay and not self.bFinished then
        self.latentFunction(self.owner)
        self.delayCounter = 0
        self.bFinished = true
    end
end

---@class GridMap3DNode is one node of the octree.
GridMap3DNode = {}

--- new creates a new GridMap3DNode.
--@param x is the center x coordinate of node.
--@param y is the center y coordinate of node.
--@param z is the center z coordinate of node.
--@param parent is the lower resolution parent node.
--@param size is full length of the square node.
function GridMap3DNode.new(x,y,z,parent,size)
    local self = setmetatable({},nil)
    self.positionX = x
    self.positionY = y
    self.positionZ = z
    self.size = size

    -- Links to neighbours and parents as references to the node tables.
    self.parent = parent
    self.children = nil
    self.xNeighbour = nil
    self.xMinusNeighbour = nil
    self.yNeighbour = nil
    self.yMinusNeighbour = nil
    self.zNeighbour = nil
    self.zMinusNeighbour = nil
    -- most highest resolution is done with two 32bits, so a 4x4x4 grid. Only within leaf node is something else than nil value.
    self.leafVoxelsBottom = nil
    self.leafVoxelsTop = nil
    return self
end

--- isSolid checks if the provided node is solid or not
--@return true if the node was solid
function GridMap3DNode.isSolid(node)
    if node == nil then
        return true
    end

    if node.children == nil then
        if (node.leafVoxelsBottom ~= nil and node.leafVoxelsBottom ~= 0) or (node.leafVoxelsTop ~= nil and node.leafVoxelsTop ~= 0) then
            return true
        else
            return false
        end
    end

    return true
end

--- isLeafFullSolid checks if the provided leaf node is fully solid
--@return true if the node was fully solid
function GridMap3DNode.isLeafFullSolid(node)

    if node == nil or GridMap3DNode.isLeaf(node) == false then
        return true
    end

    local mask = 4294967295

    if node.leafVoxelsBottom == mask and node.leafVoxelsTop == mask then
        return true
    end

    return false
end

--- isLeaf checks if the provided node is a leaf node
--@return true if the node was a leaf
function GridMap3DNode.isLeaf(node)
    if node == nil then
        return false
    end

    if node.children == nil then
        if node.leafVoxelsBottom ~= nil and node.leafVoxelsTop ~= nil then
            return true
        end
    end

    return false
end

--- checkAABBIntersection simple helper function to find if two boxes intersect.
-- aabb's provided as table as following , minX,minY,minZ,maxX,maxY,maxZ.
--@return true if two provided boxes intersect.
function GridMap3DNode.checkAABBIntersection(aabb1, aabb2)
    if aabb1 == nil or aabb2 == nil or #aabb1 ~= 6 or #aabb2 ~= 6 then
        return false
    end

    if aabb1[1] > aabb2[4] or aabb2[1] > aabb1[4] or aabb1[2] > aabb2[5] or aabb2[2] > aabb1[5] or aabb1[3] > aabb2[6] or aabb2[3] > aabb1[6] then
        return false
    else
        return true
    end
end

--- checkPointInAABB simple helper function to find if a point is inside box.
-- aabb provided as table as following , minX,minY,minZ,maxX,maxY,maxZ.
--@return true if point is inside provided box.
function GridMap3DNode.checkPointInAABB(px, py, pz, aabb)
    if px == nil or py == nil or pz == nil or aabb == nil or #aabb ~= 6 then
        return false
    end

    if px >= aabb[1] and px <= aabb[4] and py >= aabb[2] and py <= aabb[5] and pz >= aabb[3] and pz <= aabb[6] then
        return true
    else
        return false
    end
end

---@class GridMap3DUpdate exists for creating an update table for the grid reacting to deletion or creation of a placeable.
GridMap3DUpdate = {}

--- new creates a new GridMap3DUpdate.
--@param id is the id of the placeable object sold or placed.
--@param x is the coordinate of the middle of the object.
--@param y is the coordinate of the middle of the object.
--@param z is the coordinate of the middle of the object.
--@param aabb is an box that contains the whole object, table constructed as minX,minY,minZ,maxX,maxY,maxZ.
--@param isDeletion is a boolean that states if the placeable object was deleted/sold or placed.
function GridMap3DUpdate.new(id,x,y,z,aabb,isDeletion)
    local self = setmetatable({},nil)
    self.positionX = x
    self.positionY = y
    self.positionZ = z
    self.id = id
    self.aabb = aabb
    self.bDeletion = isDeletion or false
    return self
end


---@class GridMap3D.
--Custom object class for the 3d navigation grid.
GridMap3D = {}
GridMap3D.className = "GridMap3D"
GridMap3D_mt = Class(GridMap3D,Object)
InitObjectClass(GridMap3D, "GridMap3D")


--- new creates a new grid map 3d.
function GridMap3D.new(customMt)

    local self = Object.new(true,false, customMt or GridMap3D_mt)
    -- adds a debugging console command to visualize the octree.
    addConsoleCommand( 'GridMap3DOctreeDebug', 'toggle debugging for octree', 'octreeDebugToggle', self)
    -- nodeTree will contain the root node of the octree which then has references to deeper nodes.
    self.nodeTree = {}
    self.terrainSize = 2048
    self.maxVoxelResolution = 2 -- in meters
    -- leaf node is four times the size of the highest resolution, as leaf node contains the highest resolution in a 4x4x4 grid.
    self.leafNodeResolution = self.maxVoxelResolution * 4
    -- contains all the states
    self.gridMap3DStates = {}
    self.EGridMap3DStates = {UNDEFINED = 0, PREPARE = 1, GENERATE = 2, DEBUG = 3, UPDATE = 4, IDLE = 5}
    self.EDirections = {X = 0, MINUSX = 1, Y = 2, MINUSY = 3, Z = 4, MINUSZ = 5}
    self.currentGridState = self.EGridMap3DStates.UNDEFINED
    -- this bool is toggled by console command to true and false
    self.octreeDebug = false
    -- contains the ids of the map boundaries which will be ignored and not marked as solid by the grid.
    self.mapBoundaryIDs = {}
    -- contains updates queued up and waiting to set to ready queue after a certain delay, as to not try to update the grid too fast as the collision might not be registered immediately.
    self.gridUpdateQueue = {}
    -- contains the updates that are ready to be updated in the grid.
    self.gridUpdateReadyQueue = {}
    -- contains the function events that should be notified after grid update has been increased
    self.onGridUpdateQueueIncreasedEvent = {}
    -- sets activated once the update has run once, so that any placeables that are placed while in loading screen won't be triggered to be updated in the grid.
    self.bActivated = false
    -- this contains the updates that are waiting a time before they are ready to be checked out.
    self.latentUpdates = {}
    -- used by the grid states to know if collision check was solid, set in the trace callback
    self.bTraceVoxelSolid = false
    self.collisionMask = CollisionFlag.STATIC_WORLD + CollisionFlag.WATER

    -- Appends to the finalizePlacement function which is called when a placeable is placed down.
    Placeable.finalizePlacement = Utils.appendedFunction(Placeable.finalizePlacement,
        function(...)
            self:onPlaceablePlaced(unpack({...}))
        end
    )
    -- prepends on to the onSell function of a placeable to get notified when a placeable has been sold/deleted
    Placeable.onSell = Utils.prependedFunction(Placeable.onSell,
        function(...)
            self:onPlaceableSold(unpack({...}))
        end
    )

    -- Creates all the needed states and changes to the prepare state initially.
    table.insert(self.gridMap3DStates,GridMap3DStatePrepare.new())
    self.gridMap3DStates[self.EGridMap3DStates.PREPARE]:init(self)
    table.insert(self.gridMap3DStates,GridMap3DStateGenerate.new())
    self.gridMap3DStates[self.EGridMap3DStates.GENERATE]:init(self)
    table.insert(self.gridMap3DStates,GridMap3DStateDebug.new())
    self.gridMap3DStates[self.EGridMap3DStates.DEBUG]:init(self)
    table.insert(self.gridMap3DStates,GridMap3DStateUpdate.new())
    self.gridMap3DStates[self.EGridMap3DStates.UPDATE]:init(self)

    -- add a function callback when grid needs an update.
    self:addGridUpdateQueueIncreasedEvent(self,GridMap3D.onGridNeedUpdate)


    registerObjectClassName(self, "GridMap3D")

    return self
end

function GridMap3D:init()

    self:changeState(self.EGridMap3DStates.PREPARE)


end

--- onGridNeedUpdate is a callback function for when an update has been readied for the grid.
-- Prepares a grid update into the ready queue and calls forward to change to update state.
--@param gridMap3D is the self reference which is provided.
function GridMap3D.onGridNeedUpdate(gridMap3D)

    if #gridMap3D.gridUpdateQueue < 1 then
        return
    end

    table.insert(gridMap3D.gridUpdateReadyQueue,gridMap3D.gridUpdateQueue[1])
    table.remove(gridMap3D.gridUpdateQueue,1)

    gridMap3D:changeState(gridMap3D.EGridMap3DStates.UPDATE)
end

--- delete function handles cleaning up the grid.
function GridMap3D:delete()

    self.isDeleted = true
    if self.gridMap3DStates[self.currentGridState] ~= nil then
        self.gridMap3DStates[self.currentGridState]:leave()
    end

    for _, state in pairs(self.gridMap3DStates) do

        if state ~= nil then
            state:destroy()
        end

    end

    self.gridMap3DStates = nil
    self.nodeTree = nil

    self.gridUpdateQueue = nil
    self.gridUpdateReadyQueue = nil
    self.onGridUpdateQueueIncreasedEvent = nil
    self.EGridMap3DStates = nil
    self.EDirections = nil
    self.latentUpdates = nil

    GridMap3D:superClass().delete(self)

    unregisterObjectClassName(self)
end

--- getVectorDistance tiny helper function to get the distance between two vectors.
--@param x is the first vector's x coordinate
--@param y is the first vector's y coordinate
--@param z is the first vector's z coordinate
--@param x2 is the second vector's x coordinate
--@param y2 is the second vector's y coordinate
--@param z2 is the second vector's z coordinate
function GridMap3D.getVectorDistance(x,y,z,x2,y2,z2)
    return math.sqrt(math.pow((x - x2),2) + math.pow((y - y2),2) + math.pow((z - z2),2))
end

--- changeState changes the
function GridMap3D:changeState(newState)

    if newState == nil or newState == 0 then
        Logging.warning("Not a valid state given to GridMap3D:changeState() _ ".. tostring(newState))
        return
    end

    if newState == self.currentGridState then
        return
    end


    -- if there is work queued when returning to idle should set to update state instead
    if self.currentGridState ~= self.EGridMap3DStates.GENERATE and #self.gridUpdateReadyQueue > 0 then
        newState = self.EGridMap3DStates.UPDATE
    end
    -- if debug is on then when returning to idle should set to debug state instead
    if newState == self.EGridMap3DStates.IDLE and self.octreeDebug then
        newState = self.EGridMap3DStates.DEBUG
    end

    if self.gridMap3DStates[self.currentGridState] ~= nil then
        self.gridMap3DStates[self.currentGridState]:leave()
    end

    self.currentGridState = newState

    if self.gridMap3DStates[self.currentGridState] ~= nil then
        self.gridMap3DStates[self.currentGridState]:enter()
    end

end

function GridMap3D:addGridUpdateQueueIncreasedEvent(inOwner,callbackFunction)

    if callbackFunction == nil then
        return
    end

    for _,existingEventFunction in pairs(self.onGridUpdateQueueIncreasedEvent) do

        if existingEventFunction.callbackFunction == callbackFunction then
            Logging.warning("Existing queue increased event callback in GridMap3D:addGridUpdateQueueIncreasedEvent()!")
            return
        end
    end

    table.insert(self.onGridUpdateQueueIncreasedEvent,{owner = inOwner,callbackFunction = callbackFunction})
end

function GridMap3D:removeGridUpdateQueueIncreasedEvent(inOwner,callbackFunction)

    if inOwner == nil and callbackFunction == nil then
        return
    end

    local toRemove = {}
    for i,existingEventFunction in ipairs(self.onGridUpdateQueueIncreasedEvent) do

        -- if no specific function given then removes all from the provided owner
        if existingEventFunction.owner == inOwner and callbackFunction == nil then
            table.insert(toRemove,i)
        elseif existingEventFunction.callbackFunction == callbackFunction then
            table.insert(toRemove,i)
        end
    end

    for i = #toRemove, 1, -1 do
        table.remove(self.onGridUpdateQueueIncreasedEvent,toRemove[i])
    end

end

function GridMap3D:update(dt)
    GridMap3D:superClass().update(self,dt)

    self:raiseActive()

    if self.bActivated == false then
        self.bActivated = true
    end

    for i = #self.latentUpdates, 1, -1 do
        if self.latentUpdates[i] ~= nil then
            self.latentUpdates[i]:run(dt / 1000)
            if self.latentUpdates[i].bFinished then
                table.remove(self.latentUpdates,i)
            end
        end
    end

    if self.gridMap3DStates[self.currentGridState] ~= nil then
         self.gridMap3DStates[self.currentGridState]:update(dt)
    end
end


function GridMap3D:QueueGridUpdate(newWork)

    if newWork == nil then
        return
    end

    -- if the same object has been queued before, it means this time it has been deleted before the grid has been updated so deletes and returns
    for i,gridUpdate in ipairs(self.gridUpdateQueue) do
        if gridUpdate.id == newWork.id then
            table.remove(self.gridUpdateQueue,i)
            return
        end
    end
    for i,gridUpdate in ipairs(self.gridUpdateReadyQueue) do
        if gridUpdate.id == newWork.id then
            table.remove(self.gridUpdateReadyQueue,i)
            return
        end
    end


    table.insert(self.gridUpdateQueue,newWork)

    local newLatentUpdate = GridMap3DLatentUpdateAction.new(self,
        function()
            for _, eventFunction in pairs(self.onGridUpdateQueueIncreasedEvent) do
                eventFunction.callbackFunction(eventFunction.owner)
            end
        end
    ,1)

    table.insert(self.latentUpdates,newLatentUpdate)

end

function GridMap3D:onPlaceablePlaced(placeable)
    self:onPlaceableModified(placeable,false)
end

function GridMap3D:onPlaceableSold(placeable)
    self:onPlaceableModified(placeable,true)
end


function GridMap3D:onPlaceableModified(placeable,isDeletion)

    if not self.bActivated or placeable == nil or placeable.rootNode == nil or placeable.spec_fence ~= nil then
        return
    end

    local x,y,z = getTranslation(placeable.rootNode)

    -- Init a new grid queue update with the placeable's id and location, and intially has some values set which might change down below if spec_placement exists.
    local newWork = GridMap3DUpdate.new(placeable.rootNode,x,y,z,{x - 50, y - 50, z - 50, x + 50, y + 50, z + 50},isDeletion)


    if placeable.spec_placement ~= nil and placeable.spec_placement.testAreas ~= nil and #placeable.spec_placement.testAreas > 0 then
        -- init beyond possible coordinate ranges
        local minX,minY,minZ,maxX,maxY,maxZ = 99999,99999,99999,-99999,-99999,-99999

        for _, area in ipairs(placeable.spec_placement.testAreas) do

            local startX,startY,startZ = getWorldTranslation(area.startNode)
            local endX,endY,endZ = getWorldTranslation(area.endNode)

            local xDistance = math.abs(startX - endX)
            local zDistance = math.abs(startZ - endZ)
            local halfDistance = math.max(xDistance,zDistance) / 2
            local halfYDistance = math.abs(startY - endY) / 2
            minX = math.min(minX, x - halfDistance)
            minZ = math.min(minZ, z - halfDistance)
            minY = math.min(minY, y - halfYDistance)

            maxX = math.max(maxX, x + halfDistance)
            maxZ = math.max(maxZ, z + halfDistance)
            maxY = math.max(maxY, y + math.abs(startY - endY))
        end

        newWork.positionX = x
        newWork.positionY = y + (math.abs(minY - maxY) / 2)
        newWork.positionZ = z

        newWork.aabb = {minX,minY,minZ,maxX,maxY,maxZ}
    end

    self:QueueGridUpdate(newWork)

end

function GridMap3D:getNodeTreeLayer(size)
    if size < 1 or size == nil then
        return 1
    end

    local dividedBy = self.terrainSize / size
    -- + 1 as the layer 1 is the root
    return ((math.log(dividedBy)) / (math.log(2))) + 1
end


--- voxelOverlapCheck is called when a new node/leaf voxel need to be checked for collision.
-- first it checks the terrain height, if the terrain is higher than the node's y extent then can skip wasting time to collision check as it can be counted as non-solid.
--@param x is the center coordinate of node/leaf voxel to be checked.
--@param y is the center coordinate of node/leaf voxel to be checked.
--@param z is the center coordinate of node/leaf voxel to be checked.
--@param extentRadius is the radius of the node/leaf voxel to be checked.
--@return true if was a collision on checked location
function GridMap3D:voxelOverlapCheck(x,y,z, extentRadius)
    self.bTraceVoxelSolid = false

    local terrainHeight = 0
    if g_currentMission.terrainRootNode ~= nil then
        terrainHeight = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode,x,y,z)
    end

    if y + extentRadius < terrainHeight then
        return
    end


    overlapBox(x,y,z,0,0,0,extentRadius,extentRadius,extentRadius,"voxelOverlapCheckCallback",self,self.collisionMask,false,true,true,false)

end

--- voxelOverlapCheckCallback is callback function for the overlapBox.
-- Checks if there was any object id found, or if it was the terrain or if it was the boundary then can ignore those.
-- If it wasn't any of the above then it checks if it has the ClassIds.SHAPE, if it does then it is counted as solid.
--@param hitObjectId is the id of collided thing.
function GridMap3D:voxelOverlapCheckCallback(hitObjectId)

    if hitObjectId < 1 or hitObjectId == g_currentMission.terrainRootNode or self.mapBoundaryIDs[hitObjectId] then
        return true
    end


    if getHasClassId(hitObjectId,ClassIds.SHAPE) and bitAND(getCollisionMask(hitObjectId),CollisionFlag.TREE) ~= CollisionFlag.TREE  then
        self.bTraceVoxelSolid = true
        return false

    end

    return true
end


--- createLeafVoxels is called when layer index is reached for the leaf nodes.
-- the leaftvoxels are 4x4x4 voxels within the leaf node.
-- Because of limited bit manipulation, the FS bitOR&bitAND works up to 32bits.
-- So the voxels were divided into to variables, bottom 32 voxels into one, and the top 32 voxels into another.
-- Where each bit indicates if it is a solid or empty.
--@param parent node which owns these leaf voxels.
function GridMap3D:createLeafVoxels(parent)

    if parent == nil then
        return
    end


    parent.leafVoxelsBottom = 0
    parent.leafVoxelsTop = 0

    -- early check if no collision for whole leaf node then no inner 64 voxels need to be checked
    self:voxelOverlapCheck(parent.positionX,parent.positionY,parent.positionZ,parent.size / 2)
    if self.bTraceVoxelSolid == false then
        return
    end


    local startPositionX = parent.positionX - self.maxVoxelResolution - (self.maxVoxelResolution / 2)
    local startPositionY = parent.positionY - self.maxVoxelResolution - (self.maxVoxelResolution / 2)
    local startPositionZ = parent.positionZ - self.maxVoxelResolution - (self.maxVoxelResolution / 2)

    local count = 0
    for y = 0, 1 do
        for z = 0 , 3 do
            for x = 0, 3 do
                local currentPositionX = startPositionX + (self.maxVoxelResolution * x)
                local currentPositionY = startPositionY + (self.maxVoxelResolution * y)
                local currentPositionZ = startPositionZ + (self.maxVoxelResolution * z)
                self:voxelOverlapCheck(currentPositionX,currentPositionY,currentPositionZ,self.maxVoxelResolution / 2)

                -- if voxel was solid then set the bit to 1
                if self.bTraceVoxelSolid == true then
                    parent.leafVoxelsBottom = bitOR(parent.leafVoxelsBottom,( 1 * 2^count))
                end

                count = count + 1
            end
        end
    end


    count = 0
    for y = 2, 3 do
        for z = 0 , 3 do
            for x = 0, 3 do
                local currentPositionX = startPositionX + (self.maxVoxelResolution * x)
                local currentPositionY = startPositionY + (self.maxVoxelResolution * y)
                local currentPositionZ = startPositionZ + (self.maxVoxelResolution * z)
                self:voxelOverlapCheck(currentPositionX,currentPositionY,currentPositionZ,self.maxVoxelResolution / 2)

                -- if voxel was solid then set the bit to 1
                if self.bTraceVoxelSolid == true then
                    parent.leafVoxelsTop = bitOR(parent.leafVoxelsTop,( 1 * 2^count))
                end

                count = count + 1

            end
        end
    end

end

--- findNeighbours looks for the possible neighbours that the current childNumber can reach.
--@param node is the which needs its neighbours assigned.
--@param childNumber is the number of child, to know which location it is within the parent node.
function GridMap3D:findNeighbours(node,childNumber)

    if node == nil or childNumber == nil or childNumber < 1 or childNumber > 8 then
        return
    end

    if childNumber == 1 then
        self:findOutsideNeighbours(2,self.EDirections.MINUSX,node)
        self:findOutsideNeighbours(3,self.EDirections.MINUSZ,node)
        self:findOutsideNeighbours(5,self.EDirections.MINUSY,node)

    elseif childNumber == 2 then
        node.xMinusNeighbour = node.parent.children[1]
        node.parent.children[1].xNeighbour = node

        self:findOutsideNeighbours(4,self.EDirections.MINUSZ,node)
        self:findOutsideNeighbours(1,self.EDirections.X,node)
        self:findOutsideNeighbours(6,self.EDirections.MINUSY,node)

    elseif childNumber == 3 then
        node.zMinusNeighbour = node.parent.children[1]
        node.parent.children[1].zNeighbour = node

        self:findOutsideNeighbours(4,self.EDirections.MINUSX,node)
        self:findOutsideNeighbours(1,self.EDirections.Z,node)
        self:findOutsideNeighbours(7,self.EDirections.MINUSY,node)

    elseif childNumber == 4 then
        node.zMinusNeighbour = node.parent.children[2]
        node.parent.children[2].zNeighbour = node

        node.xMinusNeighbour = node.parent.children[3]
        node.parent.children[3].xNeighbour = node

        self:findOutsideNeighbours(8,self.EDirections.MINUSY,node)
        self:findOutsideNeighbours(2,self.EDirections.Z,node)
        self:findOutsideNeighbours(3,self.EDirections.X,node)



    elseif childNumber == 5 then
        node.yMinusNeighbour = node.parent.children[1]
        node.parent.children[1].yNeighbour = node

        self:findOutsideNeighbours(6,self.EDirections.MINUSX,node)
        self:findOutsideNeighbours(7,self.EDirections.MINUSZ,node)
        self:findOutsideNeighbours(1,self.EDirections.Y,node)

    elseif childNumber == 6 then
        node.yMinusNeighbour = node.parent.children[2]
        node.parent.children[2].yNeighbour = node

        node.xMinusNeighbour = node.parent.children[5]
        node.parent.children[5].xNeighbour = node

        self:findOutsideNeighbours(8,self.EDirections.MINUSZ,node)
        self:findOutsideNeighbours(5,self.EDirections.X,node)
        self:findOutsideNeighbours(2,self.EDirections.Y,node)



    elseif childNumber == 7 then
        node.yMinusNeighbour = node.parent.children[3]
        node.parent.children[3].yNeighbour = node

        node.zMinusNeighbour = node.parent.children[5]
        node.parent.children[5].zNeighbour = node


        self:findOutsideNeighbours(8,self.EDirections.MINUSX,node)
        self:findOutsideNeighbours(3,self.EDirections.Y,node)
        self:findOutsideNeighbours(5,self.EDirections.Z,node)

    elseif childNumber == 8 then
        node.yMinusNeighbour = node.parent.children[4]
        node.parent.children[4].yNeighbour = node

        node.xMinusNeighbour = node.parent.children[7]
        node.parent.children[7].xNeighbour = node

        node.zMinusNeighbour = node.parent.children[6]
        node.parent.children[6].zNeighbour = node

        self:findOutsideNeighbours(4,self.EDirections.Y,node)
        self:findOutsideNeighbours(6,self.EDirections.Z,node)
        self:findOutsideNeighbours(7,self.EDirections.X,node)

    end




end

--- findOutsideNeighbours tries to link the same resolution nodes from the parent's neighbours children.
-- if it fails to find same resolution it sets the neighbour as the lower resolution/bigger node parent's neighbour.
-- Also sets the outside neighbours opposite direction neighbour as the currently checked node.
--@param neighbourChildNumber is the child number which is suppose to be linked to the node.
--@param direction is the direction the neighbour is being checked from.
--@param node is the current node which has its neighbours linked.
function GridMap3D:findOutsideNeighbours(neighbourChildNumber,direction,node)

    if node == nil or direction == nil or neighbourChildNumber < 1 or neighbourChildNumber > 8 then
        return
    end

    local parentNode = node.parent

    if direction ==  self.EDirections.MINUSX then

        if parentNode.xMinusNeighbour ~= nil then

            local neighbourNode = parentNode.xMinusNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.xMinusNeighbour = neighbourNode
                return
            end

            node.xMinusNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].xNeighbour = node
            return
        end

    elseif direction == self.EDirections.MINUSY then

        if parentNode.yMinusNeighbour ~= nil then

            local neighbourNode = parentNode.yMinusNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.yMinusNeighbour = neighbourNode
                return
            end

            node.yMinusNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].yNeighbour = node
            return
        end


    elseif direction == self.EDirections.MINUSZ then

        if parentNode.zMinusNeighbour ~= nil then

            local neighbourNode = parentNode.zMinusNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.zMinusNeighbour = neighbourNode
                return
            end

            node.zMinusNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].zNeighbour = node
            return
        end

    elseif direction == self.EDirections.X then

        if parentNode.xNeighbour ~= nil then

            local neighbourNode = parentNode.xNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.xNeighbour = neighbourNode
                return
            end

            node.xNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].xMinusNeighbour = node
            return
        end


    elseif direction == self.EDirections.Y then

        if parentNode.yNeighbour ~= nil then

            local neighbourNode = parentNode.yNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.yNeighbour = neighbourNode
                return
            end

            node.yNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].yMinusNeighbour = node
            return
        end

    elseif direction == self.EDirections.Z then

        if parentNode.zNeighbour ~= nil then

            local neighbourNode = parentNode.zNeighbour
            -- if no children then setting the neighbour as the parents neighbour lower resolution.
            if neighbourNode.children == nil then
                node.zNeighbour = neighbourNode
                return
            end

            node.zNeighbour = neighbourNode.children[neighbourChildNumber]
            neighbourNode.children[neighbourChildNumber].zMinusNeighbour = node
            return
        end


    end



end



function GridMap3D:octreeDebugToggle()

    if self == nil then
        return
    end

    self.octreeDebug = not self.octreeDebug

    if self.octreeDebug and self.currentGridState == self.EGridMap3DStates.IDLE then
        self:changeState(self.EGridMap3DStates.DEBUG)
    elseif not self.octreeDebug and self.currentGridState == self.EGridMap3DStates.DEBUG then
        self:changeState(self.EGridMap3DStates.IDLE)
    end


end














