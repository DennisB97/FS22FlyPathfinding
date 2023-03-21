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

-- All the directions that pathfinding can take within the octree grid.
local ENavDirection = {NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4, UP = 5, DOWN = 6}

-- Lookuptable for getting opposite direction from given key direction.
local mirroredDirectionTable = {
    [ENavDirection.NORTH] = ENavDirection.SOUTH,
    [ENavDirection.EAST] = ENavDirection.WEST,
    [ENavDirection.SOUTH] = ENavDirection.NORTH,
    [ENavDirection.WEST] = ENavDirection.EAST,
    [ENavDirection.UP] = ENavDirection.DOWN,
    [ENavDirection.DOWN] = ENavDirection.UP,
}

-- Lookuptable for getting next node, which is not within leaf node's tiniest voxels.
-- node provided as table of {GridMap3DNode,leafVoxelIndex(-1 - 63)}.
local nodeAdvancementTable = {
    [ENavDirection.NORTH] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].xNeighbour,-1}
    end,
    [ENavDirection.EAST] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].zNeighbour,-1}
    end,
    [ENavDirection.SOUTH] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].xMinusNeighbour,-1}
    end,
    [ENavDirection.WEST] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].zMinusNeighbour,-1}
    end,
    [ENavDirection.UP] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].yNeighbour,-1}
    end,
    [ENavDirection.DOWN] = function(node)
        if node == nil or node[1] == nil then
            return {nil,-1}
        end
        return {node[1].yMinusNeighbour,-1}
    end,
}

-- Lookuptable for getting all 4 nodes on edge from given key direction side.
-- node provided as table of {GridMap3DNode,leafVoxelIndex(-1 - 63)}.
local gridNodeChildrenWallPerDirection = {
    [ENavDirection.NORTH] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[2],-1},{node[1].children[4],-1},{node[1].children[6],-1},{node[1].children[8],-1}}
    end,
    [ENavDirection.EAST] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[3],-1},{node[1].children[4],-1},{node[1].children[7],-1},{node[1].children[8],-1}}
    end,
    [ENavDirection.SOUTH] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[1],-1},{node[1].children[3],-1},{node[1].children[7],-1},{node[1].children[5],-1}}
    end,
    [ENavDirection.WEST] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[1],-1},{node[1].children[2],-1},{node[1].children[5],-1},{node[1].children[6],-1}}
    end,
    [ENavDirection.UP] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[5],-1},{node[1].children[6],-1},{node[1].children[7],-1},{node[1].children[8],-1}}
    end,
    [ENavDirection.DOWN] = function(node)
        if node == nil or node[1] == nil or node[1].children == nil then
            return {}
        end
        return {{node[1].children[1],-1},{node[1].children[2],-1},{node[1].children[3],-1},{node[1].children[4],-1}}
    end,
}

-- Lookuptable for getting the outer neighbour leaf voxel.
-- node provided as table of {GridMap3DNode,leafVoxelIndex(-1 - 63)}.
local gridLeafNodeChildPerDirection = {
        [ENavDirection.NORTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            -- if neighbour is same size as leaf node and not completely non-solid then can get the neighbouring leaf voxel
            if node[1].xNeighbour ~= nil and node[1].xNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].xNeighbour,node[2] - 3}
            -- else just takes the leaf node itself as possible open node
            elseif node[1].xNeighbour ~= nil then
                    return {node[1].xNeighbour,-1}
            end

            return {nil,-1}
            end,
        [ENavDirection.EAST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].zNeighbour ~= nil and node[1].zNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].zNeighbour,node[2] - 12}
            elseif node[1].zNeighbour ~= nil then
                    return {node[1].zNeighbour,-1}
            end

            return {nil,-1}
            end,
        [ENavDirection.SOUTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].xMinusNeighbour ~= nil and node[1].xMinusNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].xMinusNeighbour,node[2] + 3}
            elseif node[1].xMinusNeighbour ~= nil then
                    return {node[1].xMinusNeighbour,-1}
            end

            return {nil,-1}
            end,
        [ENavDirection.WEST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].zMinusNeighbour ~= nil and node[1].zMinusNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].zMinusNeighbour,node[2] + 12}
            elseif node[1].zMinusNeighbour ~= nil then
                    return {node[1].zMinusNeighbour,-1}
            end

            return {nil,-1}
            end,
        [ENavDirection.UP] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].yNeighbour ~= nil and node[1].yNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].yNeighbour,node[2] + 16 - 64}
            elseif node[1].yNeighbour ~= nil then
                    return {node[1].yNeighbour,-1}
            end

            return {nil,-1}
            end,
        [ENavDirection.DOWN] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].yMinusNeighbour ~= nil and node[1].yMinusNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].yMinusNeighbour,node[2] + 64 - 16}
            elseif node[1].yMinusNeighbour ~= nil then
                    return {node[1].yMinusNeighbour,-1}
            end

            return {nil,-1}
            end,
}
-- Lookuptable for getting all the leaf node's voxels on the edge for given key direction.
local gridLeafNodeChildrenWallPerDirection = {
        [ENavDirection.NORTH] = function()
            return {[3] = 3,[7] = 7,[11] = 11,[15] = 15,[19] = 19,[23] = 23,[27] = 27,[31] = 31,[35] = 35,[39] = 39,[43] = 43,[47] = 47,[51] = 51,[55] = 55,[59] = 59,[63] = 63}
            end,
        [ENavDirection.EAST] = function()
            return {[12] = 12,[13] = 13,[14] = 14,[15] = 15,[28] = 28,[29] = 29,[30] = 30,[31] = 31,[44] = 44,[45] = 45,[46] = 46,[47] = 47,[60] = 60,[61] = 61,[62] = 62,[63] = 63}
            end,
        [ENavDirection.SOUTH] = function()
            return {[0] = 0,[4] = 4,[8] = 8,[12] = 12,[16] = 16,[20] = 20,[24] = 24,[28] = 28,[32] = 32,[36] = 36,[40] = 40,[44] = 44,[48] = 48,[52] = 52,[56] = 56,[60] = 60}
            end,
        [ENavDirection.WEST] = function()
            return {[0] = 0,[1] = 1,[2] = 2,[3] = 3,[16] = 16,[17] = 17,[18] = 18,[19] = 19,[32] = 32,[33] = 33,[34] = 34,[35] = 35,[48] = 48,[49] = 49,[50] = 50,[51] = 51}
            end,
        [ENavDirection.UP] = function()
            return {[48] = 48,[49] = 49,[50] = 50,[51] = 51,[52] = 52,[53] = 53,[54] = 54,[55] = 55,[56] = 56,[57] = 57,[58] = 58,[59] = 59,[60] = 60,[61] = 61,[62] = 62,[63] = 63}
            end,
        [ENavDirection.DOWN] = function()
            return {[0] = 0,[1] = 1,[2] = 2,[3] = 3,[4] = 4,[5] = 5,[6] = 6,[7] = 7,[8] = 8,[9] = 9,[10] = 10,[11] = 11,[12] = 12,[13] = 13,[14] = 14,[15] = 15}
            end,
}
-- Lookuptable for returning the advancing to next voxel index or leaf node with given key direction.
-- node provided as table of {GridMap3DNode,leafVoxelIndex(-1 - 63)}.
local leafNodeAdvancementTable = {
        [ENavDirection.NORTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            -- if current leaf voxel index indicates being on the edge, then new node is in outer neighbour
            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end
            -- if not on edge then the next leaf voxel is within this same node, increments the index only.
            return {{node[1],node[2] + 1}}
            end,
        [ENavDirection.EAST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end

            return {{node[1],node[2] + 4}}
            end,
        [ENavDirection.SOUTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end

            return {{node[1],node[2] - 1}}
            end,
        [ENavDirection.WEST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end

            return {{node[1],node[2] - 4}}
            end,
        [ENavDirection.UP] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end

            return {{node[1],node[2] + 16}}
            end,
        [ENavDirection.DOWN] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {gridLeafNodeChildPerDirection[direction](node)}
            end

            return {{node[1],node[2] - 16}}
            end,
}


---@class AStarOpenQueue
--Min max heap for the open nodes queue.
AStarOpenQueue = {}
AStarOpenQueue_mt = Class(AStarOpenQueue,Object)
InitObjectClass(AStarOpenQueue, "AStarOpenQueue")

--- new creates a new openqueue.
function AStarOpenQueue.new()
    local self = setmetatable({},AStarOpenQueue_mt)
    self.openNodes = {}
    -- the hash table works as double table, first key will be the gridNode
    -- eg. self.hash[someGridNode]
    -- the value will be another table where the leaf voxel index is the key.
    -- eg. self.hash[someGridNode][leafvoxelIndex] -- and then finally value is index into the self.openNodes.
    self.hash = {}
    return self
end

--- getSize returns the size of the openqueue.
function AStarOpenQueue:getSize()
    return #self.openNodes
end
--- getParent returns parent index.
--@param i is the index from parent wanted.
function AStarOpenQueue:getParent(i)
    return math.floor(i / 2)
end
--- getLeftChild returns left child.
--@param i is the parent index from where left child index will be taken.
function AStarOpenQueue:getLeftChild(i)
    return 2*i
end
--- getRightChild returns right child.
--@param i is the parent index from where right child index will be taken.
function AStarOpenQueue:getRightChild(i)
    return 2*i + 1
end
--- empty cleans the openqueue for next path.
function AStarOpenQueue:empty()
    self.openNodes = nil
    self.openNodes = {}
    self.hash = nil
    self.hash = {}
    self.size = 0
end
--- insert used for adding a new node into the open queue.
-- checks if it already contains the same node and updates it if g value is lower on new node.
--@param node is the node to be added into the openqueue of type AStarNode
function AStarOpenQueue:insert(node)
    if node == nil or node.gridNode[1] == nil then
        return
    end

    if self:contains(node) then
        self:update(node)
        return
    end

    table.insert(self.openNodes,node)
    local index = #self.openNodes
    -- need to make sure there is a second table within the self.hash if this gridNode has never been added before it would be nil.
    if self.hash[node.gridNode[1]] == nil then
        self.hash[node.gridNode[1]] = {}
    end
    self.hash[node.gridNode[1]][node.gridNode[2]] = index

    local parentIndex = self:getParent(index)

    -- swap until it is in the right place
    while index > 1 and self.openNodes[parentIndex].f > self.openNodes[index].f do
        self:swap(index,parentIndex)
        index = parentIndex
        parentIndex = self:getParent(index)
    end

end

--- pop called to remove the root node which has the lowest f.
--@return AStarNode which had the lowest f value in the openqueue, nil if empty.
function AStarOpenQueue:pop()
    if #self.openNodes < 1 then
        return nil
    end

    if #self.openNodes == 1 then
        local node = table.remove(self.openNodes)
        self.hash[node.gridNode[1]][node.gridNode[2]] = nil
        return node
    end

    local node = self.openNodes[1]
    self:swap(1,#self.openNodes)
    table.remove(self.openNodes)
    self.hash[node.gridNode[1]][node.gridNode[2]] = nil
    self:heapify(1)
    return node
end

--- contains checks if a given node is already in the queue.
--@param node is the node that will be checked if already exists in the queue, of type AStarNode.
--@return true if the node exists in the queue.
function AStarOpenQueue:contains(node)

    if node ~= nil and node.gridNode[1] ~= nil then
        if self.hash[node.gridNode[1]] ~= nil then
            if self.hash[node.gridNode[1]][node.gridNode[2]] ~= nil then
                return true
            end
        end
    end

    return false
end

--- swap will swap two given indices within the self.openNodes, and then also correct the hash table.
--@param i index of one of the nodes that will be swapped.
--@param j index of the other of the nodes that will be swapped.
function AStarOpenQueue:swap(i,j)

    local temp = self.openNodes[j]
    self.openNodes[j] = self.openNodes[i]
    self.openNodes[i] = temp
    self.hash[self.openNodes[i].gridNode[1]][self.openNodes[i].gridNode[2]] = i
    self.hash[self.openNodes[j].gridNode[1]][self.openNodes[j].gridNode[2]] = j

end

--- update will try to update existing value in the queue if the new node's g value is lower than previously.
--@param node that will be updated, type AStarNode.
function AStarOpenQueue:update(node)

    if node ~= nil and node.gridNode[1] ~= nil and self.hash[node.gridNode[1]] ~= nil and self.hash[node.gridNode[1]][node.gridNode[2]] ~= nil then

        local index = self.hash[node.gridNode[1]][node.gridNode[2]]
        if self.openNodes[index].g > node.g then
            -- g was lower in new node then updates the values on the queue node and replaces parent.
            self.openNodes[index].g = node.g
            self.openNodes[index].f = node.g + node.h
            self.openNodes[index].parent = node.parent

            local parentIndex = self:getParent(index)
            -- Try to swap new updated node into place if f is lower than some older.
            while index > 1 and self.openNodes[parentIndex].f > self.openNodes[index].f do
                self:swap(index,parentIndex)
                index = parentIndex
                parentIndex = self:getParent(index)
            end

        end
    end
end

--- heapify adjust the heap to be correct.
--@param index starting parent index from where to children will be checked and adjusted.
function AStarOpenQueue:heapify(index)

    if #self.openNodes <= 1 then
        return
    end

    local leftIndex = self:getLeftChild(index)
    local rightIndex = self:getRightChild(index)
    local smallestIndex = index

    if leftIndex <= #self.openNodes and self.openNodes[leftIndex].f < self.openNodes[index].f then
        smallestIndex = leftIndex
    end

    if rightIndex <= #self.openNodes and self.openNodes[rightIndex].f < self.openNodes[smallestIndex].f then
        smallestIndex = rightIndex
    end

    if smallestIndex ~= index then
        self:swap(index,smallestIndex)
        self:heapify(smallestIndex)
    end

end


---@class AStarNode is used for the pathfinding when opening new nodes.
AStarNode = {}

--- new creates a new AStarNode.
--@param gridNode is the octree node and leaf node index, like {node,index(-1 - 63)}.
--@param g currently traveled distance from start to this node.
--@param h is the heuristic from this node to goal node.
--@param direction is the direction taken from previous node to this node.
--@return the newly created AStarNode.
function AStarNode.new(gridNode,g,h,parent, direction)
    local self = setmetatable({},nil)
    self.gridNode = gridNode
    self.g = g
    self.h = h
    self.f = g + h
    self.parent = parent
    self.direction = direction
    return self
end


---@class AStarDebug.
--Custom debugging object class for the A* pathfinding algorithm.
AStarDebug = {}
AStarDebug.className = "AStarDebug"
AStarDebug_mt = Class(AStarDebug,Object)
InitObjectClass(AStarDebug, "AStarDebug")

--- new creates a new AStarDebug object.
function AStarDebug.new()
    local self = Object.new(g_server ~= nil or g_dedicatedServerInfo ~= nil,g_client ~= nil, AStarDebug_mt)
    self.debugPaths = {}
    self.currentDebugPathIndex = 1
    self.maxSavedDebugPaths = 5
    self.bShowClosedNodes = false

    if g_inputBinding ~= nil then
        local _, _eventId = g_inputBinding:registerActionEvent(InputAction.FLYPATHFINDING_DBG_ASTAR_PATH_PREVIOUS, self, self.debugPreviousPath, true, false, false, true, true, true)
        local _, _eventId = g_inputBinding:registerActionEvent(InputAction.FLYPATHFINDING_DBG_ASTAR_PATH_NEXT, self, self.debugNextPath, true, false, false, true, true, true)
        local _, _eventId = g_inputBinding:registerActionEvent(InputAction.FLYPATHFINDING_DBG_ASTAR_SHOW_CLOSEDNODES, self, self.toggleClosedNodes, true, false, false, true, true, true)
    end

    return self
end

--- delete function called to clean up and remove input bindings from the debug functions.
function AStarDebug:delete()

    if g_inputBinding ~= nil then
        g_inputBinding:removeActionEventsByTarget(self)
    end
    self.debugPaths = nil

    AStarDebug:superClass().delete(self)
end

--- debugNextPath is bound to keyinput to increase the currentDebugPathIndex.
function AStarDebug:debugNextPath()
    self.currentDebugPathIndex = MathUtil.clamp(self.currentDebugPathIndex + 1,1,#self.debugPaths)
end

--- debugPreviousPath is bound to keyinput to decrease the currentDebugPathIndex.
function AStarDebug:debugPreviousPath()
    self.currentDebugPathIndex = MathUtil.clamp(self.currentDebugPathIndex - 1,1,#self.debugPaths)
end

--- toggleClosedNodes is bound to keyinputs to toggle boolean to visualize the closed nodes or not.
function AStarDebug:toggleClosedNodes()
    self.bShowClosedNodes = not self.bShowClosedNodes
end

--- addPath is called to add a path that can be visualized.
--@param path is the path finished, table of {x,y,z}.
--@param closedNodes is the nodes that were visited and closed in the path, closedNodes[gridNode][leafvoxelindex] = gridNode.
--@param closedNodeCount is the amount of closed nodes.
--@param timeTaken is around how long it took for the path to find goal (if goal was even reached).
function AStarDebug:addPath(path,closedNodes,closedNodeCount,timeTaken)

    if #self.debugPaths == self.maxSavedDebugPaths then
        return
    end

    table.insert(self.debugPaths,{path,closedNodes})
    self.currentDebugPathIndex = MathUtil.clamp(self.currentDebugPathIndex,1,#self.debugPaths)
    if #self.debugPaths == 1 then
        self:raiseActive()
    end

    if #self.debugPaths[#self.debugPaths][1] < 1 then
        Logging.info(string.format("Path received! closed node count: %d , No path was able to be made!" ,closedNodeCount))
    else
        Logging.info(string.format("Path was finished! Time taken around: %f, closed node count: %d " ,timeTaken,closedNodeCount))
    end
end

--- update is called every tick if a path has been added, else raiseActive isn't called and this function does not run.
-- Debug visualizes the path from start to goal, and optionally shows all the closed nodes.
--@param dt is the deltaTime , not needed in this case.
function AStarDebug:update(dt)
    AStarDebug:superClass().update(self,dt)

    if self.bShowClosedNodes and self.debugPaths[self.currentDebugPathIndex] ~= nil then
        for _,indexHolder in pairs(self.debugPaths[self.currentDebugPathIndex][2]) do
            for _,closedGridNode in pairs(indexHolder) do
                if closedGridNode ~= nil then
                    if closedGridNode[2] == -1 then
                        DebugUtil.drawSimpleDebugCube(closedGridNode[1].positionX, closedGridNode[1].positionY, closedGridNode[1].positionZ, closedGridNode[1].size, 1, 0, 0)
                    else
                        local x,y,z = GridMap3DNode.getLeafVoxelLocation(closedGridNode[1],closedGridNode[2])
                        DebugUtil.drawSimpleDebugCube(x, y, z, 1, 0, 0, 1)
                    end
                end
            end
        end
    end

    if self.debugPaths[self.currentDebugPathIndex] ~= nil then
        self:raiseActive()
        for i,location in ipairs(self.debugPaths[self.currentDebugPathIndex][1]) do
            if i ~= 1 then
                local startX = self.debugPaths[self.currentDebugPathIndex][1][i-1].x
                local startY = self.debugPaths[self.currentDebugPathIndex][1][i-1].y
                local startZ = self.debugPaths[self.currentDebugPathIndex][1][i-1].z
                local endX = location.x
                local endY = location.y
                local endZ = location.z

                DebugUtil.drawDebugLine(startX, startY, startZ,endX ,endY , endZ, 0, 1, 0, 1, false)
            end
        end
    end


end


---@class AStar.
--Custom object class for the A* pathfinding algorithm.
AStar = {}
AStar.className = "AStar"
AStar.debugObject = nil
AStar_mt = Class(AStar,Object)
InitObjectClass(AStar, "AStar")

function AStar.aStarDebugToggle()
    if g_GridMap3D == nil or g_GridMap3D.bOctreeDebug then
        Logging.info("Can't turn on AStar flypathfinding debug at same time as Octree debug mode!")
        return
    end

    if AStar.debugObject == nil then
        AStar.debugObject = AStarDebug.new()
        AStar.debugObject:register(true)
    else
        AStar.debugObject:delete()
        AStar.debugObject = nil
    end

end


--- new creates a new A* pathfinding algorithm object.
function AStar.new()
    local self = Object.new(g_server ~= nil or g_dedicatedServerInfo ~= nil,g_client ~= nil, AStar_mt)
    self.open = AStarOpenQueue.new()
    self.closed = {}
    self.goalGridNode = nil
    self.startGridNode = nil
    self.bestNode = nil
    self.bFindNearest = false
    self.closedNodeCount = 0
    self.pathingTime = 0
    self.goalPath = {}
    self.callback = nil
    self.realStartLocation = {}
    self.realGoalLocation = {}
    self:loadConfig()
    return self
end

--- loadConfig called to load some default values for the pathfinding algorithm from xml file.
function AStar:loadConfig()
    local filePath = Utils.getFilename("config/config.xml", FlyPathfinding.modDir)
    local xmlFile = loadXMLFile("TempXML", filePath)

    -- If closed node list goes beyond this stops the search early.
    self.maxSearchedNodes = 100000
    -- How many loops per update to run pathfinding.
    self.maxPathfindLoops = 100

    if xmlFile ~= nil then
        if getXMLString(xmlFile, "Config.aStarConfig#maxSearchedNodes") ~= nil then
            self.maxSearchedNodes = getXMLInt(xmlFile,"Config.aStarConfig#maxSearchedNodes")
        end
        if getXMLString(xmlFile, "Config.aStarConfig#maxPathfindLoops") ~= nil then
            self.maxPathfindLoops = getXMLInt(xmlFile,"Config.aStarConfig#maxPathfindLoops")
        end
    end

end

--- clean called to clean up the AStar object for allowing reuse.
function AStar:clean()
    if self.open ~= nil then
        self.open:empty()
    end
    self.closed = nil
    self.closed = {}
    self.goalPath = nil
    self.goalPath = {}
    self.bFindNearest = false
    self.closedNodeCount = 0
    self.pathingTime = 0
    self.callback = nil
    self.goalGridNode = nil
    self.startGridNode = nil
    self.realStartLocation = nil
    self.realStartLocation = {}
    self.realGoalLocation = nil
    self.realGoalLocation = {}

end

--- find is the function called from any object that wants to do pathfinding.
--@param x is the start x coordinate
--@param y is the start y coordinate
--@param z is the start z coordinate
--@param x2 is the end x coordinate
--@param y2 is the end y coordinate
--@param z2 is the end z coordinate
--@param findNearest is a bool to indicate if should return the closest path to goal if goal was not reached.
--@param allowSolidStart is a bool to indicate if it is okay if start location is inside a solid node.
--@param allowSolidGoal is a bool to indicate if it is okay if end location is inside a solid node.
--@param callback is a function that wants to be called after pathfinding is done, returns the path as parameter.
function AStar:find(x,y,z,x2,y2,z2,findNearest,allowSolidStart,allowSolidGoal,callback)

    if g_GridMap3D == nil then
        return
    end

    self.realStartLocation.x, self.realStartLocation.y, self.realStartLocation.z = x,y,z
    self.realGoalLocation.x, self.realGoalLocation.y, self.realGoalLocation.z = x2,y2,z2
    self.startGridNode = g_GridMap3D:getGridNode(x,y,z,allowSolidStart)
    self.goalGridNode = g_GridMap3D:getGridNode(x2,y2,z2,allowSolidGoal)

    if self.startGridNode[1] == nil or self.goalGridNode[1] == nil or (self.startGridNode[1] == self.goalGridNode[1] and self.startGridNode[2] == self.goalGridNode[2]) then
        self:finishPathfinding()
    end

    local newPathNode = self:prepareNewNode(nil,self.startGridNode,ENavDirection.NORTH)

    self:addToOpen(newPathNode)
    self.bestNode = newPathNode
    self.bFindNearest = findNearest
    self.callback = callback
    self:raiseActive()
end

--- update here the pathfinding is looped per self.maxPathfindLoops, and raiseActive is only called when actually pathfinding.
--@param dt is deltaTime, used to get estimated time it took to generate the path.
function AStar:update(dt)
    AStar:superClass().update(self,dt)

    if self.startGridNode ~= nil then
        self:raiseActive()
        self.pathingTime = self.pathingTime + (dt / 1000)
        for i = 0, self.maxPathfindLoops do
            if self:doSearch() == true then
                self:finishPathfinding()
                return
            end
        end
    end

end

--- finishPathfinding is called when pathfinding ends.
-- if a debug object exists sends path and information about generated path to it.
-- Calls the provided callback and then cleans up for next path request.
function AStar:finishPathfinding()

    if AStar.debugObject ~= nil then
        AStar.debugObject:addPath(self.goalPath,self.closed,self.closedNodeCount,self.pathingTime)
    end
    if self.callback ~= nil then
        self.callback(self.goalPath)
    end

    self:clean()
end

--- doSearch is the function that handles iterating the path search.
function AStar:doSearch()

    local currentNode = self.open:pop()

    -- if open queue is empty no goal was found, returns nearest if so wanted by the bool bFindNearest
    if currentNode == nil then
        if self.bFindNearest then
            self:finalizePath(self.bestNode,false)
        end
        return true
    end

    -- Checks if current node is goal then can finalize path
    if self:isSameGridNode(currentNode.gridNode,self.goalGridNode) then
        self:finalizePath(currentNode,true)
        return true
    end

    self:addToClosed(currentNode)

    -- track the best node so far
    if currentNode.f  < self.bestNode.f then
        if GridMap3DNode.isSolid(currentNode.gridNode[1]) == false or GridMap3DNode.isLeaf(currentNode.gridNode[1]) then
            self.bestNode = currentNode
        end
    end

    -- early exit if search goes too long
    if self.closedNodeCount >= self.maxSearchedNodes then
        self:finalizePath(self.bestNode,false)
        return true
    end

    -- if has higher resolution child nodes opens those and returns
    if currentNode.gridNode[1].children ~= nil then
        self:openChildren(currentNode)
        return false
    elseif GridMap3DNode.isLeaf(currentNode.gridNode[1]) and currentNode.gridNode[2] == -1 and GridMap3DNode.isSolid(currentNode.gridNode[1]) then
        self:openLeafVoxels(currentNode)
        return false
    end

    -- try to open new nodes in each available direction
    for _, direction in pairs(ENavDirection) do

        -- if not within a leaf node's voxels then try open a normal node
        if currentNode.gridNode[2] == -1 then
            local nextGridNode = nodeAdvancementTable[direction](currentNode.gridNode)
            if nextGridNode[1] ~= nil then
                local newNode = self:prepareNewNode(currentNode,nextGridNode,direction)
                self:addToOpen(newNode)
            end
        else
            -- within a leaf voxel so need to try get the next leaf voxel index to open
            local leafVoxelNodes = leafNodeAdvancementTable[direction](currentNode.gridNode,direction)
            for _,nextLeafVoxelNode in ipairs(leafVoxelNodes) do
                if nextLeafVoxelNode[1] ~= nil then
                    local newNode = self:prepareNewNode(currentNode,nextLeafVoxelNode,direction)
                    self:addToOpen(newNode)
                end
            end
        end

    end

    return false
end

--- openChildren is called to open the child nodes of a given node, not leaf voxel children.
--@param node is the AStarNode that has higher resolution children to be open.
function AStar:openChildren(node)
    if node == nil then
        return
    end

    -- get all the nodes along the edge on the opposite direction from direction this node was opened from
    local newChildren = gridNodeChildrenWallPerDirection[mirroredDirectionTable[node.direction]](node.gridNode)

    if newChildren == nil then
        return
    end

    for _, newChild in pairs(newChildren) do
        local newPathNode = self:prepareNewNode(node.parent,newChild,node.direction)
        self:addToOpen(newPathNode)
    end

end

--- openLeafVoxels will be called to open the leaf voxels if given node is a leaf node and has some solid leaf voxels.
--@param is the AStarNode leaf node which has some solid leaf voxels.
function AStar:openLeafVoxels(node)
    if node == nil then
        return
    end

    local newLeafVoxelIndices = gridLeafNodeChildrenWallPerDirection[mirroredDirectionTable[node.direction]]()

    for _, newLeafVoxelIndex in pairs(newLeafVoxelIndices) do
        local newPathNode = self:prepareNewNode(node.parent,{node.gridNode[1],newLeafVoxelIndex},node.direction)
        self:addToOpen(newPathNode)
    end

end


--- prepareNewNode will create a new AStarNode with the provided variables.
--@param parent is the previous AStarNode.
--@param gridNode is the octree {GridMap3DNode,leafVoxelIndex(-1 - 63)} node this AStarNode presents.
--@param direction is the direction taken from previous to this node.
function AStar:prepareNewNode(parent,gridNode,direction)

    local g = 1
    local h = 0

    -- g is a unit cost of 1 for every step.
    if parent ~= nil then
        g = parent.g + 1
    end

    -- get the heuristic which is euclidean distance * scaling dependent on octree layer of this node.
    h = self:getHeuristic(gridNode,self.goalGridNode)

    return AStarNode.new(gridNode,g,h,parent,direction)
end

--- finalizePath called to reverse the found path so it goes from start->end.
--@param node is the final node that was reached.
--@param isGoal indicates if the final node is the goal node.
function AStar:finalizePath(node,isGoal)

    if node == nil then
        return
    end

    local currentNode = node

    -- if final node reached is the goal then can add the real goal location as the location else it will be the octree node center location
    if isGoal == true then
        currentNode = node.parent
        table.insert(self.goalPath,{x = self.realGoalLocation.x,y = self.realGoalLocation.y, z = self.realGoalLocation.z})
    end

    -- loop the linked nodes through the parent reference, inserting at the front of the self.goalPath
    while currentNode.parent ~= nil do
        if currentNode.gridNode[2] ~= -1 then
            local xLoc,yLoc,zLoc = GridMap3DNode.getLeafVoxelLocation(currentNode.gridNode[1],currentNode.gridNode[2])
            table.insert(self.goalPath,1,{x = xLoc,y = yLoc, z = zLoc})
        else
            table.insert(self.goalPath,1,{x = currentNode.gridNode[1].positionX,y = currentNode.gridNode[1].positionY, z = currentNode.gridNode[1].positionZ})
        end
        currentNode = currentNode.parent
    end
    -- lastly inserting the first location which is the given start location for the path search
    table.insert(self.goalPath,1,{x = self.realStartLocation.x,y = self.realStartLocation.y, z = self.realStartLocation.z})
    return
end

--- isClosed is called to check if a given gridNode is already in the closed list.
--@param gridNode is the {GridMap3DNode,leafVoxelIndex (-1 - 63)} that needs to be checked.
function AStar:isClosed(gridNode)
    if gridNode == nil or gridNode[1] == nil then
        return true
    end

    if self.closed[gridNode[1]] == nil then
        return false
    elseif self.closed[gridNode[1]][gridNode[2]] == nil then
        return false
    end

    return true
end

--- addToClosed will add a given AStarNode into the closed list.
--@param node is the AStarNode to be added into the closed list.
function AStar:addToClosed(node)

    if self.closed[node.gridNode[1]] == nil then
        self.closed[node.gridNode[1]] = {}
    end

    -- The node's gridNode is added into the closed list, as the AStarNode's variable aren't need in there.
    self.closed[node.gridNode[1]][node.gridNode[2]] = node.gridNode

    self.closedNodeCount = self.closedNodeCount + 1
end

--- isSameGridNode is called to comapare two given gridNodes {GridMap3DNode,leafVoxelIndex (-1 - 63)}
--@param node is the first given node to compare with.
--@param node2 is the second given node to compare with.
function AStar:isSameGridNode(node,node2)
    if node == nil or node2 == nil then
        return false
    end

    if node[1] == node2[1] and node[2] == node2[2] then
        return true
    end

    return false
end

--- getHeuristic is the heuristic distance between the two given nodes.
--@param node1 is the first grid node of type {GridMap3DNode,leafVoxelIndex (-1 - 63)} table.
--@param node1 is the second grid node of type {GridMap3DNode,leafVoxelIndex (-1 - 63)} table.
--@return a distance between the two nodes.
function AStar:getHeuristic(node1,node2)
    if node1 == nil or node2 == nil or node1[1] == nil or node2[1] == nil then
        return 0
    end

    local positionX,positionY,positionZ = node1[1].positionX, node1[1].positionY, node1[1].positionZ
    local positionX2,positionY2,positionZ2 = node2[1].positionX, node2[1].positionY, node2[1].positionZ

    if node1[2] > -1 then
        positionX, positionY, positionZ = GridMap3DNode.getLeafVoxelLocation(node1[1],node1[2])
    end

    if node2[2] > -1 then
        positionX2, positionY2, positionZ2 = GridMap3DNode.getLeafVoxelLocation(node2[1],node2[2])
    end

    return AStar.euclideanDistance(positionX,positionY,positionZ,positionX2,positionY2,positionZ2) * AStar.getHeuristicScaling(node1)
end

--- getHeuristicScaling is used to get scaling value for adding higher cost for higher resolution nodes.
--@param node is from which the scaling is received from, grid node is of type {GridMap3DNode,leafVoxelIndex (-1 - 63)} table.
function AStar.getHeuristicScaling(node)
    if g_GridMap3D == nil or node == nil or node[1] == nil then
        return
    end

    local layerScaling = g_GridMap3D:getNodeTreeLayer(node[1].size)

    -- if is in a leaf voxel then +1, as giving leaf node size to getNodeTreeLayer does not count leaf voxels as a layer.
    if node[2] > -1 then
        layerScaling = layerScaling + 1
    end

    return layerScaling
end

--- Helper function to calculate euclidean distance between two points
-- returns the distance between given two points.
function AStar.euclideanDistance(x1, y1, z1, x2, y2, z2)
  -- Return the square root of the sum of the squares of the differences between the coordinates
  return math.sqrt(math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2))
end

--- addToOpen called to try add a new AStarNode into the open queue, checks first if it is possible.
--@param is the AStarNode to try to be added into the open queue.
function AStar:addToOpen(newNode)
    if newNode == nil then
        return
    end

    if self:checkgridNodePossibility(newNode.gridNode) == false then
        return
    end

    self.open:insert(newNode)
end

--- checkGridNodePossiblity is used to check if the given gridNode is a possible next location in the path.
--@param gridNode is the grid node to be checked if is not solid or closed, of type {GridMap3DNode,leafVoxelIndex (-1 - 63)} table.
function AStar:checkgridNodePossibility(gridNode)

    if gridNode == nil or gridNode[1] == nil or self:isClosed(gridNode) then
        return false
    end

    -- if it is the goal have to add it into the open
    if self:isSameGridNode(gridNode,self.goalGridNode) then
        return true
    end

    -- if it is a leaf node need to check if the leaf node is not full solid and if it is within a leaf voxel that the leaf voxel is not solid.
    if GridMap3DNode.isLeaf(gridNode[1]) then
        if GridMap3DNode.isLeafFullSolid(gridNode[1]) then
            return false
        elseif gridNode[2] > -1 and GridMap3DNode.isLeafVoxelSolidAt(gridNode[1],gridNode[2]) then
            return false
        end

    else
        -- also avoiding adding any non leaf nodes that are completely under the terrain into the open queue.
        if GridMap3DNode.isUnderTerrain(gridNode[1]) then
            return false
        end

    end

    return true
end

