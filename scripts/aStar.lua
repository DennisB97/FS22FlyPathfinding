

    local ENavDirection = {NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4, UP = 5, DOWN = 6}

    local mirroredDirectionTable = {
        [ENavDirection.NORTH] = ENavDirection.SOUTH,
        [ENavDirection.EAST] = ENavDirection.WEST,
        [ENavDirection.SOUTH] = ENavDirection.NORTH,
        [ENavDirection.WEST] = ENavDirection.EAST,
        [ENavDirection.UP] = ENavDirection.DOWN,
        [ENavDirection.DOWN] = ENavDirection.UP,
    }


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

local gridLeafNodeChildPerDirection = {
        [ENavDirection.NORTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] == -1 then
                return {nil, -1}
            end

            if node[1].xNeighbour ~= nil and node[1].xNeighbour.size == node[1].size and GridMap3DNode.isSolid(node[1]) then
                    return {node[1].xNeighbour,node[2] - 3}
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
    -- TODO open leaf voxels all the way to the edge
    local leafNodeAdvancementTable = {
        [ENavDirection.NORTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] + 1}

            end,
        [ENavDirection.EAST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] + 4}

            end,
        [ENavDirection.SOUTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] - 1}

            end,
        [ENavDirection.WEST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] - 4}

            end,
        [ENavDirection.UP] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] + 16}

            end,
        [ENavDirection.DOWN] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return gridLeafNodeChildPerDirection[direction](node)
            end
            return {node[1],node[2] - 16}

            end,
    }


---@class AStarOpenQueue
--Min max heap for the open nodes queue.
AStarOpenQueue = {}
AStarOpenQueue_mt = Class(AStarOpenQueue,Object)
InitObjectClass(AStarOpenQueue, "AStarOpenQueue")

function AStarOpenQueue.new()
    local self = setmetatable({},AStarOpenQueue_mt)
    self.openNodes = {}
    self.hash = {}
    return self
end

function AStarOpenQueue:getSize()
    return #self.openNodes
end

function AStarOpenQueue:getParent(i)
    return math.floor(i / 2)
end

function AStarOpenQueue:getLeftChild(i)
    return 2*i
end

function AStarOpenQueue:getRightChild(i)
    return 2*i + 1
end

function AStarOpenQueue:empty()
    self.openNodes = nil
    self.openNodes = {}
    self.hash = nil
    self.hash = {}
    self.size = 0
end

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
    if self.hash[node.gridNode[1]] == nil then
        self.hash[node.gridNode[1]] = {}
    end
    self.hash[node.gridNode[1]][node.gridNode[2]] = index

    local parentIndex = self:getParent(index)

    while index > 1 and self.openNodes[parentIndex].f > self.openNodes[index].f do
        self:swap(index,parentIndex)
        index = parentIndex
        parentIndex = self:getParent(index)
    end

end

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

function AStarOpenQueue:swap(i,j)

    local temp = self.openNodes[j]
    self.openNodes[j] = self.openNodes[i]
    self.openNodes[i] = temp
    self.hash[self.openNodes[i].gridNode[1]][self.openNodes[i].gridNode[2]] = i
    self.hash[self.openNodes[j].gridNode[1]][self.openNodes[j].gridNode[2]] = j

end

function AStarOpenQueue:update(node)

    if node ~= nil and node.gridNode[1] ~= nil and self.hash[node.gridNode[1]] ~= nil and self.hash[node.gridNode[1]][node.gridNode[2]] ~= nil then

        local index = self.hash[node.gridNode[1]][node.gridNode[2]]
        if self.openNodes[index].g > node.g then
            self.openNodes[index].g = node.g
            self.openNodes[index].f = node.g + node.h
            self.openNodes[index].parent = node.parent

            local parentIndex = self:getParent(index)

            while index > 1 and self.openNodes[parentIndex].f > self.openNodes[index].f do
                self:swap(index,parentIndex)
                index = parentIndex
                parentIndex = self:getParent(index)
            end

        end
    end
end

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



AStarNode = {}

--- new creates a new AStarNode.
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

---@class AStar.
--Custom object class for the A* pathfinding algorithm.
AStar = {}
AStar.className = "AStar"
AStar_mt = Class(AStar,Object)
InitObjectClass(AStar, "AStar")


--- new creates a new 3d jump point pathfinding algorithm object.
--@param customMt optional customized base table.
function AStar.new(customMt)
    local self = Object.new(true,false, customMt or AStar_mt)
    self.open = AStarOpenQueue.new()
    self.closed = {}
    self.goalGridNode = nil
    self.startGridNode = nil
    self.bDone = false
    self.bestNode = nil
    self.bFindNearest = false
    self.closedNodeCount = 0
    self.pathingTime = 0
    self.goalPath = {}
    return self
end

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

end

function AStar:find(x,y,z,x2,y2,z2,findNearest)

    if g_GridMap3D == nil then
        return
    end

    self.startGridNode = self:getGridNode(x,y,z,g_GridMap3D.nodeTree)
    self.goalGridNode = self:getGridNode(x2,y2,z2,g_GridMap3D.nodeTree)
    local newPathNode = self:prepareNewNode(nil,self.startGridNode,ENavDirection.NORTH)

    self:addToOpen(newPathNode)
    self.bestNode = newPathNode
    self.bFindNearest = findNearest

end


function AStar:update(dt)
    AStar:superClass().update(self,dt)

    self:raiseActive()

--     for _,closedNode in pairs(self.closed) do
--
--         if closedNode ~= nil then
--             for _,closedTrueNode in pairs(closedNode) do
--                 if closedTrueNode ~= nil then
--                     DebugUtil.drawSimpleDebugCube(closedTrueNode.gridNode[1].positionX, closedTrueNode.gridNode[1].positionY, closedTrueNode.gridNode[1].positionZ, closedTrueNode.gridNode[1].size, 1, 0, 0)
--                 end
--             end
--         end
--
--     end

    for i,node in ipairs(self.goalPath) do
        if i ~= 1 then
            local startX = self.goalPath[i-1].gridNode[1].positionX
            local startY = self.goalPath[i-1].gridNode[1].positionY
            local startZ = self.goalPath[i-1].gridNode[1].positionZ
            local endX = node.gridNode[1].positionX
            local endY = node.gridNode[1].positionY
            local endZ = node.gridNode[1].positionZ

            if self.goalPath[i-1].gridNode[2] > -1 then
                startX,startY,startZ = GridMap3DNode.getLeafVoxelLocation(self.goalPath[i-1].gridNode[1],self.goalPath[i-1].gridNode[2])
            end

            if node.gridNode[2] > -1 then
                endX,endY,endZ = GridMap3DNode.getLeafVoxelLocation(node.gridNode[1],node.gridNode[2])
            end


            DebugUtil.drawDebugLine(startX, startY, startZ,endX ,endY , endZ, 0, 1, 0, 1, false)
        end
    end



    if self.startGridNode ~= nil and  self.bDone == false then
        self.pathingTime = self.pathingTime + (dt / 1000)
        for i = 0, 200 do
            if self:doSearch() == true then
                Logging.info(string.format("Done finding path! Took around %f ",self.pathingTime))
                Logging.info(string.format("Checked %d nodes ",self.closedNodeCount))
                self.bDone = true
                return
            end
        end
    end


end

function AStar:doSearch()

    local currentNode = self.open:pop()

    if currentNode == nil then
        Logging.info("Could not get all the way to goal!")
        if self.bFindNearest then
        Logging.info("Getting path to nearest goal node that was reached!")
        end
        return true
    end

    if self:isSameGridNode(currentNode.gridNode,self.goalGridNode) then
        Logging.info("goal node reached for pathfinding!")
        self:finalizePath(currentNode)
        return true
    end

    self:addToClosed(currentNode)

    if currentNode.f  < self.bestNode.f then
        self.bestNode = currentNode
    end

    if currentNode.gridNode[1].children ~= nil then
        self:openChildren(currentNode)
        return
    elseif GridMap3DNode.isLeaf(currentNode.gridNode[1]) and currentNode.gridNode[2] == -1 then
        self:openLeafVoxels(currentNode)
        return
    end

    for _, direction in pairs(ENavDirection) do
        local nextGridNode = nil
        if currentNode.gridNode[2] == -1 then
            nextGridNode = nodeAdvancementTable[direction](currentNode.gridNode)
        else
            nextGridNode = leafNodeAdvancementTable[direction](currentNode.gridNode,direction)
        end
        if nextGridNode[1] ~= nil then
            local newNode = self:prepareNewNode(currentNode,nextGridNode,direction)
            self:addToOpen(newNode)
        end
    end


    return false
end


function AStar:openChildren(node)
    if node == nil then
        return
    end


    local newChildren = gridNodeChildrenWallPerDirection[mirroredDirectionTable[node.direction]](node.gridNode)

    if newChildren == nil then
        return
    end

    for _, newChild in pairs(newChildren) do
        local newPathNode = self:prepareNewNode(node.parent,newChild,node.direction)
        self:addToOpen(newPathNode)
    end

end

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



function AStar:prepareNewNode(parent,gridNode,direction)


    local g = 1
    local h = 0

    if parent ~= nil then
        g = parent.g + 1
    end

    h = self:getHeuristic(gridNode,self.goalGridNode)

    return AStarNode.new(gridNode,g,h,parent,direction)
end


function AStar:finalizePath(node)

    if node == nil then
        return self.goalPath
    end

    local currentNode = node
    while currentNode.parent ~= nil do
        table.insert(self.goalPath,1,currentNode)
        currentNode = currentNode.parent
    end
    table.insert(self.goalPath,1,currentNode)
    return self.goalPath
end




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

function AStar:addToClosed(node)

    if self.closed[node.gridNode[1]] == nil then
        self.closed[node.gridNode[1]] = {}
    end

    self.closed[node.gridNode[1]][node.gridNode[2]] = node

    self.closedNodeCount = self.closedNodeCount + 1
end


function AStar:isSameGridNode(node,node2)
    if node == nil or node2 == nil then
        return false
    end

    if node[1] == node2[1] and node[2] == node2[2] then
        return true
    end

    return false
end


function AStar:getGridNode(x,y,z,node)

    if node == nil or g_GridMap3D == nil then
        return {nil,-1}
    end

    local currentNode = node

    local aabbNode = {currentNode.positionX - (currentNode.size / 2), currentNode.positionY - (currentNode.size / 2), currentNode.positionZ - (currentNode.size / 2),currentNode.positionX + (currentNode.size / 2),
        currentNode.positionY + (currentNode.size / 2), currentNode.positionZ + (currentNode.size / 2) }

    if GridMap3DNode.checkPointInAABB(x,y,z,aabbNode) == false then
        return {nil,-1}
    end

    if currentNode.children == nil then
        return currentNode
    end


    while true do

        -- need to check the voxels in 2 x 32 bits
        if currentNode.size == g_GridMap3D.leafNodeResolution then
        return self:findLeafVoxelNode(x,y,z,currentNode)

        elseif currentNode.children == nil then
            return {currentNode,-1}
        end

        for _ ,childNode in pairs(currentNode.children) do

            aabbNode = {childNode.positionX - (childNode.size / 2), childNode.positionY - (childNode.size / 2), childNode.positionZ - (childNode.size / 2),childNode.positionX + (childNode.size / 2),
            childNode.positionY + (childNode.size / 2), childNode.positionZ + (childNode.size / 2) }

            if GridMap3DNode.checkPointInAABB(x,y,z,aabbNode) == true then
                currentNode = childNode
                break
            end

        end

    end


    return nil
end

function AStar:findLeafVoxelNode(x,y,z,node)
    if node == nil or g_GridMap3D == nil then
        return {nil,-1}
    end

    if GridMap3DNode.isLeafFullSolid(node) then
        return {node , -1}
    end


    local startPositionX = node.positionX - g_GridMap3D.maxVoxelResolution - (g_GridMap3D.maxVoxelResolution / 2)
    local startPositionY = node.positionY - g_GridMap3D.maxVoxelResolution - (g_GridMap3D.maxVoxelResolution / 2)
    local startPositionZ = node.positionZ - g_GridMap3D.maxVoxelResolution - (g_GridMap3D.maxVoxelResolution / 2)
    local count = 0


    for yIndex = 0, 3 do
        for zIndex = 0, 3 do
            for xIndex = 0, 3 do
                local currentPositionX = startPositionX + (g_GridMap3D.maxVoxelResolution * xIndex)
                local currentPositionY = startPositionY + (g_GridMap3D.maxVoxelResolution * yIndex)
                local currentPositionZ = startPositionZ + (g_GridMap3D.maxVoxelResolution * zIndex)
                local aabbNode = {currentPositionX - (g_GridMap3D.maxVoxelResolution / 2), currentPositionY - (g_GridMap3D.maxVoxelResolution / 2), currentPositionZ - (g_GridMap3D.maxVoxelResolution / 2),
                    currentPositionX + (g_GridMap3D.maxVoxelResolution / 2), currentPositionY + (g_GridMap3D.maxVoxelResolution / 2), currentPositionZ + (g_GridMap3D.maxVoxelResolution / 2) }

                if GridMap3DNode.checkPointInAABB(x,y,z,aabbNode) == true then
                    return {node , count}
                end
                count = count + 1
            end
        end
    end

    return {nil,-1}
end




--- getHeuristic is the heuristic distance between the two given nodes.
--@param node1 is the first grid node of type GridMap3DNode table.
--@param node1 is the second grid node of type GridMap3DNode table.
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

    return AStar.euclideanDistance(positionX,positionY,positionZ,positionX2,positionY2,positionZ2)  * self:getHeuristicScaling(node1)
end

function AStar:getHeuristicScaling(node)
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


-- Define a function to calculate chebyshev distance between two points
function AStar.chebyshevDistance(x1, y1, z1, x2, y2, z2)
  -- Return the maximum of the absolute differences between the coordinates
  return math.max(math.abs(x1 - x2), math.abs(y1 - y2), math.abs(z1 - z2))
end

-- Define a function to calculate euclidean distance squared between two points
function AStar.euclideanDistance_squared(x1, y1, z1, x2, y2, z2)
  -- Return the sum of the squares of the differences between the coordinates
  return math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2)
end

-- Define a function to calculate euclidean distance between two points
function AStar.euclideanDistance(x1, y1, z1, x2, y2, z2)
  -- Return the square root of the sum of the squares of the differences between the coordinates
  return math.sqrt(math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2))
end


function AStar:addToOpen(newNode)
    if newNode == nil then
        return
    end

    if self:checkgridNodePossibility(newNode.gridNode) == false then
        return
    end

    self.open:insert(newNode)
end


function AStar:checkgridNodePossibility(gridNode)

    if gridNode == nil or gridNode[1] == nil or self:isClosed(gridNode) then
        return false
    end


    if self:isSameGridNode(gridNode,self.goalGridNode) then
        return true
    end


    if GridMap3DNode.isLeaf(gridNode[1]) then
        if GridMap3DNode.isLeafFullSolid(gridNode[1]) then
            return false
        elseif gridNode[2] > -1 and GridMap3DNode.isLeafVoxelSolidAt(gridNode[1],gridNode[2]) then
            return false
        end

    else

        if GridMap3DNode.isUnderTerrain(gridNode) then
            return false
        end

    end




    return true
end

