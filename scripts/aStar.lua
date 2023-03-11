

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

    local gridLeafNodeChildrenWallPerDirection = {
        [ENavDirection.NORTH] = function()
            return {[3] = 3,[7] = 7,[11] = 11,[15] = 15,[19] = 19,[23] = 23,[27] = 27,[31] = 31,[32] = 32,[39] = 39,[43] = 43,[47] = 47,[51] = 51,[55] = 55,[59] = 59,[63] = 63}
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

    local leafNodeAdvancementTable = {
        [ENavDirection.NORTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].xNeighbour,-1}
            end
            return {node[1],node[2] + 1}

            end,
        [ENavDirection.EAST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].zNeighbour,-1}
            end
            return {node[1],node[2] + 4}

            end,
        [ENavDirection.SOUTH] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].xMinusNeighbour,-1}
            end
            return {node[1],node[2] - 1}

            end,
        [ENavDirection.WEST] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].zMinusNeighbour,-1}
            end
            return {node[1],node[2] - 4}

            end,
        [ENavDirection.UP] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].yNeighbour,-1}
            end
            return {node[1],node[2] + 16}

            end,
        [ENavDirection.DOWN] = function(node,direction)
            if node == nil or node[1] == nil or node[2] < 0 then
                return {nil,-1}
            end

            local wallLeafNodes = gridLeafNodeChildrenWallPerDirection[direction]()
            if wallLeafNodes[node[2]] ~= nil then
                return {node[1].yMinusNeighbour,-1}
            end
            return {node[1],node[2] - 16}

            end,
    }

OpenQueue = {}
OpenQueue_mt = Class(OpenQueue)
InitObjectClass(OpenQueue, "OpenQueue")

--- new creates a new PriorityQueue.
function OpenQueue.new()
    local self = setmetatable({},OpenQueue_mt)
    self.openNodes = {}
    self.hash = {}
    self.size = 0
    return self
end

function OpenQueue:getSize()
    return self.size
end

function OpenQueue:swap(i, j)
  local temp = self.openNodes[i]
  self.openNodes[i] = self.openNodes[j]
  self.openNodes[j] = temp
  self.hash[self.openNodes[i].gridNode[1]][self.openNodes[i].gridNode[2]] = i
  self.hash[self.openNodes[j].gridNode[1]][self.openNodes[j].gridNode[2]] = j
end

function OpenQueue:shiftUp(i)
  while i > 1 do
    local parent = math.floor(i / 2)
    if JPS3DNode.compareNodes(self.openNodes[i], self.openNodes[parent]) then
      self:swap(i , parent)
      i = parent
    else
      break
    end
   end
end


-- A function to move an element down in a heap until it satisfies the heap property
function OpenQueue:shiftDown(i)
   local size = #self.openNodes
   while i *2 <= size do
     local child = i * 2
     if child < size and JPS3DNode.compareNodes(self.openNodes[child +1],self.openNodes[child]) then
       child=child +1
     end
     if JPS3DNode.compareNodes(self.openNodes[child],self.openNodes[i]) then
       self:swap(i ,child)
       i=child
     else
       break
     end
   end
end


function OpenQueue:insert(node)
    if self:contains(node) then
        self:update(node)
        return false
    end

    table.insert(self.openNodes ,node)
    self.size = self.size + 1
    if self.hash[node.gridNode[1]] == nil then
        self.hash[node.gridNode[1]] = {}
    end
    self.hash[node.gridNode[1]][node.gridNode[2]] = #self.openNodes
    self:shiftUp(#self.openNodes)
    return true
end

function OpenQueue:pop()
    if #self.openNodes == 0 then
        return nil
    elseif #self.openNodes == 1 then
        local node = self.openNodes[1]
        self.openNodes[1] = nil
        self.hash[node.gridNode[1]][node.gridNode[2]] = nil
        self.size = self.size - 1
        return node
    else
        local node = self.openNodes[1]
        self:swap(1, #self.openNodes)
        table.remove(self.openNodes)
        self.hash[node.gridNode[1]][node.gridNode[2]] = nil
        self:shiftDown(1)
        self.size = self.size - 1
        return node
    end
end

function OpenQueue:contains(node)

    if self.hash[node.gridNode[1]] == nil or self.hash[node.gridNode[1]][node.gridNode[2]] == nil then
        return false
    else
        return true
    end
end


function OpenQueue:update(node)
    if self.hash[node.gridNode[1]] ~= nil then
        local i = self.hash[node.gridNode[1]][node.gridNode[2]]
        if i ~= nil then
            local oldPriority = self.openNodes[i].f
            self.openNodes[i].f = node.f
            if node.f  < oldPriority then
                self:shiftUp(i)
            else
                self:shiftDown(i)
            end
        end
    end
end


JPS3DNode = {}

--- new creates a new JPS3DNode.
function JPS3DNode.new(gridNode,g,h,parent, direction)
    local self = setmetatable({},nil)
    self.gridNode = gridNode
    self.g = g or 0
    self.h = h or 0
    self.f = g + h or 0
    self.parent = parent
    self.direction = direction
    return self
end

-- A function to compare two nodes by their f values
function JPS3DNode.compareNodes(node, node2)
  return node.f < node2.f
end


---@class JPS3D.
--Custom object class for the 3d jump point pathfinding algorithm.
JPS3D = {}
JPS3D.className = "JPS3D"
JPS3D_mt = Class(JPS3D,Object)
InitObjectClass(JPS3D, "JPS3D")


--- new creates a new 3d jump point pathfinding algorithm object.
--@param customMt optional customized base table.
function JPS3D.new(customMt)
    local self = Object.new(true,false, customMt or JPS3D_mt)
    self.open = OpenQueue.new()
    self.closed = {}
    self.goalGridNode = nil
    self.startGridNode = nil
    self.bDone = false
    self.bestNode = nil
    self.bFindNearest = false
    self.closedNodeCount = 0
    self.goalPath = {}
    return self
end

function JPS3D:find(x,y,z,x2,y2,z2,findNearest)

    if g_GridMap3D == nil then
        return
    end

    self.startGridNode = self:getGridNode(x,y,z,g_GridMap3D.nodeTree)
    self.goalGridNode = self:getGridNode(x2,y2,z2,g_GridMap3D.nodeTree)
    local newPathNode = JPS3D:prepareNewNode(nil,self.startGridNode,nil)
    self:addToOpen(newPathNode)
    self.bestNode = newPathNode
    self.bFindNearest = findNearest

end


function JPS3D:update(dt)
    JPS3D:superClass().update(self,dt)

    self:raiseActive()

    for _,closedNode in pairs(self.closed) do

        if closedNode ~= nil then
            for _,closedTrueNode in pairs(closedNode) do
                if closedTrueNode ~= nil then
                    DebugUtil.drawSimpleDebugCube(closedTrueNode.gridNode[1].positionX, closedTrueNode.gridNode[1].positionY, closedTrueNode.gridNode[1].positionZ, closedTrueNode.gridNode[1].size, 1, 0, 0)
                end
            end
        end

    end

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
        for i = 0, 50 do
            if self:doSearch() == true then
                self.bDone = true
                return
            end
        end
    end


end

function JPS3D:doSearch()

    local currentNode = self.open:pop()

    if currentNode == nil then
        Logging.info("Could not get all the way to goal!")
        if self.bFindNearest then
        Logging.info("Getting path to nearest goal node that was reached!")
        end
        return true
    end

    print(string.format("currentNode being looked through: x:%d y:%d z:%d and leaf node index: %d",currentNode.gridNode[1].positionX, currentNode.gridNode[1].positionY,currentNode.gridNode[1].positionZ,currentNode.gridNode[2]))

    if self:isSameGridNode(currentNode.gridNode,self.goalGridNode) then
        Logging.info("goal node reached for pathfinding!")
        self:finalizePath(currentNode)
        return true
    end

    self:addToClosed(currentNode)

    if currentNode.h  < self.bestNode.h then
        self.bestNode = currentNode
    end

    if currentNode.gridNode[1].children ~= nil then
        self:openChildren(currentNode)
    elseif GridMap3DNode.isLeaf(currentNode.gridNode[1]) and currentNode.gridNode[2] < 0 then
        self:openLeafVoxels(currentNode)
    end

    for _, direction in pairs(ENavDirection) do

        if currentNode.gridNode[2] == -1 then
            local nextGridNode = nodeAdvancementTable[direction](currentNode.gridNode)
            if nextGridNode[1] ~= nil then
                self:addToOpen(self:prepareNewNode(currentNode,nextGridNode,direction))
            end
        else
            local nextGridNode = leafNodeAdvancementTable[direction](currentNode.gridNode,direction)
            if nextGridNode[1] ~= nil then
                self:addToOpen(self:prepareNewNode(currentNode,nextGridNode,direction))
            end
        end
    end


    return false
end


function JPS3D:openChildren(node)
    if node == nil then
        return
    end

    local newChildren = nil
    if node.direction == nil then
        for _, gridNodeChild in pairs(node.gridNode[1].children) do
            table.insert(newChildren,{gridNodeChild,-1})
        end
    else
        newChildren = gridNodeChildrenWallPerDirection[mirroredDirectionTable[node.direction]](node.gridNode)
    end

    if newChildren == nil then
        return
    end

    for _, newChild in pairs(newChildren) do
        local newPathNode = JPS3D:prepareNewNode(node.parent,newChild,node.direction)
        self:addToOpen(newPathNode)
    end

end

function JPS3D:openLeafVoxels(node)
    if node == nil then
        return
    end

    local newLeafVoxelIndices = gridLeafNodeChildrenWallPerDirection[mirroredDirectionTable[node.direction]]()
    if newLeafVoxelIndices == nil then
        return
    end

    for _, newLeafVoxelIndex in pairs(newLeafVoxelIndices) do
        local newPathNode = JPS3D:prepareNewNode(node.parent,{node.gridNode[1],newLeafVoxelIndex},node.direction)
        self:addToOpen(newPathNode)
    end

end



function JPS3D:prepareNewNode(parent,gridNode,direction)


    local g = 0
    local h = 0

    if parent ~= nil then
        g = parent.g + JPS3D.getHeuristic(parent.gridNode,gridNode)
    end

    h = JPS3D.getHeuristic(gridNode,self.goalGridNode)

    return JPS3DNode.new(gridNode,g,h,parent,direction)
end


function JPS3D:finalizePath(node)

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




function JPS3D:isClosed(gridNode)
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

function JPS3D:addToClosed(node)

    if self.closed[node.gridNode[1]] == nil then
        self.closed[node.gridNode[1]] = {}
    end

    self.closed[node.gridNode[1]][node.gridNode[2]] = node

    self.closedNodeCount = self.closedNodeCount + 1
end


function JPS3D:isSameGridNode(node,node2)
    if node == nil or node2 == nil then
        return false
    end

    if node[1] == node2[1] and node[2] == node2[2] then
        return true
    end

    return false
end


function JPS3D:getGridNode(x,y,z,node)

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

function JPS3D:findLeafVoxelNode(x,y,z,node)
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
function JPS3D.getHeuristic(node1,node2)
    if node1 == nil or node2 == nil then
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


    return JPS3D.euclidean_distance_squared(positionX,positionY,positionZ,positionX2,positionY2,positionZ2)
end

-- Define a function to calculate chebyshev distance between two points
function JPS3D.chebyshev_distance(x1, y1, z1, x2, y2, z2)
  -- Return the maximum of the absolute differences between the coordinates
  return math.max(math.abs(x1 - x2), math.abs(y1 - y2), math.abs(z1 - z2))
end

-- Define a function to calculate euclidean distance squared between two points
function JPS3D.euclidean_distance_squared(x1, y1, z1, x2, y2, z2)
  -- Return the sum of the squares of the differences between the coordinates
  return math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2)
end

-- Define a function to calculate euclidean distance between two points
function JPS3D.euclidean_distance(x1, y1, z1, x2, y2, z2)
  -- Return the square root of the sum of the squares of the differences between the coordinates
  return math.sqrt(math.pow((x1 - x2),2) + math.pow((y1 - y2),2) + math.pow((z1 - z2),2))
end


function JPS3D:addToOpen(newNode)
    if newNode == nil then
        return
    end

    if self:checkgridNodePossibility(newNode.gridNode) == false then
        return
    end

    self.open:insert(newNode)
end


function JPS3D:checkgridNodePossibility(gridNode)

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

