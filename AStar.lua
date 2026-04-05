local astar = {}

local jumpHeight = 8
local dropHeight = 40

local agentRadius = 1
local agentHeight = 5
local jumpHeight = 6

local gridSize = 2
local maxIterations = 10000
local yieldInterval = 200

local HEURISTIC_WEIGHT = 2

local function CreateNode()
	return {
		Position = Vector3.zero,
		Parent = nil,
		HCost = 0,
		FCost = 0,
		GCost = 0,
		Jump = false
	}
end

local function key(v)
	return v.X .. "," .. v.Y .. "," .. v.Z
end

local function snap(pos)
	return Vector3.new(
		math.floor(pos.X/gridSize+0.5)*gridSize,
		math.floor(pos.Y/gridSize+0.5)*gridSize,
		math.floor(pos.Z/gridSize+0.5)*gridSize
	)
end

local function hasNodes(t)
	for _ in pairs(t) do
		return true
	end
	return false
end

local directions = {
	Vector3.new(1,0,0),
	Vector3.new(-1,0,0),
	Vector3.new(0,0,1),
	Vector3.new(0,0,-1),
	Vector3.new(1,0,1),
	Vector3.new(-1,0,-1),
	Vector3.new(1,0,-1),
	Vector3.new(-1,0,1)
}

local function heuristic(a,b)
	return (a-b).Magnitude
end

local function getNeighbors(node, rp)
	local neighbors = {}
	local verticalSearchRange = 15 
	local stepHeight = 2.5

	for _, dir in ipairs(directions) do
		local targetPos = node + (dir * gridSize)
		local rayOrigin = targetPos + Vector3.new(0, stepHeight, 0)
		local rayDir = Vector3.new(0, -(stepHeight + verticalSearchRange), 0)
		local groundRay = workspace:Raycast(rayOrigin, rayDir, rp)

		if groundRay then
			local landing = snap(groundRay.Position)
			local heightDiff = landing.Y - node.Y

			if heightDiff <= jumpHeight and heightDiff >= -dropHeight then
				local headCheck = workspace:Raycast(landing + Vector3.new(0, 0.5, 0), Vector3.new(0, agentHeight, 0), rp)
				if not headCheck then
					table.insert(neighbors, {
						pos = landing, 
						jump = heightDiff > 1.5,
						isLandingNode = false
					})
					continue 
				end
			end
		end

		for d = 2, 4, 1 do
			local gapTarget = node + (dir * (gridSize/2) * d)
			local jumpRay = workspace:Raycast(gapTarget + Vector3.new(0, jumpHeight, 0), Vector3.new(0, -(jumpHeight + dropHeight), 0), rp)

			if jumpRay then
				local landing = snap(jumpRay.Position)
				if landing.Y - node.Y <= jumpHeight then
					table.insert(neighbors, {
						pos = landing, 
						jump = true, 
						isLandingNode = true 
					})
					break 
				end
			end
		end
	end

	return neighbors
end

local Heap = {}
Heap.__index = Heap

function Heap.new()
	return setmetatable({ contents = {} }, Heap)
end

function Heap:Push(node)
	table.insert(self.contents, node)
	local index = #self.contents
	while index > 1 do
		local parentIndex = math.floor(index / 2)
		if self.contents[index].FCost < self.contents[parentIndex].FCost then
			self.contents[index], self.contents[parentIndex] = self.contents[parentIndex], self.contents[index]
			index = parentIndex
		else
			break
		end
	end
end

function Heap:Pop()
	if #self.contents == 0 then return nil end
	local root = self.contents[1]
	local last = table.remove(self.contents)
	if #self.contents > 0 then
		self.contents[1] = last
		local index = 1
		while true do
			local left = index * 2
			local right = index * 2 + 1
			local smallest = index

			if left <= #self.contents and self.contents[left].FCost < self.contents[smallest].FCost then
				smallest = left
			end
			if right <= #self.contents and self.contents[right].FCost < self.contents[smallest].FCost then
				smallest = right
			end

			if smallest ~= index then
				self.contents[index], self.contents[smallest] = self.contents[smallest], self.contents[index]
				index = smallest
			else
				break
			end
		end
	end
	return root
end

--function astar:PostProcessPath(path, raycastParams)
--	if not path or #path <= 2 then return path end

--	local pprocessedPath = {path[1]}
--	local currentIndex = 1
	
--	local heightVec = Vector3.new(0,agentHeight,0)
--	local checkOffset = Vector3.new(0, 2, 0)

--	while currentIndex < #path do
--		local startNode = pprocessedPath[#pprocessedPath]
--		local furthestVisibleIndex = currentIndex + 1

--		for i = currentIndex + 1, #path do
--			local targetPos = path[i].Position
--			local startPos = startNode.Position

--			local direction = targetPos - startPos
--			local ray = workspace:Raycast(startPos + checkOffset, direction, raycastParams)

--			local isFloorStable = true
--			local segments = 4
--			for s = 1, segments do
--				local samplePoint = startPos:Lerp(targetPos, s/segments)
--				local floorCheck = workspace:Raycast(samplePoint + checkOffset, Vector3.new(0, -10, 0), raycastParams)
--				if not floorCheck then
--					isFloorStable = false
--					break
--				end
--			end

--			if not ray and isFloorStable then
--				furthestVisibleIndex = i
--			else
--				break
--			end
--		end

--		table.insert(pprocessedPath, path[furthestVisibleIndex])

--		if furthestVisibleIndex == #path then
--			break
--		end

--		currentIndex = furthestVisibleIndex
--	end

--	return pprocessedPath
--end

function astar:PostProcessPath(path)
	if not path or #path < 2 then return path end

	local simplified = {path[1]}

	for i = 2, #path - 1 do
		local prevNode = path[i-1]
		local currentNode = path[i]
		local nextNode = path[i+1]

		if currentNode.Jump or nextNode.Jump or prevNode.Jump then
			table.insert(simplified, currentNode)
		else
			local lastPos = simplified[#simplified].Position
			local currentPos = currentNode.Position
			local nextPos = nextNode.Position

			local dir1 = (currentPos - lastPos).Unit
			local dir2 = (nextPos - currentPos).Unit

			if (dir1 - dir2).Magnitude > 0.05 then
				table.insert(simplified, currentNode)
			end
		end
	end

	table.insert(simplified, path[#path])
	return simplified
end

--create notes to urself

function astar:ComputePath(start, goal, raycastParams)
	start = snap(start)
	goal = snap(goal)

	local openHeap = Heap.new()
	local openLookup = {} 
	local closed = {}

	local startNode = CreateNode()
	startNode.Position = start
	startNode.HCost = heuristic(start, goal)
	startNode.FCost = startNode.HCost

	openHeap:Push(startNode)
	openLookup[key(start)] = startNode

	local closestNode = startNode
	local minHCost = startNode.HCost
	local iterations = 0

	while #openHeap.contents > 0 do
		local current = openHeap:Pop()
		local currentKey = key(current.Position)

		if closed[currentKey] then continue end

		iterations += 1

		if current.HCost < minHCost then
			minHCost = current.HCost
			closestNode = current
		end

		if iterations > maxIterations then 
			return self:ReconstructPath(closestNode), true 
		end

		if (current.Position - goal).Magnitude <= gridSize * 1.5 then
			return self:ReconstructPath(current), false
		end

		closed[currentKey] = true

		for _, neighbor in ipairs(getNeighbors(current.Position, raycastParams)) do
			local neighborKey = key(neighbor.pos)
			if closed[neighborKey] then continue end

			local g = current.GCost + (neighbor.pos - current.Position).Magnitude
			local h = heuristic(neighbor.pos, goal)
			local f = g + h * HEURISTIC_WEIGHT

			local existing = openLookup[neighborKey]

			if not existing or g < existing.GCost then
				local node = existing or CreateNode()
				node.Position = neighbor.pos
				node.Parent = current
				node.GCost = g
				node.HCost = h
				node.FCost = f
				node.Jump = neighbor.jump

				openHeap:Push(node)
				openLookup[neighborKey] = node
			end
		end
	end

	return self:ReconstructPath(closestNode), true
end

function astar:ReconstructPath(endNode)
	local path = {}
	local current = endNode

	while current do
		if current.Jump and current.Parent then
			local buff = CreateNode()
			buff.Position = current.Position
			buff.Jump = false
			table.insert(path, buff)
		end

		table.insert(path, current)
		current = current.Parent
	end

	local reversed = {}
	for i = #path, 1, -1 do 
		table.insert(reversed, path[i]) 
	end
	return reversed
end

return astar
