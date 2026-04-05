--lit the easier way for ppl who dont wanna use the astar module themselves

local pathfinder = {}

local AStar = require(script.AStar)

function pathfinder:FindPath(start, goal, npc, postProcess)
	local raycastParams = RaycastParams.new()
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	
	if npc then
		raycastParams:AddToFilter(npc)
	end
	
	local path = AStar:ComputePath(start, goal, raycastParams)
	
	if postProcess then
		path = AStar:PostProcessPath(path, raycastParams)
	end
	
	return path
end

return pathfinder
