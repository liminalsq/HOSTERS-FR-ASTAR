--lit the easier way for ppl who dont wanna use the astar module themselves

local pathfinder = {}

local AStar = loadstring(game:HttpGet("https://raw.githubusercontent.com/liminalsq/HOSTERS-FR-ASTAR/refs/heads/main/AStar.lua"))()

function pathfinder:FindPath(start, goal, npc, postProcess, maxIterations)
	local raycastParams = RaycastParams.new()
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	
	if npc then
		raycastParams:AddToFilter(npc)
	end
	
	local path = AStar:ComputePath(start, goal, raycastParams, maxIterations)
	
	if postProcess then
		path = AStar:PostProcessPath(path, raycastParams)
	end
	
	return path
end

return pathfinder
