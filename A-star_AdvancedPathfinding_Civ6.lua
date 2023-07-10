-- ===========================================================================
-- Utitity function: A* Pathfinding Algorithm
-- Author: Nightemaire
------------------------------------------------------------------------------
-- Overview:
-- This implementation follows the basic A* algorithm. Two lists of tiles are
-- populated, and a method of weighting features calculates the move cost to 
-- travel from one tile to another.
-- ===========================================================================

local debug = false;

------------- WEIGHT SETTINGS -------------
local Base_Cost = 1

-- Terrain
local Hills_Cost = 2
local Mountain_Cost = 3
local Snow_Cost = 2

-- Features
local Forest_Cost = 1
local Jungle_Cost = 2
local Marsh_Cost = 2
local Floodplains_Cost = 1

-- Adjacency
local River_Crossing_Cost = 2
local Direction_Change_Cost = 2
--------------------------------------------
-- Searches for the fastest route over land from start to end within a specified range
-- Returns a table of the plot indices (in reverse order) as well as the total move cost
-- Attempts to maintain a straight course by weighting changes in direction heavier
function GetLandRoute(startPlot : object, endPlot : object, range)
	if startPlot ~= nil and endPlot ~= nil then
		local startX = startPlot:GetX()
		local startY = startPlot:GetY()
		local endX = endPlot:GetX()
		local endY = endPlot:GetY()

		if (range == nil) then range = 999999; end

		-- Take care of a few simple cases first
		if startPlot:IsWater() or endPlot:IsWater() then
			print_if_debugging("WARNING: One of the tiles in the route is water...")
			return {}, 99999;
		end
		if startPlot:GetIndex() == endPlot:GetIndex() then
			print_if_debugging("WARNING: Start and end plots are the same...why are you trying to find a path?")
			return {startPlot}, 0
		end
		minDist = Map.GetPlotDistance(startX, startY, endX, endY)
		if minDist <= 1 then
			print_if_debugging("NO PATH: Start and end plots are adjacent...pretty simple one here...")
			return {endPlot, startPlot}, 1
		end
		if minDist > range then
			print_if_debugging("NO PATH: Plots are too far from one another (specified range: "..range..")")
			return {}, 99999;
		end

		local localPlayerVis:table = PlayersVisibility[startPlot:GetOwner()]

		-- Okay, the path isn't so simple after all	
		print_if_debugging("Routing from <"..startX..","..startY.."> to <"..endX..","..endY..">");
		local fastestPath = {}
		local totalMoveCost = 99999

		-- Local tables for storing plots
		local OpenList = {}
		local ClosedList = {}

		-- A* ALGORITHM HELPER FUNCTIONS --
		local function CalcG(plot, initialCost, adjData)
			local G = initialCost

			local ePlotFeature = plot:GetFeatureType();
            local ePlotTerrain = plot:GetTerrainType();

			if plot:IsMountain() then
				G = G + Mountain_Cost
			elseif plot:IsHills() then
				G = G + Hills_Cost
			else
				G = G + Base_Cost
			end

			local HasSnow = ePlotTerrain == iSnow or ePlotTerrain == iSnowHills or ePlotTerrain == iSnowMountain;
            if HasSnow then G = G + Snow_Cost; end

            if notNilOrNegative(ePlotFeature) then
                if ePlotFeature == iForest then
                    G = G + Forest_Cost
                elseif ePlotFeature == iJungle then
                    G = G + Jungle_Cost
                elseif ePlotFeature == iMarsh then
                    G = G + Marsh_Cost
                elseif ePlotFeature == iFloodplains then
                    G = G + Floodplains_Cost
                end
            end

            if adjData.bDirectionChange then G = G + Direction_Change_Cost; end
            if adjData.bRiverCrossing then G = G + River_Crossing_Cost; end

			return G
		end

		-- Gets a table of adjacency data that's used in the open and update plot functions
		local function GetAdjacencyInfo(i, pPlot, adjPlot, lastdir)
			local crossesRiver = pPlot:IsRiverCrossingToPlot(adjPlot)
			local dirMatches = i == lastdir

			local adjacencyTable = {
				bRiverCrossing = crossesRiver,
				iDirection = i,
				bDirectionChange = not(dirMatches)
			}

			return adjacencyTable
		end

		-- Adds a plot to the OpenList and calculates its G, H, and F values
		local function OpenPlot(currPlot : object, initialCost, bIsStart, adjData)
			if currPlot ~= nil then
				if adjData == nil then
					adjData = {}
					adjData.iDirection = 0
					adjData.bRiverCrossing = false
					adjData.bDirectionChange = false
				end
				local plotOwner = currPlot:GetOwner()
				-- For building the railroads, we care that its the correct owner or no owner
				if plotOwner == -1 or plotOwner == startPlot:GetOwner() then
					local G = 0
					local H = Map.GetPlotDistance(currPlot:GetX(), currPlot:GetY(), endX, endY)

					-- If it's not the starting plot, we care about the movement cost
					if not(bIsStart) then
						G = CalcG(currPlot, initialCost, adjData)
					end
					
					local F = G + H

					local plotID = currPlot:GetIndex()

					OpenList[plotID] = {}
					OpenList[plotID].plot = currPlot
					OpenList[plotID].dir = adjData.iDirection
					OpenList[plotID].G = G
					OpenList[plotID].H = H
					OpenList[plotID].F = F

					local printStr = "in dir: "
					--if dirMatch then printStr = "in M dir: "; end
					print_if_debugging(">> Opened plot: "..plotID.." <"..H..", "..G..", "..F.."> "..printStr..adjData.iDirection);
				end
			else
				print("ERROR: Why'd you try to add a nil plot to the open list?");
			end
		end

		-- Updates a plot in the OpenList if the newCost results in a lower F value
		local function UpdatePlot(plot : object, newCost, adjData)
			local plotIndex = plot:GetIndex()
			-- See if the plot exists in the open list, but not the closed list
			--if OpenList[plotIndex] ~= nil  and ClosedList[plotIndex] == nil then
			local entry = OpenList[plotIndex]
			local oldF = entry.F
			local newG = CalcG(plot, newCost, adjData)
			local newF = newG + entry.H

			if newF < oldF then
				OpenList[plotIndex].G = newG
				OpenList[plotIndex].F = newF
				OpenList[plotIndex].dir = adjData.iDirection
			end
			--end
		end

		-- Used in ClosePlot() to check whether an adjacent plot is a valid candidate
		local function AdjacentIsPlotValid(adjPlot)
			if adjPlot ~= nil then
				local adjIndex = adjPlot:GetIndex()
				local adjX = adjPlot:GetX()
				local adjY = adjPlot:GetY()
				
				local isVisible = localPlayerVis:IsRevealed(adjX, adjY)
				local isVolcano = adjPlot:GetFeatureType() == iVolcano

				-- Check if the plot is not water, a volcano, or a natural wonder
				local invalidPlot = adjPlot:IsWater() or (adjPlot:IsImpassable() and not(adjPlot:IsMountain()) or isVolcano or adjPlot:IsNaturalWonder())
				
				-- Check validitiy, visibility, distance, and whether this plot is already in the closed list
				local dist = Map.GetPlotDistance(startX, startY, adjX, adjY)
				local isValid = not(invalidPlot) and ClosedList[adjIndex] == nil and dist <= range and isVisible

				return isValid
			end

			return false
		end

		-- Closes out a plot and adds or updates each valid adjacent plot to the OpenList
		local function ClosePlot(plot : object)
			local thisID = plot:GetIndex()
			local thisEntry = OpenList[thisID]
			local thisCost = thisEntry.G
			local thisDir = thisEntry.dir

			-- Remove from the OpenList, and add to the ClosedList
			OpenList[thisID] = nil
			ClosedList[thisID] = thisEntry
			print_if_debugging("Closing plot: "..thisID.." with direction: "..thisDir)
			local thisX = plot:GetX()
			local thisY = plot:GetY()
			-- Iterate over all adjacent plots
			for i = 0, 5 do
				local adjPlot = Map.GetAdjacentPlot(thisX, thisY, i)
				--print_if_debugging("Checking adjacent plot "..adjPlot:GetIndex())
				if adjPlot ~= nil then
					local adjIndex = adjPlot:GetIndex()
					if AdjacentIsPlotValid(adjPlot) then
						if OpenList[adjIndex] == nil then
							-- Plot is not in the open list, add it
							OpenPlot(adjPlot, thisCost, false, GetAdjacencyInfo(i, plot, adjPlot, thisDir))
						else
							-- Plot is in the open list, update it
							UpdatePlot(adjPlot, thisCost, GetAdjacencyInfo(i, plot, adjPlot, thisDir))
						end
					else
						--print_if_debugging("Cannot open adjacent plot: "..i)
					end
				end
			end
		end

		-- Returns the plot in the OpenList with the lowest F or G value
		local function GetNextPlot()
			local Fmin = 99999
			local Gmin = 99999
			local nextEntry = nil
			for k,entry in orderedPairs(OpenList) do
				--print("Evaluating open list item "..k)
				--if entry.F < Fmin and entry.G > Gmax then
				--[[
				if entry.G < Gmin then
					Gmin = entry.G
					nextEntry = entry
				end
				--]]
				--
				if entry.F < Fmin then
					Fmin = entry.F
					nextEntry = entry
				end
				--]]
			end

			local nextPlot = nil
			if nextEntry ~= nil then
				nextPlot = nextEntry.plot
				--print("Selected plot "..nextEntry.plot:GetIndex())
			end
				
			return nextPlot
		end

		-- BEGIN A* ALGORITHM --
		-- Initialize the algorithm to our starting location
		OpenPlot(startPlot, 0, true, nil)

		local searching = true
		local routeFound = false

		-- Iterate towards the destination
		print_if_debugging("Initiating search...")
		while searching do
			local next = GetNextPlot()
			if next ~= nil then
				ClosePlot(next)
			
				if next:GetIndex() == endPlot:GetIndex() then
					print_if_debugging("Found the end...")
					routeFound = true
					searching = false
				end
			else
				-- ran out of plots to check, no route is available
				print("ERROR: Search ended without finding end")
				searching = false
				routeFound = true
			end
			last = next
		end

		local routeComplete = false
		-- Backtrack to find the route
		if routeFound then
			local backtracking = true
			local currPlot = endPlot
			local maxIterations = 1000
			local thisIteration = 0
			local lastDir = -1

			local endEntry = ClosedList[endPlot:GetIndex()]

			table.insert(fastestPath, currPlot:GetIndex())

			print_if_debugging("Initiating backtrack...")
			while backtracking do
				thisIteration = thisIteration + 1
				local costMin = 99999
				local nextPlot = nil
				local thisDur = -1
				
				-- Remove the current plot from the closed list
				ClosedList[currPlot:GetIndex()] = nil

				local currX = currPlot:GetX()
				local currY = currPlot:GetY()

				-- iterate over all adjacent plots to find the lowest F cost
				--print("Checking adjacent plots... i = "..thisIteration)
				for i = 0, 5 do
					local adjPlot = Map.GetAdjacentPlot(currX, currY, i)				
					
					if adjPlot ~= nil then
						-- Check if the plot is closed
						local adjIndex = adjPlot:GetIndex()
						local entry = ClosedList[adjIndex]
						if entry ~= nil then
							-- if it is, we see whether it's the lowest G cost
							local thisCost = entry.G
							if thisCost < costMin or (thisCost == costMin and lastDir == i) then
								-- if it is, store it
								if thisCost == costMin then
									print_if_debugging("Plot "..adjIndex.." is better.  (Straight")
								else
									print_if_debugging("Plot "..adjIndex.." is better.  (G = "..thisCost..", D = "..lastDir..","..i..")")
								end
								costMin = thisCost
								nextPlot = adjPlot
								thisDur = i
							end
						end
					end
				end

				-- if we found a plot, then add it and set it as the current plot
				if nextPlot ~= nil then
					print_if_debugging("Found a closed neighbor")
					-- add the plot to the fastest path
					table.insert(fastestPath, nextPlot:GetIndex())

					if nextPlot:GetIndex() == startPlot:GetIndex() then 
						-- if we're back at the beginning, end the loop
						backtracking = false
						routeComplete = true
					else
						-- otherwise update the current plot
						currPlot = nextPlot
						lastDir = thisDur
					end	
				else
					-- Couldn't find a tile for some reason
					backtracking = false
					print("ERROR: Failed to find an adjacent plot on the closed list... i = "..thisIteration)
				end

				if thisIteration >= maxIterations then
					backtracking = false
					print("ERROR: Failed to find a route within the max iterations allowed (1000)")
				end
			end

			if routeComplete then
				totalMoveCost = endEntry.G
				print_if_debugging("Path Complete! Cost = "..totalMoveCost)
			else
				print("ERROR: Route wasn't completed")
				fastestPath = {}
				totalMoveCost = 9999999
			end
		end

		return fastestPath, totalMoveCost
	else
		print("ERROR: One of the provided plots was nil...")
	end
end

function print_if_debugging(msg)
	if debug == true then print(msg); end
end

-- ===========================================================================