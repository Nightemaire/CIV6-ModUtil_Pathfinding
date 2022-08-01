-- ===========================================================================
-- Utitity function: A* Pathfinding Algorithm
-- Author: Nightemaire
------------------------------------------------------------------------------
-- Overview:
-- This implementation follows the basic A* algorithm. Two lists of tiles are
-- populated and a simple heuristic calculated based on the distance and
-- move cost of the tile to the destination.
-- ===========================================================================

-- Searches for the fastest route over land from start to end within a specified range
-- Returns a table of the plots (in reverse order) as well as the total move cost
function GetLandRoute(startPlot : object, endPlot : object, range)
	-- Take care of a few simple cases first
	if startPlot:IsWater() or endPlot:IsWater() then
		print("One of the tiles in the route is water...")
		return {}, 99999;
	end
	if startPlot:GetIndex() == endPlot:GetIndex() then
		print("Start and end plots are the same...why are you trying to find a path?")
		return {startPlot}, 0
	end
	minDist = Map.GetPlotDistance(startPlot:GetX(), startPlot:GetY(), endPlot:GetX(), endPlot:GetY())
	if minDist <= 1 then
		print("Start and end plots are adjacent...pretty simple one here...")
		return {endPlot, startPlot}, 1
	end
	if minDist > range then
		print("Plots are too far from one another (specified range: "..range..")")
		return {}, 99999;
	end

	-- Okay, the path isn't so simple after all	
	print("Routing from <"..startPlot:GetX()..","..startPlot:GetY().."> to <"..endPlot:GetX()..","..endPlot:GetY()..">");
	local fastestPath = {}
	local totalMoveCost = 99999
	
	-- Local tables for storing plots
	local OpenList = {}
	local ClosedList = {}

	-- A* ALGORITHM HELPER FUNCTIONS --
	-- Adds a plot to the OpenList and calculates its G, H, and F values
	local function OpenPlot(currPlot : object, initialCost, bIsStart, bIsAcrossRiver)
		local G = 0
		local H = Map.GetPlotDistance(currPlot:GetX(), currPlot:GetY(), endPlot:GetX(), endPlot:GetY())

		-- If it's not the starting plot, we care about the movement cost
		if not(bIsStart) then
			G = initialCost + currPlot:GetMovementCost()
			if bIsAcrossRiver then
				G = G + 1
			end
		end
		
		local F = G + H

		if currPlot ~= nil then
			plotID = currPlot:GetIndex()

			OpenList[plotID] = {}
			OpenList[plotID].plot = currPlot
			OpenList[plotID].G = G
			OpenList[plotID].H = H
			OpenList[plotID].F = F

			--print("Opened plot "..plotID.."// G: "..G.."// H: "..H.."// F: "..F);
		else
			print("Why'd you try to add a nil plot to the open list?");
		end
	end

	-- Updates a plot in the OpenList if the newCost results in a lower F value
	local function UpdatePlot(plot : object, newCost, bIsAcrossRiver)
		-- See if the plot exists in the open list, but not the closed list
		if OpenList[plot:GetIndex()] ~= nil  and ClosedList[plot:GetIndex()] == nil then
			local oldF = OpenList[plot:GetIndex()].F
			local newCost = newCost + plot:GetMovementCost()
			if bIsAcrossRiver then
				newCost = newCost + 1
			end
			local newF = newCost + OpenList[plot:GetIndex()].H

			if newF < oldF then
				OpenList[plot:GetIndex()].G = newCost
				OpenList[plot:GetIndex()].F = newF
			end
		end
	end

	-- Closes out a plot and adds or updates each valid adjacent plot to the OpenList
	local function ClosePlot(plot : object)
		local thisID = plot:GetIndex()
		local thisEntry = OpenList[thisID]
		local thisCost = thisEntry.G

		-- Remove from the OpenList, and add to the ClosedList
		OpenList[thisID] = nil
		ClosedList[thisID] = thisEntry
		--print("Closed plot "..thisID)

		-- Iterate over all adjacent plots
		for i = 0, 5 do
			local adjPlot = Map.GetAdjacentPlot(plot:GetX(), plot:GetY(), i)
			--print("Checking adjacent plot "..adjPlot:GetIndex())

			-- Check to see if we cross a river, and if we even care
			local crossesRiver = plot:IsRiverCrossingToPlot(adjPlot) and Minimize_River_Crossings

			if adjPlot ~= nil then
				-- Check if the plot is not water, impassable, or already closed
				local dist = Map.GetPlotDistance(startPlot:GetX(), startPlot:GetY(), adjPlot:GetX(), adjPlot:GetY())
				local validPlot = not(adjPlot:IsImpassable()) and not(adjPlot:IsWater())
				local canOpen = validPlot and ClosedList[adjPlot:GetIndex()] == nil and dist <= range

				if canOpen then
					if OpenList[adjPlot:GetIndex()] == nil then
						-- Plot is not in the open list, add it
						OpenPlot(adjPlot, thisCost, false, crossesRiver)
					else
						-- Plot is in the open list, update it
						UpdatePlot(adjPlot, thisCost, crossesRiver)
					end
				end
			end
		end
	end

	-- Returns the plot in the OpenList with the lowest F value
	local function GetNextPlot()
		local Fmin = 99999
		local Gmin = -1
		local nextEntry = nil
		for k,entry in orderedPairs(OpenList) do
			--print("Evaluating open list item "..k)
			--if entry.F < Fmin and entry.G > Gmax then
			if entry.F < Fmin then
				Fmin = entry.F
				nextEntry = entry
			end
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
	OpenPlot(startPlot, 0, true)
		
	local searching = true
	local routeFound = false
	-- Iterate towards the destination
	print("Initiating search...")
	while searching do
		local next = GetNextPlot()
		if next ~= nil then
			ClosePlot(next)
		
			if next:GetIndex() == endPlot:GetIndex() then
				print("Found the end...")
				routeFound = true
				searching = false
			end
		else
			-- ran out of plots to check, no route is available
			print("Search ended without finding end")
			searching = false
		end
	end

	local routeComplete = false
	-- Backtrack to find the route
	if routeFound then
		local backtracking = true
		local currPlot = endPlot
		local maxIterations = 1000
		local thisIteration = 0

		print ("Initiating backtrack...")
		while backtracking do
			thisIteration = thisIteration + 1
			local costMin = 99999
			local nextPlot = nil

			-- iterate over all adjacent plots to find the lowest F cost
			--print("Checking adjacent plots... i = "..thisIteration)
			for i = 0, 5 do
				local adjPlot = Map.GetAdjacentPlot(currPlot:GetX(), currPlot:GetY(), i)				
				
				if adjPlot ~= nil then
					-- Check if the plot is closed
					local entry = ClosedList[adjPlot:GetIndex()]
					if entry ~= nil then
						--print ("Found a closed neighbor")
						-- if it is, we see whether it's the lowest G cost
						local thisCost = entry.G
						if thisCost < costMin then
							-- if it is, store it
							costMin = thisCost
							nextPlot = adjPlot
						end
					end
				end
			end

			-- if we found a plot, then add it and set it as the 
			if nextPlot ~= nil then
				-- add the plot to the fastest path
				table.insert(fastestPath, nextPlot)

				if nextPlot:GetIndex() == startPlot:GetIndex() then 
					-- if we're back at the beginning, end the loop
					backtracking = false
					routeComplete = true
				else
					-- otherwise update the current plot
					currPlot = nextPlot
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
			totalMoveCost = ClosedList[endPlot:GetIndex()].G
			print("Path Complete! Cost = "..totalMoveCost)
		else
			print("ERROR: Route wasn't completed")
			fastestPath = {}
			totalMoveCost = 9999999
		end
	end

	return fastestPath, totalMoveCost
end

-- ===========================================================================