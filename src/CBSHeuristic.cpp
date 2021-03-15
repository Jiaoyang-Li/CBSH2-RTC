//#pragma warning(disable: 4996) // I added this line to disable error C4996 caused by CPLEX
//#include <ilcplex/ilocplex.h>
#include "CBSHeuristic.h"
#include "CBS.h"
#include <queue>


void CBSHeuristic::computeQuickHeuristics(CBSNode& node) const // for non-root node
{
	node.h_val = max(0, node.parent->g_val + node.parent->h_val - node.g_val); // pathmax
	set<pair<int, int>> conflicting_agents;
    for (const auto& conflict : node.unknownConf)
    {
        auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
        if (conflicting_agents.find(agents) == conflicting_agents.end())
        {
            conflicting_agents.insert(agents);
        }
    }
    node.tie_breaking = (int)(node.conflicts.size() + conflicting_agents.size());
	copyConflictGraph(node, *node.parent);
}


bool CBSHeuristic::computeInformedHeuristics(CBSNode& curr, double _time_limit)
{
	curr.h_computed = true;
	// create conflict graph
	start_time = clock();
	this->time_limit = _time_limit;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
	int h = -1;
	switch (type)
	{
	case heuristics_type::ZERO:
		h = 0;
		break;
	case heuristics_type::CG:
		buildCardinalConflictGraph(curr, HG, num_of_CGedges);
		// Minimum Vertex Cover
		if (curr.parent == nullptr ||  // root node of CBS tree
		    target_reasoning || corridor_reasoning == corridor_strategy::STC || corridor_reasoning == corridor_strategy::GC ||
                corridor_reasoning == corridor_strategy::DC || disjoint_splitting) // when we are allowed
		    // to replan for multiple agents, the incremental method is not correct any longer.
			h = minimumVertexCover(HG);
		else
        {
		    assert(curr.paths.size() == 1);
            h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);

        }
		break;
	case heuristics_type::DG:
		if (!buildDependenceGraph(curr, HG, num_of_CGedges))
			return false;
		// Minimum Vertex Cover
		if (curr.parent == nullptr || // root node of CBS tree
		    target_reasoning || corridor_reasoning == corridor_strategy::STC || corridor_reasoning == corridor_strategy::GC ||
		    corridor_reasoning == corridor_strategy::DC || disjoint_splitting) // when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
			h = minimumVertexCover(HG);
		else
			h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
		break;
	case heuristics_type::WDG:
		if (!buildWeightedDependencyGraph(curr, HG))
			return false;
		h = minimumWeightedVertexCover(HG);
		break;
	}
	if (h < 0)
		return false;

	curr.h_val = max(h, curr.h_val);
	return true;
}


void CBSHeuristic::buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges)
{
	num_of_CGedges = 0;
	for (const auto& conflict : curr.conflicts)
	{
		if (conflict->priority == conflict_priority::CARDINAL)
		{
			int a1 = conflict->a1;
			int a2 = conflict->a2;
			if (!CG[a1 * num_of_agents + a2])
			{
				CG[a1 * num_of_agents + a2] = true;
				CG[a2 * num_of_agents + a1] = true;
				num_of_CGedges++;
			}
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
}


bool CBSHeuristic::buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges)
{
	for (auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (conflict->priority == conflict_priority::CARDINAL)
		{
			node.conflictGraph[idx] = 1;
		}
		else if(node.conflictGraph.find(idx) == node.conflictGraph.end())
		{
			auto got = lookupTable[a1][a2].find(DoubleConstraintsHasher(a1, a2, &node));
			if (got != lookupTable[a1][a2].end()) // check the lookup table first
			{
				num_memoization++;
				node.conflictGraph[idx] = got->second;
			}
			else
			{
				node.conflictGraph[idx] = dependent(a1, a2, node)? 1 : 0;
				lookupTable[a1][a2][DoubleConstraintsHasher(a1, a2, &node)] = node.conflictGraph[idx];
			}
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.conflictGraph.find(i * num_of_agents + j);
			if (got != node.conflictGraph.end() && got->second > 0)
			{
				CG[i * num_of_agents + j] = got->second;
				CG[j * num_of_agents + i] = got->second;
				num_of_CGedges++;
			}
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


bool CBSHeuristic::buildWeightedDependencyGraph(CBSNode& node, vector<int>& CG)
{
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (node.conflictGraph.find(idx) != node.conflictGraph.end())
			continue;
		auto got = lookupTable[a1][a2].find(DoubleConstraintsHasher(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
			node.conflictGraph[idx] = got->second;
		}
		else if (rectangle_reasoning == rectangle_strategy::RM ||
                rectangle_reasoning == rectangle_strategy::GR ||
                rectangle_reasoning == rectangle_strategy::DR ||
                save_stats)
		{
			node.conflictGraph[idx] = solve2Agents(a1, a2, node, false);
			assert(node.conflictGraph[idx] >= 0);
			lookupTable[a1][a2][DoubleConstraintsHasher(a1, a2, &node)] = node.conflictGraph[idx];
		}
		else
		{ 
			bool cardinal = conflict->priority == conflict_priority::CARDINAL;
			if (!cardinal && !mutex_reasoning) // using merging MDD methods before runing 2-agent instance
			{
				cardinal = dependent(a1, a2, node);
			}
			if (cardinal) // run 2-agent solver only for dependent agents
			{
				node.conflictGraph[idx] = solve2Agents(a1, a2, node, cardinal);		
				assert(node.conflictGraph[idx] >= 1);
			}
			else
			{
				node.conflictGraph[idx] = 0;
			}
			lookupTable[a1][a2][DoubleConstraintsHasher(a1, a2, &node)] = node.conflictGraph[idx];
		}

		if (node.conflictGraph[idx] == MAX_COST) // no solution
		{
			return false;
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.conflictGraph.find(i * num_of_agents + j);
			if (got != node.conflictGraph.end() && got->second > 0)
			{
				CG[i * num_of_agents + j] = got->second;
				CG[j * num_of_agents + i] = got->second;
			}
		}
	}
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


int CBSHeuristic::solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal)
{
	vector<SingleAgentSolver*> engines(2);
	engines[0] = search_engines[a1];
	engines[1] = search_engines[a2];
	vector<vector<PathEntry>> initial_paths(2);
	initial_paths[0] = *paths[a1];
	initial_paths[1] = *paths[a2];
	vector<ConstraintTable> constraints{
		ConstraintTable(initial_constraints[a1]),
		ConstraintTable(initial_constraints[a2]) };
	constraints[0].build(node, a1);
	constraints[1].build(node, a2);
	CBS cbs(engines, constraints, initial_paths, screen);
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setNodeLimit(node_limit);

	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	int root_g = (int)initial_paths[0].size() - 1 + (int)initial_paths[1].size() - 1;
	int lowerbound = root_g;
	int upperbound = MAX_COST;
	if (cardinal)
		lowerbound += 1;
	cbs.solve(time_limit - runtime, lowerbound, upperbound);
	num_solve_2agent_problems++;
	int rst;
	if (cbs.runtime > time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		rst = (int)cbs.min_f_val - root_g; // using lowerbound to approximate
	else if (cbs.solution_cost  < 0) // no solution
		rst = MAX_COST;
	else
	{
		rst = cbs.solution_cost - root_g;
	}

	// For statistic study!!!
	/*if (cbs.num_HL_expanded > 1 && rst >= 1)
	{
		cout << a1 << "," << a2 << "," << node.time_generated << "," << cbs.num_HL_expanded << "," << rst << endl;
	}*/
	if (save_stats)
	{
		sub_instances.emplace_back(a1, a2, &node, cbs.num_HL_expanded, rst);
		// cout << sub_instances.size() << endl;
	}

	assert(rst >= 0);
	return rst;
}


int CBSHeuristic::MVConAllConflicts(CBSNode& curr)
{
	auto G = buildConflictGraph(curr);
	return  minimumVertexCover(G);
}


vector<int> CBSHeuristic::buildConflictGraph(const CBSNode& curr) const
{
	vector<int> G(num_of_agents * num_of_agents, 0);
	for (const auto& conflict : curr.conflicts)
	{
		int a1 = conflict->a1;
		int a2 = conflict->a2;
		if (!G[a1 * num_of_agents + a2])
		{
			G[a1 * num_of_agents + a2] = true;
			G[a2 * num_of_agents + a1] = true;
		}
	}
	return G;
}

int CBSHeuristic::minimumVertexCover(const vector<int>& CG)
{
    clock_t t = clock();
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> indices;
		indices.reserve(num_of_agents);
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
				else if (CG[k * num_of_agents + j] > 0)
				{
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
		}
		if ((int) indices.size() == 1) //one node -> no edges -> mvc = 0
			continue;
		else if ((int) indices.size() == 2) // two nodes -> only one edge -> mvc = 1
		{
			rst += 1; // add edge weight
			continue;
		}

		std::vector<int> subgraph(indices.size() * indices.size(), 0);
		int num_edges = 0;
		for (int j = 0; j < (int) indices.size(); j++)
		{
			for (int k = j + 1; k < (int) indices.size(); k++)
			{
				subgraph[j * indices.size() + k] = CG[indices[j] * num_of_agents + indices[k]];
				subgraph[k * indices.size() + j] = CG[indices[k] * num_of_agents + indices[j]];
				if (subgraph[j * indices.size() + k] > 0)
					num_edges++;
			}
		}
        if ((int)indices.size() > DP_node_threshold)
        {
            rst += greedyMatching(subgraph, (int)indices.size());
            double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
            if (runtime > time_limit)
            {
                runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
                return -1; // run out of time
            }
        }
        else {
            for (int i = 1; i < (int) indices.size(); i++) {
                if (KVertexCover(subgraph, (int) indices.size(), num_edges, i, (int) indices.size())) {
                    rst += i;
                    break;
                }
                double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
                if (runtime > time_limit)
                    return -1; // run out of time
            }
        }
	}
    runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}


int CBSHeuristic::minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_CGedges)
{
    assert(old_mvc >= 0);
	clock_t t = clock();
	int rst = 0;
	if (num_of_CGedges < 2)
		return num_of_CGedges;
	// Compute #CG nodes that have edges
	int num_of_CGnodes = 0;
	for (int i = 0; i <  cols; i++)
	{
		for (int j = 0; j <  cols; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	if (num_of_CGnodes > DP_node_threshold)
	{
        runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
		return minimumVertexCover(CG);
	}
	else
	{
		if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols))
			rst = old_mvc - 1;
		else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols))
			rst = old_mvc;
		else
			rst = old_mvc + 1;
	}
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

// Whether there exists a k-vertex cover solution
bool CBSHeuristic::KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols)
{
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return true; // run out of time
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	std::vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < cols - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < cols && flag; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		std::vector<int> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < cols; j++)
		{
			if (CG_copy[node[i] * cols + j] > 0)
			{
				CG_copy[node[i] * cols + j] = 0;
				CG_copy[j * cols + node[i]] = 0;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols))
			return true;
	}
	return false;
}

int CBSHeuristic::greedyMatching(const std::vector<int>& CG,  int cols)
{
	int rst = 0;
	std::vector<bool> used(cols, false);
	while(true)
	{
		int maxWeight = 0;
		int ep1, ep2;
		for (int i = 0; i < cols; i++)
		{
			if(used[i])
				continue;
			for (int j = i + 1; j < cols; j++)
			{
				if (used[j])
					continue;
				else if (maxWeight < CG[i * cols + j])
				{
					maxWeight = CG[i * cols + j];
					ep1 = i;
					ep2 = j;
				}
			}
		}
		if (maxWeight == 0)
			return rst;
		rst += maxWeight;
		used[ep1] = true;
		used[ep2] = true;
	}
}

int CBSHeuristic::minimumWeightedVertexCover(const vector<int>& HG)
{
	clock_t t = clock();
	int rst = weightedVertexCover(HG);
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}


int CBSHeuristic::weightedVertexCover(const std::vector<int>& CG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		int num_states = 1;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0); // TODO::this line can be deleted?
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}		
				else if (CG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num  == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		//if (num > DP_threshold) // solve by ILP
		//	rst += ILPForWMVC(G, range);
        if (num > DP_node_threshold) // solve by greedy matching
        {
            rst += greedyMatching(G, (int)range.size());
        }
		else // solve by dynamic programming
		{
			std::vector<int> x(num); 
			int best_so_far = MAX_COST;
			rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
		}
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = DPForWMVC(x, 0, 0, CG, range, best_so_far);
	if( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

// recusive component of dynamic programming for weighted vertex cover
int CBSHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();
	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost;
	}

	return best_so_far;
}


void CBSHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent) const
{
	//copy conflict graph
	if (type == heuristics_type::DG || type == heuristics_type::WDG)
	{
		unordered_set<int> changed;
		for (const auto& p : child.paths) {
			changed.insert(p.first);
		}
		for (auto e : parent.conflictGraph) {
			if (changed.find(e.first / num_of_agents) == changed.end() &&
				changed.find(e.first % num_of_agents) == changed.end())
				child.conflictGraph[e.first] = e.second;
		}

	}
}

bool CBSHeuristic::dependent(int a1, int a2, CBSNode& node) // return true if the two agents are dependent
{
	const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size()); // get mdds
	const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	if (mdd1->levels.size() > mdd2->levels.size()) // swap
		std::swap(mdd1, mdd2);
	num_merge_MDDs++;
	return !SyncMDDs(*mdd1, *mdd2);
}

// return true if the joint MDD exists.
bool CBSHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
{
	if (other.levels.size() <= 1) // Either of the MDDs was already completely pruned already
		return false;

	SyncMDD copy(mdd);
	if (copy.levels.size() < other.levels.size())
	{
		size_t i = copy.levels.size();
		copy.levels.resize(other.levels.size());
		for (; i < copy.levels.size(); i++)
		{
			SyncMDDNode* parent = copy.levels[i - 1].front();
			auto node = new SyncMDDNode(parent->location, parent);
			parent->children.push_back(node);
			copy.levels[i].push_back(node);

		}
	}
	// Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
	copy.levels[0].front()->coexistingNodesFromOtherMdds.push_back(other.levels[0].front());

	// what if level.size() = 1?
	for (size_t i = 1; i < copy.levels.size(); i++)
	{
		for (auto node = copy.levels[i].begin(); node != copy.levels[i].end();)
		{
			// Go over all the node's parents and test their coexisting nodes' children for co-existance with this node
			for (auto parent = (*node)->parents.begin(); parent != (*node)->parents.end(); parent++)
			{
				//bool validParent = false;
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location || // vertex conflict
							((*node)->location == parentCoexistingNode->location &&
							(*parent)->location == childOfParentCoexistingNode->location)) // edge conflict
							continue;
						//validParent = true;

						auto it = (*node)->coexistingNodesFromOtherMdds.cbegin();
						for (; it != (*node)->coexistingNodesFromOtherMdds.cend(); ++it)
						{
							if (*it == childOfParentCoexistingNode)
								break;
						}
						if (it == (*node)->coexistingNodesFromOtherMdds.cend())
						{
							(*node)->coexistingNodesFromOtherMdds.push_back(childOfParentCoexistingNode);
						}
					}
				}
				//if (!validParent)
				//{
				//	// delete the edge, and continue up the levels if necessary
				//	SyncMDDNode* p = *parent;
				//	parent = (*node)->parents.erase(parent);
				//	p->children.remove((*node));
				//	if (p->children.empty())
				//		copy.deleteNode(p);
				//}
				//else
				//{
				//	parent++;
				//}
			}
			if ((*node)->coexistingNodesFromOtherMdds.empty())
			{
				// delete the node, and continue up the levels if necessary
				SyncMDDNode* p = *node;
				node++;
				copy.deleteNode(p, i);
			}
			else
				node++;
		}
		if (copy.levels[i].empty())
		{
			copy.clear();
			return false;
		}
	}
	copy.clear();
	return true;
}
