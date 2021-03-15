#pragma once

#include "MDD.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"

enum heuristics_type { ZERO, CG, DG, WDG, STRATEGY_COUNT };


typedef unordered_map<DoubleConstraintsHasher, int, DoubleConstraintsHasher::Hasher, DoubleConstraintsHasher::EqNode> HTable;


class CBSHeuristic
{
public:
	heuristics_type type;
	rectangle_strategy rectangle_reasoning; // using rectangle reasoning
	corridor_strategy corridor_reasoning; // using corridor reasoning
	bool target_reasoning; // using target reasoning
	bool mutex_reasoning; // using mutex reasoning
	bool disjoint_splitting; // disjoint splitting
	bool PC; // prioritize conflicts
	bool save_stats;

	double runtime_build_dependency_graph = 0;
	double runtime_solve_MVC = 0;

	uint64_t num_merge_MDDs = 0;
	uint64_t num_solve_2agent_problems = 0;
	uint64_t num_memoization = 0; // number of times when memeorization helps

	 //stats
	list<tuple<int, int, const CBSNode*, uint64_t, int> > sub_instances; 	// <agent 1, agent 2, node, number of expanded CT nodes, h value> 


	CBSHeuristic(int num_of_agents,
               const vector<Path*>& paths,
               vector<SingleAgentSolver*>& search_engines,
               const vector<ConstraintTable>& initial_constraints,
               MDDTable& mdd_helper) : num_of_agents(num_of_agents),
                                       paths(paths), search_engines(search_engines),
                                       initial_constraints(initial_constraints), mdd_helper(mdd_helper) {}

	void init()
	{
		if (type == heuristics_type::DG || type == heuristics_type::WDG)
		{
			lookupTable.resize(num_of_agents);
			for (int i = 0; i < num_of_agents; i++)
			{
				lookupTable[i].resize(num_of_agents);
			}
		}
	}

	bool computeInformedHeuristics(CBSNode& curr, double time_limit); // this function is called when poping a CT node for the first time
	void computeQuickHeuristics(CBSNode& curr) const; // this function is called when generating a CT node
	void copyConflictGraph(CBSNode& child, const CBSNode& parent) const;
	void clear() { lookupTable.clear(); }

private:
	int screen = 0;
	int num_of_agents;
	int DP_node_threshold = 8; // run dynamic programming (=brute-force search) only when #nodes <= th
	// int DP_product_threshold = 1024;  // run dynamic programming (=brute-force search) only when #\product range <= th
	vector<vector<HTable> > lookupTable;

	double time_limit;
	int node_limit = 10;  // terminate the sub CBS solver if the number of its expanded nodes exceeds the node limit.
	double start_time;

	const vector<Path*>& paths;
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;
	MDDTable& mdd_helper;


	vector<int> buildConflictGraph(const CBSNode& curr) const;
	void buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges);
	bool buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges);
	bool buildWeightedDependencyGraph(CBSNode& curr, vector<int>& CG);

	bool dependent(int a1, int a2, CBSNode& node); // return true if the two agents are dependent
	int solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal);
	static bool SyncMDDs(const MDD &mdd1, const MDD& mdd2); 	// Match and prune MDD according to another MDD.

	int minimumVertexCover(const vector<int>& CG);
	int minimumVertexCover(const vector<int>& CG, int old_mvc, int cols, int num_of_edges);
	bool KVertexCover(const vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);
	static int greedyMatching(const vector<int>& CG, int cols);
	int minimumWeightedVertexCover(const vector<int>& CG);
	int weightedVertexCover(const vector<int>& CG);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);
	// int ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value) const;
	int MVConAllConflicts(CBSNode& curr);
};