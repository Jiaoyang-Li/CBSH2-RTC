#pragma once

#include "common.h"
#include "Conflict.h"

enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };

class CBSNode
{
public:
	// the following is used to compare nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const CBSNode* n1, const CBSNode* n2) const 
		{
			if (n1->tie_breaking == n2->tie_breaking)
				return rand() % 2;
			return n1->tie_breaking >= n2->tie_breaking;
		}
	};  // used by FOCAL to compare nodes by tie_breaking value (top of the heap has min tie_breaking value)

	typedef boost::heap::pairing_heap<CBSNode*, boost::heap::compare<CBSNode::compare_node>>::handle_type open_handle_t;
	typedef boost::heap::pairing_heap<CBSNode*, boost::heap::compare<CBSNode::secondary_compare_node>>::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by  for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specialized template inside std namespace
	struct ICBSNodeHasher 
	{
		std::size_t operator()(const CBSNode* n) const
		{
			return std::hash<uint64_t>()(n->time_generated);
		}
	};

	// conflicts in the current paths
	list<shared_ptr<Conflict>> conflicts;
	list<shared_ptr<Conflict>> unknownConf;

	// The chosen conflict
	shared_ptr<Conflict> conflict;

	boost::unordered_map<int, int> conflictGraph; //<edge index, weight> // TODO: This can be deleted.
	CBSNode* parent;

	list<pair<int, Path>> paths; // new paths
	list<Constraint> constraints; // new constraints


	int g_val;
	int h_val;
	int depth; // depth of this CT node
	size_t makespan = 0; // makespan over all paths
	int tie_breaking = 0; // tie breaking for node selection
	bool h_computed = false;

	uint64_t time_expanded;
	uint64_t time_generated;


	void clear();
	void printConflictGraph(int num_of_agents) const;
	void printConstraints(int agent) const; // print constraints on the given agent
};


std::ostream& operator<<(std::ostream& os, const CBSNode& node);


struct ConstraintsHasher // Hash a CT node by constraints on one agent
{
	int a{};
	const CBSNode* n{};
	ConstraintsHasher(int a, CBSNode* n) : a(a), n(n) {};

	struct EqNode
	{
		bool operator()(const ConstraintsHasher& c1, const ConstraintsHasher& c2) const
		{
			if (c1.a != c2.a)
				return false;
				
			std::set<Constraint> cons1, cons2;
			const CBSNode* curr = c1.n;
			while (curr->parent != nullptr)
			{
				for (const auto& constraint : curr->constraints)
				{
					if (get<4>(constraint) == constraint_type::LEQLENGTH ||
						get<4>(constraint) == constraint_type::POSITIVE_VERTEX ||
						get<4>(constraint) == constraint_type::POSITIVE_EDGE ||
						get<0>(constraint) == c1.a)
					{
						cons1.insert(constraint);
					}
				}
				curr = curr->parent;
			}
			curr = c2.n;
			while (curr->parent != nullptr)
			{
				for (const auto& constraint : curr->constraints)
				{
					if (get<4>(constraint) == constraint_type::LEQLENGTH ||
						get<4>(constraint) == constraint_type::POSITIVE_VERTEX ||
						get<4>(constraint) == constraint_type::POSITIVE_EDGE ||
						get<0>(constraint) == c2.a) 
					{
						cons2.insert(constraint);
					}
				}
				curr = curr->parent;
			}

			return equal(cons1.begin(), cons1.end(), cons2.begin(), cons2.end());
		}
	};

	struct Hasher
	{
		std::size_t operator()(const ConstraintsHasher& entry) const
		{
			const CBSNode* curr = entry.n;
			size_t cons_hash = 0;
			while (curr->parent != nullptr)
			{
				if (get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE ||
					get<0>(curr->constraints.front()) == entry.a)
				{
					for (auto con : curr->constraints)
					{
						cons_hash += 3 * std::hash<int>()(std::get<0>(con)) +
									 5 * std::hash<int>()(std::get<1>(con)) +
									 7 * std::hash<int>()(std::get<2>(con)) +
									 11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons_hash;
		}
	};
};

struct DoubleConstraintsHasher // Hash a CT node by constraints on two agents
{
	int a1{};
	int a2{};
	CBSNode* n{};

	DoubleConstraintsHasher() = default;
	DoubleConstraintsHasher(int a1, int a2, CBSNode* n) : a1(a1), a2(a2), n(n) {};

	struct EqNode
	{
		bool operator() (const DoubleConstraintsHasher& h1, const DoubleConstraintsHasher& h2) const
		{
			if (h1.a1 != h2.a1 || h1.a2 != h2.a2)
				return false;
			std::set<Constraint> cons1[2], cons2[2];
			const CBSNode* curr = h1.n;
			while (curr->parent != nullptr)
			{
				for (const auto& constraint : curr->constraints)
				{
					if (get<4>(constraint) == constraint_type::LEQLENGTH ||
						get<4>(constraint) == constraint_type::POSITIVE_VERTEX ||
						get<4>(constraint) == constraint_type::POSITIVE_EDGE)
					{
						cons1[0].insert(constraint);
						cons2[0].insert(constraint);
					}
					else if (get<0>(constraint) == h1.a1)
						cons1[0].insert(constraint);
					else if (get<0>(constraint) == h1.a2)
						cons2[0].insert(constraint);
				}
				curr = curr->parent;
			}
			curr = h2.n;
			while (curr->parent != nullptr)
			{
				for (const auto& constraint : curr->constraints)
				{
					if (get<4>(constraint) == constraint_type::LEQLENGTH ||
						get<4>(constraint) == constraint_type::POSITIVE_VERTEX ||
						get<4>(constraint) == constraint_type::POSITIVE_EDGE)
					{
						cons1[1].insert(constraint);
						cons2[1].insert(constraint);
					}
					else if (get<0>(constraint) == h2.a1)
						cons1[1].insert(constraint);
					else if (get<0>(constraint) == h2.a2)
						cons2[1].insert(constraint);
				}
				curr = curr->parent;
			}
			if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
				return false;

			if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
				return false;
			return equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
		}
	};


	struct Hasher
	{
		size_t operator()(const DoubleConstraintsHasher& entry) const
		{
			CBSNode* curr = entry.n;
			size_t cons1_hash = 0, cons2_hash = 0;
			while (curr->parent != nullptr)
			{
				for (const auto& constraint : curr->constraints)
				{
					if (get<0>(constraint) == entry.a1)
					{
						cons1_hash += 3 * std::hash<int>()(std::get<0>(constraint)) +
							5 * std::hash<int>()(std::get<1>(constraint)) +
							7 * std::hash<int>()(std::get<2>(constraint)) +
							11 * std::hash<int>()(std::get<3>(constraint));
					}
					else if (get<0>(constraint) == entry.a2)
					{
						cons2_hash += 3 * std::hash<int>()(std::get<0>(constraint)) +
							5 * std::hash<int>()(std::get<1>(constraint)) +
							7 * std::hash<int>()(std::get<2>(constraint)) +
							11 * std::hash<int>()(std::get<3>(constraint));
					}
					else if (get<4>(constraint) == constraint_type::LEQLENGTH ||
						get<4>(constraint) == constraint_type::POSITIVE_VERTEX ||
						get<4>(constraint) == constraint_type::POSITIVE_EDGE)
					{
						cons1_hash += 3 * std::hash<int>()(std::get<0>(constraint)) +
							5 * std::hash<int>()(std::get<1>(constraint)) +
							7 * std::hash<int>()(std::get<2>(constraint)) +
							11 * std::hash<int>()(std::get<3>(constraint));
						cons2_hash += 3 * std::hash<int>()(std::get<0>(constraint)) +
							5 * std::hash<int>()(std::get<1>(constraint)) +
							7 * std::hash<int>()(std::get<2>(constraint)) +
							11 * std::hash<int>()(std::get<3>(constraint));
					}
				}
				curr = curr->parent;
			}
			return cons1_hash ^ (cons2_hash << 1);
		}
	};
};
