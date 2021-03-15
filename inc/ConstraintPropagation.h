// #include <vector>

#ifndef CONS_PROP_H
#define CONS_PROP_H

#include <boost/unordered_set.hpp>
#include "MDD.h"

typedef std::pair<MDDNode*, MDDNode*> node_pair;
typedef std::pair<node_pair, node_pair> edge_pair;

bool is_edge_mutex(edge_pair ep);

class ConstraintPropagation{
protected:
  // TODO ownership?
  // std::vector<MDD*> mdds;
  MDD* mdd0;
  MDD* mdd1;


	// check whether two nodes could be mutexed
	// return true if they are mutexed
	bool should_be_fwd_mutexed(MDDNode*, MDDNode*);

	bool should_be_fwd_mutexed(MDDNode* node_a, MDDNode* node_a_to,
							   MDDNode* node_b, MDDNode* node_b_to);

	bool should_be_bwd_mutexed(MDDNode*, MDDNode*);
	bool should_be_bwd_mutexed(MDDNode* node_a, MDDNode* node_a_to,
							   MDDNode* node_b, MDDNode* node_b_to);


	void add_bwd_node_mutex(MDDNode* node_a, MDDNode* node_b);

	void add_fwd_node_mutex(MDDNode* node_a, MDDNode* node_b);
	void add_fwd_edge_mutex(MDDNode* node_a, MDDNode* node_a_to,
							MDDNode* node_b, MDDNode* node_b_to);

	// boost::unordered_set<node_pair> node_cons;

	/*
	  A Mutex is in the form of <<node*, node*>, <node*, node*>>
	  if second node* in each pair are nullptr, then it is a node mutex

	 */

	// void fwd_mutex_prop_generalized_helper(MDD* mdd_0, MDD* mdd_1, int level);

public:
	ConstraintPropagation(MDD* mdd0, MDD* mdd1) :
			mdd0(mdd0), mdd1(mdd1) {}

	boost::unordered_set<edge_pair> fwd_mutexes;
	boost::unordered_set<edge_pair> bwd_mutexes;

  virtual void init_mutex();
  virtual void fwd_mutex_prop();
  // void fwd_mutex_prop_generalized();

  virtual void bwd_mutex_prop();

  virtual bool has_mutex(edge_pair);
  virtual bool has_mutex(MDDNode*, MDDNode*);

  virtual bool has_fwd_mutex(edge_pair);
  virtual bool has_fwd_mutex(MDDNode*, MDDNode*);

  // MDD 0 of level_0 and MDD 1 of level_1 mutexed at goal
  virtual bool mutexed(int level_0, int level_1);
  virtual bool feasible(int level_0, int level_1);
  virtual int _feasible(int level_0, int level_1);

  // virtual bool semi_cardinal(int level, int loc);

  virtual std::pair<std::vector<Constraint>, std::vector<Constraint>> generate_constraints(int, int);

  virtual ~ConstraintPropagation(){};
};

#endif
