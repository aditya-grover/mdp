/*
 *  Copyright (C) 2011 Universidad Simon Bolivar
 *
 *  Permission is hereby granted to distribute this software for
 *  non-commercial research purposes, provided that this copyright
 *  notice is included with any such distribution.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 *  EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
 *  SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
 *  ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 *
 *  Blai Bonet, bonet@ldc.usb.ve
 *
 */

#ifndef UCT_H
#define UCT_H

#include "policy.h"

#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <limits>
#include <vector>
#include <math.h>
#include <map>
#include <array>

//#define DEBUG


namespace Online {

namespace Policy {

namespace UCT {

////////////////////////////////////////////////
//
// Tree
//

template<typename T> struct node_t {
    mutable std::vector<unsigned> counts_;
    mutable std::vector<float> values_;
    node_t(int num_actions)
      : counts_(1+num_actions, 0),
        values_(1+num_actions, 0) {
    }
    ~node_t() { }
};

////////////////////////////////////////////////
//
// Hash Table
//

template<typename T> struct map_functions_t {
    size_t operator()(const std::pair<unsigned, T> &p) const {
        return p.second.hash();
    }
};

struct data_t {
    std::vector<float> values_;
    std::vector<int> counts_;
    data_t(const std::vector<float> &values, const std::vector<int> &counts)
      : values_(values), counts_(counts) { }
    data_t(const data_t &data)
      : values_(data.values_), counts_(data.counts_) { }
#if 0
    data_t(data_t &&data)
      : values_(std::move(data.values_)), counts_(std::move(data.counts_)) { }
#endif
};

template<typename T> class hash_t :
  public Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                  data_t,
                                  map_functions_t<T> > {
  public:
    typedef typename Hash::generic_hash_map_t<std::pair<unsigned, T>,
                                              data_t,
                                              map_functions_t<T> >
            base_type;
    typedef typename base_type::const_iterator const_iterator;
    const_iterator begin() const { return base_type::begin(); }
    const_iterator end() const { return base_type::end(); }

  public:
    hash_t() { }
    virtual ~hash_t() { }
    void print(std::ostream &os) const {
        for( const_iterator it = begin(); it != end(); ++it ) {
            os << "(" << it->first.first << "," << it->first.second << ")" << std::endl;
        }
    }
};

////////////////////////////////////////////////
//
// Policy
//

template<typename T> class uct_t : public improvement_t<T> {
  using policy_t<T>::problem;

  protected:
    unsigned width_;
    unsigned horizon_;
    float parameter_;
    bool random_ties_;
    mutable hash_t<T> table_;
    mutable hash_t<T> temp_table_;       // SAU: Declaration Added
    typedef std::pair<unsigned, T> tree_node_; // SAU
    //mutable std::map<tree_node_,bool> isLeaf_table_; // SAU
    mutable std::map<tree_node_, std::vector<tree_node_>> ground_to_abstract_; // SAU
    //mutable std::map<tree_node_,data_t> abstract_node_data_; //SAU _Ankit

  public:
    uct_t(const policy_t<T> &base_policy,
          unsigned width,
          unsigned horizon,
          float parameter,
          bool random_ties)
      : improvement_t<T>(base_policy),
        width_(width),
        horizon_(horizon),
        parameter_(parameter),
        random_ties_(random_ties) {
        std::stringstream name_stream;
        name_stream << "uct("
                    << "width=" << width_
                    << ",horizon=" << horizon_
                    << ",par=" << parameter_
                    << ",random-ties=" << (random_ties_ ? "true" : "false")
                    << ")";
        policy_t<T>::set_name(name_stream.str());
    }
    virtual ~uct_t() { }

    virtual Problem::action_t operator()(const T &s) const {
        ++policy_t<T>::decisions_;
        table_.clear();
        ground_to_abstract_.clear();  
        // isLeaf_table_.clear();
        // abstract_node_data_.clear(); 
        // SAU: code modified as per Algorithm 2 pseudo code
        unsigned l=10;
        //std::cout<<"\nWidth:"<<width_<<" l:"<<l<<"\n";
        for (unsigned m = 0; m < l; ++m) {
            for( unsigned i = 0; i < width_/l; ++i ) {
                search_tree(s, 0);
            }
         ground_to_abstract_.clear();  
         //isLeaf_table_.clear();
         //abstract_node_data_.clear(); 
        // Abstraction applied and new tree stored in temp_table_
        construct_abstract_tree(s, 0);
        //temp_table_.swap(table_);
        //temp_table_.clear();


        }
        //code modification ends

        /* Original code
        for( unsigned i = 0; i < width_/l; ++i ) {
                search_tree(s, 0);
            }
        */

        typename hash_t<T>::iterator it = table_.find(std::make_pair(0, s));
        /* SAU : //std::cout<<"State Depth:"<<(it->second).values_[0]++<<"\n";
        temp_table_.swap(table_);
        it = temp_table_.find(std::make_pair(0, s));
        //std::cout<<"Temp State Depth:"<<(it->second).values_[0]<<"\n";
        std::exit(0);
        */
        assert(it != table_.end());
        Problem::action_t action = select_action(s, it->second, 0, false, random_ties_);
        assert(problem().applicable(s, action));
        return action;
    }
    virtual const policy_t<T>* clone() const {
        return new uct_t(improvement_t<T>::base_policy_, width_, horizon_, parameter_, random_ties_);
    }
    virtual void print_stats(std::ostream &os) const {
        os << "stats: policy=" << policy_t<T>::name() << std::endl;
        os << "stats: decisions=" << policy_t<T>::decisions_ << std::endl;
        improvement_t<T>::base_policy_.print_stats(os);
    }

    float value(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.values_[1+a];
    }
    unsigned count(const T &s, Problem::action_t a) const {
        typename hash_t<T>::const_iterator it = table_.find(std::make_pair(0, s));
        assert(it != table_.end());
        return it->second.counts_[1+a];
    }
    size_t size() const { return table_.size(); }
    void print_table(std::ostream &os) const {
        table_.print(os);
    }

    /*SAU Checks whether even if one of action is not sampled till now*/
    bool Check_state_undersampled(const T &s, unsigned depth) const{
        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        //typename std::map <tree_node_, data_t>::iterator it;
        //it=table_.find(std::make_pair(depth, s));
        int no_applicable_actions=0;
        int nactions = problem().number_actions(s);
        Problem::action_t a;
        for(a = 0; a < nactions; ++a ) {
            if( problem().applicable(s, a))  //SAU
                no_applicable_actions++;
        }
        if(it->second.counts_[0]>=no_applicable_actions)
            return false;

        return true;

    }


    /* SAU */
    float construct_abstract_tree(const T &s, unsigned depth) const {

        //std::cout<<"\nConstruct Abstract Tree begins. Depth Parameter is:"<<depth<<"\n";

        std::map <T,T> reverseMap, reverseMapLeaf;          //ground to abstract mapping stored only for previous level
        T repLeafNode;
        bool flagLeaf=0;


        std::map <tree_node_, data_t> orderedMap;
        typename hash_t<T>::const_iterator it;
       // typename std::map<tree_node_,bool>::iterator isLeafit;
        for (it = table_.begin(); it != table_.end(); ++it)
        {
			//std::cout<<"\nTree Entry"<<it->first.first<<" State"<<it->first.second;
            orderedMap.insert(std::pair<tree_node_, data_t>(it->first,it->second));
        }
		typename std::map <tree_node_, data_t>::reverse_iterator rit;
		for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        {
			//std::cout<<"\nOrdered Map Entry"<<rit->first.first<<" State"<<rit->first.second;
		}
        //bool leaf=false;
        unsigned oldDepth=depth;
        //std::cout<<"Line 224: Reverse Iterating over orderedMap\n";
        for ( rit= orderedMap.rbegin(); rit != orderedMap.rend(); ++rit)
        {
            unsigned currentDepth=(rit->first).first;
            //std::cout<<"Current Depth:"<<currentDepth<<"\n";
            T currentState=(rit->first).second;

            //check if node is a leaf
            //isLeafit=isLeaf_table_.find(rit->first);
            //assert(isLeafit!=table_.end());
            typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
            unsigned abstractDepth; T abstractState;
            if (currentDepth!=oldDepth){

                oldDepth=currentDepth;
                reverseMap=reverseMapLeaf;

                for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit)
                    {
                        abstractDepth=(rgit->first).first;
                        abstractState=(rgit->first).second;
                        if (abstractDepth==currentDepth+1){
                            std::vector<tree_node_> nextDepthVector=rgit->second;
                            for (unsigned j=0; j< nextDepthVector.size(); j++)
                                reverseMap.insert(std::pair<T,T>(nextDepthVector[j].second,abstractState));
                            }
                        else if (abstractDepth<currentDepth+1)
                            break;
                    }
                reverseMapLeaf.clear();
                flagLeaf=0;
            }

            if(Check_state_undersampled(currentState,currentDepth)){ //SAU_Ankit_Checking Undersampled State
            //if( isLeafit->second ) { // ADD THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
               // leaf=true;
                //std::cout<<currentState<<" depth"<<currentDepth<<" is LEAF"<<leaf<<"\n";
                temp_table_.insert(std::pair<tree_node_, data_t>(rit->first,rit->second));
                if (flagLeaf==0) {
                    repLeafNode=currentState;
                    flagLeaf=1;
                    //Sau_Ankit Adding undersampled nodes to abstract to groudn mapping
                    std::vector<tree_node_> groundStateVector(1,rit->first);
                    //std::cout<<"CURRENT STATE Added to abstract state:"<< currentState<<"\n";
                    //ground_to_abstract_.insert(std::pair<tree_node_,std::vector<tree_node_>>(rit->first,groundStateVector));
                    //abstract_node_data_.insert(std::pair<tree_node_,data_t>(rit->first,rit->second)); //updating abstract node data


                }
                reverseMapLeaf.insert(std::pair<T,T>(currentState,repLeafNode));
                //Sau_Ankit Adding pushing undersampled node to abstract to groudn mapping at that level
                  //typename std::map <tree_node_,std::vector<tree_node_> >::iterator git;
                  //git=ground_to_abstract_.find(std::make_pair(currentDepth,repLeafNode));
                  //(git->second).push_back(std::make_pair(currentDepth,currentState));


            }   else {  // ADD THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                //leaf=false;
                //std::cout<<currentState<<" depth"<<currentDepth<<" is NOT LEAF"<<leaf<<"\n";

                bool isNewNode=false;

                for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit)
                {
                    abstractDepth=(rgit->first).first;
                   // //std::cout<<"Abstract Depth:"<<abstractDepth<<"\n";
                    if (abstractDepth<currentDepth){
                        //make a new abstract node
                        isNewNode=true;
                        break;
                    }
                    else if (abstractDepth == currentDepth){
                        std::vector<tree_node_> groundStateVector = rgit->second;
                        tree_node_ absRepNode=groundStateVector[0];  //abstract representative node
                       // //std::cout<<"TREE"<<absRepNode.first<<"\n";
                        //check abstraction
                        bool isEquiv=true;
                        int nactions = problem().number_actions(absRepNode.second);
						abstractState=absRepNode.second; //Added By SAU_2
                        Problem::action_t a;
                        //std::cout<<"CURRENT STATE:"<< currentState<< " ABSTRACT STATE:"<<abstractState<<"\n";
                        for(a = 0; a < nactions; ++a ) {
                            if( problem().applicable(currentState, a) && problem().applicable(absRepNode.second,a)) { //SAU
                                continue;
                            }
                            else
								//std::cout<<"\nThis action is problematic-"<<a<<problem().applicable(currentState, a)<<"abs"<<problem().applicable(absRepNode.second,a);
                                break;
                        }
                        if (a==nactions)
                            //std::cout<<"CANDIDATE CHECK\n";
                        for(a = 0; a < nactions; ++a ) {
                            if( problem().applicable(currentState, a) && problem().applicable(absRepNode.second,a)) { //SAU
                                // Givan  check

                                //costs
                                if (problem().cost(currentState,a)!=problem().cost(absRepNode.second,a)){
                                    isEquiv=false;
                                    break;
                                }
                                //transition
                                std::vector<std::pair<T, float> > outcomesCurrent, outcomesAbsRep;
                                problem().next(currentState, a, outcomesCurrent);
                                problem().next(absRepNode.second, a, outcomesAbsRep);
                                unsigned osizeCurrent = outcomesCurrent.size(), osizeAbsRep = outcomesAbsRep.size();
                                std::map <T,std::array<float,2>> transComparison;
                                typename std::map <T,std::array<float,2>>::iterator transIter;
                                std::array<float,2> transTemp;
                                transTemp[0]=0; //Sau_Ankit
                                transTemp[1]=0; //Sau_Ankit
                                T temp_outcome_abstract_node;
                                //Modified Code _ Implementation Detail-2 // The original code shows error as well
                                for (unsigned o=0; o<osizeCurrent; o++)
                                {
                                    if(outcomesCurrent[o].second==0) //Sau_ankit
                                        continue;
                                    if(reverseMap.find(outcomesCurrent[o].first)!=reverseMap.end())
                                        temp_outcome_abstract_node=reverseMap[outcomesCurrent[o].first];
                                    else
                                        temp_outcome_abstract_node=outcomesCurrent[o].first;
                                    transIter=transComparison.find(temp_outcome_abstract_node);
                                    if (transIter==transComparison.end()){
                                        transTemp[0]=outcomesCurrent[o].second;
                                        transComparison.insert(std::pair<T,std::array<float,2>>(reverseMap[outcomesCurrent[o].first],transTemp));
                                    } else {
                                        transTemp=transIter->second;
                                        transTemp[0]=transTemp[0]+outcomesCurrent[o].second;
                                        transIter->second=transTemp;
                                    }
                                }
                                for (unsigned o=0; o<osizeAbsRep; o++)
                                {
                                     if(outcomesAbsRep[o].second==0) //Sau_ankit
                                        continue;
                                    if(reverseMap.find(outcomesAbsRep[o].first)!=reverseMap.end())
                                        temp_outcome_abstract_node=reverseMap[outcomesAbsRep[o].first];
                                    else
                                        temp_outcome_abstract_node=outcomesAbsRep[o].first;
                                    transIter=transComparison.find(temp_outcome_abstract_node);
                                    if (transIter==transComparison.end()){
                                        isEquiv=false;
                                        //std::cout<<"problelm in Equivalence here:not reaching some abstract state-"<<a<<outcomesAbsRep[o].first;
                                        break;
                                    } else {
                                        transTemp=transIter->second;
                                        transTemp[1]=transTemp[1]+outcomesAbsRep[o].second;
                                        transIter->second=transTemp;
                                    }
                                }
                                                //original code
                                // for (unsigned o=0; o<osizeCurrent; o++)
                                // {
                                //     if(outcomesCurrent[o].second==0) //Sau_ankit
                                //         continue;
                                //     transIter=transComparison.find(reverseMap[outcomesCurrent[o].first]);
                                //     if (transIter==transComparison.end()){
                                //         transTemp[0]=outcomesCurrent[o].second;
                                //         transComparison.insert(std::pair<T,std::array<float,2>>(reverseMap[outcomesCurrent[o].first],transTemp));
                                //     } else {
                                //         transTemp=transIter->second;
                                //         transTemp[0]=transTemp[0]+outcomesCurrent[o].second;
                                //         transIter->second=transTemp;
                                //     }
                                // }
                                // for (unsigned o=0; o<osizeAbsRep; o++)
                                // {
                                //      if(outcomesAbsRep[o].second==0) //Sau_ankit
                                //         continue;
                                //     transIter=transComparison.find(reverseMap[outcomesAbsRep[o].first]);
                                //     if (transIter==transComparison.end()){
                                //         isEquiv=false;
                                //         //std::cout<<"problelm in Equivalence here:not reaching some abstract state-"<<a<<outcomesAbsRep[o].first;
                                //         break;
                                //     } else {
                                //         transTemp=transIter->second;
                                //         transTemp[1]=transTemp[1]+outcomesAbsRep[o].second;
                                //         transIter->second=transTemp;
                                //     }
                                // }

                                if (!isEquiv)
                                    break;

                                for (transIter=transComparison.begin(); transIter!=transComparison.end(); ++transIter)
                                {
                                    transTemp=transIter->second;
                                    if (transTemp[0]!=transTemp[1]){
                                        isEquiv=false;
                                        //std::cout<<"problelm in Equivalence here:transcomparison-"<<a<<transTemp[0]<<transTemp[1];
                                        break;
                                    }
                                }

                                if(!isEquiv)
                                    break;
                            }
                            else if (problem().applicable(currentState, a) != problem().applicable(absRepNode.second,a)){
                                isEquiv=false;
                                break;
                            }
                        }
                        //std::cout<<"isEquivalent:"<<isEquiv<<"\n";
                        if (isEquiv)
                            {
                                //std::cout<<"Equivalence Found"<<"\n";
                                (rgit->second).push_back(tree_node_(currentDepth,currentState));

                                //SAU_ankit taking average
                                //typename std::map<tree_node_,data_t>::iterator abit;
                                //assert(abstract_node_data_.find(rgit->first)!=abstract_node_data_.end());
                                //abit=abstract_node_data_.find(rgit->first);
                                //if(abit==abstract_node_data_.end())
                                {
                                    //std::cout<<"Error:Some issue in abstract node data";
                                  //  exit(0);
                                }
                                   // exit(0);
                                // for(a = 0; a < nactions; ++a ) {
                                //    abit->second.values_[1+a]=(abit->second.values_[1+a]*abit->second.counts_[1+a]+rit->second.values_[1+a]*rit->second.counts_[1+a]);
                                //     //if((abit->second.counts_[1+a]+rit->second.counts_[1+a]))
                                //     abit->second.values_[1+a]/=(abit->second.counts_[1+a]+rit->second.counts_[1+a]);                                
                                //     abit->second.counts_[1+a]=abit->second.counts_[1+a]+rit->second.counts_[1+a];
                                // }
                                // abit->second.counts_[0]=abit->second.counts_[0]+rit->second.counts_[0];


                                break;
                            }

                    }

                    ////std::cout<<"RGIT"<<(rgit->first).first<<"\n";
                }

                if (isNewNode || ground_to_abstract_.empty()|| (rgit == ground_to_abstract_.rend())){ //Line changed if abstract to ground table ended
                    std::vector<tree_node_> groundStateVector(1,rit->first);
                    //std::cout<<"CURRENT STATE Added to abstract state:"<< currentState<<"\n";
                    ground_to_abstract_.insert(std::pair<tree_node_,std::vector<tree_node_>>(rit->first,groundStateVector));
                    //abstract_node_data_.insert(std::pair<tree_node_,data_t>(rit->first,rit->second)); // SAU_ankit adding first entry intoabstract_node_data
                    temp_table_.insert(std::pair<tree_node_, data_t>(rit->first,rit->second));
                }
                //}

            } // ADD THIS LATER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        }


        return 0;
    }
     // SAU_Ankit ///
     int update_equivalent_nodes(const T &s, unsigned depth, float new_value,Problem::action_t a) const {

        typename std::map <tree_node_,std::vector<tree_node_> >::reverse_iterator rgit;
       // typename std::map <tree_node_,data_t >::iterator abit;
        typename std::vector<tree_node_>::iterator it;
        typename std::vector<tree_node_>::iterator jt;

        for (rgit = ground_to_abstract_.rbegin(); rgit != ground_to_abstract_.rend(); ++rgit){
            if((rgit->first.second)<depth)
                break;
            else if((rgit->first.first)==depth)
            {
                it = std::find (rgit->second.begin(), rgit->second.end(), std::make_pair(depth,s));

                if(it!=rgit->second.end()){
                    //abit=abstract_node_data_.find(rgit->first);
                   //  assert(abit!=abstract_node_data_.end());                   
                   //  abit->second.values_[1+a]=abit->second.values_[1+a]*abit->second.counts_[1+a]+3*new_value;
                   //  abit->second.counts_[1+a]+=3;
                   // abit->second.values_[1+a]/=(abit->second.counts_[1+a]);

                    for(jt=(rgit->second).begin();jt!=(rgit->second).end();jt++){
                        if(it!=jt){
                            typename hash_t<T>::iterator equiv_t = table_.find(std::make_pair(jt->first, jt->second));
                            assert(equiv_t!=table_.end());
                            if(equiv_t==table_.end())
                            {
                                std::cout<<"\nhere "<<"depth"<<jt->first<<" State"<<jt->second;
                                exit(0);
                            }
                            ++equiv_t->second.counts_[0];
                            ++equiv_t->second.counts_[1+a];
                            float &old_value = equiv_t->second.values_[1+a];
                            float  n=equiv_t->second.counts_[1+a];
                            old_value += (new_value - old_value) / n;

                        }
                    }

                }
            }
        }
        return 0;
     }

    float search_tree(const T &s, unsigned depth) const {
        ////std::cout<<"Search tree called\n";
#ifdef DEBUG
        //std::cout << std::setw(2*depth) << "" << "search_tree(" << s << "):";
#endif

        if( (depth == horizon_) || problem().terminal(s) ) {
#ifdef DEBUG
            //std::cout << " end" << std::endl;
#endif
            return 0;
        }

        if( problem().dead_end(s) ) {
#ifdef DEBUG
            //std::cout << " dead-end" << std::endl;
#endif
            return problem().dead_end_value();
        }

        typename hash_t<T>::iterator it = table_.find(std::make_pair(depth, s));
        typename std::map<tree_node_,bool>::iterator isLeafit;
        if( it == table_.end() ) {
            std::vector<float> values(1 + problem().number_actions(s), 0);
            std::vector<int> counts(1 + problem().number_actions(s), 0);
            table_.insert(std::make_pair(std::make_pair(depth, s), data_t(values, counts)));
            float value = evaluate(s, depth);
#ifdef DEBUG
            //std::cout << " insert in tree w/ value=" << value << std::endl;
#endif
            //isLeaf_table_.insert(std::pair<tree_node_,bool>(std::make_pair(depth, s), true)); // SAU
            ////std::cout<<"\nValue at random sampling is "<<value<<" state"<<s<<" depth"<<depth;

            return value;
        } else {

            // select action for this node and increase counts
            //Problem::action_t a = select_action(s, it->second, depth, true, random_ties_);
            Problem::action_t a = select_action(s, it->second, depth, true, true);//SAU Added_A
            //std::cout<<"\nAction Selected-"<<a<<s;
            ++it->second.counts_[0];
            ++it->second.counts_[1+a];

            // sample next state
            std::pair<const T, bool> p = problem().sample(s, a);
            float cost = problem().cost(s, a);

#ifdef DEBUG
            //std::cout << " count=" << it->second.counts_[0]-1
                      << " fetch " << std::setprecision(5) << it->second.values_[1+a]
                      << " a=" << a
                      << " next=" << p.first
                      << std::endl;
#endif
			//isLeafit = isLeaf_table_.find(it->first); // SAU
             //if( !((1+depth == horizon_) || problem().terminal(p.first)) ) // boundary case SAU
				//isLeafit->second=false;
            // do recursion and update value
            float &old_value = it->second.values_[1+a];
            float n = it->second.counts_[1+a];
            float new_value = cost +
              problem().discount() * search_tree(p.first, 1 + depth);
            old_value += (new_value - old_value) / n;

            // SAU_Ankit - update the corresponding equivalent nodes
            if(!Check_state_undersampled(s,depth)) //undersampled states should not call this function
                update_equivalent_nodes(s,depth,new_value,a);

            return old_value; // TO BE DISCUSSED
        }
    }

    Problem::action_t select_action(const T &state,
                                    const data_t &data,
                                    int depth,
                                    bool add_bonus,
                                    bool random_ties) const {
        float log_ns = logf(data.counts_[0]);
        std::vector<Problem::action_t> best_actions;
        int nactions = problem().number_actions(state);
        float best_value = std::numeric_limits<float>::max();

        best_actions.reserve(random_ties ? nactions : 1);
        for( Problem::action_t a = 0; a < nactions; ++a ) {
            if( problem().applicable(state, a) ) {
                // if this action has never been taken in this node, select it
                if( data.counts_[1+a] == 0 ) {
                    return a;
                }

                // compute score of action adding bonus (if applicable)
                assert(data.counts_[0] > 0);
                float par = parameter_ == 0 ? -data.values_[1+a] : parameter_;
                float bonus = add_bonus ? par * sqrtf(2 * log_ns / data.counts_[1+a]) : 0;
                float value = data.values_[1+a] + bonus;

                // update best action so far
                if( value <= best_value ) {
                    if( value < best_value ) {
                        best_value = value;
                        best_actions.clear();
                    }
                    if( random_ties || best_actions.empty() )
                        best_actions.push_back(a);
                }
            }
        }
        assert(!best_actions.empty());
        return best_actions[Random::uniform(best_actions.size())];
    }

    float evaluate(const T &s, unsigned depth) const {
        return Evaluation::evaluation(improvement_t<T>::base_policy_,
                                      s, 1, horizon_ - depth);
    }
};

}; // namespace UCT

template<typename T>
inline const policy_t<T>* make_uct(const policy_t<T> &base_policy,
                                   unsigned width,
                                   unsigned horizon,
                                   float parameter,
                                   bool random_ties) {
    return new UCT::uct_t<T>(base_policy, width, horizon, parameter, random_ties);
}

}; // namespace Policy

}; // namespace Online

#undef DEBUG

#endif

