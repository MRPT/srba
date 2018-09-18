/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/config/CConfigFileBase.h> // MRPT_LOAD_CONFIG_VAR

namespace srba {
namespace ecps {

/** Edge creation policy: A linear graph where each KF is always connected to the precedding one. 
    * Occasional loop closure edges are also inserted depending on thresholding parameters related to 
	* the number of observations whose base KF is too distant.
    *
    * \ingroup mrpt_srba_ecps
	*/
struct classic_linear_rba
{
	struct parameters_t
	{
		size_t              min_obs_to_loop_closure; //!< Default:4, reduce to 1 for relative graph-slam

		/** Ctor for default values */
		parameters_t() :
			min_obs_to_loop_closure ( 4 )
		{ }

		/** See docs of mrpt::utils::CLoadableOptions */
		void loadFromConfigFile(const mrpt::config::CConfigFileBase & source,const std::string & section)
		{
			MRPT_LOAD_CONFIG_VAR(min_obs_to_loop_closure,uint64_t,source,section)
		}

		/** See docs of mrpt::utils::CLoadableOptions */
		void saveToConfigFile(mrpt::config::CConfigFileBase & out,const std::string & section) const
		{
			out.write(section,"min_obs_to_loop_closure",static_cast<uint64_t>(min_obs_to_loop_closure), /* text width */ 30, 30, "Min. num. of covisible observations to add a loop closure edge");
		}
	};
	
	/** Implements the edge-creation policy. 
	 * \tparam traits_t Use rba_joint_parameterization_traits_t<kf2kf_pose_t,landmark_t,obs_t>
	 */
	template <class traits_t,class rba_engine_t>
	void eval(
		const TKeyFrameID               new_kf_id,
		const typename traits_t::new_kf_observations_t   & obs,
		std::vector<TNewEdgeInfo> &new_k2k_edge_ids,
		rba_engine_t       & rba_engine,
		const parameters_t &params)
	{
		using namespace std;
		ASSERT_(new_kf_id>=1);

		// (1/2) Always add an edge (n-1) => (n)
		// -------------------------------------------
		const typename traits_t::original_kf2kf_pose_t::pose_t   init_inv_pose;

		TNewEdgeInfo nei1;
		nei1.has_approx_init_val = true; // In a linear graph it's a reasonable approx. to make each KF start at the last KF pose, which is what means a null pose init val.

		nei1.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( new_kf_id-1, new_kf_id), obs, init_inv_pose);
		new_k2k_edge_ids.push_back(nei1);

		// (2/2) Need to add loop closures?
		// -------------------------------------------
		const topo_dist_t min_dist_for_loop_closure = rba_engine.parameters.srba.max_tree_depth + 1; // By definition of loop closure in the SRBA framework

		// Go thru all observations and for those already-seen LMs, check the distance between their base KFs and (i_id):
		// Make a list of base KFs of my new observations, ordered in descending order by # of shared observations:
		base_sorted_lst_t         obs_for_each_base_sorted;
		srba::internal::make_ordered_list_base_kfs<traits_t,typename rba_engine_t::rba_problem_state_t>(obs, rba_engine.get_rba_state(), obs_for_each_base_sorted);

		for (base_sorted_lst_t::const_iterator it=obs_for_each_base_sorted.begin();it!=obs_for_each_base_sorted.end();++it)
		{
			const size_t      num_obs_this_base = it->first;
			
			// Find the distance between "central_kf_id" <=> "new_kf_id"
			const TKeyFrameID from_id = new_kf_id;
			const TKeyFrameID to_id = it->second;

			typename rba_engine_t::rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_from = rba_engine.get_rba_state().spanning_tree.sym.next_edge.find(from_id);

			topo_dist_t  found_distance = numeric_limits<topo_dist_t>::max();

			if (it_from != rba_engine.get_rba_state().spanning_tree.sym.next_edge.end()) {
				const map<TKeyFrameID,TSpanTreeEntry> &from_Ds = it_from->second;
				map<TKeyFrameID,TSpanTreeEntry>::const_iterator it_to_dist = from_Ds.find(to_id);
				if (it_to_dist != from_Ds.end())
					found_distance = it_to_dist->second.distance;
			}
			else {
				// The new KF doesn't still have any edge created to it, that's why we didn't found any spanning tree for it.
				// Since this means that the KF is aisolated from the rest of the world, leave the topological distance to infinity.
			}

			if ( found_distance>=min_dist_for_loop_closure && num_obs_this_base>=params.min_obs_to_loop_closure)
			{
				// The KF is TOO FAR: We will need to create an additional edge:
				TNewEdgeInfo nei;

				nei.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( to_id, new_kf_id), obs);

				nei.has_approx_init_val = false; // By default: Will need to estimate this one
				new_k2k_edge_ids.push_back(nei);

				mrpt::system::setConsoleColor(mrpt::system::CONCOL_BLUE);
				//VERBOSE_LEVEL(2) << "[edge_creation_policy] Created edge #"<< nei.id << ": "<< central_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base<< endl;
				mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
			}
		} // end for each base KF

	} // end of eval()

};  // end of struct

} } // End of namespaces
