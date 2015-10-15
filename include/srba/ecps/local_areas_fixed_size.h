/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/utils/CConfigFileBase.h> // MRPT_LOAD_CONFIG_VAR

namespace srba {
namespace ecps {

/** Edge creation policy: The sub-map method introduced in the ICRA2013 paper. Keyframes are 
    * as submaps of a fixed maximum number of KFs. Loop closure edges are inserted depending on 
    * thresholding parameters related to the number of shared observations across distant areas.
    *
    * \ingroup mrpt_srba_ecps
	*/
struct local_areas_fixed_size
{
	struct parameters_t
	{
		size_t              submap_size;  //!< Default:15, Fixed submap size (number of keyframes)
		size_t              min_obs_to_loop_closure; //!< Default:4, reduce to 1 for relative graph-slam

		/** Ctor for default values */
		parameters_t() :
			submap_size          ( 15 ),
			min_obs_to_loop_closure ( 4 )
		{ }

		/** See docs of mrpt::utils::CLoadableOptions */
		void loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section)
		{
			MRPT_LOAD_CONFIG_VAR(submap_size,uint64_t,source,section)
			MRPT_LOAD_CONFIG_VAR(min_obs_to_loop_closure,uint64_t,source,section)
		}

		/** See docs of mrpt::utils::CLoadableOptions */
		void saveToConfigFile(mrpt::utils::CConfigFileBase & out,const std::string & section) const
		{
			out.write(section,"submap_size",static_cast<uint64_t>(submap_size), /* text width */ 30, 30, "Max. local optimization distance");
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
		ASSERT_(new_kf_id>=1)

		const size_t MINIMUM_OBS_TO_LOOP_CLOSURE = params.min_obs_to_loop_closure;
		const size_t SUBMAP_SIZE = params.submap_size; // In # of KFs
		const TKeyFrameID cur_localmap_center = SUBMAP_SIZE*((new_kf_id-1)/SUBMAP_SIZE);

		// Go thru all observations and for those already-seen LMs, check the distance between their base KFs and (i_id):
		// Make a list of base KFs of my new observations, ordered in descending order by # of shared observations:
		base_sorted_lst_t         obs_for_each_base_sorted;
		srba::internal::make_ordered_list_base_kfs<traits_t,typename rba_engine_t::rba_problem_state_t>(obs, rba_engine.get_rba_state(), obs_for_each_base_sorted);

		// Make vote list for each central KF:
		map<TKeyFrameID,size_t>  obs_for_each_area;
		for (base_sorted_lst_t::const_iterator it=obs_for_each_base_sorted.begin();it!=obs_for_each_base_sorted.end();++it)
		{
			const size_t      num_obs_this_base = it->first;
			const TKeyFrameID base_id = it->second;

			const TKeyFrameID this_localmap_center = SUBMAP_SIZE*(base_id/SUBMAP_SIZE);

			obs_for_each_area[this_localmap_center] += num_obs_this_base;
		}

		// Sort by votes:
		base_sorted_lst_t   obs_for_each_area_sorted;
		for (map<TKeyFrameID,size_t>::const_iterator it=obs_for_each_area.begin();it!=obs_for_each_area.end();++it)
			obs_for_each_area_sorted.insert( make_pair(it->second,it->first) );

		// Go thru candidate areas:
		for (base_sorted_lst_t::const_iterator it=obs_for_each_area_sorted.begin();it!=obs_for_each_area_sorted.end();++it)
		{
			const size_t      num_obs_this_base = it->first;
			const TKeyFrameID central_kf_id = it->second;
			const bool        is_strongest_connected_edge =  (it==obs_for_each_area_sorted.begin()); // Is this the first one?

			//VERBOSE_LEVEL(2) << "[edge_creation_policy] Consider: area central kf#"<< central_kf_id << " with #obs:"<< num_obs_this_base << endl;

			// Create edges to all these central KFs if they're too far:

			// Find the distance between "central_kf_id" <=> "new_kf_id"
			const TKeyFrameID from_id = new_kf_id;
			const TKeyFrameID to_id   = central_kf_id;

			typename rba_engine_t::rba_problem_state_t::TSpanningTree::next_edge_maps_t::const_iterator it_from = rba_engine.get_rba_state().spanning_tree.sym.next_edge.find(from_id);

			topo_dist_t  found_distance = numeric_limits<topo_dist_t>::max();

			if (it_from != rba_engine.get_rba_state().spanning_tree.sym.next_edge.end())
			{
				const map<TKeyFrameID,TSpanTreeEntry> &from_Ds = it_from->second;
				map<TKeyFrameID,TSpanTreeEntry>::const_iterator it_to_dist = from_Ds.find(to_id);

				if (it_to_dist != from_Ds.end())
					found_distance = it_to_dist->second.distance;
			}
			else
			{
				// The new KF doesn't still have any edge created to it, that's why we didn't found any spanning tree for it.
				// Since this means that the KF is aisolated from the rest of the world, leave the topological distance to infinity.
			}

			const topo_dist_t min_dist_for_loop_closure = rba_engine.parameters.srba.max_tree_depth + 1; // By definition of loop closure in the SRBA framework

			if ( found_distance>=min_dist_for_loop_closure)
			{
				// Skip if there is already a topo connection between the two areas:
				const std::pair<TKeyFrameID,TKeyFrameID> c2c_pair = make_pair( std::min(central_kf_id,cur_localmap_center), std::max(central_kf_id,cur_localmap_center) );
				const bool already_connected = 
					(is_strongest_connected_edge ? false : true) 
					&& 
					central2central_connected_areas.find(c2c_pair)!=central2central_connected_areas.end();

				if (num_obs_this_base>=MINIMUM_OBS_TO_LOOP_CLOSURE && !already_connected)
				{
					// The KF is TOO FAR: We will need to create an additional edge:
					TNewEdgeInfo nei;

					nei.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( central_kf_id, new_kf_id), obs);

					nei.has_approx_init_val = false; // // By default: Will need to estimate this one

					// look at last kf's kf2kf edges for an initial guess to ease optimization:
					if ( last_timestep_touched_kfs.count(central_kf_id) != 0 )
					{
						// Get the relative post from the numeric spanning tree, which should be up-to-date:
						typename kf2kf_pose_traits<typename traits_t::original_kf2kf_pose_t>::TRelativePosesForEachTarget::const_iterator it_tree4_central = rba_engine.get_rba_state().spanning_tree.num.find(central_kf_id);
						ASSERT_(it_tree4_central!=rba_engine.get_rba_state().spanning_tree.num.end())

						typename kf2kf_pose_traits<typename traits_t::original_kf2kf_pose_t>::frameid2pose_map_t::const_iterator it_nei_1 = 
							it_tree4_central->second.find(new_kf_id-1);
						if (it_nei_1!=it_tree4_central->second.end())
						{
							// Found: reuse this relative pose as a good initial guess for the estimation
							rba_engine.get_rba_state().k2k_edges[nei.id].inv_pose = -it_nei_1->second.pose; // Note the "-" inverse operator, it is important
							nei.has_approx_init_val = true;
						}
					}

					if (!nei.has_approx_init_val)
					{
						// Otherwise: estimate
						MRPT_TODO("Important: provide a mech to estimate init rel poses on loop closures for each sensor impl");
						std::cout << "TODO: init rel pos bootstrap\n";
					}

					new_k2k_edge_ids.push_back(nei);
					central2central_connected_areas.insert(c2c_pair);

					mrpt::system::setConsoleColor(mrpt::system::CONCOL_BLUE);
					//VERBOSE_LEVEL(2) << "[edge_creation_policy] Created edge #"<< nei.id << ": "<< central_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base<< endl;
					mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
				}
				else
				{
					//VERBOSE_LEVEL(1) << "[edge_creation_policy] Skipped extra edge " << central_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base << " and already_connected="<< (already_connected?"TRUE":"FALSE") << endl;
				}
			}
		}

		// At least we must create 1 edge!
		if (new_k2k_edge_ids.empty())
		{
			// Try linking to a "non-central" KF but at least having the min. # of desired shared observations:
			if (!obs_for_each_base_sorted.empty())
			{
				const size_t     most_connected_nObs  = obs_for_each_base_sorted.begin()->first;
				const TKeyFrameID most_connected_kf_id = obs_for_each_base_sorted.begin()->second;
				if (most_connected_nObs>=MINIMUM_OBS_TO_LOOP_CLOSURE)
				{
					TNewEdgeInfo nei;

					nei.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( most_connected_kf_id, new_kf_id), obs);
					nei.has_approx_init_val = false; // Will need to estimate this one
					new_k2k_edge_ids.push_back(nei);

					//VERBOSE_LEVEL(0) << "[edge_creation_policy] Created edge of last resort #"<< nei.id << ": "<< most_connected_kf_id <<"->"<<new_kf_id << " with #obs: "<< most_connected_nObs<< endl;
				}
			}
		}

		// Recheck: if even with the last attempt we don't have any edge, it's bad:
		ASSERTMSG_(new_k2k_edge_ids.size()>=1, mrpt::format("Error for new KF#%u: no suitable linking KF found with a minimum of %u common observation: the node becomes isolated of the graph!", static_cast<unsigned int>(new_kf_id),static_cast<unsigned int>(MINIMUM_OBS_TO_LOOP_CLOSURE) ))

		// save for the next timestep:
		last_timestep_touched_kfs.clear();
		for (size_t i=0;i<new_k2k_edge_ids.size();i++) {
			last_timestep_touched_kfs.insert( rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[i].id].from );
			last_timestep_touched_kfs.insert( rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[i].id].to );
		}

		// Debug:
		if (new_k2k_edge_ids.size()>1) // && m_verbose_level>=1)
		{
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_BLUE);
			cout << "\n[edge_creation_policy] Loop closure detected for KF#"<< new_kf_id << ", edges: ";
			for (size_t j=0;j<new_k2k_edge_ids.size();j++)
			{
				cout << rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[j].id].from <<"->"<<rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[j].id].to<<", ";
			}
			cout << endl;
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
		}
	
	}

private:
	std::set<std::pair<TKeyFrameID,TKeyFrameID> > central2central_connected_areas; // 1st the lowest id, to avoid duplicates
	std::set<size_t> last_timestep_touched_kfs;

};  // end of struct

} } // End of namespaces
