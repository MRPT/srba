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
		void loadFromConfigFile(const mrpt::config::CConfigFileBase & source,const std::string & section)
		{
			MRPT_LOAD_CONFIG_VAR(submap_size,uint64_t,source,section)
			MRPT_LOAD_CONFIG_VAR(min_obs_to_loop_closure,uint64_t,source,section)
		}

		/** See docs of mrpt::utils::CLoadableOptions */
		void saveToConfigFile(mrpt::config::CConfigFileBase & out,const std::string & section) const
		{
			out.write(section,"submap_size",static_cast<uint64_t>(submap_size), /* text width */ 30, 30, "Max. local optimization distance");
			out.write(section,"min_obs_to_loop_closure",static_cast<uint64_t>(min_obs_to_loop_closure), /* text width */ 30, 30, "Min. num. of covisible observations to add a loop closure edge");
		}
	};

	/** Determines the area/submap of the given KF and returns its center KF (the one defining the submap local origin of coordinates) */
	TKeyFrameID get_center_kf_for_kf(const TKeyFrameID kf_id, const parameters_t &params) const
	{
		const size_t SUBMAP_SIZE = params.submap_size; // In # of KFs
		return SUBMAP_SIZE*(kf_id/SUBMAP_SIZE);
	}

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
		ASSERT_(new_kf_id>=1); // We can run an ECP only if we have 2 KFs in the map

		const size_t MINIMUM_OBS_TO_LOOP_CLOSURE = params.min_obs_to_loop_closure;
		const TKeyFrameID current_center_kf_id = get_center_kf_for_kf(new_kf_id, params);
		const topo_dist_t min_dist_for_loop_closure = rba_engine.parameters.srba.max_tree_depth + 1; // By definition of loop closure in the SRBA framework

		// Go thru all observations and for those already-seen LMs, check the distance between their base KFs and (i_id):
		// Make a list of base KFs of my new observations, ordered in descending order by # of shared observations:
		base_sorted_lst_t         obs_for_each_base_sorted;
		srba::internal::make_ordered_list_base_kfs<traits_t,typename rba_engine_t::rba_problem_state_t>(obs, rba_engine.get_rba_state(), obs_for_each_base_sorted);

		// Make vote list for each central KF:
		map<TKeyFrameID,size_t>  obs_for_each_area;
		map<TKeyFrameID,bool>    base_is_center_for_all_obs_in_area;  // Detect whether the base KF for observations is the area center or not (needed to determine exact worst-case topological distances)
		map<TKeyFrameID,map<TKeyFrameID,size_t> >  obs_for_base_KF_grouped_by_area;
		for (base_sorted_lst_t::const_iterator it=obs_for_each_base_sorted.begin();it!=obs_for_each_base_sorted.end();++it)
		{
			const size_t      num_obs_this_base = it->first;
			const TKeyFrameID base_id = it->second;

			const TKeyFrameID this_localmap_center = get_center_kf_for_kf(base_id, params);
			obs_for_each_area[this_localmap_center] += num_obs_this_base;
			obs_for_base_KF_grouped_by_area[this_localmap_center][base_id] += num_obs_this_base;

			// Fist time this area is observed?
			if (base_is_center_for_all_obs_in_area.find(this_localmap_center)==base_is_center_for_all_obs_in_area.end())
				base_is_center_for_all_obs_in_area[this_localmap_center] = true;
			// Filter:
			if (base_id!=this_localmap_center)  base_is_center_for_all_obs_in_area[this_localmap_center] = false;
		}

		// Sort submaps by votes:
		base_sorted_lst_t   obs_for_each_area_sorted;
		for (map<TKeyFrameID,size_t>::const_iterator it=obs_for_each_area.begin();it!=obs_for_each_area.end();++it)
			obs_for_each_area_sorted.insert( make_pair(it->second,it->first) );

		// Within each submap, sort by the most voted base KF, so we can detect the most connected KF in the case of a loop closure:
		map<TKeyFrameID,base_sorted_lst_t>  obs_for_base_KF_grouped_by_area_sorted;
		for (map<TKeyFrameID,map<TKeyFrameID,size_t> >::const_iterator it=obs_for_base_KF_grouped_by_area.begin();it!=obs_for_base_KF_grouped_by_area.end();++it)
		{
			base_sorted_lst_t &bsl = obs_for_base_KF_grouped_by_area_sorted[it->first];
			for (map<TKeyFrameID,size_t>::const_iterator it2=it->second.begin();it2!=it->second.end();++it2)
				bsl.insert( make_pair(it2->second,it2->first) );
		}

		// First: always create one edge:
		//  Regular KFs:      new KF                         ==> current_center_kf_id
		//  New area center:  new KF (=current_center_kf_id) ==> center of previous 
		{
			if (current_center_kf_id == new_kf_id) {
				// We are about to start an empty, new area: link with the most connected area (in the general code above)
			}
			else {
				// Connect to the local area center:
				TNewEdgeInfo nei;
				nei.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( current_center_kf_id, new_kf_id), obs);
				nei.has_approx_init_val = false; // By default: Will need to estimate this one

				// Add to list of newly created kf2kf edges:
				new_k2k_edge_ids.push_back(nei);
			}
		}

		// Go thru candidate areas for loop closures:
		for (base_sorted_lst_t::const_iterator it=obs_for_each_area_sorted.begin();it!=obs_for_each_area_sorted.end();++it)
		{
			const size_t      num_obs_this_base = it->first;
			const TKeyFrameID remote_center_kf_id = it->second;
			const bool        is_strongest_connected_edge =  (it==obs_for_each_area_sorted.begin()); // Is this the first one?

			//VERBOSE_LEVEL(2) << "[edge_creation_policy] Consider: area central kf#"<< remote_center_kf_id << " with #obs:"<< num_obs_this_base << endl;

			// Create edges to all these central KFs if they're too far:

			// Find the distance between "remote_center_kf_id" <=> "new_kf_id"
			const TKeyFrameID from_id = current_center_kf_id; //new_kf_id;
			const TKeyFrameID to_id   = remote_center_kf_id;
			if (from_id==to_id)
				continue; // We are observing a LM within our local submap; it is fine.

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

			// We may have to add the 2 edges:
			//    OBSERVER_KF ==(1)==> CENTER1->CENTER2 ===(2)==> BASE_KF
			// to determine the exact topological distance to the base of the currently observed LMs and whether a loop closure actually happened.
			topo_dist_t dist_extra_edges = 2; 
			if (current_center_kf_id == new_kf_id)                       dist_extra_edges--;
			if (base_is_center_for_all_obs_in_area[remote_center_kf_id]) dist_extra_edges--;

			if ( found_distance >= min_dist_for_loop_closure - dist_extra_edges )  // Note: DO NOT sum `dist_extra_edges` to the left side of the equation, since found_distance may be numeric_limits::max<>!!
			{
				if (num_obs_this_base>=MINIMUM_OBS_TO_LOOP_CLOSURE)
				{
					// The KF is TOO FAR: We will need to create an additional edge:
					TNewEdgeInfo nei;

					nei.id = rba_engine.create_kf2kf_edge(from_id, TPairKeyFrameID( to_id, from_id), obs);
					nei.has_approx_init_val = false; // By default: Will need to estimate this one
					
					// Fill these loop closure helper fields:
					nei.loopclosure_observer_kf = new_kf_id;
					{
						// Take the KF id of the strongest connection:
						const base_sorted_lst_t & bsl = obs_for_base_KF_grouped_by_area_sorted[remote_center_kf_id];
						ASSERT_(!bsl.empty());
						nei.loopclosure_base_kf = bsl.begin()->second;
					}
					new_k2k_edge_ids.push_back(nei);
				}
				else {
					//VERBOSE_LEVEL(1) << "[edge_creation_policy] Skipped extra edge " << remote_center_kf_id <<"->"<<new_kf_id << " with #obs: "<< num_obs_this_base << " and already_connected="<< (already_connected?"TRUE":"FALSE") << endl;
				}
			}
		}

		ASSERTMSG_(new_k2k_edge_ids.size()>=1, mrpt::format("Error for new KF#%u: no suitable linking KF found with a minimum of %u common observation: the node becomes isolated of the graph!", static_cast<unsigned int>(new_kf_id),static_cast<unsigned int>(MINIMUM_OBS_TO_LOOP_CLOSURE) ));

		// Debug:
		if (new_k2k_edge_ids.size()>1) // && m_verbose_level>=1)
		{
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_GREEN);
			cout << "\n[edge_creation_policy] Loop closure detected for KF#"<< new_kf_id << ", edges: ";
			for (size_t j=0;j<new_k2k_edge_ids.size();j++)
				cout << rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[j].id].from <<"->"<<rba_engine.get_rba_state().k2k_edges[new_k2k_edge_ids[j].id].to<<", ";
			cout << endl;
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
		}

	} // end eval<>()

};  // end of struct

} } // End of namespaces
