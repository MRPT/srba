/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace srba {


/** Determines and creates the new kf2fk edges given the set of new observations: */
template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE,class RBA_OPTIONS>
void RbaEngine<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE,RBA_OPTIONS>::determine_kf2kf_edges_to_create(
	const TKeyFrameID               new_kf_id,
	const typename traits_t::new_kf_observations_t   & obs,
	std::vector<TNewEdgeInfo> &new_k2k_edge_ids )
{
	// (1) Invoke edge creation policy
	// ------------------------------------------------
	new_k2k_edge_ids.clear();
	if (rba_state.keyframes.size()==1)
		return; // If this is the first KF, there's no other one to which we can connect! -> Return an empty set of edges.
	
	edge_creation_policy.template eval<traits_t,rba_engine_t>(new_kf_id,obs,new_k2k_edge_ids, *this,parameters.ecp);

	// (2) Common part: 
	// try to figure out the initial relative poses of those kf2kf edges whose relative pose was not guessed by the ECP.
	// ----------------------------------------------------------

	// For each edge in "new_k2k_edge_ids"
	const size_t nNEI = new_k2k_edge_ids.size();
	for (size_t i=0;i<nNEI;i++)
	{
		TNewEdgeInfo & nei = new_k2k_edge_ids[i];
		if (nei.has_approx_init_val)
			continue; // This edge already has an initial guess
		
		k2k_edge_t & nei_edge = rba_state.k2k_edges[nei.id];
		
		// Method #1: look at last kf's kf2kf edges for an initial guess to ease optimization:
		if ( rba_state.last_timestep_touched_kfs.count(nei_edge.from) != 0 )
		{
			// Get the relative post from the numeric spanning tree, which should be up-to-date:
			typename kf2kf_pose_traits<typename traits_t::original_kf2kf_pose_t>::TRelativePosesForEachTarget::const_iterator it_tree4_central = rba_state.spanning_tree.num.find(nei_edge.from);
			ASSERT_(it_tree4_central!=rba_state.spanning_tree.num.end())

			typename kf2kf_pose_traits<typename traits_t::original_kf2kf_pose_t>::frameid2pose_map_t::const_iterator it_nei_1 = 
				it_tree4_central->second.find(new_kf_id-1);
			if (it_nei_1!=it_tree4_central->second.end())
			{
				// Found: reuse this relative pose as a good initial guess for the estimation
				const bool edge_dir_to_newkf  = (nei_edge.to==new_kf_id);
				if (edge_dir_to_newkf)
				     nei_edge.inv_pose = - it_nei_1->second.pose; // Note the "-" inverse operator, it is important
				else nei_edge.inv_pose =   it_nei_1->second.pose;
				nei.has_approx_init_val = true;
			}
		}

		// Method #2: use relative pose from sensor model implementation, if provided:
		if (!nei.has_approx_init_val)
		{
			// Landmarks in this new KF are in `obs`: const typename traits_t::new_kf_observations_t   & obs
			// Landmarks in the old reference KF are in: `other_k2f_edges`
			const bool edge_dir_to_newkf  = (nei_edge.to==new_kf_id);
			const TKeyFrameID other_kf_id = (nei_edge.to==new_kf_id) ? nei_edge.from : nei_edge.to;
			const std::deque<k2f_edge_t*>  & other_k2f_edges   = rba_state.keyframes[other_kf_id].adjacent_k2f_edges;

			// Make two lists of equal length with corresponding observations 
			// (i.e. new_kf_obs[i] and old_kf_obs[i] correspond to observations of the same landmark from different poses)
			typename mrpt::aligned_containers<typename obs_t::obs_data_t>::vector_t  new_kf_obs, old_kf_obs;

			{
				new_kf_obs.reserve(obs.size());  // maximum potential size: all observed features match against one old KF
				old_kf_obs.reserve(obs.size());

				// Temporary construction: associative container with all observed LMs in this new KF:
				std::map<TLandmarkID,size_t> newkf_obs_feats;
				for (size_t i=0;i<obs.size();i++)
					newkf_obs_feats[ obs[i].obs.feat_id ] = i;

				// Search in `other_k2f_edges`:
				for (size_t i=0;i<other_k2f_edges.size();i++) 
				{
					const TLandmarkID lm_id = other_k2f_edges[i]->obs.obs.feat_id;
					std::map<TLandmarkID,size_t>::const_iterator it_id = newkf_obs_feats.find(lm_id);
					if (it_id == newkf_obs_feats.end()) 
						continue; // No matching feature
					// Yes, we have a match:
					old_kf_obs.push_back( other_k2f_edges[i]->obs.obs.obs_data );
					new_kf_obs.push_back( obs[it_id->second].obs.obs_data );
				}
			}

			// Run matcher:
			pose_t pose_new_kf_wrt_old_kf;
			const bool found_ok = srba::observations::landmark_matcher_find_relative_pose<obs_t>(new_kf_obs, old_kf_obs,pose_new_kf_wrt_old_kf);
			if (found_ok)
			{
				// Found: reuse this relative pose as a good initial guess for the estimation
				nei.has_approx_init_val = true;
				if (edge_dir_to_newkf)
				     nei_edge.inv_pose = - pose_new_kf_wrt_old_kf;
				else nei_edge.inv_pose =   pose_new_kf_wrt_old_kf;
			}
			else
			{
				// Otherwise: we cannot provide any reasonable initial value, which may degrade performance...
				VERBOSE_LEVEL(2) << "Warning: Could not provide an initial value to relative pose between KFs " <<nei_edge.from << "<=>" << nei_edge.to << "\n";
			}
		}
	} // end for each nei

	// save for the next timestep:
	rba_state.last_timestep_touched_kfs.clear();
	for (size_t i=0;i<nNEI;i++) {
		const k2k_edge_t & nei_edge = rba_state.k2k_edges[new_k2k_edge_ids[i].id];
		rba_state.last_timestep_touched_kfs.insert( nei_edge.from );
		rba_state.last_timestep_touched_kfs.insert( nei_edge.to );
	}


}

} // End of namespaces
