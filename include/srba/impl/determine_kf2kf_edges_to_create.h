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
		
		// New edges are: FROM (old KF) ==> TO (new KF)
		// In loop closures, neither "nei.to" nor "nei.from" are the latest KF, both may be existing center KFs:
		k2k_edge_t & nei_edge = rba_state.k2k_edges[nei.id];
		const bool nei_edge_does_not_touch_cur_kf = (nei_edge.to!=new_kf_id) && (nei_edge.from!=new_kf_id);
		
		// Method #1: look at last kf's kf2kf edges for an initial guess to ease optimization:
		if ( !nei_edge_does_not_touch_cur_kf && rba_state.last_timestep_touched_kfs.count(nei_edge.from) != 0 )
		{
			const pose_t *rel_pose = get_kf_relative_pose(new_kf_id-1, nei_edge.from);
			if (rel_pose)
			{
				// Found: reuse this relative pose as a good initial guess for the estimation
				const bool edge_dir_to_newkf  = (nei_edge.to==new_kf_id);
				if (edge_dir_to_newkf)
				     nei_edge.inv_pose = - (*rel_pose); // Note the "-" inverse operator, it is important
				else nei_edge.inv_pose =    *rel_pose;
				nei.has_approx_init_val = true;
			}
		}

		// Method #2: use relative pose from sensor model implementation, if provided:
		// (1st attempt) Direct relative pose between the two KFs at each end of the new edge.
		if (!nei.has_approx_init_val)
		{
			// Landmarks in this new KF are in `obs`: const typename traits_t::new_kf_observations_t   & obs
			// Landmarks in the old reference KF are in: `other_k2f_edges`
			TKeyFrameID last_kf_id, other_kf_id;  // Required to tell if we have to take observations from "k2f_edges" or from the latest "obs":
			if (nei_edge_does_not_touch_cur_kf)
			{  // Arbitrarily pick "last" and "other" as "from" and "to":
				last_kf_id   = nei_edge.from;
				other_kf_id  = nei_edge.to;
			}
			else
			{  // Pick the latest and the "other" kf
				last_kf_id  = new_kf_id;
				other_kf_id = (nei_edge.to==new_kf_id) ? nei_edge.from : nei_edge.to;
			}

			const std::deque<k2f_edge_t*>  & other_k2f_edges   = rba_state.keyframes[other_kf_id].adjacent_k2f_edges;

			// Make two lists of equal length with corresponding observations 
			// (i.e. new_kf_obs[i] and old_kf_obs[i] correspond to observations of the same landmark from different poses)
			typename mrpt::aligned_containers<typename obs_t::obs_data_t>::vector_t  new_kf_obs, old_kf_obs;

			{
				new_kf_obs.reserve(obs.size());  // maximum potential size: all observed features match against one old KF
				old_kf_obs.reserve(obs.size());

				// Temporary construction: associative container with all observed LMs in this new KF:
				std::map<TLandmarkID,size_t> newkf_obs_feats;
				const std::deque<k2f_edge_t*>  * last_k2f_edges   = NULL;
				if (nei_edge_does_not_touch_cur_kf)
				{
					last_k2f_edges   = &rba_state.keyframes[last_kf_id].adjacent_k2f_edges;
					for (size_t i=0;i<last_k2f_edges->size();i++) {
						const TLandmarkID lm_id = (*last_k2f_edges)[i]->obs.obs.feat_id;
						newkf_obs_feats[ lm_id ] = i;
					}
				}
				else {
					for (size_t i=0;i<obs.size();i++)
						newkf_obs_feats[ obs[i].obs.feat_id ] = i;
				}

				// Search in `other_k2f_edges`:
				for (size_t i=0;i<other_k2f_edges.size();i++) 
				{
					const TLandmarkID lm_id = other_k2f_edges[i]->obs.obs.feat_id;
					std::map<TLandmarkID,size_t>::const_iterator it_id = newkf_obs_feats.find(lm_id);
					if (it_id == newkf_obs_feats.end()) 
						continue; // No matching feature
					// Yes, we have a match:
					old_kf_obs.push_back( other_k2f_edges[i]->obs.obs.obs_data );
					if (nei_edge_does_not_touch_cur_kf)
					     new_kf_obs.push_back( (*last_k2f_edges)[it_id->second]->obs.obs.obs_data );
					else new_kf_obs.push_back( obs[it_id->second].obs.obs_data );
				}
			}


			// (1st attempt) Run matcher:
			pose_t pose_new_kf_wrt_old_kf;
			m_profiler.enter("define_new_keyframe.determine_edges.lm_matcher");
			bool found_ok = srba::observations::landmark_matcher<obs_t>::find_relative_pose(new_kf_obs, old_kf_obs, parameters.sensor,pose_new_kf_wrt_old_kf);
			m_profiler.leave("define_new_keyframe.determine_edges.lm_matcher");

			// (2nd attempt) Run matcher between another set of KFs, only possible in the case of a loop closure:
			if (!found_ok && nei.loopclosure_observer_kf!=SRBA_INVALID_KEYFRAMEID && nei.loopclosure_base_kf!=SRBA_INVALID_KEYFRAMEID)
			{
				// We may have up to 4 KFs involved here: 
				// - nei.loopclosure_observer_kf
				// - nei.loopclosure_base_kf
				// - nei_edge.to
				// - nei_edge.from
				// Any of the latter 2 *might* coincide with the former 2 KFs.
				// We already tried to find a match between "to" and "from" without luck, 
				// so let's try now with the former pair.
				last_kf_id  = nei.loopclosure_observer_kf;
				other_kf_id = nei.loopclosure_base_kf;

				const std::deque<k2f_edge_t*>  & other2_k2f_edges   = rba_state.keyframes[other_kf_id].adjacent_k2f_edges;
				const bool last_kf_is_new_kf = (last_kf_id==new_kf_id);

				// Temporary construction: associative container with all observed LMs in this new KF:
				new_kf_obs.clear();
				old_kf_obs.clear();
				std::map<TLandmarkID,size_t> newkf_obs_feats;
				const std::deque<k2f_edge_t*>  * last_k2f_edges   = NULL;
				if (!last_kf_is_new_kf)
				{
					last_k2f_edges   = &rba_state.keyframes[last_kf_id].adjacent_k2f_edges;
					for (size_t i=0;i<last_k2f_edges->size();i++) {
						const TLandmarkID lm_id = (*last_k2f_edges)[i]->obs.obs.feat_id;
						newkf_obs_feats[ lm_id ] = i;
					}
				}
				else {
					for (size_t i=0;i<obs.size();i++)
						newkf_obs_feats[ obs[i].obs.feat_id ] = i;
				}

				// Search in `other2_k2f_edges`:
				for (size_t i=0;i<other2_k2f_edges.size();i++) 
				{
					const TLandmarkID lm_id = other2_k2f_edges[i]->obs.obs.feat_id;
					std::map<TLandmarkID,size_t>::const_iterator it_id = newkf_obs_feats.find(lm_id);
					if (it_id == newkf_obs_feats.end()) 
						continue; // No matching feature
					// Yes, we have a match:
					old_kf_obs.push_back( other2_k2f_edges[i]->obs.obs.obs_data );
					if (!last_kf_is_new_kf)
					     new_kf_obs.push_back( (*last_k2f_edges)[it_id->second]->obs.obs.obs_data );
					else new_kf_obs.push_back( obs[it_id->second].obs.obs_data );
				}

				// Run matcher:
				m_profiler.enter("define_new_keyframe.determine_edges.lm_matcher");
				found_ok = srba::observations::landmark_matcher<obs_t>::find_relative_pose(new_kf_obs, old_kf_obs, parameters.sensor,pose_new_kf_wrt_old_kf);
				m_profiler.leave("define_new_keyframe.determine_edges.lm_matcher");
			} // end 2nd attempt

			if (found_ok)
			{
				// Take into account the sensor pose wrt the KF: Rotate/translate if the sensor is not at the robot origin of coordinates: 
				mrpt::poses::CPose3D sensor_pose;
				RBA_OPTIONS::sensor_pose_on_robot_t::template robot2sensor<mrpt::poses::CPose3D>(mrpt::poses::CPose3D(), sensor_pose, this->parameters.sensor_pose);
				pose_new_kf_wrt_old_kf = pose_t( (sensor_pose + pose_new_kf_wrt_old_kf)+ (-sensor_pose) );

				const bool edge_dir_to_newkf  = (nei_edge.to==new_kf_id);
				nei.has_approx_init_val = true;

				// Found: reuse this relative pose as a good initial guess for the estimation
				if (!nei_edge_does_not_touch_cur_kf)
				{ // Found relative pose is directly for the two KFs at each end of the new kf2kf edge:
					if (edge_dir_to_newkf)
						 nei_edge.inv_pose = - pose_new_kf_wrt_old_kf;
					else nei_edge.inv_pose =   pose_new_kf_wrt_old_kf;
				}
				else
				{	// Found relative pose is for the "2nd attempt" approach in loop closures, so 
					// we must now transform "pose_new_kf_wrt_old_kf" into "nei_edge.inv_pose":
					//
					// loopclosure_observer_kf  <============   loopclosure_base_kf
					//       ^              pose_new_kf_wrt_old_kf           ^
					//       |                                               |
					//       | pose_observer_wrt_local                       | pose_base_wrt_remote
					//       |                                               |
					//       |                 nei_edge.inv_pose             |
					//       +--- TO or FROM  <======?======>  FROM or TO ---+
					//           local_kf_id                  remote_kf_id
					//
					ASSERT_(nei.loopclosure_observer_kf!=SRBA_INVALID_KEYFRAMEID && nei.loopclosure_base_kf!=SRBA_INVALID_KEYFRAMEID);

					const pose_t default_identity_pose;

					const pose_t *pose_observer_wrt_to   = (nei.loopclosure_observer_kf==nei_edge.to)   ? &default_identity_pose : get_kf_relative_pose(nei.loopclosure_observer_kf, nei_edge.to);
					const pose_t *pose_base_wrt_to       = (nei.loopclosure_base_kf==nei_edge.to)       ? &default_identity_pose : get_kf_relative_pose(nei.loopclosure_base_kf    , nei_edge.to);
					const pose_t *pose_observer_wrt_from = (nei.loopclosure_observer_kf==nei_edge.from) ? &default_identity_pose : get_kf_relative_pose(nei.loopclosure_observer_kf, nei_edge.from);
					const pose_t *pose_base_wrt_from     = (nei.loopclosure_base_kf==nei_edge.from)     ? &default_identity_pose : get_kf_relative_pose(nei.loopclosure_base_kf    , nei_edge.from);
					
					const bool observer_is_near_to = (pose_observer_wrt_to || pose_base_wrt_from) || !(pose_observer_wrt_from || pose_base_wrt_to);
					const TKeyFrameID local_kf_id  =  observer_is_near_to ? nei_edge.to   : nei_edge.from;
					const TKeyFrameID remote_kf_id =  observer_is_near_to ? nei_edge.from : nei_edge.to;

					const pose_t *pose_observer_wrt_local   = observer_is_near_to ? 
						(pose_observer_wrt_to ? pose_observer_wrt_to : &default_identity_pose)
						: 
						(pose_observer_wrt_from ? pose_observer_wrt_from : &default_identity_pose);
					const pose_t *pose_base_wrt_remote   = observer_is_near_to ? 
						(pose_base_wrt_from ? pose_base_wrt_from : &default_identity_pose)
						: 
						(pose_base_wrt_to ? pose_base_wrt_to : &default_identity_pose);
					
					// Pose transforms (from the graph of poses in the ASCII art above):
					//
					// Nwr = inv(BwR) * LwR * OwL
					// BwR * Nwr = LwR * OwL
					// BwR * Nwr * inv(OwL) = LwR
					//
					const pose_t pose_local_wrt_remote  =  ((*pose_base_wrt_remote)+(pose_new_kf_wrt_old_kf))+(-(*pose_observer_wrt_local));

					if (edge_dir_to_newkf)
						 nei_edge.inv_pose = - pose_local_wrt_remote;
					else nei_edge.inv_pose =   pose_local_wrt_remote;
				}
			}

			if (!nei.has_approx_init_val)
			{ // Otherwise: we cannot provide any reasonable initial value, which may degrade performance...
				VERBOSE_LEVEL_COLOR(2,mrpt::system::CONCOL_RED) << "[determine_kf2kf_edges_to_create] Could not provide initial value to relative pose " <<nei_edge.from << "<=>" << nei_edge.to << "\n";
				VERBOSE_LEVEL_COLOR_POST();
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
