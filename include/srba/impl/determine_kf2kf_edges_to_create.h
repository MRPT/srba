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
template <class RBA_SETTINGS_T>
void RbaEngine<RBA_SETTINGS_T>::determine_kf2kf_edges_to_create(
	const TKeyFrameID               new_kf_id,
	const typename traits_t::new_kf_observations_t   & obs,
	std::vector<TNewEdgeInfo> &new_k2k_edge_ids )
{
	new_k2k_edge_ids.clear();

	if (rba_state.keyframes.size()==1)
	{
		// If this is the first KF, there's no other one to which we can connect! -> Return an empty set of edges.
		return;
	}

	edge_creation_policy.eval<traits_t,rba_engine_t>(new_kf_id,obs,new_k2k_edge_ids, *this,parameters.ecp);
}

} // End of namespaces
