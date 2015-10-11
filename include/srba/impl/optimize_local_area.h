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

template <class RBA_SETTINGS_T>
void RbaEngine<RBA_SETTINGS_T>::optimize_local_area(
	const TKeyFrameID  root_id,
	const unsigned int win_size,
	TOptimizeExtraOutputInfo & out_info,
	const TOptimizeLocalAreaParams &params,
	const std::vector<size_t> & observation_indices_to_optimize
	)
{
	m_profiler.enter("optimize_local_area");

	// Use prebuilt spanning trees if possible (should be always!)
	const bool use_prebuilt_st = (win_size<= parameters.srba.max_tree_depth);
	if (!use_prebuilt_st)
	{
		VERBOSE_LEVEL(1) << "[optimize_local_area] *WARNING* Optimize win_size > max_tree_depth of prebuilt spanning trees. This is not efficient!\n";
	}

	// 1st) Find list of edges to optimize:
	// --------------------------------------------------
	m_profiler.enter("optimize_local_area.find_edges2opt");

	VisitorOptimizeLocalArea my_visitor(this->rba_state,params);

	this->bfs_visitor(
		root_id,  // Starting keyframe
		win_size, // max. depth
		use_prebuilt_st, // Use prebuilt spanning trees for speed-up
		my_visitor, //kf_visitor,
		my_visitor, //feat_visitor,
		my_visitor, //k2k_edge_visitor,
		my_visitor  //k2f_edge_visitor
		);

	m_profiler.leave("optimize_local_area.find_edges2opt");

	// 2nd) Optimize them:
	// -------------------------------
	if (!my_visitor.k2k_edges_to_optimize.empty() || !my_visitor.lm_IDs_to_optimize.empty())
	{
		this->optimize_edges(my_visitor.k2k_edges_to_optimize,my_visitor.lm_IDs_to_optimize, out_info, observation_indices_to_optimize);
	}

	m_profiler.leave("optimize_local_area");
}



} // end NS
