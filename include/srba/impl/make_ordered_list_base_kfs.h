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
namespace internal {

template <class traits_t,class rba_problem_state_t>
void make_ordered_list_base_kfs(
	const typename traits_t::new_kf_observations_t & obs,
	const rba_problem_state_t & rba_state,
	base_sorted_lst_t            & obs_for_each_base_sorted,
	std::map<TKeyFrameID,size_t>       *out_obs_for_each_base = NULL )
{
	using namespace std;
	// Make a first pass to make a sorted list of base KFs, ordered by # of observations so we prefer edges to
	// strongly connected base KFs:
	map<TKeyFrameID,size_t> obs_for_each_base;

	for (typename traits_t::new_kf_observations_t::const_iterator itObs=obs.begin();itObs!=obs.end();++itObs)
	{
		const TLandmarkID lm_id = itObs->obs.feat_id;
		if (lm_id>=rba_state.all_lms.size()) continue; // It's a new LM

		const typename landmark_traits<typename traits_t::original_landmark_t>::TLandmarkEntry &lme = rba_state.all_lms[lm_id];
		if (!lme.rfp) continue; // It's a new LM.

		const TKeyFrameID base_id = lme.rfp->id_frame_base;
		obs_for_each_base[base_id]++; // vote for this.
	}

	// Sort map<> by values:
	for (map<TKeyFrameID,size_t>::const_iterator it=obs_for_each_base.begin();it!=obs_for_each_base.end();++it)
		obs_for_each_base_sorted.insert( make_pair(it->second,it->first) );

	if (out_obs_for_each_base)
		out_obs_for_each_base->swap(obs_for_each_base);
}

} } // End of namespaces
