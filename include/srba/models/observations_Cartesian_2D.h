/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/tfest.h>  // for use in landmark_matcher<>

namespace srba {
namespace observations {
	/** \addtogroup mrpt_srba_observations
		* @{ */

	/** Observation = XY coordinates of landmarks relative to the sensor */
	struct Cartesian_2D
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is a pair of coordinates (x,y) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::math::TPoint2D  pt;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = pt.x; obs[1] = pt.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	/** Landmark matcher overloaded function. Used to provide a first initial guess for the relative pose in loop closures. \tparam POSE can be mrpt::poses::CPose2D or mrpt::poses::CPose3D */
	template <> struct landmark_matcher<Cartesian_2D>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_std_vector<Cartesian_2D::obs_data_t> & new_kf_obs,
			const mrpt::aligned_std_vector<Cartesian_2D::obs_data_t> & old_kf_obs,
			const Cartesian_2D::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf)
		{
			ASSERT_(new_kf_obs.size()==old_kf_obs.size());
			const size_t N=new_kf_obs.size();
			mrpt::tfest::TMatchingPairList matches;
			matches.reserve(N);
			for (size_t i=0;i<N;i++)
				matches.push_back( mrpt::tfest::TMatchingPair(i,i, old_kf_obs[i].pt.x,old_kf_obs[i].pt.y,0,  new_kf_obs[i].pt.x,new_kf_obs[i].pt.y,0 ) );
			// Least-square optimal transformation:
			if (POSE::rotation_dimensions==2)
			{ // SE(2)
				mrpt::math::TPose2D found_pose;
				if (!mrpt::tfest::se2_l2(matches,found_pose))
					return false;
				pose_new_kf_wrt_old_kf = POSE( mrpt::poses::CPose2D(found_pose));
			}
			else
			{  // SE(3)
				mrpt::poses::CPose3DQuat found_pose;
				double found_scale;
				if (!mrpt::tfest::se3_l2(matches,found_pose,found_scale))
					return false;
				pose_new_kf_wrt_old_kf = POSE(mrpt::poses::CPose3D(found_pose));
			}
			return true;
		}
	};

	/** @} */
}
} // end NS
