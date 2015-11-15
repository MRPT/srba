/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/TCamera.h>
#include <mrpt/tfest.h>  // for use in landmark_matcher<>

namespace srba {
namespace observations {
	/** \addtogroup mrpt_srba_observations
		* @{ */

	/** Observation = one monocular camera feature (the coordinates of one pixel) */
	struct MonocularCamera
	{
		static const size_t  OBS_DIMS = 2; //!< Each observation is one pixel (px,py) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::utils::TPixelCoordf  px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = px.x;
				obs[1] = px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::utils::TCamera  camera_calib;
		};
	};

	/** Landmark matcher overloaded function. Used to provide a first initial guess for the relative pose in loop closures. \tparam POSE can be mrpt::poses::CPose2D or mrpt::poses::CPose3D */
	template <> struct landmark_matcher<MonocularCamera>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_containers<MonocularCamera::obs_data_t>::vector_t & new_kf_obs,
			const mrpt::aligned_containers<MonocularCamera::obs_data_t>::vector_t & old_kf_obs,
			const MonocularCamera::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf)
		{
			return false; // We cannot find any absolute relative pose only from 2 sets of monocular features, right?
		}
	};

	/** @} */
}
} // end NS
