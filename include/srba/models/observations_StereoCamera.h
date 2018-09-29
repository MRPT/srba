/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

/** \defgroup mrpt_srba_observations Observation types
	* \ingroup mrpt_srba_grp */

#include "observations_MonocularCamera.h"


#include <mrpt/img/TCamera.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/tfest.h>  // for use in landmark_matcher<>

namespace srba {
namespace observations {
	/** \addtogroup mrpt_srba_observations
		* @{ */

	/** Observation = one stereo camera feature, the coordinates of two pixels (one in each left/right image). 
	  * Assumptions: rectified images, without distortion, reference image is left camera, right camera after rectification is exactly along the X axis of the left camera.
	  */
	struct StereoCamera
	{
		static const size_t  OBS_DIMS = 4; //!< Each observation is a pair of pixels (px_l,py_l,px_r,py_r) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			mrpt::img::TPixelCoordf  left_px, right_px;

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = left_px.x;  obs[1] = left_px.y;
				obs[2] = right_px.x; obs[3] = right_px.y;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			mrpt::img::TStereoCamera camera_calib;
		};
	};

	/** Landmark matcher overloaded function. Used to provide a first initial guess for the relative pose in loop closures. \tparam POSE can be mrpt::poses::CPose2D or mrpt::poses::CPose3D */
	template <> struct landmark_matcher<StereoCamera>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_std_vector<StereoCamera::obs_data_t> & new_kf_obs,
			const mrpt::aligned_std_vector<StereoCamera::obs_data_t> & old_kf_obs,
			const StereoCamera::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf
			)
		{
			ASSERT_(new_kf_obs.size()==old_kf_obs.size());
			const size_t N=new_kf_obs.size();
			// project stereo points to 3D and use them to find out the relative pose:
			const double cx = params.camera_calib.leftCamera.cx(), cy = params.camera_calib.leftCamera.cy(), baseline = params.camera_calib.rightCameraPose.x, f = params.camera_calib.leftCamera.fx();
			mrpt::tfest::TMatchingPairList matches;
			matches.reserve(N);
			for (size_t i=0;i<N;i++)
			{
				// Point 1:
				const double disparity_old = old_kf_obs[i].left_px.x - old_kf_obs[i].right_px.x;
				if (disparity_old<=.0) continue; // Invalid 3D point
				const mrpt::math::TPoint3D pt_old(
					( old_kf_obs[i].left_px.x - cx )*baseline/disparity_old,
					( old_kf_obs[i].left_px.y - cy )*baseline/disparity_old,
					f*baseline/disparity_old );
				// Point 2:
				const double disparity_new = new_kf_obs[i].left_px.x - new_kf_obs[i].right_px.x;
				if (disparity_new<=.0) continue; // Invalid 3D point
				const mrpt::math::TPoint3D pt_new(
					( new_kf_obs[i].left_px.x - cx )*baseline/disparity_new,
					( new_kf_obs[i].left_px.y - cy )*baseline/disparity_new,
					f*baseline/disparity_new );

				matches.emplace_back(i,i, pt_old.x,pt_old.y,pt_old.z, pt_new.x,pt_new.y,pt_new.z );
			}
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
