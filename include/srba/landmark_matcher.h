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
namespace observations {
	/** Landmark matcher overloaded function.
	 * Used to provide a first initial guess for the relative pose in loop closures.
	 * \tparam OBS_T can be any of srba::observations
	 * See specializations in models/observations.h
	 */
	template <class OBS_T>
	struct landmark_matcher;
}
}
