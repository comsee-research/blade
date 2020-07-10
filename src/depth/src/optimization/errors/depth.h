#pragma once

#include <libv/lma/lma.hpp>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera
#include <pleno/geometry/mia.h> //MicroImage

struct BlurAwareDisparityCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //SAD
	
	const Image img_i; 
	const Image img_j;
	
	const MicroImage mi_i;
	const MicroImage mi_j;
	
	const PlenopticCamera& mfpc;
	
    BlurAwareDisparityCostError(
    	const Image& img_i_, const Image& img_j_, 
    	const MicroImage& mi_i_, const MicroImage& mi_j_, 
    	const PlenopticCamera& mfpc_
    );
    BlurAwareDisparityCostError(const BlurAwareDisparityCostError& o);
    BlurAwareDisparityCostError(BlurAwareDisparityCostError&& o);
    
    bool operator()( 
    	const Depth& depth,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<BlurAwareDisparityCostError> 
	{ 
		static std::string name(){ return "BlurAwareDisparityCostError"; } 
	};
} // namespace ttt
