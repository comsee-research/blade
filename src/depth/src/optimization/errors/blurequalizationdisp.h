#pragma once

#include <libv/lma/lma.hpp>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera
#include <pleno/geometry/mia.h> //MicroImage

struct BlurEqualizationDisparityCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //SAD
	
	const Image img_i; 
	const Image img_j;
	
	const MicroImage mi_i;
	const MicroImage mi_j;
	
	const PlenopticCamera& mfpc;
	
    BlurEqualizationDisparityCostError(
    	const Image& img_i_, const Image& img_j_, 
    	const MicroImage& mi_i_, const MicroImage& mi_j_, 
    	const PlenopticCamera& mfpc_
    );
    BlurEqualizationDisparityCostError(const BlurEqualizationDisparityCostError& o);
    BlurEqualizationDisparityCostError(BlurEqualizationDisparityCostError&& o);
    
    bool operator()( 
    	const VirtualDepth& depth,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<BlurEqualizationDisparityCostError> 
	{ 
		static std::string name(){ return "BlurEqualizationDisparityCostError"; } 
	};
} // namespace ttt
