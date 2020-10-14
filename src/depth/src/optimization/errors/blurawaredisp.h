#pragma once

#include <libv/lma/lma.hpp>

#include <pleno/types.h>
#include "../../types.h"

#include <pleno/geometry/camera/plenoptic.h> //PlenopticCamera
#include <pleno/geometry/mia.h> //MicroImage

struct BlurAwareDisparityCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //SAD
	enum BlurMethod { GAUSSIAN_BLUR = 0, APPROX_GAUSSIAN_BLUR, S_TRANSFORM };
	
	const Image img_i; 
	const Image img_j;
	
	const MicroImage mi_i;
	const MicroImage mi_j;
	
	const PlenopticCamera& mfpc;
	
	const BlurMethod method;
	
    BlurAwareDisparityCostError(
    	const Image& img_i_, const Image& img_j_, 
    	const MicroImage& mi_i_, const MicroImage& mi_j_, 
    	const PlenopticCamera& mfpc_,
    	BlurMethod meth = BlurMethod::S_TRANSFORM
    );
    BlurAwareDisparityCostError(const BlurAwareDisparityCostError& o);
    BlurAwareDisparityCostError(BlurAwareDisparityCostError&& o);
    
    bool operator()( 
    	const VirtualDepth& depth,
    	ErrorType& error
    ) const;
    
    double weight(double v) const;
};

namespace ttt
{
	template<> 
	struct Name<BlurAwareDisparityCostError> 
	{ 
		static std::string name(){ return "BlurAwareDisparityCostError"; } 
	};
} // namespace ttt
