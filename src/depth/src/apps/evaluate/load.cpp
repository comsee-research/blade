#include "load.h"

#include "processing/depth/initialization.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
XYZs read_xyz(std::string path)
{
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);
	
    int count;
    ifs >> count;
    ifs.ignore(1, '\n');
    ifs.ignore(1, '\n');
    DEBUG_VAR(count);
    
    XYZs pts; pts.reserve(count);
    
    for(int i = 0; i < count; i++)
    {
        std::string line;
        if(!std::getline(ifs, line)) {PRINT_ERR("("<<i << ") no line"); break; }
        
        //DEBUG_VAR(line);
        std::istringstream iss(line);
		
		xyz p;

        iss >> p.x >> p.y >> p.z;
        
        //DEBUG_VAR(p.x);DEBUG_VAR(p.y);DEBUG_VAR(p.z);
        pts.emplace_back(p);    
        iss.clear(); 
    }	
    
    ifs.close();
	return pts;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::map<Index, XYZs> load(const XYZsConfig& config)
{
	std::map<Index, XYZs> maps;
	for (auto & xyz_cfg : config.xyzs())
	{
		maps.emplace(xyz_cfg.frame(), read_xyz(xyz_cfg.path()));	
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, RawCoarseDepthMap> load(const DepthMapsConfig& config, const PlenopticCamera& mfpc)
{
	std::map<Index, RawCoarseDepthMap> maps;
	
	const auto [mind, maxd] = initialize_min_max_distance(mfpc);
	
	for (auto & dm_cfg : config.maps())
	{
		RawCoarseDepthMap dm{mfpc, mfpc.obj2v(maxd), mfpc.obj2v(mind)};
		v::load(dm_cfg.path(), v::make_serializable(&dm));
		
		maps.emplace(dm_cfg.frame(), std::move(dm));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, RawCoarseDepthMap> load_from_csv(std::string path, const PlenopticCamera& mfpc)
{
	std::map<Index, RawCoarseDepthMap> maps;
	
	using KLD = std::tuple<Index, Index, double>;
	std::map<Index, std::vector<KLD>> depths;
	
	std::ifstream ifs(path);
	PRINT_DEBUG("Open file = " << path);

    
    const int nbvalues = mfpc.mia().width() * mfpc.mia().height();
    
    std::string line;
    char comma;
    
    std::getline(ifs, line, '\n');//ignore header
    
    while (std::getline(ifs, line, '\n'))
    {
     	std::istringstream iss(line);

		Index frame, k, l;
		double d;
		
        iss >> frame >> comma >> k >> comma >> l >> comma >> d;
        
        depths[frame].reserve(nbvalues);
        depths[frame].emplace_back(k,l,d);    
        
        //iss.clear(); 
    }
	ifs.close();

	const auto [mind, maxd] = initialize_min_max_distance(mfpc);
	
	for (auto & [frame, klds] : depths)
	{
		RawCoarseDepthMap dm{mfpc, mind, maxd, false}; //metric depth map
		
		for (const auto& [k, l, d] : klds)
		{
			dm.depth(k,l) = d;
			dm.state(k,l) = DepthInfo::State::COMPUTED;
		}

		maps.emplace(frame, std::move(dm));
	}
	
	return maps;
}

//******************************************************************************
std::map<Index, Pose> load(const CalibrationPosesConfig& config)
{
	std::map<Index, Pose> maps;
	for (auto & pose_cfg : config.poses())
	{
		maps.emplace(pose_cfg.frame(), pose_cfg.pose());	
	}
	return maps;
}

//******************************************************************************
std::map<Index, double> load_gt_dist(std::string path)
{
	std::ifstream ifs(path, std::ios::in);
	PRINT_DEBUG("Open file = " << path);
             				
	std::map<Index, double> maps;
	
    std::string line;
	while (std::getline(ifs, line))
	{
		double d = 0.; int frame = 0;
        std::istringstream iss(line);
        
        iss >> frame >> d;
        
        maps[frame] = d;
	}
	
	ifs.close();
	return maps;
}
