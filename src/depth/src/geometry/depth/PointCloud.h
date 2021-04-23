#pragma once

#include <pleno/types.h>
#include <pleno/io/archive.h>

//******************************************************************************
class PointCloud 
{	
private:
	P3DS 		features_;
	P2DS		pixels_;
	Colors	 	colors_; 

//******************************************************************************	
public:
	PointCloud(std::size_t n = 0.) { reserve(n); }
	PointCloud(const PointCloud& o) : features_{o.features_}, pixels_{o.pixels_}, colors_{o.colors_} {}
	PointCloud(PointCloud&& o) : features_{std::move(o.features_)}, pixels_{std::move(o.pixels_)}, colors_{std::move(o.colors_)} {}
	
//accessors
	P3DS& features() { return features_; }
	const P3DS& features() const { return features_; }
	
	P3D& feature(std::size_t i) { assert(i < features().size()); return features()[i]; }
	const P3D& feature(std::size_t i) const { assert(i < features().size()); return features()[i]; }

	P2DS& pixels() { return pixels_; }
	const P2DS& pixels() const { return pixels_; }

	P2D& pixel(std::size_t i) { assert(i < pixels().size()); return pixels()[i]; }
	const P2D& pixel(std::size_t i) const { assert(i < pixels().size()); return pixels()[i]; }
	
	Colors& colors() { return colors_; }
	const Colors& colors() const { return colors_; }
	
	RGBA& color(std::size_t i) { assert(i < colors().size()); return colors()[i]; }
	const RGBA& color(std::size_t i) const { assert(i < colors().size()); return colors()[i]; }
	
	std::size_t size() const { return features().size(); }
	std::size_t nbPoints() const { return size(); }
	std::size_t capacity() const { return features().capacity(); }

//modifiers	
	std::size_t shrink() { features().shrink_to_fit(); pixels().shrink_to_fit(); colors().shrink_to_fit(); return size(); }
	std::size_t reserve(std::size_t size) { features().reserve(size); pixels().reserve(size); colors().reserve(size); return capacity(); }

//add
	void add(const P3D& data, const P2D& pix = P2D{-1.,-1.}, const RGBA& col = RGBA{255.,255.,255.,255.}) 
	{
		features().emplace_back(data);
		pixels().emplace_back(pix);
		colors().emplace_back(col);
	} 

protected:
	//******************************************************************************
	friend void save(v::OutputArchive& archive, const PointCloud& pc);
	friend void load(v::InputArchive& archive, PointCloud& pc);	
};

//******************************************************************************
//******************************************************************************
inline void save(v::OutputArchive& archive, const PointCloud& pc)
{
	archive
		("features", pc.features_)
		("pixels", pc.pixels_)
		("colors", pc.colors_);
}

inline void load(v::InputArchive& archive, PointCloud& pc)
{
	archive
		("features", pc.features_)
		("pixels", pc.pixels_)
		("colors", pc.colors_);
}

//******************************************************************************
//******************************************************************************
