/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HF_ENGINE_H
#define HF_ENGINE_H

#include <cg3/meshes/dcel/dcel.h>
#include "hf_box.h"

class HFEngine
{
public:
	HFEngine();
	void clear();
	void setMesh(const cg3::Dcel& _mesh);
	void setOriginalMesh(const cg3::Dcel& _mesh);
	void setUseSmoothedMesh(bool b);
	void pushBox(const HFBox& box);
	void popBox();
	const std::vector<HFBox>& boxes() const;
	std::vector<cg3::Vec3d> restoreHighFrequenciesDirs() const;
	void restoreHighFrequencies(uint nIterations, double flipAngle);
	double hausdorffDistance() const;
	std::vector<cg3::Dcel> decomposition() const;
	cg3::Dcel mesh() const;
	cg3::Dcel originalMesh() const;

private:
	cg3::Dcel _mesh;
	cg3::Dcel _originalMesh;
	bool useSmoothedMesh;
	std::vector<HFBox> _boxes;
};

#endif // HF_ENGINE_H
