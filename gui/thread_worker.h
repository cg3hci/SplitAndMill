/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef THREADCOMPUTATIONS_H
#define THREADCOMPUTATIONS_H

#include <QObject>
#include <QThread>

#include <cg3/meshes/dcel/dcel.h>
#include "data_structures/hf_engine.h"

class ThreadWorker  : public QObject
{
    Q_OBJECT
    QThread workerThread;
public:
    ThreadWorker();

public slots:

	void taubinSmoothing(cg3::Dcel mesh, uint nIt, double lambda, double mu);

	void optimalOrientation(cg3::Dcel mesh, uint nDirs);

	void cut(cg3::Dcel mesh, HFBox hfbox);

	void restoreHighFrequencies(HFEngine* hfEngine, uint nIt, double flipAngle);

	void computeDecomposition(HFEngine* hfEngine);

	void computeDecompositionExact(HFEngine* hfEngine);

	void packInOneStock(std::vector<cg3::Dcel> blocks, cg3::BoundingBox3 stock, double distanceBetweenBlocks);

signals:
	void setProgressBarValue(uint);
	void taubinSmoothingCompleted(cg3::Dcel);
	void optimalOrientationCompleted(Eigen::Matrix3d);
	void cutCompleted(cg3::Dcel);
	void restoreHighFrequenciesCompleted();
	void computeDecompositionCompleted(std::vector<cg3::Dcel>);
	void packInOneStockCompleted(bool, double, std::vector<std::pair<int, cg3::Point3d>>);


};

#endif // THREADCOMPUTATIONS_H
