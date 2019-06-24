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

	void restoreHighFrequencies(HFEngine* hfEngine, uint nIt, double flipAngle);

	void computeDecomposition(HFEngine* hfEngine);

signals:
	void setProgressBarValue(uint);
	void taubinSmoothingCompleted(cg3::Dcel);
	void optimalOrientationCompleted(Eigen::Matrix3d);
	void restoreHighFrequenciesCompleted();
	void computeDecompositionCompleted(std::vector<cg3::Dcel>);


};

#endif // THREADCOMPUTATIONS_H
