/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef HF_ENGINE_THREAD_H
#define HF_ENGINE_THREAD_H

#include <QObject>
#include <QThread>
#include "data_structures/hf_engine.h"


class HFEngineThread  : public QObject, public HFEngine
{
	Q_OBJECT
	QThread workerThread;
public:
	HFEngineThread();
	HFEngineThread(const HFEngine& he);

public slots:
	void taubinSmoothing(uint nIt, double lambda, double mu);
	Eigen::Matrix3d optimalOrientation(uint nDirs);
	void cut(cg3::Dcel mesh, HFBox hfBox);
	void restoreHighFrequencies(uint nIterations, double flipAngle);
	void computeDecomposition();
	void computeDecompositionExact();
	void computePackingFromDecomposition(
			const cg3::BoundingBox3 &stock,
			double toolLength,
			const cg3::Point2d& frameThicknessStock,
			double zOffset,
			double distanceBetweenBlocks = 5,
			cg3::Point2d clearnessStock = cg3::Point2d(5, 2),
			double clearnessTool = 1, double factor = 1, bool computeNegative = false);
	bool computeOneStockPackingFromDecomposition(
			const cg3::BoundingBox3 &stock,
			double toolLength,
			cg3::Point2d frameThicknessStock,
			double zOffset,
			double distanceBetweenBlocks = 5,
			cg3::Point2d clearnessStock = cg3::Point2d(5, 2),
			double clearnessTool = 1, bool computeNegative = false);

signals:
	void setProgressBarValue(uint);
	void taubinSmoothingCompleted();
	void optimalOrientationCompleted(Eigen::Matrix3d);
	void cutCompleted(cg3::Dcel);
	void restoreHighFrequenciesCompleted();
	void computeDecompositionCompleted();
	void computePackingFromDecompositionCompleted(bool);
};

#endif // HF_ENGINE_THREAD_H
