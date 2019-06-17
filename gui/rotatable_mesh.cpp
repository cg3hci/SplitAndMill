/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "rotatable_mesh.h"
#include "arrow.h"
#include <cg3/meshes/dcel/dcel_builder.h>

RotatableMesh::RotatableMesh(cg3::viewer::GLCanvas &canvas)  :
	canvas(canvas), init(false)
{
	setTranslationSensitivity(0);
}

RotatableMesh::RotatableMesh(cg3::viewer::GLCanvas& canvas, const cg3::DrawableDcel &mesh) :
	canvas(canvas)
{
	setMesh(mesh);
}

void RotatableMesh::setMesh(const cg3::DrawableDcel &mesh)
{
	resetRotation();
	init = true;
	this->mesh = mesh;
	grabbedArrow = -1;
	this->mesh.update();
	double sf = mesh.boundingBox().diag() / 100;
	arrowColor[0] = cg3::RED;
	arrowColor[1] = cg3::GREEN;
	arrowColor[2] = cg3::BLUE;
	for (uint i = 0; i < 3; ++i){
		arrow[i] = createArrow();
		arrow[i].setFlatShading();
		arrow[i].scale(sf);
		arrow[i].setFaceColors(arrowColor[i]);
	}
	arrow[0].rotate(cg3::Y_AXIS, -M_PI / 2);
	arrow[1].rotate(cg3::X_AXIS, M_PI / 2);
	double tf = (mesh.boundingBox().diag() / 2) * 0.66;
	arrow[0].translate(cg3::Vec3(0, tf, tf));
	arrow[1].translate(cg3::Vec3(tf, 0, tf));
	arrow[2].translate(cg3::Vec3(tf, tf, 0));
	for (uint i = 0; i < 3; ++i){
		arrow[i].update();
		bc[i] = arrow[i].barycenter();
	}
	constraints[0].setRotationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(1,0,0));
	constraints[1].setRotationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(0,1,0));
	constraints[2].setRotationConstraint(qglviewer::AxisPlaneConstraint::AXIS, qglviewer::Vec(0,0,1));
}

void RotatableMesh::draw() const
{
	if (init) {
		mesh.draw();
		for (uint i = 0; i < 3; ++i){
			arrow[i].draw();
		}
	}
}

cg3::Point3d RotatableMesh::sceneCenter() const
{
	return mesh.boundingBox().center();
}

double RotatableMesh::sceneRadius() const
{
	return mesh.boundingBox().diag();
}

void RotatableMesh::drawHighlighted() const
{
	if (init) {
		mesh.draw();
		for (uint i = 0; i < 3; ++i){
			arrow[i].draw();
		}
	}
}

void RotatableMesh::checkIfGrabsMouse(int x, int y, const qglviewer::Camera * const camera)
{
	if (init) {
		//set center and radius
		qglviewer::Vec center[3];
		for (uint i = 0; i < 3; ++i){
			center[i] = qglviewer::Vec(bc[i].x(), bc[i].y(), bc[i].z());
		}
		double radius = arrow[0].boundingBox().diag();// = sceneRadius();
		double gf = 0.1;

		bool grabs = false;
		for (uint i = 0; i < 3; i++){
			const qglviewer::Quaternion qt = rotation();
			center[i] = qt.rotate(center[i]);
			const qglviewer::Vec extreme(center[i].x + radius, center[i].y + radius, center[i].z + radius);
			const qglviewer::Vec pos(position().x(), position().y(), position().z());
			const qglviewer::Vec proj = camera->projectedCoordinatesOf(pos + center[i]);
			const qglviewer::Vec projex = camera->projectedCoordinatesOf(pos + extreme);
			const int threshold = std::sqrt ( std::pow((proj.x - projex.x), 2) +
										std::pow((proj.y - projex.y), 2) ) * gf;
			bool over = (fabs(x - proj.x) < threshold) && (fabs(y - proj.y) < threshold);
			grabs = keepsGrabbingMouse() || over || grabs;
			if (over && !keepsGrabbingMouse()) {
				if (grabbedArrow != (int)i){
					if (grabbedArrow != -1) {
						arrow[grabbedArrow].setFaceColors(arrowColor[grabbedArrow]);
						arrow[grabbedArrow].update();
					}
					arrow[i].setFaceColors(cg3::YELLOW);
					arrow[i].update();
					this->setConstraint(&constraints[i]);
				}
				grabbedArrow = i;
			}
		}
		if (!grabs){
			if (grabbedArrow != -1){
				arrow[grabbedArrow].setFaceColors(arrowColor[grabbedArrow]);
				arrow[grabbedArrow].update();
				this->setConstraint(nullptr);
			}
			grabbedArrow = -1;
		}
		setGrabsMouse(grabs);
	}
}
