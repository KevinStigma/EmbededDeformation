#ifndef EMBEDEDDEFORMATION3D_H
#define EMBEDEDDEFORMATION3D_H

#include <QtWidgets/QMainWindow>
#include <QKeyEvent>
#include "ui_embededdeformation3d.h"

class EmbededDeformation3D : public QMainWindow
{
	Q_OBJECT

public:
	EmbededDeformation3D(QWidget *parent = 0);
	~EmbededDeformation3D();

public slots:
	void importMesh();
	void MeshShowCheck();
	void NormalShowCheck();
	void TriangleShowCheck();
	void GraphShowCheck();
	void DeformNodeShowCheck();
	void BoxShowCheck();
	void plusId();
	void minusId();
	void weightDefine();
	void importMeshBatch();
	void importGraph();
	void exportGraph();
	void inputBoxes();
	void outputBoxes();
	void none_shader();
	void orenyar_shader();
	void glass_shader();
	void xray_shader();
protected:
	void keyPressEvent(QKeyEvent *e);
	void keyReleaseEvent(QKeyEvent *e);
private:
	Ui::EmbededDeformation3DClass ui;
};

#endif // EMBEDEDDEFORMATION3D_H
