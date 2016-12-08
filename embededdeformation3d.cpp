#include "embededdeformation3d.h"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
#include <QFileDialog>

EmbededDeformation3D::EmbededDeformation3D(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	connect(ui.actionImport_Mesh,SIGNAL(triggered()),this,SLOT(importMesh()));
	connect(ui.actionImport_Mesh_Batch,SIGNAL(triggered()),this,SLOT(importMeshBatch()));
	connect(ui.actionImport_Graph,SIGNAL(triggered()),this,SLOT(importGraph()));
	connect(ui.actionExport_Graph,SIGNAL(triggered()),this,SLOT(exportGraph()));
	connect(ui.actionInput_Box_Info,SIGNAL(triggered()),this,SLOT(inputBoxes()));
	connect(ui.actionOutput_Box_Info,SIGNAL(triggered()),this,SLOT(outputBoxes()));
	connect(ui.actionNone,SIGNAL(triggered()),this,SLOT(none_shader()));
	connect(ui.actionOrennayar,SIGNAL(triggered()),this,SLOT(orenyar_shader()));
	connect(ui.actionGlass,SIGNAL(triggered()),this,SLOT(glass_shader()));
	connect(ui.actionXray,SIGNAL(triggered()),this,SLOT(xray_shader()));
	connect(ui.MeshShow,SIGNAL(clicked()),this,SLOT(MeshShowCheck()));
	connect(ui.NormalShow,SIGNAL(clicked()),this,SLOT(NormalShowCheck()));
	connect(ui.TriangleShow,SIGNAL(clicked()),this,SLOT(TriangleShowCheck()));
	connect(ui.GraphNodeShow,SIGNAL(clicked()),this,SLOT(GraphShowCheck()));
	connect(ui.DeformNodeShow,SIGNAL(clicked()),this,SLOT(DeformNodeShowCheck()));
	connect(ui.BoxShow,SIGNAL(clicked()),this,SLOT(BoxShowCheck()));
	connect(ui.DefineWeight,SIGNAL(clicked()),this,SLOT(weightDefine()));
	connect(ui.plusButton,SIGNAL(clicked()),this,SLOT(plusId()));
	connect(ui.minusButton,SIGNAL(clicked()),this,SLOT(minusId()));
	
	ui.render_widget->setLineEdit(ui.RotEdit,ui.RegEdit,ui.ConEdit);
	ui.render_widget->setLineWeight();
}

EmbededDeformation3D::~EmbededDeformation3D()
{

}
void EmbededDeformation3D::none_shader()
{
	ui.render_widget->setShaderMode(0);
	
}
void EmbededDeformation3D::orenyar_shader()
{
	ui.render_widget->setShaderMode(1);
}
void EmbededDeformation3D::glass_shader()
{
	ui.render_widget->setShaderMode(2);
}
void EmbededDeformation3D::xray_shader()
{
	ui.render_widget->setShaderMode(3);
}
void EmbededDeformation3D::importMesh()
{
	QString filename=QFileDialog::getOpenFileName(this,tr("Import Mesh"),".",
		"Mesh Files(*.obj)", 0);

	if(filename.size()==0)
		return;

	//ui.render_widget->importMesh(filename.toStdString());
	ui.render_widget->importTriMesh(filename.toStdString());
}

void EmbededDeformation3D::importMeshBatch()
{
	QString filename=QFileDialog::getOpenFileName(this,tr("Import Mesh Batch"),".",
		"Mesh Script Files(*.txt)", 0);

	if(filename.size()==0)
		return;

	//ui.render_widget->importMesh(filename.toStdString());
	ui.render_widget->importTriMeshBatch(filename.toStdString());
}

void EmbededDeformation3D::importGraph()
{
	QString filename=QFileDialog::getOpenFileName(this,tr("Import Graph Info"),".",
		"Graph Info Files(*.gri)", 0);

	if(filename.isNull())
		return;
	ui.render_widget->inputGraphInfo(filename.toStdString());
}

void EmbededDeformation3D::exportGraph()
{
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Graph Info"),
		"",
		tr("Graph Info (*.gri)"));
	if(!fileName.isNull())
		ui.render_widget->outputGraphInfo(fileName.toStdString());
}

void EmbededDeformation3D::inputBoxes()
{
	QString filename=QFileDialog::getOpenFileName(this,tr("Input Boxes Info"),".",
		"Boxes Info Files(*.bif)", 0);
	if(!filename.isNull())
		ui.render_widget->inputBoxes(filename.toStdString());
}

void EmbededDeformation3D::outputBoxes()
{
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Boxes Info"),"",tr("Boxes Info (*.bif)"));
	if(!fileName.isNull())
		ui.render_widget->outputBoxes(fileName.toStdString());
}

void EmbededDeformation3D::MeshShowCheck()
{
	bool is=ui.MeshShow->isChecked();
	ui.render_widget->setMeshShow(is);
}
void EmbededDeformation3D::BoxShowCheck()
{
	bool is=ui.BoxShow->isChecked();
	ui.render_widget->setBoxShow(is);
}
void EmbededDeformation3D::NormalShowCheck()
{
	bool is=ui.NormalShow->isChecked();
	ui.render_widget->setNormalShow(is);
}
void EmbededDeformation3D::TriangleShowCheck()
{
	bool is=ui.TriangleShow->isChecked();
	ui.render_widget->setTriangleShow(is);
}

void EmbededDeformation3D::GraphShowCheck()
{
	bool is=ui.GraphNodeShow->isChecked();
	ui.render_widget->setGraphNodeShow(is);
}
void EmbededDeformation3D::DeformNodeShowCheck()
{
	bool is=ui.DeformNodeShow->isChecked();
	ui.render_widget->setDeformNodeShow(is);
}

void EmbededDeformation3D::keyPressEvent(QKeyEvent *e)
{
	switch(e->key())
	{
	case Qt::Key_G:
		{
			ui.render_widget->init_deform();
			break;
		}
	case Qt::Key_R:
		{
			ui.render_widget->resetView();
			break;
		}
	case Qt::Key_T:
		{
			ui.render_widget->resetModel();
			break;
		}
	case Qt::Key_Q:
		{
			ui.render_widget->setKeyStatus(KEY_SELECT_POINT);
			break;
		}
	case Qt::Key_W:
		{
			ui.render_widget->setKeyStatus(KEY_DELETE_POINT);
			break;
		}
	case Qt::Key_A:
		{
			ui.render_widget->setKeyStatus(KEY_SELECT_BOX);
			break;
		}
	case Qt::Key_J:
		{
			ui.render_widget->setKeyStatus(KEY_SEL_BOXPAIR);
			break;
		}
	case Qt::Key_F:
		{
			ui.render_widget->delete_boxpair();
			break;
		}
	case Qt::Key_V:
		{
			ui.render_widget->setKeyStatus(KEY_DRAG_BOX);
			break;
		}
	case Qt::Key_N:
		{
			ui.render_widget->setKeyStatus(KEY_SEL_NODE);
			break;
		}
	case Qt::Key_L:
		{
			ui.render_widget->write_mesh();
			break;
		}
	case Qt::Key_E:
		{
			ui.render_widget->cancel_sel_box();
			break;
		}
	case Qt::Key_B:
		{
			ui.render_widget->generateNew_AABB();
			break;
		}
	
	case Qt::Key_X:
		{
			ui.render_widget->setTranslateAxis(0);
			break;
		}
	case Qt::Key_Y:
		{
			ui.render_widget->setTranslateAxis(1);
			break;
		}
	case Qt::Key_Z:
		{
			ui.render_widget->setTranslateAxis(2);
			break;
		}
	case Qt::Key_O:
		{
			ui.render_widget->deform();
			break;
		}
	case Qt::Key_D:
		{
			ui.render_widget->delete_AABB();
			break;
		}
	case Qt::Key_4:
		{
			ui.render_widget->minus_modifyX();
			break;
		}
	case Qt::Key_6:
		{
			ui.render_widget->plus_modifyX();
			break;
		}
	case Qt::Key_8:
		{
			ui.render_widget->plus_modifyY();
			break;
		}
	case Qt::Key_5:
		{
			ui.render_widget->minus_modifyY();
			break;
		}
	case Qt::Key_0:
		{
			ui.render_widget->readRenderInfo("render_info");
			break;
		}
	}
}

void EmbededDeformation3D::keyReleaseEvent(QKeyEvent *e)
{
	ui.render_widget->setKeyStatus(KEY_NONE);
}

void EmbededDeformation3D::plusId()
{
	ui.render_widget->plus_testID();
}

void EmbededDeformation3D::minusId()
{
	ui.render_widget->minus_testID();
}

void EmbededDeformation3D::weightDefine()
{
	ui.render_widget->weightDefine();
}

