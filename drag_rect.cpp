#include "drag_rect.h"
#include <algorithm>
#include <iostream>
using namespace std;
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

drag_rect::drag_rect(float x, float y)
{
	rect_[0] = rect_[2] = x;
	rect_[1] = rect_[3] = y;
}

void drag_rect::reset(float x, float y)
{
    rect_[0] = rect_[2] = x;
    rect_[1] = rect_[3] = y;
}

void drag_rect::move_to(float x, float y)
{
	rect_[2] = x; rect_[3] = y;
}

void drag_rect::render(void) const
{
	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, vp[2], vp[3], 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
	glBegin(GL_QUADS);
	glVertex2fv(rect_+0);
	glVertex2f(rect_[0], rect_[3]);
	glVertex2fv(rect_+2);
	glVertex2f(rect_[2], rect_[1]);
	glEnd();
	glPopAttrib();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}
