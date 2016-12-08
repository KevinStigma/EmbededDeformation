#ifndef HJ_DRAG_RECT_H
#define HJ_DRAG_RECT_H

class drag_rect
{
public:
	drag_rect(float x, float y);
    void reset(float x, float y);
	void move_to(float x, float y);
	void render(void) const;

	template <typename T1, typename T2>
	bool inline contain(T1 x, T2 y) const {
		return ((x-rect_[0])*(x-rect_[2]) < 0)
			&& ((y-rect_[1])*(y-rect_[3]) < 0);
	}
//private:
	float rect_[4];	// left, top, right, bottom
};

#endif
