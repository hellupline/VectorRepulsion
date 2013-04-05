#include <iostream>
#include <vector>
#include <math.h>

#include "quadTree.h"

quadTree::quadTree(rect _bounds) {
    bounds = _bounds;
    for (int i = 0; i < QT_NODE_CAPACITY; i++)
        points[i] = xy();
    sz = 0;
    ne = nullptr;
    se = nullptr;
    nw = nullptr;
    sw = nullptr;
}

bool quadTree::insert(xy p) {
    if (!bounds.contains(p))
        return false;
    if (sz < QT_NODE_CAPACITY) {
        points[sz++] = p;
        return true;
    } else {
        if (nw == nullptr)
            subDivide();
        return (nw->insert(p) || ne->insert(p) || sw->insert(p) || se->insert(p));
    }
}

bool quadTree::subDivide() {
    if (ne != nullptr && nw != nullptr && sw != nullptr && se != nullptr)
        return false;
    nw = new quadTree(rect(bounds.x, bounds.y, bounds.w >> 1, bounds.h >> 1));
    ne = new quadTree(rect(bounds.x + (bounds.w >> 1), bounds.y, bounds.w >> 1, bounds.h >> 1));
    sw = new quadTree(rect(bounds.x, bounds.y + (bounds.h >> 1), bounds.w >> 1, bounds.h >> 1));
    se = new quadTree(rect(bounds.x + (bounds.w >> 1), bounds.y + (bounds.h >> 1), bounds.w >> 1, bounds.h >> 1));

    return true;
}

void quadTree::delChildren() {
if (nw != nullptr) {
        nw->delChildren();
        delete nw;
        nw = nullptr;
    }
    if (ne != nullptr) {
        ne->delChildren();
        delete ne;
        ne = nullptr;
    }
    if (sw != nullptr) {
        sw->delChildren();
        delete sw;
        sw = nullptr;
    }
    if (se != nullptr) {
        se->delChildren();
        delete se;
        se = nullptr;
    }
}

void quadTree::resize(rect newBounds) {
    bounds = newBounds;
    if (nw != nullptr)
        nw->resize(rect(bounds.x, bounds.y, bounds.w >> 1, bounds.h >> 1));
    if (ne != nullptr)
        ne->resize(rect(bounds.x + (bounds.w >> 1), bounds.y, bounds.w >> 1, bounds.h >> 1));
    if (sw != nullptr)
        sw->resize(rect(bounds.x, bounds.y + (bounds.h >> 1), bounds.w >> 1, bounds.h >> 1));
    if (se != nullptr)
        se->resize(rect(bounds.x + (bounds.w >> 1), bounds.y + (bounds.h >> 1), bounds.w >> 1, bounds.h >> 1));
}

void AddGroup(std::vector<xy*>& vtr, std::vector<xy*>& toAdd) {
	for (unsigned int i = 0; i < toAdd.size(); i++)
		vtr.push_back(toAdd[i]);
}

std::vector<xy*> quadTree::queryRange(rect& range) {
	std::vector<xy*> pointsInRange;

	if (!bounds.intersects(range))
		return pointsInRange;

    for (int p = 0; p < sz; p++)
		if (range.contains(points[p]))
			pointsInRange.push_back(&points[p]);

	if (nw != nullptr) {
        std::vector<xy*> tmp;

        tmp = nw->queryRange(range);
        AddGroup(pointsInRange, tmp);

        tmp = ne->queryRange(range);
        AddGroup(pointsInRange, tmp);

        tmp = sw->queryRange(range);
        AddGroup(pointsInRange, tmp);

        tmp = se->queryRange(range);
        AddGroup(pointsInRange, tmp);
    }
    return pointsInRange;
}
