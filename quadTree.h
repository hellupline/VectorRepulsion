struct xy {
    int x, y, value;
    xy(int _x, int _y, int _value) : x(_x), y(_y), value(_value) { }
    xy(int _x, int _y) : x(_x), y(_y), value(0) { }
    xy() : x(0), y(0), value(0) { }
};

struct rect {
    int x, y, w, h;
    rect(int _x, int _y, int _w, int _h) : x(_x), y(_y), w(_w), h(_h) { }
    rect() : x(0), y(0), w(0), h(0) { }
    inline bool contains(xy& p) { return (p.x >= x && p.y >= y && p.x < x+w && p.y < y+h); }
    inline bool intersects(rect& r) { return !(r.x > (x+w) || (r.x+r.w) < x || r.y > (y+h) || (r.y+r.h) < y); }
};

class quadTree {
    public:
        static const int QT_NODE_CAPACITY = 16;
        rect bounds;
        int sz;
        xy points[QT_NODE_CAPACITY];

        quadTree* nw;
        quadTree* ne;
        quadTree* sw;
        quadTree* se;

        quadTree(rect _bounds);
        bool insert(xy p);
        //bool remove(xy p);
        bool subDivide();
        void resize(rect newBounds);
        void delChildren();
        std::vector<xy*> queryRange(rect& range);
};
