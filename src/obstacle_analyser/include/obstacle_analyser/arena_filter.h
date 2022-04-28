//判断一个点是否在矩形内部
#include<cstdio>
#include<iostream>

using namespace std;

struct Point
{
	float x;
	float y;
	Point(float x,float y)
	{
		this->x = x;
		this->y = y;
	}

	friend ostream& operator<<(ostream& os,const Point& p);
};

ostream& operator<<(ostream& os,const Point& p)
{
	return os << p.x << ", " << p.y;
}

// 计算 |p1 p2| X |p1 p|
float GetCross(Point& p1, Point& p2,Point& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}
//改完数据后catkin_make obstacle_filter_node一下
bool IsPointInB3(Point& p, float t)
{
	Point p1(1.60-t, 1.19+t);
	Point p2(1.60-t, 0.09-t);
	Point p3(1.92+t, 0.09-t);
	Point p4(1.92+t, 1.19+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB6(Point& p, float t)
{
    Point p1(3.67-t, 1.32+t);
    Point p2(3.67-t, 1.00-t);
    Point p3(4.78+t, 1.00-t);
    Point p4(4.78+t, 1.32+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB9(Point& p, float t)
{
    //Point p1(6.50-t, 1.80+t);
    //Point p2(6.50-t, 0.50-t);
    //Point p3(8.70+t, 0.50-t);
    //Point p4(8.70+t, 1.80+t);
	Point p1(7.24-t, 1.48+t);
    Point p2(7.24-t, 1.08-t);
    Point p3(8.30+t, 1.08-t);
    Point p4(8.30+t, 1.48+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB2(Point& p, float t)
{
	Point p1(1.62-t, 2.54+t);
	Point p2(1.62-t, 2.21-t);
	Point p3(2.40+t, 2.21-t);
	Point p4(2.40+t, 2.54+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB5(Point& p, float t)
{
	//菱形的膨胀比方形小，同等膨胀需要根号2t
	Point p1(3.80-1.5*t, 2.38);
	Point p2(4.22, 2.00-1.5*t);
	Point p3(4.53+1.5*t, 2.32);
	Point p4(4.22, 2.74+1.5*t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB8(Point& p, float t)
{
	Point p1(5.96-t, 2.56+t);
	Point p2(5.96-t, 2.24-t);
	Point p3(6.68+t, 2.24-t);
	Point p4(6.68+t, 2.56+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB1(Point& p, float t)
{
	Point p1(0.18-t, 3.60+t);
	Point p2(0.18-t, 3.30-t);
	Point p3(1.23+t, 3.30-t);
	Point p4(1.23+t, 3.60+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB4(Point& p, float t)
{
    Point p1(3.58-t, 3.76+t);
    Point p2(3.58-t, 3.37-t);
    Point p3(4.73+t, 3.37-t);
    Point p4(4.73+t, 3.76+t);
	//Point p1(3.34-t, 4.13+t);
    //Point p2(3.34-t, 3.87-t);
    //Point p3(4.60+t, 3.87-t);
    //Point p4(4.60+t, 4.13+t);
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInB7(Point& p, float t)
{
    Point p1(6.44-t, 4.44+t);
    Point p2(6.44-t, 3.55-t);
    Point p3(6.80+t, 3.55-t);
    Point p4(6.80+t, 4.44+t);
    return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge1(Point& p, float t)
{
	Point p1(-10.00-t, 4.89+t);
	Point p2(-10.00-t, 0.00-t);
	Point p3( 0.18+t, 0.00-t);
	Point p4( 0.18+t, 4.89+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge2(Point& p, float t)
{
	Point p1(-10.00-t, 60.00+t);
	Point p2(-10.00-t, 4.44-t);
	Point p3( 90.00+t, 4.44-t);
	Point p4( 90.00+t, 60.00+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge3(Point& p, float t)
{
	Point p1( 8.15-t, 4.52+t);
	Point p2( 8.15-t, 0.00-t);
	Point p3( 90.00+t, 0.00-t);
	Point p4( 90.00+t, 4.52+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}

bool IsPointInEdge4(Point& p, float t)
{
	Point p1(-10.00-t,  0.28+t);
	Point p2(-10.00-t, -10.00-t);
	Point p3( 90.00+t, -10.00-t);
	Point p4( 90.00+t,  0.28+t);
	return GetCross(p1,p2,p) * GetCross(p3,p4,p) >= 0 && GetCross(p2,p3,p) * GetCross(p4,p1,p) >= 0;
}


bool IsPointInAerna(Point& p, float t)
{
	return IsPointInB1(p, t) || IsPointInB2(p, t) || IsPointInB3(p, t) || IsPointInB4(p, t) || IsPointInB5(p, t) || IsPointInB6(p, t) || IsPointInB7(p, t) || IsPointInB8(p, t) || IsPointInB9(p, t) || IsPointInEdge1(p, t) || IsPointInEdge2(p, t) || IsPointInEdge3(p, t) || IsPointInEdge4(p, t);
}

