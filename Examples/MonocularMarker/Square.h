#ifndef SQUARE_H
#define SQUARE_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>

// qpoint class, similar behavior as Qt QPoint, so I can easily copy some codes
class QPoint{
public:
	inline int x() const{
		return _x;
	}
	inline int y() const{
		return _y;
	}
	inline int &rx(){
		return _x;
	}
	inline int &ry(){
		return _y;
	}
	inline QPoint(int X, int Y) :_x(X), _y(Y){
		//nvm
	}
	inline QPoint() : _x(0), _y(0){
		//nvm
	}
	inline int manhattanLength() const{
		return abs(_x) + abs(_y);
	}
	inline QPoint &operator +=(const QPoint &p){
		_x += p._x; _y += p._y; return *this;
	}
	inline QPoint &operator -=(const QPoint &p){
		_x -= p._x; _y -= p._y; return *this;
	}
	friend inline const QPoint operator+(const QPoint &p1, const QPoint &p2){
		return QPoint(p1._x + p2._x, p1._y + p2._y);
	}
	friend inline const QPoint operator-(const QPoint &p1, const QPoint &p2){
		return QPoint(p1._x - p2._x, p1._y - p2._y);
	}
	friend inline const QPoint operator+(const QPoint &p){
		return p;
	}
	friend inline const QPoint operator-(const QPoint &p){
		return QPoint(-p._x, -p._y);
	}

private:
	int _x, _y;
};

class Square
{
public:
	inline Square();
	inline Square(QPoint, QPoint, QPoint, QPoint);
	inline Square(std::vector<cv::Point> &);

	//smaller, better
	double squareness();

	inline double area() const;
	inline QPoint sum() const;
	inline int similarity(const Square &another) const;
	inline std::vector<cv::Point2f> toVector() const;
	cv::Point2f mid();

	void reOrder();

	bool contains(const QPoint point);
	int adapt(const std::vector<cv::KeyPoint> &keypoints);


private:
	QPoint p0, p1, p2, p3;
	//saved squareness
	//double sn;
};

//inline functions
Square::Square() :p0(0, 0), p1(0, 0), p2(0, 0), p3(0, 0)//,sn(180.0)
{

}

Square::Square(QPoint point1, QPoint point2, QPoint point3, QPoint point4) : p0(point1), p1(point2), p2(point3), p3(point4)//,sn(180.0)
{

}

Square::Square(std::vector<cv::Point> &vec) : p0(vec[0].x, vec[0].y), p1(vec[1].x, vec[1].y), p2(vec[2].x, vec[2].y), p3(vec[3].x, vec[3].y)//,sn(180.0)
{

}

//return the area size of the square
inline double Square::area() const{
	return abs(0.5*(p0.x()*p1.y() + p1.x()*p2.y() + p2.x()*p0.y() - p0.x()*p2.y() - p2.x()*p1.y() - p1.x()*p0.y())) + abs(0.5*(p1.x()*p2.y() + p2.x()*p3.y() + p3.x()*p1.y() - p1.x()*p3.y() - p3.x()*p2.y() - p2.x()*p1.y()));
}
//nvm
inline QPoint Square::sum() const{
	return p0 + p1 + p2 + p3;
}
//nvm
inline int Square::similarity(const Square &another) const{
	return (sum() - another.sum()).manhattanLength();
}
//convert the square to a set of cv::point2f, for opencv functions
inline std::vector<cv::Point2f> Square::toVector() const{
	std::vector<cv::Point2f> vector;
	vector.push_back(cv::Point2f(p0.x(), p0.y()));
	vector.push_back(cv::Point2f(p1.x(), p1.y()));
	vector.push_back(cv::Point2f(p2.x(), p2.y()));
	vector.push_back(cv::Point2f(p3.x(), p3.y()));
	return vector;
}

#endif