#include "square.h"
#include "KeyFrame.h"
//vector2d class, similar behavior as a small part of Qt QVector2D, so I can easily copy some codes
//double instead of float

typedef unsigned int uint;
class Vector2D{
public:
	inline Vector2D(QPoint p) :vx(p.x()), vy(p.y()){
		//nvm
	}
	inline Vector2D() : vx(0), vy(0){
		//nvm
	}
	inline double length() const{
		return sqrt(vx*vx + vy*vy);
	}
	static inline double dotProduct(const Vector2D& v1, const Vector2D& v2){
		return v1.vx*v2.vx + v1.vy*v2.vy;
	}

private:
	double vx, vy;
};

//vector3d class, similar behavior as a small part of Qt QVector3D, so I can easily copy some codes
class Vector3D{
public:
	inline Vector3D(QPoint p) :vx(p.x()), vy(p.y()), vz(0){
		//nvm
	}
	inline Vector3D(double x, double y, double z) : vx(x), vy(y), vz(z){
		//nvm
	}
	friend inline const Vector3D operator-(const Vector3D &v1, const Vector3D &v2){
		return Vector3D(v1.vx - v2.vx, v1.vy - v2.vy, v1.vz - v2.vz);
	}
	static inline Vector3D crossProduct(const Vector3D& v1, const Vector3D& v2){
		return Vector3D(v1.vy*v2.vz + v1.vz*v2.vy, v1.vx*v2.vz + v1.vz*v2.vx, v1.vy*v2.vx + v1.vx*v2.vy);
	}
	inline double x() const{
		return vx;
	}
	inline double y() const{
		return vy;
	}
	inline double z() const{
		return vz;
	}
private:
	double vx, vy, vz;
};


//square class implement

double Square::squareness(){
	Vector2D vec1(p1 - p0), vec2(p2 - p1), vec3(p3 - p2), vec4(p0 - p3);
	double t1 = acos(abs(Vector2D::dotProduct(vec1, vec3) / (vec1.length()*vec3.length())));
	double t2 = acos(abs(Vector2D::dotProduct(vec2, vec4) / (vec2.length()*vec4.length())));
	return t1 + t2;
}

void Square::reOrder(){
	//qDebug() << "pre-ordered:" << p0 << p1 << p2 << p3;
	QPoint p[4] = { p3, p2, p1, p0 };
	cv::Point2f midn = mid();
	if (Vector3D::crossProduct(Vector3D(p0) - Vector3D(midn.x, midn.y, 0), Vector3D(p1) - Vector3D(midn.x, midn.y, 0)).z()>0){
		p[0] = p0;
		p[1] = p1;
		p[2] = p2;
		p[3] = p3;
	}
	int max = 0;
	for (int i = 1; i<4; ++i){
		if (p[i].y()>p[max].y())max = i;
	}
	if (Vector2D(p[max] - p[(max + 1) % 4]).length()>Vector2D(p[max] - p[(max - 1) % 4]).length())
		max = (max - 1) % 4;
	p0 = p[max];
	p1 = p[(max + 1) % 4];
	p2 = p[(max + 2) % 4];
	p3 = p[(max + 3) % 4];
	//qDebug() << "ordered:" << p0 << p1 << p2 << p3;
}

cv::Point2f Square::mid(){
	double a1 = p2.y() - p0.y(), b1 = p0.x() - p2.x(), c1 = (p2.x() - p0.x())*p0.y() - (p2.y() - p0.y())*p0.x();
	double a2 = p3.y() - p1.y(), b2 = p1.x() - p3.x(), c2 = (p3.x() - p1.x())*p1.y() - (p3.y() - p1.y())*p1.x();
	return cv::Point2f((b1*c2 - b2*c1) / (a1*b2 - a2*b1), (a2*c1 - a1*c2) / (a1*b2 - a2*b1));
}

//returns a whether the square contains the point
//algorithm from https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
bool Square::contains(const QPoint point){
	bool result = false;
	if (((p0.y() > point.y()) != (p3.y() > point.y())) && (point.x() < (p3.x() - p0.x()) * (point.y() - p0.y()) / (p3.y() - p0.y()) + p0.x()))
		result = !result;
	if (((p1.y() > point.y()) != (p0.y() > point.y())) && (point.x() < (p0.x() - p1.x()) * (point.y() - p1.y()) / (p0.y() - p1.y()) + p1.x()))
		result = !result;
	if (((p2.y() > point.y()) != (p1.y() > point.y())) && (point.x() < (p1.x() - p2.x()) * (point.y() - p2.y()) / (p1.y() - p2.y()) + p2.x()))
		result = !result;
	if (((p3.y() > point.y()) != (p2.y() > point.y())) && (point.x() < (p2.x() - p3.x()) * (point.y() - p3.y()) / (p2.y() - p3.y()) + p3.x()))
		result = !result;
	return result;
}

//Finds candidates near the corners of the square and replace the corners.
//Because the input of canny edge detector was blurred, the result may not be as accurate as candidated fast corners.
//On the other hand, the exposure of mobile phone are usually too long for slam, thus lots of do not have good fast features.
int Square::adapt(const std::vector<cv::KeyPoint> &candidates){
	cv::Point2f r0(0, 0), r1(0, 0), r2(0, 0), r3(0, 0);
	bool b0 = false, b1 = false, b2 = false, b3 = false;
	for (uint i = 0; i < candidates.size(); ++i){
		if (abs(candidates[i].pt.x - p0.x()) + abs(candidates[i].pt.y - p0.y())
			< abs(r0.x - p0.x()) + abs(r0.y - p0.y()) &&
			(candidates[i].pt.x - p0.x())*(candidates[i].pt.x - p0.x()) + (candidates[i].pt.y - p0.y())*(candidates[i].pt.y - p0.y())<19){
			r0 = candidates[i].pt;
			b0 = true;
		}
		if (abs(candidates[i].pt.x - p1.x()) + abs(candidates[i].pt.y - p1.y())
			< abs(r1.x - p1.x()) + abs(r1.y - p1.y()) &&
			(candidates[i].pt.x - p1.x())*(candidates[i].pt.x - p1.x()) + (candidates[i].pt.y - p1.y())*(candidates[i].pt.y - p1.y())<19){
			r1 = candidates[i].pt;
			b1 = true;
		}
		if (abs(candidates[i].pt.x - p2.x()) + abs(candidates[i].pt.y - p2.y())
			< abs(r2.x - p2.x()) + abs(r2.y - p2.y()) &&
			(candidates[i].pt.x - p2.x())*(candidates[i].pt.x - p2.x()) + (candidates[i].pt.y - p2.y())*(candidates[i].pt.y - p2.y())<19){
			r2 = candidates[i].pt;
			b2 = true;
		}
		if (abs(candidates[i].pt.x - p3.x()) + abs(candidates[i].pt.y - p3.y())
			< abs(r3.x - p3.x()) + abs(r3.y - p3.y()) &&
			(candidates[i].pt.x - p3.x())*(candidates[i].pt.x - p3.x()) + (candidates[i].pt.y - p3.y())*(candidates[i].pt.y - p3.y())<19){
			r3 = candidates[i].pt;
			b3 = true;
		}
	}
	if (b0)
		p0 = QPoint(r0.x, r0.y);
	if (b1)
		p1 = QPoint(r1.x, r1.y);
	if (b2)
		p2 = QPoint(r2.x, r2.y);
	if (b3)
		p3 = QPoint(r3.x, r3.y);
	return b0 + b1 + b2 + b3;
}