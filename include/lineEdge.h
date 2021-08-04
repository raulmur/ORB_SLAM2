//
// Created by lan on 18-1-12.
//

#ifndef ORB_SLAM2_LINEEDGE_H
#define ORB_SLAM2_LINEEDGE_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace types_six_dof_expmap {
    void init();
}

//class VertexLine : public g2o::BaseVertex<3, Eigen::Vector3d> {
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//    VertexLine() {}
//
//    virtual bool read(std::istream &is) {
//        Eigen::Vector3d lv;
//        for (int i = 0; i < 3; i++)
//            is >> _estimate[i];
//        return true;
//    }
//
//    virtual bool write(std::ostream &os) const {
//        Eigen::Vector3d lv = estimate();
//        for (int i = 0; i < 3; i++) {
//            os << lv[i] << " ";
//        }
//        return os.good();
//    }
//
//    virtual void setToOriginImpl() {
//        _estimate.fill(0.);
//    }
//
//    virtual void oplusImpl(const double *update) {
//        Eigen::Map<const Eigen::Vector3d> v(update);
//        _estimate += v;
//    }
//};

class EdgeLineProjectXYZ
        : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeLineProjectXYZ() {}

    virtual void computeError() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector3d obs = _measurement;
        Eigen::Vector2d proj = cam_project(v1->estimate().map(v2->estimate()));
        _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);
        _error(1) = 0;
        _error(2) = 0;
    }

    virtual void linearizeOplus() {
        const g2o::VertexSE3Expmap *vj = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *vi = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        g2o::SE3Quat T(vj->estimate());
        Eigen::Vector3d xyz_trans = T.map(vi->estimate());

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        _jacobianOplusXj(0, 0) = -fy * ly - fx * lx * x * y * invz_2 - fy * ly * y * y * invz_2;
        _jacobianOplusXj(0, 1) = fx * lx + fx * lx * x * x * invz_2 + fy * ly * x * y * invz_2;
        _jacobianOplusXj(0, 2) = -fx * lx * y * invz + fy * ly * x * invz;
        _jacobianOplusXj(0, 3) = fx * lx * invz;
        _jacobianOplusXj(0, 4) = fy * ly * invz;
        _jacobianOplusXj(0, 5) = -(fx * lx * x + fy * ly * y) * invz_2;
        _jacobianOplusXj(1, 0) = 0;
        _jacobianOplusXj(1, 1) = 0;
        _jacobianOplusXj(1, 2) = 0;
        _jacobianOplusXj(1, 3) = 0;
        _jacobianOplusXj(1, 4) = 0;
        _jacobianOplusXj(1, 5) = 0;
        _jacobianOplusXj(2, 0) = 0;
        _jacobianOplusXj(2, 1) = 0;
        _jacobianOplusXj(2, 2) = 0;
        _jacobianOplusXj(2, 3) = 0;
        _jacobianOplusXj(2, 4) = 0;
        _jacobianOplusXj(2, 5) = 0;

        Eigen::Matrix<double, 3, 3, Eigen::ColMajor> tmp;
        tmp = Eigen::Matrix3d::Zero();
        tmp(0, 0) = fx * lx;
        tmp(0, 1) = fy * ly;
        tmp(0, 2) = -(fx * lx * x + fy * ly * y) * invz;

        Eigen::Matrix<double, 3, 3> R;
        R = T.rotation().toRotationMatrix();

        _jacobianOplusXi = 1. * invz * tmp * R;
    }

    bool read(std::istream &is) {
        for (int i = 0; i < 3; i++) {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    bool write(std::ostream &os) const {
        for (int i = 0; i < 3; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i, j);
            }
        }
        return os.good();
    }

    Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz) {
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        return res;
    }

    double fx, fy, cx, cy;
};

class EdgeLineProjectXYZOnlyPose : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeLineProjectXYZOnlyPose() {}

    virtual void computeError() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d obs = _measurement;
        Eigen::Vector2d proj = cam_project(v1->estimate().map(Xw));
        _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);
        _error(1) = 0;
        _error(2) = 0;
    }

    double chiline() {
        return _error(0) * _error(0);
    }

    virtual void linearizeOplus() {
        //        BaseUnaryEdge::linearizeOplus();
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        _jacobianOplusXi(0, 0) = -fy * ly - fx * lx * x * y * invz_2 - fy * ly * y * y * invz_2;
        _jacobianOplusXi(0, 1) = fx * lx + fx * lx * x * x * invz_2 + fy * ly * x * y * invz_2;
        _jacobianOplusXi(0, 2) = -fx * lx * y * invz + fy * ly * x * invz;
        _jacobianOplusXi(0, 3) = fx * lx * invz;
        _jacobianOplusXi(0, 4) = fy * ly * invz;
        _jacobianOplusXi(0, 5) = -(fx * lx * x + fy * ly * y) * invz_2;
        _jacobianOplusXi(1, 0) = 0;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = 0;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = 0;
        _jacobianOplusXi(1, 5) = 0;
        _jacobianOplusXi(2, 0) = 0;
        _jacobianOplusXi(2, 1) = 0;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = 0;
    }

    bool read(std::istream &is) {
        for (int i = 0; i < 3; i++) {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    bool write(std::ostream &os) const {
        for (int i = 0; i < 3; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i, j);
            }
        }
        return os.good();
    }

    Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz) {
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        return res;
    }

    Eigen::Vector3d Xw;
    double fx, fy, cx, cy;
};

class EdgeLineProjectXYZOnlyTranslation : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeLineProjectXYZOnlyTranslation() {}

    virtual void computeError() {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d obs = _measurement;
        Eigen::Vector2d proj = cam_project(v1->estimate().mapTrans(Xc));
        _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);
        _error(1) = 0;
        _error(2) = 0;
    }

    double chiline() {
        return _error(0) * _error(0);
    }

    virtual void linearizeOplus() {
        //        BaseUnaryEdge::linearizeOplus();
        const g2o::VertexSE3Expmap *vi = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().mapTrans(Xc);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = 0;
        _jacobianOplusXi(0, 2) = 0;
        _jacobianOplusXi(0, 3) = fx * lx * invz;
        _jacobianOplusXi(0, 4) = fy * ly * invz;
        _jacobianOplusXi(0, 5) = -(fx * lx * x + fy * ly * y) * invz_2;
        _jacobianOplusXi(1, 0) = 0;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = 0;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = 0;
        _jacobianOplusXi(1, 5) = 0;
        _jacobianOplusXi(2, 0) = 0;
        _jacobianOplusXi(2, 1) = 0;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = 0;
    }

    bool read(std::istream &is) {
        for (int i = 0; i < 3; i++) {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    bool write(std::ostream &os) const {
        for (int i = 0; i < 3; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i, j);
            }
        }
        return os.good();
    }

    Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz) {
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        return res;
    }

    Eigen::Vector3d Xc;
    double fx, fy, cx, cy;
};

class EdgeLineSim3Project : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSim3Expmap>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgeLineSim3Project() {}

            void computeError()
            {
                const g2o::VertexSim3Expmap* v1 = static_cast<const g2o::VertexSim3Expmap*>(_vertices[1]);
                const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

                Eigen::Vector3d obs(_measurement);
                Eigen::Vector2d proj = v1->cam_map1(g2o::project(v1->estimate().map(v2->estimate())));
                _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);
                _error(1) = 0;
                _error(2) = 0;
            }

            virtual bool read(std::istream &is) {
                for (int i = 0; i < 3; i++) {
                    is >> _measurement[i];
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = i; j < 3; ++j) {
                        is >> information()(i, j);
                        if (i != j)
                            information()(j, i) = information()(i, j);
                    }
                }
                return true;
            }

            virtual bool write(std::ostream &os) const {
                for (int i = 0; i < 3; i++) {
                    os << measurement()[i] << " ";
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = i; j < 3; ++j) {
                        os << " " << information()(i, j);
                    }
                }
                return os.good();
            }

            virtual void linearizeOplus() {
                BaseBinaryEdge::linearizeOplus();

                _jacobianOplusXi(1, 0) = 0;
                _jacobianOplusXi(1, 1) = 0;
                _jacobianOplusXi(1, 2) = 0;
                _jacobianOplusXi(1, 3) = 0;
                _jacobianOplusXi(1, 4) = 0;
                _jacobianOplusXi(1, 5) = 0;
                _jacobianOplusXi(2, 0) = 0;
                _jacobianOplusXi(2, 1) = 0;
                _jacobianOplusXi(2, 2) = 0;
                _jacobianOplusXi(2, 3) = 0;
                _jacobianOplusXi(2, 4) = 0;
                _jacobianOplusXi(2, 5) = 0;
            }
        };

class EdgeLineInverseSim3Project : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSim3Expmap>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgeLineInverseSim3Project() {}

            void computeError()
            {
                const g2o::VertexSim3Expmap* v1 = static_cast<const g2o::VertexSim3Expmap*>(_vertices[1]);
                const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

                Eigen::Vector3d obs(_measurement);
                Eigen::Vector2d proj = v1->cam_map2(g2o::project(v1->estimate().inverse().map(v2->estimate())));
                _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);
                _error(1) = 0;
                _error(2) = 0;
            }

            virtual bool read(std::istream &is) {
                for (int i = 0; i < 3; i++) {
                    is >> _measurement[i];
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = i; j < 3; ++j) {
                        is >> information()(i, j);
                        if (i != j)
                            information()(j, i) = information()(i, j);
                    }
                }
                return true;
            }

            virtual bool write(std::ostream &os) const {
                for (int i = 0; i < 3; i++) {
                    os << measurement()[i] << " ";
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = i; j < 3; ++j) {
                        os << " " << information()(i, j);
                    }
                }
                return os.good();
            }

            virtual void linearizeOplus() {
                BaseBinaryEdge::linearizeOplus();

                _jacobianOplusXi(1, 0) = 0;
                _jacobianOplusXi(1, 1) = 0;
                _jacobianOplusXi(1, 2) = 0;
                _jacobianOplusXi(1, 3) = 0;
                _jacobianOplusXi(1, 4) = 0;
                _jacobianOplusXi(1, 5) = 0;
                _jacobianOplusXi(2, 0) = 0;
                _jacobianOplusXi(2, 1) = 0;
                _jacobianOplusXi(2, 2) = 0;
                _jacobianOplusXi(2, 3) = 0;
                _jacobianOplusXi(2, 4) = 0;
                _jacobianOplusXi(2, 5) = 0;
            }
        };

#endif //ORB_SLAM2_LINEEDGE_H
