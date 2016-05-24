#pragma once
#include "isam/isam.h"

using namespace isam;

// the following two template classes allow the addition of a timestamp and an id to a regular node.

template <class T>
class TimeStampedT_Node : public NodeT<T>
{
  int64_t _ts;

 public:
 TimeStampedT_Node(int64_t ts) : NodeT<T>(), _ts(ts) {}
 TimeStampedT_Node(const char* name, int64_t ts) : NodeT<T>(name),_ts(ts) {}

  int64_t ts(){return _ts;}

  void write(std::ostream &out) const {
    NodeT<T>::write(out);
    out << " t = " << _ts;
  }

};


typedef TimeStampedT_Node<Pose3d> Pose3dTS_Node;
typedef TimeStampedT_Node<Pose2d> Pose2dTS_Node;

template <class T>
class IDT_Node : public Point3dT_Node<T>
{
  int32_t _id;

 public:
 IDT_Node(int32_t id) : Point3dT_Node<T>(), _id(id) {}
 IDT_Node(const char* name, int32_t id) : Point3dT_Node<T>(name),_id(id) {}

  int32_t id(){return _id;}
  void write(std::ostream &out) const {
    NodeT<T>::write(out);
    out << " id = " << _id;
  }


};


typedef IDT_Node<Point3d> Point3dID_Node;

class Virtual_Pose3d_Pose3d_Factor : public FactorT<Pose3d> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The GLOBAL measurement from pose1 to pose2, 
   * @param noise The 6x6 square root information matrix (upper triangular).
   * @param anchor1 Optional anchor node for trajectory to which pose1 belongs to.
   * @param anchor2 Optional anchor node for trajectory to which pose2 belongs to.
   */
  Virtual_Pose3d_Pose3d_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2,
      const Pose3d& measure, const Noise& noise)
    : FactorT<Pose3d>("Virtual_Pose3d_Pose3d_Factor", 6, noise, measure), _pose1(pose1), _pose2(pose2) {
    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
  }

  void initialize() {
    require(_pose1->initialized() || _pose2->initialized(),
        "slam3d: Pose3d_Pose3d_Factor requires pose1 or pose2 to be initialized");

    if (!_pose1->initialized() && _pose2->initialized()) {
      // Reverse constraint 
      Pose3d p2 = _pose2->value();
      Pose3d z;
      Pose3d predict(p2.vector() - _measure.vector());
      _pose1->init(predict);
    } else if (_pose1->initialized() && !_pose2->initialized()) {
      Pose3d p1 = _pose1->value();
      Pose3d predict(p1.vector() + _measure.vector());
      _pose2->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    Pose3d predicted(p2.vector() - p1.vector());
    Eigen::VectorXd err = predicted.vector() - _measure.vector();
    err(3) = standardRad(err(3));
    err(4) = standardRad(err(4));
    err(5) = standardRad(err(5));
    return err;
  }
  // TODO jacobians?
};

class Virtual_Pose3d_Point3d_Factor : public FactorT<Point3d> {
  Pose3d_Node* _pose;
  Point3d_Node* _point;

public:

  /**
   * Constructor.
   * @param pose The pose from which the landmark is observed.
   * @param point The point or landmark that is observed
   * @param measure The GLOBAL observation of the landmark in the pose's frame.
   * @param noise The 3x3 square root information matrix (upper triangular).
   */
  Virtual_Pose3d_Point3d_Factor(Pose3d_Node* pose, Point3d_Node* point,
      const Point3d& measure, const Noise& noise)
    : FactorT<Point3d>("Virtual_Pose3d_Point3d_Factor", 3, noise, measure), _pose(pose), _point(point) {
    _nodes.resize(2);
    _nodes[0] = pose;
    _nodes[1] = point;
  }

  void initialize() {
    require(_pose->initialized(), "slam3d: Pose3d_Point3d_Factor requires pose to be initialized");
    if (!_point->initialized()) {
      Pose3d p = _pose->value();
      Point3d predict(p.trans().vector() + _measure.vector());
      _point->init(predict);
    }
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& po = _pose->value(s);
    const Point3d& pt = _point->value(s);
    Point3d p(pt.vector() - po.trans().vector());
    Eigen::VectorXd predicted = p.vector();
    return (predicted - _measure.vector());
  }
//TODO Jacobians?
};

class Pose3d_Pose3d_RangeFactor : public FactorT<double> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;

public:

  Pose3d_Pose3d_RangeFactor(Pose3d_Node* pose1, Pose3d_Node* pose2,
                              const double& measure, const Noise& noise)
    : FactorT<double>("Pose3d_Pose3d_RangeFactor", 1, noise, measure), _pose1(pose1), _pose2(pose2) {
    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
  }

  void initialize() {
    require(_nodes[0]->initialized(), "slam_auv: Pose3d_Pose3d_RangeFactor requires pose1 to be initialized");
    require(_nodes[1]->initialized(), "slam_auv: Pose3d_Pose3d_RangeFactor requires pose2 to be initialized");
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    Point3d p1(_pose1->value(s).trans());
    Point3d p2(_pose2->value(s).trans());
    double predicted;
    predicted = (p1.vector() - p2.vector()).norm(); // sqrt( (x1-x2)^2 + (y1-y2)^2 )
    double temp = predicted - _measure;
    Eigen::VectorXd err(1);
    err << temp;
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " " << _measure << " " << noise_to_string(_noise);
    if (_nodes.size()==4) {
      out << " " << _nodes[2]->unique_id() << " " << _nodes[3]->unique_id();
    }
  }
};

class Pose3d_PartialYPR_Factor : public FactorT<Rot3d>{
  const Pose3d_Node* _pose;

public:

  /**
   * Constructor.
   * @param pose The pose node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 3x3 square root information matrix (upper triangular).
   */
  Pose3d_PartialYPR_Factor(Pose3d_Node* pose, const Rot3d& prior, const Noise& noise)
    : FactorT<Rot3d>("Pose3d_PartialYPR_Factor", 3, noise, prior), _pose(pose)
  {
    _nodes.resize(1);
    _nodes[0] = pose;
  }

  void initialize() {
    // Partial prior is not used for initialization
  }

  Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
    // associated pose x,y,z,h,p,r
    const Pose3d& pose = _pose->value(s);
    Eigen::VectorXd err(3);
    err << pose.yaw() - _measure.yaw(), 
           pose.pitch() - _measure.pitch(), 
           pose.roll() - _measure.roll();
    err(0) = standardRad(err(0));
    err(1) = standardRad(err(1));
    err(2) = standardRad(err(2));
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " (" << _measure.yaw() << ", "<< _measure.pitch() << ", "<<_measure.roll() << ") " << noise_to_string(_noise);
  }
};

class Pose3d_PartialZ_Factor : public FactorT<double> {
  const Pose3d_Node* _pose;

public:
  /**
   * Constructor.
   * @param pose The pose node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Pose3d_PartialZ_Factor(Pose3d_Node* pose, const double  prior, const Noise& noise)
    : FactorT<double>("Pose3d_PartialZ_Factor", 1, noise, prior), _pose(pose)
  {
    _nodes.resize(1);
    _nodes[0] = pose;
  }

  void initialize() {
    // Partial prior is not used for initialization
  }

  Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
    // associated pose x,y,z,h,p,r
    const Pose3d& pose = _pose->value(s);
    Eigen::VectorXd err(1);
    err << pose.z() - _measure;
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " (" << _measure << ") " << noise_to_string(_noise);
  }
};


class Pose3d_PartialXY_Factor : public FactorT<Eigen::Vector2d> {
  const Pose3d_Node* _pose;

public:
  /**
   * Constructor.
   * @param pose The pose node the prior acts on.
   * @param prior The actual prior measurement.
   * @param noise The 1x1 square root information matrix (upper triangular).
   */
  Pose3d_PartialXY_Factor(Pose3d_Node* pose, const Eigen::Vector2d&  prior, const Noise& noise)
    : FactorT<Eigen::Vector2d>("Pose3d_PartialXY_Factor", 1, noise, prior), _pose(pose)
  {
    _nodes.resize(1);
    _nodes[0] = pose;
  }

  void initialize() {
    // Partial prior is not used for initialization
  }

  Eigen::VectorXd basic_error(Selector s = LINPOINT) const {
    // associated pose x,y,z,h,p,r
    const Pose3d& pose = _pose->value(s);
    Eigen::VectorXd err(2);
    Eigen::Vector2d pos(pose.x(),pose.y());
    err =  pos - _measure;
    return err;
  }

  void write(std::ostream &out) const {
    Factor::write(out);
    out << " (" << _measure << ") " << noise_to_string(_noise);
  }
};


// A class for "position odometry"

class Pose3d_Pose3d_PartialXY_Factor : public FactorT<Eigen::Vector2d> {
  Pose3d_Node* _pose1;
  Pose3d_Node* _pose2;

public:

  /**
   * Constructor.
   * @param pose1 The pose from which the measurement starts.
   * @param pose2 The pose to which the measurement extends.
   * @param measure The relative partial measurement from Vector2d to Vector2d: X, Y.
   * @param noise The 2x2 square root information matrix (upper triangular).
   */
  Pose3d_Pose3d_PartialXY_Factor(Pose3d_Node* pose1, Pose3d_Node* pose2,
                                  const Eigen::Vector2d& measure, const Noise& noise)
    : FactorT<Eigen::Vector2d>("Pose3d_Pose3d_PartialXY_Factor", 2, noise, measure), 
    _pose1(pose1), _pose2(pose2)
  {
    _nodes.resize(2);
    _nodes[0] = pose1;
    _nodes[1] = pose2;
  }

  void initialize() {
    require(_pose1->initialized() && _pose2->initialized(),
        "slam3d: Pose3d_Pose3d_PartialXY_Factor requires pose1 and pose2 to be initialized");
  }

  Eigen::VectorXd basic_error(Selector s = ESTIMATE) const {
    const Pose3d& p1 = _pose1->value(s);
    const Pose3d& p2 = _pose2->value(s);
    Eigen::VectorXd predicted = p2.vector() - p1.vector();
    Eigen::Vector2d predicted2(predicted(0), predicted(1));
    Eigen::Vector2d err = predicted2 - _measure;
    return err;
  }

};

