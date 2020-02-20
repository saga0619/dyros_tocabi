#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H

#define DEG2RAD (0.01745329251994329576923690768489)
// constexpr size_t MAX_DOF=50;

#include <Eigen/Dense>
//#include <Eigen/SVD>
#include <iostream>


#define MODEL_DOF 33
#define ENDEFFECTOR_NUMBER 4
#define LINK_NUMBER 34
#define MODEL_DOF_VIRTUAL 39
#define MODEL_DOF_QVIRTUAL 40


#define GRAVITY 9.80665
#define MAX_DOF 50U
#define RAD2DEG 1 / DEG2RAD

namespace Eigen
{
// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)    \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
  typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
  typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

typedef double rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 8, 8)
//EIGEN_MAKE_TYPEDEFS(rScalar, d, MODEL_DOF, MODEL_DOF)
//EIGEN_MAKE_TYPEDEFS(rScalar, d, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL)
//EIGEN_MAKE_TYPEDEFS(rScalar, d, MODEL_DOF_QVIRTUAL, MODEL_DOF_QVIRTUAL)

// typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;  // typedef Transform< double, 3, Isometry > 	Eigen::Isometry3d

typedef Matrix<rScalar, 1, 3> Matrix1x3d;
typedef Matrix<rScalar, 1, 4> Matrix1x4d;
typedef Matrix<rScalar, 4, 3> Matrix4x3d;
typedef Matrix<rScalar, 6, 3> Matrix6x3d;
typedef Matrix<rScalar, 6, 7> Matrix6x7d;
typedef Matrix<rScalar, 8, 4> Matrix8x4d;
typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
typedef Matrix<rScalar, -1, 1, 0, 12, 1> VectorLXd; //Leg IK
typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

typedef Matrix<rScalar, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL> MatrixVVd;

typedef Matrix<rScalar, 12, 1> Vector12d;

typedef Matrix<rScalar, MODEL_DOF, 1> VectorQd;
typedef Matrix<rScalar, MODEL_DOF_VIRTUAL, 1> VectorVQd;
typedef Matrix<rScalar, MODEL_DOF_QVIRTUAL, 1> VectorQVQd;

typedef Matrix<rScalar, 6, MODEL_DOF_VIRTUAL> Matrix6Vd;
typedef Matrix<rScalar, 3, MODEL_DOF_VIRTUAL> Matrix3Vd;

typedef Matrix<rScalar, 6, MODEL_DOF> Matrix6Qd;
typedef Matrix<rScalar, 3, MODEL_DOF> Matrix3Qd;

//Complex
typedef Matrix<std::complex<double>, 8, 4> Matrix8x4cd;

} // namespace Eigen

namespace DyrosMath
{

//constexpr double GRAVITY {9.80665};
//constexpr double DEG2RAD {};

static Eigen::Matrix3d skm(Eigen::Vector3d x)
{
  Eigen::Matrix3d Skew_temp1(3, 3);
  Skew_temp1.setZero();
  Skew_temp1(0, 1) = -x(2);
  Skew_temp1(0, 2) = x(1);
  Skew_temp1(1, 0) = x(2);
  Skew_temp1(1, 2) = -x(0);
  Skew_temp1(2, 0) = -x(1);
  Skew_temp1(2, 1) = x(0);
  return Skew_temp1;
}

static double cubic(double time,    ///< Current time
                    double time_0,  ///< Start time
                    double time_f,  ///< End time
                    double x_0,     ///< Start state
                    double x_f,     ///< End state
                    double x_dot_0, ///< Start state dot
                    double x_dot_f  ///< End state dot
)
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

          + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

          + (-2 * total_x / total_time3 +
             (x_dot_0 + x_dot_f) / total_time2) *
                elapsed_time * elapsed_time * elapsed_time;
  }

  return x_t;
}

static double cubicDot(double time,    ///< Current time
                       double time_0,  ///< Start time
                       double time_f,  ///< End time
                       double x_0,     ///< Start state
                       double x_f,     ///< End state
                       double x_dot_0, ///< Start state dot
                       double x_dot_f, ///< End state dot
                       double hz       ///< control frequency
)
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_dot_0;
  }
  else if (time > time_f)
  {
    x_t = x_dot_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x = x_f - x_0;

    x_t = x_dot_0

          + 2 * (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time

          + 3 * (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2) * elapsed_time * elapsed_time;
  }

  return x_t;
}

template <int N>
static Eigen::Matrix<double, N, 1> cubicVector(double time,                         ///< Current time
                                               double time_0,                       ///< Start time
                                               double time_f,                       ///< End time
                                               Eigen::Matrix<double, N, 1> x_0,     ///< Start state
                                               Eigen::Matrix<double, N, 1> x_f,     ///< End state
                                               Eigen::Matrix<double, N, 1> x_dot_0, ///< Start state dot
                                               Eigen::Matrix<double, N, 1> x_dot_f  ///< End state dot
)
{

  Eigen::Matrix<double, N, 1> res;
  for (unsigned int i = 0; i < N; i++)
  {
    res(i) = cubic(time, time_0, time_f, x_0(i), x_f(i), x_dot_0(i), x_dot_f(i));
  }
  return res;
}

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                              Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++)
  {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5 * phi;

  return phi;
}

static Eigen::Isometry3d multiplyIsometry3d(Eigen::Isometry3d A,
                                            Eigen::Isometry3d B)
{
  Eigen::Isometry3d AB;

  AB.linear() = A.linear() * B.linear();
  AB.translation() = A.linear() * B.translation() + A.translation();
  return AB;
}

static Eigen::Isometry3d inverseIsometry3d(Eigen::Isometry3d A)
{
  Eigen::Isometry3d A_inv;

  A_inv.linear() = A.linear().transpose();
  A_inv.translation() = -A.linear().transpose() * A.translation();
  return A_inv;
}

static Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
  Eigen::Matrix3d rotate_wth_z(3, 3);

  rotate_wth_z(0, 0) = cos(yaw_angle);
  rotate_wth_z(1, 0) = sin(yaw_angle);
  rotate_wth_z(2, 0) = 0.0;

  rotate_wth_z(0, 1) = -sin(yaw_angle);
  rotate_wth_z(1, 1) = cos(yaw_angle);
  rotate_wth_z(2, 1) = 0.0;

  rotate_wth_z(0, 2) = 0.0;
  rotate_wth_z(1, 2) = 0.0;
  rotate_wth_z(2, 2) = 1.0;

  return rotate_wth_z;
}

static Eigen::Matrix3d rotateWithY(double pitch_angle)
{
  Eigen::Matrix3d rotate_wth_y(3, 3);

  rotate_wth_y(0, 0) = cos(pitch_angle);
  rotate_wth_y(1, 0) = 0.0;
  rotate_wth_y(2, 0) = -sin(pitch_angle);

  rotate_wth_y(0, 1) = 0.0;
  rotate_wth_y(1, 1) = 1.0;
  rotate_wth_y(2, 1) = 0.0;

  rotate_wth_y(0, 2) = sin(pitch_angle);
  rotate_wth_y(1, 2) = 0.0;
  rotate_wth_y(2, 2) = cos(pitch_angle);

  return rotate_wth_y;
}

static Eigen::Matrix3d rotateWithX(double roll_angle)
{
  Eigen::Matrix3d rotate_wth_x(3, 3);

  rotate_wth_x(0, 0) = 1.0;
  rotate_wth_x(1, 0) = 0.0;
  rotate_wth_x(2, 0) = 0.0;

  rotate_wth_x(0, 1) = 0.0;
  rotate_wth_x(1, 1) = cos(roll_angle);
  rotate_wth_x(2, 1) = sin(roll_angle);

  rotate_wth_x(0, 2) = 0.0;
  rotate_wth_x(1, 2) = -sin(roll_angle);
  rotate_wth_x(2, 2) = cos(roll_angle);

  return rotate_wth_x;
}

static Eigen::Vector3d rot2Euler(Eigen::Matrix3d Rot)
{
  double beta;
  Eigen::Vector3d angle;
  beta = -asin(Rot(2, 0));

  if (abs(beta) < 90 * DEG2RAD)
    beta = beta;
  else
    beta = 180 * DEG2RAD - beta;

  angle(0) = atan2(Rot(2, 1), Rot(2, 2) + 1E-37); //roll
  angle(2) = atan2(Rot(1, 0), Rot(0, 0) + 1E-37); //pitch
  angle(1) = beta;                                //yaw

  return angle;
}

static Eigen::MatrixXd pinv_SVD(const Eigen::MatrixXd &A, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);
  return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

static Eigen::MatrixXd pinv_QR(const Eigen::MatrixXd &A) //faster than pinv_SVD,
{
  //FullPivHouseholderQR<MatrixXd> qr(A);
  //qr.compute(A);
  //qr.setThreshold(10e-10);
  //return qr.inverse();

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
  qr.setThreshold(10e-10);
  int rank = qr.rank();

  if (rank == 0)
  {
    std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
    return A;
  }
  else
  {
    if (A.rows() > A.cols())
    {
      //ColPivHouseholderQR<MatrixXd> qr(A);
      qr.compute(A);
      Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
      Eigen::MatrixXd Rpsinv2(A.cols(), A.rows());

      Rpsinv2.setZero();
      Rpsinv2.topLeftCorner(rank, rank) = R.inverse();

      return qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose();
    }
    else if (A.cols() > A.rows())
    {
      //ColPivHouseholderQR<MatrixXd> qr(A.transpose());
      qr.compute(A.transpose());
      Eigen::MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>();
      Eigen::MatrixXd Rpsinv2(A.rows(), A.cols());

      Rpsinv2.setZero();
      Rpsinv2.topLeftCorner(rank, rank) = R.inverse();
      return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
    }
    else if (A.cols() == A.rows())
    {
      if (rank == A.cols() && rank == A.cols()) //full rank suqre matrix
      {
        return A.inverse();
      }
      else //rank deficient matrix
      {
#ifdef EIGEN_3_3
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A);
        return cod.pseudoInverse();
#else
        pinv_SVD(A, 10e-8);
#endif
      }
    }
  }
}

static void floatGyroframe(Eigen::Isometry3d trunk, Eigen::Isometry3d reference, Eigen::Isometry3d new_trunk)
{
  Eigen::Vector3d rpy_ang;
  rpy_ang = DyrosMath::rot2Euler(reference.linear());

  Eigen::Matrix3d temp;
  temp = DyrosMath::rotateWithZ(-rpy_ang(2));

  new_trunk.linear() = temp * trunk.linear();
  new_trunk.translation() = temp * (trunk.translation() - reference.translation());
}

// template <int _State_Size_, int _Input_Size_>
// Eigen::Matrix<double, _State_Size_, _State_Size_> discreteRiccatiEquation(
//     Eigen::Matrix<double, _State_Size_, _State_Size_> a,
//     Eigen::Matrix<double, _State_Size_, _Input_Size_> b,
//     Eigen::Matrix<double, _Input_Size_, _Input_Size_> r,
//     Eigen::Matrix<double, _State_Size_, _State_Size_> q)
// {
//   Eigen::Matrix4d z11, z12, z21, z22;
//   z11 = a.inverse();
//   z12 = a.inverse() * b * r.inverse() * b.transpose();
//   z21 = q * a.inverse();
//   z22 = a.transpose() + q * a.inverse() * b * r.inverse() * b.transpose();

//   Eigen::Matrix<double, 2 * _State_Size_, 2 * _State_Size_> z;
//   z.setZero();
//   z.topLeftCorner(4, 4) = z11;
//   z.topRightCorner(4, 4) = z12;
//   z.bottomLeftCorner(4, 4) = z21;
//   z.bottomRightCorner(4, 4) = z22;

//   std::vector<double> eigVal_real(8);
//   std::vector<double> eigVal_img(8);
//   std::vector<Eigen::Vector8d> eigVec_real(8);
//   std::vector<Eigen::Vector8d> eigVec_img(8);

//   for (int i = 0; i < 8; i++)
//   {
//     eigVec_real[i].setZero();
//     eigVec_img[i].setZero();
//   }

//   Eigen::Matrix<double, 2 * _State_Size_, 1> deigVal_real, deigVal_img;
//   Eigen::Matrix<double, 2 * _State_Size_, 2 * _State_Size_> deigVec_real, deigVec_img;
//   deigVal_real.setZero();
//   deigVal_img.setZero();
//   deigVec_real.setZero();
//   deigVec_img.setZero();
//   deigVal_real = z.eigenvalues().real();
//   deigVal_img = z.eigenvalues().imag();

//   Eigen::EigenSolver<Eigen::Matrix<double, 2 * _State_Size_, 2 * _State_Size_>> ev(z);
//   //EigenVector Solver
//   //Matrix3D ones = Matrix3D::Ones(3,3);
//   //EigenSolver<Matrix3D> ev(ones);
//   //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ev.eigenvectors().col(1) << endl;

//   for (int i = 0; i < 8; i++)
//   {
//     for (int j = 0; j < 8; j++)
//     {
//       deigVec_real(j, i) = ev.eigenvectors().col(i)(j).real();
//       deigVec_img(j, i) = ev.eigenvectors().col(i)(j).imag();
//     }
//   }

//   //Order the eigenvectors
//   //move e-vectors correspnding to e-value outside the unite circle to the left

//   Eigen::Matrix8x4d tempZ_real, tempZ_img;
//   tempZ_real.setZero();
//   tempZ_img.setZero();
//   int c = 0;

//   for (int i = 0; i < 8; i++)
//   {
//     if ((deigVal_real(i) * deigVal_real(i) + deigVal_img(i) * deigVal_img(i)) > 1.0) //outside the unit cycle
//     {
//       for (int j = 0; j < 8; j++)
//       {
//         tempZ_real(j, c) = deigVec_real(j, i);
//         tempZ_img(j, c) = deigVec_img(j, i);
//       }
//       c++;
//     }
//   }

//   Eigen::Matrix8x4cd tempZ_comp;
//   for (int i = 0; i < 8; i++)
//   {
//     for (int j = 0; j < 4; j++)
//     {
//       tempZ_comp.real()(i, j) = tempZ_real(i, j);
//       tempZ_comp.imag()(i, j) = tempZ_img(i, j);
//     }
//   }

//   Eigen::Matrix4cd U11, U21, X;
//   for (int i = 0; i < 4; i++)
//   {
//     for (int j = 0; j < 4; j++)
//     {
//       U11(i, j) = tempZ_comp(i, j);
//       U21(i, j) = tempZ_comp(i + 4, j);
//     }
//   }
//   X = U21 * (U11.inverse());
//   Eigen::Matrix4d X_sol;
//   for (int i = 0; i < 4; i++)
//   {
//     for (int j = 0; j < 4; j++)
//     {
//       X_sol(i, j) = X.real()(i, j);
//     }
//   }

//   return X_sol;
// }

static Eigen::Vector3d QuinticSpline(
    double time,     ///< Current time
    double time_0,   ///< Start time
    double time_f,   ///< End time
    double x_0,      ///< Start state
    double x_dot_0,  ///< Start state dot
    double x_ddot_0, ///< Start state ddot
    double x_f,      ///< End state
    double x_dot_f,  ///< End state
    double x_ddot_f) ///< End state ddot
{
  double a1, a2, a3, a4, a5, a6;
  double time_s;

  Eigen::Vector3d result;

  if (time < time_0)
  {
    result << x_0, x_dot_0, x_ddot_0;
    return result;
  }
  else if (time > time_f)
  {
    result << x_f, x_dot_f, x_ddot_f;
    return result;
  }

  time_s = time_f - time_0;
  a1 = x_0;
  a2 = x_dot_0;
  a3 = x_ddot_0 / 2.0;

  Eigen::Matrix3d Temp;
  Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
      3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
      6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

  Eigen::Vector3d R_temp;
  R_temp << x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0,
      x_dot_f - x_dot_0 - x_ddot_0 * time_s,
      x_ddot_f - x_ddot_0;

  Eigen::Vector3d RES;

  RES = Temp.inverse() * R_temp;

  a4 = RES(0);
  a5 = RES(1);
  a6 = RES(2);

  double time_fs = time - time_0;

  double position = a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5);
  double velocity = a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4);
  double acceleration = 2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3);

  result << position, velocity, acceleration;

  return result;
}

static double minmax_cut(double val, double min_, double max_)
{
  if (val < min_)
    return min_;
  else if (val > max_)
    return max_;
  else
    return val;
}

static double check_border(double x, double y, double x0, double x1, double y0, double y1, double sign)
{
  return -sign * ((y1 - y0) * (x - x0) + (x1 - x0) * (y0 - y));
}
static inline double lowPassFilter(double input, double prev, double ts, double tau)
{
  return (tau * prev + ts * input) / (tau + ts);
}
template <int N>
static Eigen::Matrix<double, N, 1> lowPassFilter(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev, double ts, double tau)
{
  Eigen::Matrix<double, N, 1> res;
  for (int i = 0; i < N; i++)
  {
    res(i) = lowPassFilter(input(i), prev(i), ts, tau);
  }
  return res;
}
} // namespace DyrosMath
#endif
