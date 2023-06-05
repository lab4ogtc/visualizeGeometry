#include <iostream>
#include <iomanip>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

#include <pangolin/pangolin.h>

struct RotationMatrix {
  bool edited = false;
  Matrix3d matrix = Matrix3d::Identity();
};

ostream &operator<<(ostream &out, const RotationMatrix &r) {
  out.setf(ios::fixed);
  Matrix3d matrix = r.matrix;
  out << '=';
  out << "[" << setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
      << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
      << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
  return out;
}

istream &operator>>(istream &in, RotationMatrix &r) {
  return in;
}

struct TranslationVector {
  bool edited = false;
  Vector3d trans = Vector3d(0, 0, 0);
};

ostream &operator<<(ostream &out, const TranslationVector &t) {
  out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
  return out;
}

istream &operator>>(istream &in, TranslationVector &t) {
  std::string input;
  std::getline(in, input);
  auto data = t.trans.data();
  sscanf(input.c_str(), "=[%lf,%lf,%lf]", &data[0], &data[1], &data[2]);
  std::cout << "Override TranslationVector with [" << data[0] << "," << data[1] << "," << data[2] << "]" << std::endl;
  t.edited = true;
  return in;
}

struct QuaternionDraw {
  bool edited = false;
  Quaterniond q;
};

ostream &operator<<(ostream &out, const QuaternionDraw &quat) {
  auto c = quat.q.coeffs();
  out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
  return out;
}

istream &operator>>(istream &in, QuaternionDraw &quat) {
  std::string input;
  std::getline(in, input);
  auto data = quat.q.coeffs().data();
  sscanf(input.c_str(), "=[%lf,%lf,%lf,%lf]", &data[0], &data[1], &data[2], &data[3]);
  std::cout << "Override QuaternionDraw with [" << data[0] << "," << data[1] << "," << data[2] << "," << data[3] << "]" << std::endl;
  quat.edited = true;
  return in;
}

struct RotationVector {
  bool edited = false;
  AngleAxisd r;
};

ostream &operator<<(ostream &out, const RotationVector &rv) {
  auto angle = rv.r.angle();
  auto axis = rv.r.axis();
  out << "=[" << angle << "," << axis[0] << "," << axis[1] << "," << axis[2] << "]";
  return out;
}

istream &operator>>(istream &in, RotationVector &rv) {
  std::string input;
  std::getline(in, input);
  auto& angle = rv.r.angle();
  auto& data = rv.r.axis();
  sscanf(input.c_str(), "=[%lf,%lf,%lf,%lf]", &angle, &data[0], &data[1], &data[2]);
  std::cout << "Override RotationVector with [" << angle << "," << data[0] << "," << data[1] << "," << data[2] << "]" << std::endl;
  rv.edited = true;
  return in;
}

int main(int argc, char **argv) {
  pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
    pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
  );

  const int UI_WIDTH = 500;

  pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).
    SetHandler(new pangolin::Handler3D(s_cam));

  // ui
  pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
  pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
  pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
  pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
  pangolin::Var<RotationVector> rotation_vector("ui.rv", RotationVector());
  pangolin::Var<QuaternionDraw> quaternion_lh("ui.q_lh", QuaternionDraw());
  pangolin::Var<RotationVector> rotation_vector_lh("ui.rv_lh", RotationVector());

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& edit_q = quaternion.Get();
    if (edit_q.edited) {
      pangolin::OpenGlMatrix& m = s_cam.GetModelViewMatrix();
      auto R = edit_q.q.toRotationMatrix();
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          m(j, i) = R(i, j);

      std::cout << "OnDrawUpdate QuaternionDraw with [" << edit_q.q.coeffs()[0] << "," << edit_q.q.coeffs()[1] << "," << edit_q.q.coeffs()[2] << "," << edit_q.q.coeffs()[3] << "]" << std::endl;
    }

    auto& edit_rv = rotation_vector.Get();
    if (edit_rv.edited) {
      pangolin::OpenGlMatrix& m = s_cam.GetModelViewMatrix();
      auto R = edit_rv.r.toRotationMatrix();
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          m(j, i) = R(i, j);

      auto angle = edit_rv.r.angle();
      auto data = edit_rv.r.axis();
      std::cout << "OnDrawUpdate RotationVector with [" << angle << "," << data[0] << "," << data[1] << "," << data[2] << "]" << std::endl;
    }

    auto& edit_t = translation_vector.Get();
    if (edit_t.edited) {
      pangolin::OpenGlMatrix& m = s_cam.GetModelViewMatrix();
      RotationMatrix R;
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          R.matrix(i, j) = m(j, i);
      
      Vector3d t = edit_t.trans;
      t = -R.matrix.inverse() * t;
      m(0, 3) = t[0];
      m(1, 3) = t[1];
      m(2, 3) = t[2];
      std::cout << "OnDrawUpdate TranslationVector with [" << edit_t.trans[0] << "," << edit_t.trans[1] << "," << edit_t.trans[2] << "]" << std::endl;
    }

    d_cam.Activate(s_cam);

    pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
    Matrix<double, 4, 4> m = matrix;

    RotationMatrix R;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R.matrix(i, j) = m(j, i);
    rotation_matrix = R;

    TranslationVector t;
    t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
    t.trans = -R.matrix * t.trans;
    translation_vector = t;

    TranslationVector euler;
    euler.trans = R.matrix.eulerAngles(2, 1, 0);
    euler_angles = euler;

    QuaternionDraw quat;
    quat.q = Quaterniond(R.matrix);
    quaternion = quat;

    RotationVector rv;
    rv.r.fromRotationMatrix(R.matrix);
    rotation_vector = rv;

    RotationVector rv_lh;
    rv_lh.r = AngleAxisd(-rv.r.angle(), Vector3d(rv.r.axis()[0], rv.r.axis()[1], -rv.r.axis()[2]));
    rotation_vector_lh = rv_lh;

    QuaternionDraw quat_lh;
    quat_lh.q = rv_lh.r;
    quaternion_lh = quat_lh;

    glColor3f(1.0, 1.0, 1.0);

    pangolin::glDrawColouredCube();
    // draw the original axis
    glLineWidth(3);
    glColor3f(0.8f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(10, 0, 0);
    glColor3f(0.f, 0.8f, 0.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 10, 0);
    glColor3f(0.2f, 0.2f, 1.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glEnd();

    pangolin::FinishFrame();
  }
}
