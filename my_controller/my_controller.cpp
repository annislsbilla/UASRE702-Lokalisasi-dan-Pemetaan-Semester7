
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include "iostream"
#include "webots/InertialUnit.hpp"
#include <icecream.hpp>

#include <iostream>
#include <fstream>
#include <vector>

#include </usr/include/eigen3/Eigen/Dense> 
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

struct Point2D
{
  float x, y, theta;
};

namespace kinematic
{
  class Motor
  {
  public:
    Motor() {}
    Motor(float a1, float a2, float a3) : a1(a1 * M_PI / 180), a2(a2 * M_PI / 180), a3(a3 * M_PI / 180)
    {
    }

    float a1, a2, a3;
    float v1, v2, v3;
    float r = 0.07;
  };
}

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
using namespace Eigen;

float glob_yaw = 0.0;

constexpr double MAX_RANGE = 2000.0; // Maximum observation range
constexpr double M_DIST_TH = 4.0; // Mahalanobis distance threshold
constexpr int STATE_SIZE = 3; // [x, y, yaw]
constexpr int LM_SIZE = 2; // Landmark state size [x, y]

Matrix3d Cx = (Matrix3d() << 0.005, 0, 0, 0, 0.25, 0, 0, 0, pow(M_PI / 6, 2)).finished();
Matrix2d Q_sim = (Matrix2d() << 0.0002*8.0/1000.0, 0, 0, pow(M_PI / 960, 2)*8.0/1000.0).finished();
Matrix2d R_sim = (Matrix2d() << 0.002, 0, 0, pow(0.2 * M_PI / 180.0, 2)).finished();


Motor *motors[3];
PositionSensor *pw[3];
InertialUnit *imu;
std::vector<double> ps{0, 0, 0};

std::vector<double> prev_pulse{0, 0, 0};
std::vector<double> v = {0, 0, 0};
kinematic::Motor mtr_(60, 180, 300);
Point2D pos;
Point2D velocity;

kinematic::Motor motor_out;
Point2D vel_world;
double yaw;
Camera *camera = NULL;
const unsigned char *image;

double toRad(double a)
{
  return a * M_PI / 180;
}

void calcOdom()
{
  for (int i = 0; i < 3; i++)
  {
    v[i] = (ps[i] - prev_pulse[i]) * 6;
    // v[i] = tick * 6;
  }

  mtr_.v1 = v[0];
  mtr_.v2 = v[1];
  mtr_.v3 = v[2];
  // IC(mtr_.v1, mtr_.v2, mtr_.v3);
  float heading = 0 * M_PI / 180;
  // IC(velocity.theta);
  // IC(pos.theta);
  velocity.x = ((-sqrt(3) * mtr_.v1) + (sqrt(3) * mtr_.v3)) / 3;
  velocity.y = ((-2 * mtr_.v2) + mtr_.v1 + mtr_.v3) / 3;
  velocity.theta = ((mtr_.v1 + mtr_.v2 + mtr_.v3)) / (3 * 20.8);
  // IC(velocity.x, velocity.y);

  // IC(velocity.theta);

  // velocity.x = ((-1 * mtr_.v1) + (1 * mtr_.v3));
  // velocity.y = (((-sqrt(3) + 2) * mtr_.v1) + ((2 * sqrt(3) - 4) * mtr_.v1) + ((-sqrt(3) + 2) * mtr_.v3));
  // velocity.theta = (((-sqrt(3) + 2) * mtr_.v1) + (((2 * sqrt(3) - 3)) * mtr_.v2) + (-sqrt(3) + 2) * mtr_.v3) / (3 * 20.8);

  // IC(pos.theta);
  double radian = fmod(pos.theta, M_PI * 2);
  // IC(radian);
  if (radian < 0)
    radian += M_PI * 2;

  pos.theta = radian;

  // IC(radian, sin(radian), cos(radian));
  // IC((velocity.x * sin(radian)), (cos(radian) * velocity.y));

  Point2D vel_glob;
  vel_glob.x = (std::cos(radian) * velocity.x) - (std::sin(radian) * velocity.y);
  vel_glob.y = (std::cos(radian) * velocity.y) + (std::sin(radian) * velocity.x);

  // vel_glob.x = (std::cos(yaw) * velocity.x) - (std::sin(yaw) * velocity.y);
  // vel_glob.y = (std::cos(yaw) * velocity.y) + (std::sin(yaw) * velocity.x);

  // IC(velocity.x, velocity.y);

  pos.x += vel_glob.x;
  pos.y += vel_glob.y;
  pos.theta += velocity.theta;
  prev_pulse = ps;
}

void save_to_csv(const std::string& filename, const vector<Vector3d>& hxEst,
                 const vector<Vector3d>& hxTrue, const vector<Vector3d>& hxDR) {
    std::ofstream file(filename);

    // Check if file is open
    if (!file.is_open()) {
        std::cerr << "Failed to open the file!" << std::endl;
        return;
    }

    // Write header to the CSV file
    file << "hxEst_x, hxEst_y, hxEst_theta, hxTrue_x, hxTrue_y, hxTrue_theta, hxDR_x, hxDR_y, hxDR_theta\n";

    // Assuming all vectors have the same size, loop through the vectors
    for (size_t i = 0; i < hxEst.size(); ++i) {
        // Write data from each vector (hxEst, hxTrue, hxDR)
        file << hxEst[i][0] << ", " << hxEst[i][1] << ", " << hxEst[i][2] << ", "
             << hxTrue[i][0] << ", " << hxTrue[i][1] << ", " << hxTrue[i][2] << ", "
             << hxDR[i][0] << ", " << hxDR[i][1] << ", " << hxDR[i][2] << "\n";
    }

    file.close(); // Close the file
    std::cout << "Data saved to " << filename << std::endl;
}

// Utility function to normalize angle between -pi and pi
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Motion model
Vector3d motion_model(const Vector3d& x, const Vector2d& u, double angle) {
    float ddt = 8.0/1000.0;
    // IC(ddt, u[0], u[1]);
    Vector3d x_new = x;
    x_new[0] += ddt * u[0] * cos(angle);
    x_new[1] += ddt * u[0] * sin(angle);
    x_new[2] += ddt * u[1];
    x_new[2] = normalize_angle(x_new[2]);
    return x_new;
}

// Calculate input (velocity and yaw rate)
Vector2d calc_input(float vel, float yawrad) {
    return Vector2d(vel, yawrad); // Velocity: 1 m/s, Yaw rate: 0.1 rad/s
}

// Observation model (simulate landmark observation)
void observation(const Vector3d& xTrue, const MatrixXd& RFID,
                 Vector3d& xDR, Vector2d& ud, vector<Vector3d>& z, float vel, float yawrad, double angle) {
    // Update true position
    Vector3d xTrue_new = motion_model(xTrue, calc_input(vel, yawrad), angle);

    z.clear();
    for (int i = 0; i < RFID.rows(); ++i) {
        // double dx = RFID(i, 0) - xTrue_new[0];
        // double dy = RFID(i, 1) - xTrue_new[1];
        double dx = RFID(i, 0) - (xTrue_new[0]);
        double dy = RFID(i, 1) - (xTrue_new[1]);

        double dist = sqrt(dx * dx + dy * dy);

        if (dist <= MAX_RANGE) {
            double angle = normalize_angle(atan2(dy, dx) - xTrue_new[2]);
            double dist_noise = dist + sqrt(Q_sim(0, 0)) * ((double)rand() / RAND_MAX);
            double angle_noise = angle + sqrt(Q_sim(1, 1)) * ((double)rand() / RAND_MAX);
            z.push_back(Vector3d(dist, angle, i));
        }
    }

    // Add noise to input
    Vector2d input = calc_input(vel, yawrad);
    ud[0] = input[0] + sqrt(R_sim(0, 0)) * ((double)rand() / RAND_MAX);
    ud[1] = input[1] + sqrt(R_sim(1, 1)) * ((double)rand() / RAND_MAX);

    xDR = motion_model(xDR, ud, angle);
}

// Jacobian of the motion model
void jacob_motion(const VectorXd& x, const Vector2d& u, MatrixXd& G, MatrixXd& Fx) {
    Fx = MatrixXd::Zero(STATE_SIZE, x.size());
    Fx.topLeftCorner(STATE_SIZE, STATE_SIZE) = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    float tt = 8.0/1000.0;
    MatrixXd jF(STATE_SIZE, STATE_SIZE);
    jF << 0, 0, -tt * u[0] * sin(x[2]),
          0, 0,  tt * u[0] * cos(x[2]),
          0, 0, 0;

    G = MatrixXd::Identity(x.size(), x.size()) + Fx.transpose() * jF * Fx;
}

// Calculate innovation
void calc_innovation(const Vector2d& lm, const VectorXd& xEst, const MatrixXd& PEst,
                     const Vector2d& z, int LMid, Vector2d& y, MatrixXd& S, MatrixXd& H) {
    Vector2d delta = lm - xEst.head(2);
    double q = delta.squaredNorm();
    double z_angle = normalize_angle(atan2(delta[1], delta[0]) - xEst[2]);

    Vector2d zp(sqrt(q), z_angle);
    y = z - zp;
    y[1] = normalize_angle(y[1]);

    // Calculate Jacobian H
    double sq = sqrt(q);
    MatrixXd G(2, 5); // Jacobian matrix size
    G << -sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1],
         delta[1], -delta[0], -q, -delta[1], delta[0];
    G /= q;

    H = G; // Update H matrix
    S = H * PEst * H.transpose() + Cx.topLeftCorner(2, 2);
}

// Search for corresponding landmark ID
int search_correspond_landmark_id(const VectorXd& xEst, const MatrixXd& PEst,
                                  const Vector2d& zi, double mDistTh) {
    int nLM = (xEst.size() - STATE_SIZE) / LM_SIZE;
    vector<double> distances;

    for (int i = 0; i < nLM; ++i) {
        Vector2d lm = xEst.segment(STATE_SIZE + i * LM_SIZE, LM_SIZE);
        Vector2d y;
        MatrixXd S, H;
        calc_innovation(lm, xEst, PEst, zi, i, y, S, H);
        double dist = y.transpose() * S.inverse() * y;
        distances.push_back(dist);
    }

    distances.push_back(mDistTh); // Add threshold for new landmark
    return min_element(distances.begin(), distances.end()) - distances.begin();
}

// Main EKF-SLAM algorithm
pair<VectorXd, MatrixXd> ekf_slam(VectorXd& xEst, MatrixXd& PEst,
                                  const Vector2d& u, const vector<Vector3d>& z, double angle) {
    // Predict step
    xEst.head(STATE_SIZE) = motion_model(xEst.head(STATE_SIZE), u, angle);
    MatrixXd G, Fx;
    jacob_motion(xEst, u, G, Fx);
    PEst = G * PEst * G.transpose() + Fx.transpose() * Cx * Fx;

    // Update step
    for (const auto& obs : z) {
        Vector2d zi = obs.head(2);
        int min_id = search_correspond_landmark_id(xEst, PEst, zi, M_DIST_TH);

        int nLM = (xEst.size() - STATE_SIZE) / LM_SIZE;
        IC(nLM);
        if (min_id == nLM) {
            // Add new landmark
            VectorXd xAug = VectorXd::Zero(xEst.size() + LM_SIZE);
            xAug.head(xEst.size()) = xEst;
            xAug.tail(LM_SIZE) = xEst.head(2) + Vector2d(zi[0] * cos(xEst[2] + zi[1]), zi[0] * sin(xEst[2] + zi[1]));
            xEst = xAug;

            MatrixXd PAug = MatrixXd::Zero(PEst.rows() + LM_SIZE, PEst.cols() + LM_SIZE);
            PAug.topLeftCorner(PEst.rows(), PEst.cols()) = PEst;
            PAug.bottomRightCorner(LM_SIZE, LM_SIZE) = MatrixXd::Identity(LM_SIZE, LM_SIZE);
            PEst = PAug;
        }

        Vector2d y;
        MatrixXd S, H;
        Vector2d lm = xEst.segment(STATE_SIZE + min_id * LM_SIZE, LM_SIZE);
        calc_innovation(lm, xEst, PEst, zi, min_id, y, S, H);

        MatrixXd K = PEst * H.transpose() * S.inverse();
        xEst += K * y;
        PEst = (MatrixXd::Identity(xEst.size(), xEst.size()) - K * H) * PEst;
    }
     IC(xEst);
    return {xEst, PEst};
}
int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  motors[0] = new Motor("wheel1");
  motors[0]->setPosition(INFINITY);
  motors[1] = new Motor("wheel2");
  motors[1]->setPosition(INFINITY);
  motors[2] = new Motor("wheel3");
  motors[2]->setPosition(INFINITY);

  pw[0] = robot->getPositionSensor("pw1");
  pw[0]->enable(timeStep);
  pw[1] = robot->getPositionSensor("pw2");
  pw[1]->enable(timeStep);
  pw[2] = robot->getPositionSensor("pw3");
  pw[2]->enable(timeStep);
  imu = robot->getInertialUnit("IMU");
  imu->enable(timeStep);
  camera = new Camera("camera");
  camera->enable(timeStep);
  camera->recognitionEnable(timeStep);
  
 // RFID positions
  // MatrixXd RFID(4, 2);
  // RFID.setZero();
  // RFID << 10.0, -2.0;

  // Initial states
  VectorXd xEst = VectorXd::Zero(STATE_SIZE);
  Vector3d xTrue = Vector3d::Zero();
  MatrixXd PEst = MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  Vector3d xDR = Vector3d::Zero();

  vector<Vector3d> hxEst, hxTrue, hxDR;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller

  // ps.reserve(3);

  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    const double *orientation = imu->getRollPitchYaw();
    yaw = orientation[2];

    Point2D velocity;
    velocity.x = 20;
    velocity.y = 0;
    velocity.theta = 0.2;

    vel_world.x = (cos(pos.theta) * velocity.x + sin(pos.theta ) * velocity.y);
    vel_world.y = (-sin(pos.theta) * velocity.x + cos(pos.theta) * velocity.y);
    
    // IC(yaw);
    // IC(pos.theta);

    // IC(vel_world.y);

    vel_world.theta = velocity.theta;

    motor_out.v1 = ((-sqrt(3) * vel_world.x / 2) + (vel_world.y / 2) + (20.8 * vel_world.theta)) / 6;
    motor_out.v2 = (-vel_world.y + (20.8 * vel_world.theta)) / 6;
    motor_out.v3 = ((sqrt(3) * vel_world.x / 2) + (vel_world.y / 2) + (20.8 * vel_world.theta)) / 6;


    motors[0]->setVelocity(motor_out.v1);
    motors[1]->setVelocity(motor_out.v2);
    motors[2]->setVelocity(motor_out.v3);

    for (int i = 0; i < 3; i++)
    {
      // motors[i]->setVelocity(2);
      ps[i] = pw[i]->getValue();
    }

    calcOdom();
    IC(pos.x, pos.y, pos.theta);
    xTrue << pos.x/100.0,
            pos.y/100.0,
            pos.theta;
    
    IC(motors[0]->getVelocity(), motors[1]->getVelocity(), motors[2]->getVelocity());
    
    int object_count = camera->getRecognitionNumberOfObjects();
    const webots::CameraRecognitionObject *objects = camera->getRecognitionObjects();
    
    MatrixXd RFID(object_count, 2);
    RFID.setZero();
    float kn = yaw;
    glob_yaw = yaw;
    
    for (int i = 0; i < object_count; i++) {
      const webots::CameraRecognitionObject &obj = objects[i];
      RFID(i, 0) = pos.x/100 + (-sin(kn) * obj.position[1] + cos(kn) * obj.position[2]);
      RFID(i, 1) = pos.y/100 + (cos(kn) * obj.position[1] + sin(kn) * obj.position[2]);
      IC(RFID(i, 0), RFID(i, 1));
    }
    
    float vvx = velocity.x/100;
    float vvy = velocity.y/100;
    
    float vv = sqrt(vvx * vvx + vvy * vvy);
    double angle = atan2(vvy, vvx);
    // IC(angle);
    float yws = (velocity.theta);
    // IC(yws);

    Vector2d u = calc_input(vv, yws);
    Vector2d ud;
    vector<Vector3d> z;
    observation(xTrue, RFID, xDR, ud, z, vv, yws, angle);

    tie(xEst, PEst) = ekf_slam(xEst, PEst, u, z, angle);
    // IC(hxTrue);
    // Store history
    hxEst.push_back(xEst.head(STATE_SIZE));
    hxTrue.push_back(xTrue);
    hxDR.push_back(xDR);
    
    
    // save_to_csv("data", hxEst, hxTrue, hxDR);
};

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
