#include <GL/glut.h>
#include <GL/glu.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <array>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <cstdint>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// ============================================================
//  Quaternion utilities
// ============================================================
struct Quat {
  // store as (w, x, y, z)
  double w{1}, x{0}, y{0}, z{0};
};

static Quat normalize(const Quat& q) {
  double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n <= 0) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static Quat mul(const Quat& a, const Quat& b) {
  // Hamilton product: a*b
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static Quat fromAxisAngle(double ax, double ay, double az, double angle_rad) {
  double n = std::sqrt(ax*ax + ay*ay + az*az);
  if (n <= 1e-12) return {1,0,0,0};
  ax /= n; ay /= n; az /= n;
  double s = std::sin(angle_rad * 0.5);
  return normalize({std::cos(angle_rad * 0.5), ax*s, ay*s, az*s});
}

static std::array<float, 16> quatToMat4(const Quat& q_in) {
  Quat q = normalize(q_in);
  const double w = q.w, x = q.x, y = q.y, z = q.z;

  const double xx = x*x, yy = y*y, zz = z*z;
  const double xy = x*y, xz = x*z, yz = y*z;
  const double wx = w*x, wy = w*y, wz = w*z;

  double r00 = 1.0 - 2.0*(yy + zz);
  double r01 = 2.0*(xy - wz);
  double r02 = 2.0*(xz + wy);

  double r10 = 2.0*(xy + wz);
  double r11 = 1.0 - 2.0*(xx + zz);
  double r12 = 2.0*(yz - wx);

  double r20 = 2.0*(xz - wy);
  double r21 = 2.0*(yz + wx);
  double r22 = 1.0 - 2.0*(xx + yy);

  std::array<float,16> m{};
  // Column-major for OpenGL: m[col*4 + row]
  m[0]  = (float)r00; m[4]  = (float)r01; m[8]  = (float)r02; m[12] = 0.0f;
  m[1]  = (float)r10; m[5]  = (float)r11; m[9]  = (float)r12; m[13] = 0.0f;
  m[2]  = (float)r20; m[6]  = (float)r21; m[10] = (float)r22; m[14] = 0.0f;
  m[3]  = 0.0f;       m[7]  = 0.0f;       m[11] = 0.0f;       m[15] = 1.0f;
  return m;
}

// ============================================================
//  Visualization
// ============================================================
static Quat g_q;                 // current orientation
static std::mutex g_q_mtx;
static float g_rot_step = 2.0f;  // degrees per keypress
static int g_width = 900, g_height = 600;

static std::atomic<bool> g_running{true};
static std::thread g_imuThread;  // so we can join on exit

static void drawAxes(float len=1.2f) {
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(len,0,0);
  glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,len,0);
  glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,len);
  glEnd();
}

static void drawCubeWire(float s=1.0f) {
  glColor3f(1,1,1);
  glutWireCube(s);
}

static void drawCubeSolid(float s=1.0f) {
  glColor3f(0.7f, 0.7f, 0.9f);
  glutSolidCube(s);
}

static void printQuat() {
  std::lock_guard<std::mutex> lk(g_q_mtx);
  Quat q = normalize(g_q);
  std::cout << "q = [w x y z] = "
            << q.w << " " << q.x << " " << q.y << " " << q.z << "\n";
}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Camera
  gluLookAt(0, 0, 3.0,
            0, 0, 0,
            0, 1, 0);

  // World axes (fixed)
  drawAxes();

  // Copy quaternion
  Quat q_copy;
  {
    std::lock_guard<std::mutex> lk(g_q_mtx);
    q_copy = g_q;
  }

  // Apply quaternion rotation to cube
  auto M = quatToMat4(q_copy);
  glPushMatrix();
  glMultMatrixf(M.data());

  // Cube axes (rotating with cube)
  drawAxes(0.9f);

  // Cube
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  drawCubeSolid(1.0f);
  glDisable(GL_POLYGON_OFFSET_FILL);

  drawCubeWire(1.01f);
  glPopMatrix();

  glutSwapBuffers();
}

static void idle() {
  // Redraw continuously; avoids calling GLUT functions from the IMU thread.
  glutPostRedisplay();
}

static void reshape(int w, int h) {
  g_width = w; g_height = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (h>0)? (double)w/(double)h : 1.0, 0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
}

static void applyIncrementalRotation(double ax, double ay, double az, double deg) {
  double rad = deg * M_PI / 180.0;
  Quat dq = fromAxisAngle(ax, ay, az, rad);
  std::lock_guard<std::mutex> lk(g_q_mtx);
  g_q = normalize(mul(dq, g_q));
}

static void shutdownAndExit(int code) {
  g_running.store(false);
  if (g_imuThread.joinable()) g_imuThread.join();
  std::exit(code);
}

static void keyboard(unsigned char key, int, int) {
  switch (key) {
    case 27: // ESC
    case 'q':
      shutdownAndExit(0);

    case 'r':
      {
        std::lock_guard<std::mutex> lk(g_q_mtx);
        g_q = {1,0,0,0};
      }
      printQuat();
      break;

    // Optional manual rotation still supported
    case 'e': applyIncrementalRotation(0,0,1, +g_rot_step); break;
    case 'c': applyIncrementalRotation(0,0,1, -g_rot_step); break;
    case 'w': applyIncrementalRotation(1,0,0, +g_rot_step); break;
    case 's': applyIncrementalRotation(1,0,0, -g_rot_step); break;
    case 'a': applyIncrementalRotation(0,1,0, +g_rot_step); break;
    case 'd': applyIncrementalRotation(0,1,0, -g_rot_step); break;

    case '+': g_rot_step = std::min(20.0f, g_rot_step + 0.5f); break;
    case '-': g_rot_step = std::max(0.5f,  g_rot_step - 0.5f); break;
    default: break;
  }
}

static void initGL() {
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
}

// ============================================================
//  IMU + Madgwick (embedded in same file)
// ============================================================
static inline int16_t combine(uint8_t lo, uint8_t hi) {
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

static inline float invSqrt(float x) {
  return 1.0f / std::sqrt(x);
}

class I2CDevice {
public:
  I2CDevice(const char* dev, uint8_t addr) {
    fd = open(dev, O_RDWR);
    if (fd < 0) throw std::runtime_error("Failed to open I2C device");

    if (ioctl(fd, I2C_SLAVE, addr) < 0)
      throw std::runtime_error("Failed to set I2C slave address");
  }

  ~I2CDevice() { if (fd >= 0) close(fd); }

  uint8_t readReg(uint8_t reg) {
    if (write(fd, &reg, 1) != 1) throw std::runtime_error("I2C write(reg) failed");
    uint8_t v{};
    if (read(fd, &v, 1) != 1) throw std::runtime_error("I2C read(reg) failed");
    return v;
  }

  void writeReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    if (write(fd, buf, 2) != 2) throw std::runtime_error("I2C write(reg,val) failed");
  }

  void readBytes(uint8_t reg, uint8_t* buf, size_t n) {
    if (write(fd, &reg, 1) != 1) throw std::runtime_error("I2C write(reg) failed");
    if (read(fd, buf, n) != (ssize_t)n) throw std::runtime_error("I2C read(bytes) failed");
  }

private:
  int fd{-1};
};

class LIS3MDL {
public:
  LIS3MDL(const char* dev="/dev/i2c-1", uint8_t addr=0x1e) : i2c(dev, addr) {}

  bool begin() {
    uint8_t who = i2c.readReg(0x0F);
    if (who != 0x3D) {
      std::cerr << "LIS3MDL WHO_AM_I mismatch: 0x" << std::hex << (int)who << std::dec << "\n";
      return false;
    }
    // CTRL_REG1: Ultra-high perf XY, ODR=10Hz -> 0x70
    i2c.writeReg(0x20, 0x70);
    // CTRL_REG2: FS=±4 gauss -> 0x00
    i2c.writeReg(0x21, 0x00);
    // CTRL_REG3: Continuous mode -> 0x00
    i2c.writeReg(0x22, 0x00);
    // CTRL_REG4: Ultra-high perf Z -> 0x0C
    i2c.writeReg(0x23, 0x0C);
    return true;
  }

  void readRaw(int16_t& mx, int16_t& my, int16_t& mz) {
    uint8_t b[6];
    i2c.readBytes(uint8_t(0x28 | 0x80), b, 6);
    mx = combine(b[0], b[1]);
    my = combine(b[2], b[3]);
    mz = combine(b[4], b[5]);
  }

private:
  I2CDevice i2c;
};

class Madgwick {
public:
  explicit Madgwick(float beta_=0.10f) : beta(beta_) {}

  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz,
              float dt)
  {
    float q1=q.w, q2=q.x, q3=q.y, q4=q.z;

    float norm = ax*ax + ay*ay + az*az;
    if (norm <= 0.0f) return;
    norm = invSqrt(norm);
    ax*=norm; ay*=norm; az*=norm;

    norm = mx*mx + my*my + mz*mz;
    if (norm <= 0.0f) return;
    norm = invSqrt(norm);
    mx*=norm; my*=norm; mz*=norm;

    float _2q1mx = 2.0f*q1*mx;
    float _2q1my = 2.0f*q1*my;
    float _2q1mz = 2.0f*q1*mz;
    float _2q2mx = 2.0f*q2*mx;

    float _2q1 = 2.0f*q1;
    float _2q2 = 2.0f*q2;
    float _2q3 = 2.0f*q3;
    float _2q4 = 2.0f*q4;

    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q1q4 = q1*q4;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q2q4 = q2*q4;
    float q3q3 = q3*q3;
    float q3q4 = q3*q4;
    float q4q4 = q4*q4;

    float hx = mx*q1q1 - _2q1my*q4 + _2q1mz*q3 + mx*q2q2 + _2q2*my*q3 + _2q2*mz*q4 - mx*q3q3 - mx*q4q4;
    float hy = _2q1mx*q4 + my*q1q1 - _2q1mz*q2 + _2q2mx*q3 - my*q2q2 + my*q3q3 + _2q3*mz*q4 - my*q4q4;
    float _2bx = std::sqrt(hx*hx + hy*hy);
    float _2bz = -_2q1mx*q3 + _2q1my*q2 + mz*q1q1 + _2q2mx*q4 - mz*q2q2 + _2q3*my*q4 - mz*q3q3 + mz*q4q4;
    float _4bx = 2.0f*_2bx;
    float _4bz = 2.0f*_2bz;

    float s1 = -_2q3*(2.0f*(q2q4 - q1q3) - ax) + _2q2*(2.0f*(q1q2 + q3q4) - ay)
             - _2bz*q3*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (-_2bx*q4 + _2bz*q2)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + _2bx*q3*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s2 =  _2q4*(2.0f*(q2q4 - q1q3) - ax) + _2q1*(2.0f*(q1q2 + q3q4) - ay)
             - 4.0f*q2*(1.0f - 2.0f*(q2q2 + q3q3) - az)
             + _2bz*q4*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (_2bx*q3 + _2bz*q1)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + (_2bx*q4 - _4bz*q2)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s3 = -_2q1*(2.0f*(q2q4 - q1q3) - ax) + _2q4*(2.0f*(q1q2 + q3q4) - ay)
             - 4.0f*q3*(1.0f - 2.0f*(q2q2 + q3q3) - az)
             + (-_4bx*q3 - _2bz*q1)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (_2bx*q2 + _2bz*q4)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + (_2bx*q1 - _4bz*q3)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    float s4 =  _2q2*(2.0f*(q2q4 - q1q3) - ax) + _2q3*(2.0f*(q1q2 + q3q4) - ay)
             + (-_4bx*q4 + _2bz*q2)*(_2bx*(0.5f - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
             + (-_2bx*q1 + _2bz*q3)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
             + _2bx*q2*(_2bx*(q1q3 + q2q4) + _2bz*(0.5f - q2q2 - q3q3) - mz);

    norm = invSqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    s1*=norm; s2*=norm; s3*=norm; s4*=norm;

    float qDot1 = 0.5f*(-q2*gx - q3*gy - q4*gz) - beta*s1;
    float qDot2 = 0.5f*( q1*gx + q3*gz - q4*gy) - beta*s2;
    float qDot3 = 0.5f*( q1*gy - q2*gz + q4*gx) - beta*s3;
    float qDot4 = 0.5f*( q1*gz + q2*gy - q3*gx) - beta*s4;

    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    norm = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q.w=q1*norm; q.x=q2*norm; q.y=q3*norm; q.z=q4*norm;
  }

  struct { float w=1, x=0, y=0, z=0; } q;
  float beta;
};

static void imuLoopThread() {
  try {
    I2CDevice imu("/dev/i2c-1", 0x6b);
    LIS3MDL mag;

    uint8_t who = imu.readReg(0x0F);
    if (who != 0x69) throw std::runtime_error("LSM6DS33 WHO_AM_I mismatch");

    imu.writeReg(0x10, 0x40); // accel 104Hz ±2g
    imu.writeReg(0x11, 0x40); // gyro  104Hz 245dps

    if (!mag.begin()) throw std::runtime_error("LIS3MDL init failed");

    const float ACCEL_G_PER_LSB  = 0.000061f;
    const float GYRO_DPS_PER_LSB = 0.00875f;
    const float DEG2RAD          = (float)M_PI / 180.0f;

    Madgwick f(0.10f);
    auto last = std::chrono::steady_clock::now();

    while (g_running.load()) {
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<float> dt = now - last;
      last = now;

      float dts = dt.count();
      if (dts <= 0.0f || dts > 0.2f) dts = 0.01f;

      uint8_t b[12];
      imu.readBytes(0x22, b, 12);

      int16_t gx_raw = combine(b[0], b[1]);
      int16_t gy_raw = combine(b[2], b[3]);
      int16_t gz_raw = combine(b[4], b[5]);

      int16_t ax_raw = combine(b[6],  b[7]);
      int16_t ay_raw = combine(b[8],  b[9]);
      int16_t az_raw = combine(b[10], b[11]);

      int16_t mx_raw, my_raw, mz_raw;
      mag.readRaw(mx_raw, my_raw, mz_raw);

      float ax = ax_raw * ACCEL_G_PER_LSB;
      float ay = ay_raw * ACCEL_G_PER_LSB;
      float az = az_raw * ACCEL_G_PER_LSB;

      float gx = gx_raw * GYRO_DPS_PER_LSB * DEG2RAD;
      float gy = gy_raw * GYRO_DPS_PER_LSB * DEG2RAD;
      float gz = gz_raw * GYRO_DPS_PER_LSB * DEG2RAD;

      f.update(gx, gy, gz, ax, ay, az,
               (float)mx_raw, (float)my_raw, (float)mz_raw, dts);

      {
        std::lock_guard<std::mutex> lk(g_q_mtx);
        g_q = normalize({(double)f.q.w, (double)f.q.x, (double)f.q.y, (double)f.q.z});
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~100Hz
    }
  } catch (const std::exception& e) {
    std::cerr << "IMU thread error: " << e.what() << "\n";
    g_running.store(false);
  }
}

// ============================================================
//  main (FIXED):
//    - GLUT initialized BEFORE starting IMU thread
//    - No glutPostRedisplay from IMU thread
// ============================================================
int main(int argc, char** argv) {
  if (argc == 5) {
    std::lock_guard<std::mutex> lk(g_q_mtx);
    g_q.w = std::atof(argv[1]);
    g_q.x = std::atof(argv[2]);
    g_q.y = std::atof(argv[3]);
    g_q.z = std::atof(argv[4]);
    g_q = normalize(g_q);
  } else {
    std::lock_guard<std::mutex> lk(g_q_mtx);
    g_q = {1,0,0,0};
  }

  std::cout << "Cube Quaternion Viewer (IMU-driven)\n"
            << "Controls:\n"
            << "  r: reset quaternion\n"
            << "  q or ESC: quit\n"
            << "Optional manual rotate still works:\n"
            << "  w/s pitch, a/d yaw, e/c roll, +/- step\n";
  printQuat();

  // 1) Init GLUT / Create window first
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(g_width, g_height);
  glutCreateWindow("Quaternion Cube Orientation (IMU + Madgwick)");

  initGL();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  // 2) Start IMU thread AFTER GLUT is ready
  g_imuThread = std::thread(imuLoopThread);

  // 3) Enter GLUT loop
  glutMainLoop();

  // If glutMainLoop returns (some freeglut builds can), clean up:
  shutdownAndExit(0);
  return 0;
}
