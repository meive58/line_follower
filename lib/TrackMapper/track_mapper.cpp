/**
 * @file track_mapper.cpp
 * @brief Implementação do mapeamento da pista
 */

#include "track_mapper.h"

TrackMapper::TrackMapper()
    : _left(nullptr), _right(nullptr), _mpu(nullptr),
      _wheelDiameter(0),
      _count(0), _x(0), _y(0),
      _lastSavedDistance(0), _lastDistance(0) {
}

void TrackMapper::init(Encoder* left, Encoder* right, MPU6050Sensor* mpu, float wheelDiameterMm) {
    _left = left;
    _right = right;
    _mpu = mpu;
    _wheelDiameter = wheelDiameterMm;
    reset();
}

void TrackMapper::reset() {
    _left->resetCount();
    _right->resetCount();
    _mpu->resetYaw();
    _count = 0;
    _x = 0.0f;
    _y = 0.0f;
    _lastSavedDistance = 0.0f;
    _lastDistance = 0.0f;
}

float TrackMapper::estimateCurveRadius() {
    // Velocidade linear do centro do robô (mm/s)
    float avgRPM = (_left->getRPM() + _right->getRPM()) / 2.0f;
    float speedMmS = fabsf(avgRPM) * PI * _wheelDiameter / 60.0f;

    float gyroZ = _mpu->getAngularVelocityZ();  // graus/s
    if (fabsf(gyroZ) < GYRO_STRAIGHT_THRESHOLD_DPS || speedMmS < 1.0f) {
        return INFINITY;
    }

    return speedMmS / fabsf(gyroZ * DEG_TO_RAD);
}

float TrackMapper::getDistanceTraveled() {
    return (_left->getDistance(_wheelDiameter) +
            _right->getDistance(_wheelDiameter)) / 2.0f;
}

void TrackMapper::update() {
    float distance = getDistanceTraveled();

    // Odometria: integra posição usando o yaw atual
    float yaw = _mpu->getYaw();
    float step = distance - _lastDistance;
    _lastDistance = distance;
    _x += step * cosf(yaw * DEG_TO_RAD);
    _y += step * sinf(yaw * DEG_TO_RAD);

    if (distance - _lastSavedDistance < TRACK_SAMPLE_DISTANCE_MM) return;
    if (_count >= TRACK_MAX_POINTS) return;

    _lastSavedDistance = distance;
    _points[_count++] = { distance, yaw, estimateCurveRadius(), _x, _y };
}

uint16_t TrackMapper::getPointCount() {
    return _count;
}

const TrackPoint& TrackMapper::getPoint(uint16_t index) {
    return _points[index];
}

void TrackMapper::printMap(Print& out) {
    out.println("distancia_mm,yaw_graus,raio_mm,x_mm,y_mm");

    for (uint16_t i = 0; i < _count; i++) {
        const TrackPoint& p = _points[i];
        if (isinf(p.radius)) {
            out.printf("%.1f,%.1f,inf,%.1f,%.1f\n", p.distance, p.yaw, p.x, p.y);
        } else {
            out.printf("%.1f,%.1f,%.1f,%.1f,%.1f\n", p.distance, p.yaw, p.radius, p.x, p.y);
        }
    }
}
