#include <Arduino.h>
#include <vector3.h>

Vector3::Vector3(float x, float y, float z) {
    this->set(x, y, z);
}

bool Vector3::operator==(const Vector3& vector) const {
    return this->x == vector.x && this->y == vector.y && this->z == vector.z;
}

bool Vector3::operator!=(const Vector3& vector) const {
    return !(*this == vector);
}

const Vector3 Vector3::operator+() const {
    return *this;
}

const Vector3 Vector3::operator-() const {
    return Vector3(-this->x, -this->y, -this->z);
}

Vector3& Vector3::operator+=(const Vector3& vector) {
    this->x += vector.x;
    this->y += vector.y;
    this->z += vector.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& vector) {
    this->x -= vector.x;
    this->y -= vector.y;
    this->z -= vector.z;
    return *this;
}

Vector3& Vector3::operator*=(float value) {
    this->x *= value;
    this->y *= value;
    this->z *= value;
    return *this;
}

Vector3& Vector3::operator/=(float value) {
    this->x /= value;
    this->y /= value;
    this->z /= value;
    return *this;
}

const Vector3 Vector3::operator+(const Vector3& vector) const {
    return Vector3(*this) += vector;
}

const Vector3 Vector3::operator-(const Vector3& vector) const {
    return Vector3(*this) -= vector;
}

const Vector3 Vector3::operator*(float value) const {
    return Vector3(*this) *= value;
}

const Vector3 Vector3::operator/(float value) const {
    return Vector3(*this) /= value;
}

Vector3& Vector3::set(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
    return *this;
}

float Vector3::squaredLength() const {
    return this->x * this->x + this->y * this->y + this->z * this->z;
}

float Vector3::length() const {
    return sqrt(this->squaredLength());
}

const Vector3 Vector3::norm() const {
    const float length = this->length();
    return length == 0 ? Vector3() : *this / length;
}

float Vector3::dot(const Vector3& vector) const {
    return this->x * vector.x + this->y * vector.y + this->z * vector.z;
}

const Vector3 Vector3::cross(const Vector3& vector) const {
    return Vector3(this->y * vector.z - this->z * vector.y, this->z * vector.x - this->x * vector.z,
        this->x * vector.y - this->y * vector.x);
}

const Vector3 Vector3::minEdge(Vector3& vector) const {
    return Vector3(min(this->x, vector.x), min(this->y, vector.y), min(this->z, vector.z));
}

const Vector3 Vector3::maxEdge(Vector3& vector) const {
    return Vector3(max(this->x, vector.x), max(this->y, vector.y), max(this->z, vector.z));
}
