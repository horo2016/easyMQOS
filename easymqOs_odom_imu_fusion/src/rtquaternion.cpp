

#include "rtquaternion.h"
#include "sensor_msg.h"
#include <math.h>

RTQuaternion::RTQuaternion()
{
    zero();
}

RTQuaternion::RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = scalar;
    m_data[1] = x;
    m_data[2] = y;
    m_data[3] = z;
}


void RTQuaternion::zero()
{
    for (int i = 0; i < 4; i++)
        m_data[i] = 0;
}

void RTQuaternion::normalize()
{
    RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3]);

    if ((length == 0) || (length == 1))
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
    m_data[3] /= length;
}

void RTQuaternion::toEuler(Vector3 *vec)
{
    vec->x =(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
            1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

    vec->y =(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

    vec->z=(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
            1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));
}
//欧拉角转4元数
void RTQuaternion::fromEuler(Vector3 vec)
{
    RTFLOAT cosX2 = cos(vec.x / 2.0f);
    RTFLOAT sinX2 = sin(vec.x / 2.0f);
    RTFLOAT cosY2 = cos(vec.y / 2.0f);
    RTFLOAT sinY2 = sin(vec.y / 2.0f);
    RTFLOAT cosZ2 = cos(vec.z / 2.0f);
    RTFLOAT sinZ2 = sin(vec.z / 2.0f);

    m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;//w
    m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;//x
    m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;//y
    m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;//z
    normalize();
}

RTQuaternion RTQuaternion::conjugate() const
{
    RTQuaternion q;
    q.setScalar(m_data[0]);
    q.setX(-m_data[1]);
    q.setY(-m_data[2]);
    q.setZ(-m_data[3]);
    return q;
}

void RTQuaternion::toAngleVector(RTFLOAT& angle, Vector3 vec)
{
    RTFLOAT halfTheta;
    RTFLOAT sinHalfTheta;

    halfTheta = acos(m_data[0]);
    sinHalfTheta = sin(halfTheta);

    if (sinHalfTheta == 0) {
        vec.x=(1.0);
        vec.y=(0);
        vec.z=(0);
    } else {
        vec.x=(m_data[1] / sinHalfTheta);
        vec.y=(m_data[1] / sinHalfTheta);
        vec.z=(m_data[1] / sinHalfTheta);
    }
    angle = 2.0 * halfTheta;
}

void RTQuaternion::fromAngleVector(const RTFLOAT& angle, const Vector3 vec)
{
    RTFLOAT sinHalfTheta = sin(angle / 2.0);
    m_data[0] = cos(angle / 2.0);
    m_data[1] = vec.x * sinHalfTheta;
    m_data[2] = vec.y * sinHalfTheta;
    m_data[3] = vec.z * sinHalfTheta;
}


