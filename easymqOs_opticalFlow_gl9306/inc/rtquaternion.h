
#ifndef _RTQUATERNION_H_
#define _RTQUATERNION_H_
#include "odometry.h"
#include <string.h>
typedef float RTFLOAT;
class RTQuaternion
{
public:
    RTQuaternion();
    RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z);

    
  
    void normalize();
    void toEuler(Vector3 vec);
    void fromEuler(Vector3 vec);
    RTQuaternion conjugate() const;
    void toAngleVector(RTFLOAT& angle, Vector3 vec);
    void fromAngleVector(const RTFLOAT& angle, const Vector3 vec);

    void zero();
    const char *display();

    inline RTFLOAT scalar() const { return m_data[0]; }
    inline RTFLOAT x() const { return m_data[1]; }
    inline RTFLOAT y() const { return m_data[2]; }
    inline RTFLOAT z() const { return m_data[3]; }
    inline RTFLOAT data(const int i) const { return m_data[i]; }

    inline void setScalar(const RTFLOAT val) { m_data[0] = val; }
    inline void setX(const RTFLOAT val) { m_data[1] = val; }
    inline void setY(const RTFLOAT val) { m_data[2] = val; }
    inline void setZ(const RTFLOAT val) { m_data[3] = val; }
    inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }
    inline void fromArray(RTFLOAT *val) { memcpy(m_data, val, 4 * sizeof(RTFLOAT)); }
    inline void toArray(RTFLOAT *val) const { memcpy(val, m_data, 4 * sizeof(RTFLOAT)); }

private:
    RTFLOAT m_data[4];
};


#endif