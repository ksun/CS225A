// *******************************************************************
// PrQuaternion.h
//
// This class provides a quaternion, specialized for rotation.
//
// A quaternion is similar to a complex number, except that there are
// three separate imaginary numbers i, j, and k.  Multiplication is
// defined by the rule "i*i = j*j = k*k = i*j*k = -1"; it is
// associative but not commutative.  Other rules (such as i*j = k and
// j*i = -k) can de derived from this basic definition.
//
// A quaternion can also be thought of as a combination of a scalar
// and a 3x1 vector (w, v).  In this case, multiplication is defined
// as (w1, v1) * (w2, v2) = ( w1*w2 - v1.v2 , w1*v2 + w2*v1 + v1Xv2 ).
// "v1.v2" and "v1Xv2" represents the dot-product and the cross-
// product of the vectors, respectively.
//
// Unit quaternions are useful as Euler Parameters, which can rotate a
// vector around an axis of rotation r by an angle u.  We create a
// unit quaternion q = (cos(u/2), sin(u/2) * r) and its conjugate q' =
// (cos(u/2), -sin(u/2) * r).  Now, if we take an arbitrary quaternion
// (w,v), the product q * (w,v) * q' rotates v without changing w.  (w
// is usually 0, anyways.)
//
// The four quaternion parameters are often called lambda_0, lambda_1,
// lambda_2, and lambda_3 (pretend that you're looking at Greek
// letters with subscripts), where lambda_0 is the scalar part and
// (lambda_1, lambda_2, lambda_3) is the vector.
//
// Quaternions are more compact than rotation matrices, and easier to
// normalize with minimal error.  If we want to combine several
// rotations, it's faster to multiply two quaternions than two
// rotation matrices.  However, if we want to rotate a vector, it's
// faster to multiply by a rotation matrix than to apply a quaternion.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Added comments
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#ifndef _PrQuaternion_h
#define _PrQuaternion_h

#include "PrGlobalDefn.h"
#include "PrVector3.h"
#include "PrMatrix3.h"

// ===================================================================
// PrQuaternion class declaration
// ===================================================================
class PrQuaternion
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrQuaternion() : m_scalar( 1.0 ) {} // initialized to [I]
  PrQuaternion( const PrQuaternion& rhs );
  PrQuaternion( Float scalar, const PrVector& vector );
  PrQuaternion( Float scalar, Float vector0, Float vector1, Float vector2 );
  PrQuaternion( const PrVector& axis, Float angle );
  PrQuaternion( const PrVector& vFrom, const PrVector& vTo );
  PrQuaternion( const PrMatrix& rotationMatrix );
  ~PrQuaternion() {}

  // -----------------------------------------------------------------
  // Operations that load data into quaternion
  // -----------------------------------------------------------------

  PrQuaternion& operator=( const PrQuaternion& rhs );

  void values( Float scalar, Float vector0, Float vector1, Float vector2 );
  void values( const PrVector& axis, Float angle );
  void values( const PrVector& vFrom, const PrVector& vTo );
  void values( const PrMatrix& rotationMatrix );

  const PrQuaternion& identity();

  // -----------------------------------------------------------------
  // Lookup and conversion operations
  // -----------------------------------------------------------------

  const PrVector3& v() const { return m_vector; }
  Float w() const            { return m_scalar; }

  // convert to axis-angle notation
  void axisAngle( PrVector& axis, Float& angle ) const;

  // convert to Euler angles (x-convention)
  void eulerAngles( Float& psi, Float& theta, Float& phi ) const;
  void loadFromEulerAngles( Float psi, Float theta, Float phi );

  // convert to Euler angles (y-convention)
  void eulerAnglesY( Float& psi, Float& theta, Float& phi ) const;
  void loadFromEulerAnglesY( Float psi, Float theta, Float phi );

  // Returns rotation matrix.
  PrMatrix3 rotationMatrix() const;
  void rotationMatrix( PrMatrix& dest ) const;
  PrMatrix3 matrix() const;
  void matrix( PrMatrix& dest ) const      { rotationMatrix( dest ); }

  // One column of rotation matrix
  PrVector3 column( int col ) const;
  void column( int col, PrVector& dest ) const;

  // -----------------------------------------------------------------
  // Basic arithmetic operations
  // -----------------------------------------------------------------

  int operator==( const PrQuaternion& rhs );

  PrQuaternion operator-() const;
  void negate( PrQuaternion& dest ) const;

  // inversion operator only works for unit quaternions
  PrQuaternion operator~() const;
  void inverse( PrQuaternion& dest ) const;

  PrQuaternion operator+( const PrQuaternion& rhs ) const;
  void add( const PrQuaternion& rhs, PrQuaternion& dest ) const;

  PrQuaternion operator-( const PrQuaternion& rhs ) const;
  void subtract( const PrQuaternion& rhs, PrQuaternion& dest ) const;

  PrQuaternion operator*( const PrQuaternion& rhs ) const;
  void multiply( const PrQuaternion& rhs, PrQuaternion& dest ) const;

  PrVector3 operator*( const PrVector& rhs ) const;
  void multiply( const PrVector& rhs, PrVector& dest ) const;

  PrQuaternion operator*( Float rhs ) const;
  void multiply( Float rhs, PrQuaternion& dest ) const;

  PrQuaternion operator/( Float rhs ) const;
  void divide( Float rhs, PrQuaternion& dest ) const { multiply(1/rhs, dest); }

  PrQuaternion& operator+=( const PrQuaternion& rhs );
  PrQuaternion& operator-=( const PrQuaternion& rhs );
  PrQuaternion& operator*=( const PrQuaternion& rhs );
  PrQuaternion& operator*=( Float rhs );
  PrQuaternion& operator/=( Float rhs );

  Float dot( const PrQuaternion& rhs ) const;
  void normalize();
  PrQuaternion& unit() { normalize(); return *this; }

  // -----------------------------------------------------------------
  // Miscellaneous
  // -----------------------------------------------------------------

  // dPhi = E_inv * (this - qd)
  PrVector3 angularError( const PrQuaternion& qd ) const;
  void angularError( const PrQuaternion& qd, PrVector& dPhi ) const;

  // dq = E * omega
  PrQuaternion velocity( const PrVector& omega ) const;
  void velocity( const PrVector& omega, PrQuaternion& dq ) const;

  // interpolates btw quaternions to the specified fraction using spherical interpolation
  static PrQuaternion interpolate( const PrQuaternion& qA, const PrQuaternion& qB, double fraction );

  void display( const char* name = NULL ) const;

private:
  Float m_scalar;
  PrVector3 m_vector;
};

// ===================================================================
// Inline methods
// ===================================================================

inline PrQuaternion::PrQuaternion( const PrQuaternion& rhs )
  : m_scalar( rhs.m_scalar ), m_vector( rhs.m_vector )
{
}

inline PrQuaternion::PrQuaternion( Float scalar, const PrVector& vector )
  : m_scalar( scalar ), m_vector( vector )
{
}

inline PrQuaternion::PrQuaternion( Float scalar, Float vector0,
                                   Float vector1, Float vector2 )
{
  values( scalar, vector0, vector1, vector2 );
}

inline PrQuaternion::PrQuaternion( const PrVector& axis, Float angle )
{
  values( axis, angle );
}

inline PrQuaternion::PrQuaternion( const PrVector& vFrom, const PrVector& vTo )
{
  values( vFrom, vTo );
}

inline PrQuaternion::PrQuaternion( const PrMatrix& rotationMatrix )
{
  values( rotationMatrix );
}

inline PrQuaternion& PrQuaternion::operator=( const PrQuaternion& rhs )
{
  if( this != &rhs )
  {
    m_scalar = rhs.m_scalar;
    m_vector = rhs.m_vector;
  }
  return (*this );
}

inline const PrQuaternion& PrQuaternion::identity()
{
  m_scalar = 1.0;
  m_vector.zero();
  return *this;
}

inline void PrQuaternion::values( Float scalar, Float vector0,
                                  Float vector1, Float vector2 )
{
  m_scalar = scalar;
  m_vector.values( vector0, vector1, vector2 ); 
}

inline void PrQuaternion::values( const PrVector& axis, Float angle )
{
  SAIAssert( axis.size() == 3 );
  m_scalar = (Float) cos( 0.5 * angle );
  axis.multiply( (Float) sin( 0.5 * angle ), m_vector );
}

inline PrMatrix3 PrQuaternion::rotationMatrix() const
{
  PrMatrix3 tmp;
  rotationMatrix(tmp);
  return tmp;
}

inline PrMatrix3 PrQuaternion::matrix() const
{
  PrMatrix3 tmp;
  matrix(tmp);
  return tmp;
}

inline PrVector3 PrQuaternion::column( int col ) const
{
  PrVector3 tmp;
  column(col, tmp);
  return tmp;
}

inline PrQuaternion PrQuaternion::operator-() const
{
  return PrQuaternion( -m_scalar, -m_vector[0], -m_vector[1], -m_vector[2] );
}

inline PrQuaternion PrQuaternion::operator~() const
{
  return PrQuaternion( m_scalar, -m_vector[0], -m_vector[1], -m_vector[2] );
}

inline void PrQuaternion::inverse( PrQuaternion& dest ) const
{
  dest.m_scalar = m_scalar;
  m_vector.negate(dest.m_vector);
}

inline PrQuaternion PrQuaternion::operator+( const PrQuaternion& rhs ) const
{
  return PrQuaternion( m_scalar + rhs.m_scalar,
                       m_vector[0] + rhs.m_vector[0],
                       m_vector[1] + rhs.m_vector[1],
                       m_vector[2] + rhs.m_vector[2] );
}

inline void PrQuaternion::add( const PrQuaternion& rhs,
                               PrQuaternion& dest ) const
{
  dest.m_scalar = m_scalar + rhs.m_scalar;
  m_vector.add( rhs.m_vector, dest.m_vector );
}

inline PrQuaternion PrQuaternion::operator-( const PrQuaternion& rhs ) const
{
  return PrQuaternion( m_scalar - rhs.m_scalar,
                       m_vector[0] - rhs.m_vector[0],
                       m_vector[1] - rhs.m_vector[1],
                       m_vector[2] - rhs.m_vector[2] );
}

inline void PrQuaternion::subtract( const PrQuaternion& rhs,
                                    PrQuaternion& dest ) const
{
  dest.m_scalar = m_scalar - rhs.m_scalar;
  m_vector.subtract( rhs.m_vector, dest.m_vector );
}

inline PrQuaternion PrQuaternion::operator*( const PrQuaternion& rhs ) const
{
  PrQuaternion tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector3 PrQuaternion::operator*( const PrVector& rhs ) const
{
  PrVector3 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrQuaternion PrQuaternion::operator*( Float rhs ) const
{
  return PrQuaternion( m_scalar * rhs,
                       m_vector[0] * rhs,
                       m_vector[1] * rhs,
                       m_vector[2] * rhs );
}

inline void PrQuaternion::multiply( Float rhs, PrQuaternion& dest ) const
{
  dest.m_scalar = m_scalar * rhs;
  m_vector.multiply( rhs, dest.m_vector );
}

inline PrQuaternion PrQuaternion::operator/( Float rhs ) const
{
  Float rhsInv = 1 / rhs;
  return PrQuaternion( m_scalar * rhsInv,
                       m_vector[0] * rhsInv,
                       m_vector[1] * rhsInv,
                       m_vector[2] * rhsInv );
}

inline PrQuaternion& PrQuaternion::operator+=( const PrQuaternion& rhs )
{
  m_scalar += rhs.m_scalar;
  m_vector += rhs.m_vector;
  return *this;
}

inline PrQuaternion& PrQuaternion::operator-=( const PrQuaternion& rhs )
{
  m_scalar -= rhs.m_scalar;
  m_vector -= rhs.m_vector;
  return *this;
}

inline PrQuaternion& PrQuaternion::operator*=( const PrQuaternion& rhs )
{
  PrQuaternion lhs = *this;
  lhs.multiply( rhs, *this );
  return *this;
}

inline PrQuaternion& PrQuaternion::operator*=( Float rhs )
{
  m_scalar *= rhs;
  m_vector *= rhs;
  return *this;
}

inline PrQuaternion& PrQuaternion::operator/=( Float rhs )
{
  Float rhsInv = 1 / rhs;
  m_scalar *= rhsInv;
  m_vector *= rhsInv;
  return *this;
}

inline Float PrQuaternion::dot( const PrQuaternion& rhs ) const
{
  return ( m_scalar * rhs.m_scalar + m_vector.dot( rhs.m_vector ) );
}

inline void PrQuaternion::normalize()
{
  ( *this ) /= (Float) sqrt( dot( *this ) );
}

inline PrVector3 PrQuaternion::angularError( const PrQuaternion& qd ) const
{
  PrVector3 tmp;
  angularError( qd, tmp );
  return tmp;
}

inline PrQuaternion PrQuaternion::velocity( const PrVector& omega ) const
{
  PrQuaternion tmp;
  velocity( omega, tmp );
  return tmp;
}

#endif // _PrQuaternion_h



