// *******************************************************************
// PrTransform.h
//
// This class provides an affine transform [R, P], where R is a
// rotation and P is a translation.
//
// modification history
// --------------------
//
// 06/21/04: Dan Merget: Added comments, made compatible with new
//                       PrMatrix6 and PrVector6 structures
// 11/20/97: K.C. Chang: added inline methods.
// 11/06/97: K.C. Chang: created.
// *******************************************************************
#ifndef _PrTransform_h
#define _PrTransform_h

#include "PrGlobalDefn.h"
#include "PrVector3.h"
#include "PrQuaternion.h"
#include "PrVector6.h"
#include "PrMatrix6.h"

// ===================================================================
// PrTransform class declaration
// ===================================================================

class PrTransform
{
public:
  // -----------------------------------------------------------------
  // Constructors & Destructors
  // -----------------------------------------------------------------
  PrTransform() {}  // initialized to [I 0]
  PrTransform( const PrTransform& rhs );
  PrTransform( const PrQuaternion& rot, const PrVector& trans );
  PrTransform( const PrQuaternion& rot ) : m_rot( rot ) {}
  PrTransform( const PrVector& trans )   : m_trans( trans ) {}
  ~PrTransform() {}

  // -----------------------------------------------------------------
  // Access methods
  // -----------------------------------------------------------------

  void identity();

  PrQuaternion&       rotation()          { return m_rot; }
  const PrQuaternion& rotation()    const { return m_rot; }
  PrVector3&          translation()       { return m_trans; }
  const PrVector3&    translation() const { return m_trans; }

  // -----------------------------------------------------------------
  // Arithmetic operations
  // -----------------------------------------------------------------

  PrTransform& operator=( const PrTransform& rhs );

  // [r1, p1] * [r2, p2] = [r1*r2, r1*p2 + p1]
  PrTransform operator*( const PrTransform& rhs ) const;
  void multiply( const PrTransform& rhs, PrTransform& dest ) const;

  // [r, p] * v = r*v + p
  PrVector3 operator*( const PrVector& v ) const;
  void multiply( const PrVector& v, PrVector& dest ) const;

  PrTransform& operator*=( const PrTransform& rhs );

  // ~[r, p] = [~r, -(~r * p)]
  PrTransform operator~() const;
  void inverse( PrTransform& dest ) const;

  // -----------------------------------------------------------------
  // Transformations applied to 6x1 vectors,
  // where the vector consists of the linear & angular velocity,
  // or the linear & angular error.
  // -----------------------------------------------------------------

  // e = T - Td
  PrVector6 error( const PrTransform& Td );
  void error( const PrTransform& Td, PrVector& dest );

  PrVector6 Xform( const PrVector6& rhs ) const;
  PrVector6 Xform( const PrVector& rhs ) const;
  void Xform( const PrVector6& rhs, PrVector6& dest ) const;
  void Xform( const PrVector& rhs, PrVector& dest ) const;

  PrVector6 XformT( const PrVector6& rhs ) const;
  PrVector6 XformT( const PrVector& rhs ) const;
  void XformT( const PrVector6& rhs, PrVector6& dest ) const;
  void XformT( const PrVector& rhs, PrVector& dest ) const;

  // -----------------------------------------------------------------
  // Miscellaneous
  // -----------------------------------------------------------------
  void display( const char* name = NULL ) const;

private:
  PrQuaternion m_rot;   // rotation
  PrVector3    m_trans; // translation
};

// ===================================================================
// Inline methods
// ===================================================================
inline PrTransform::PrTransform( const PrTransform& rhs )
  : m_rot( rhs.m_rot ), m_trans( rhs.m_trans )
{
}

inline PrTransform::PrTransform( const PrQuaternion& rot,
                                 const PrVector& trans )
  : m_rot( rot ), m_trans( trans )
{
}

inline void PrTransform::identity()
{
  m_rot.identity();
  m_trans.zero();
}

inline PrTransform& PrTransform::operator=( const PrTransform& rhs )
{
  if( this != &rhs )
  {
    m_rot   = rhs.m_rot;
    m_trans = rhs.m_trans;
  }
  return (*this );
}

inline PrTransform PrTransform::operator*( const PrTransform& rhs ) const
{
  PrTransform tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrVector3 PrTransform::operator*( const PrVector& rhs ) const
{
  PrVector3 tmp;
  multiply( rhs, tmp );
  return tmp;
}

inline PrTransform& PrTransform::operator*=( const PrTransform& rhs )
{
  PrTransform lhs = *this;
  lhs.multiply( rhs, *this );
  return *this;
}

inline PrTransform PrTransform::operator~() const
{
  PrTransform tmp;
  inverse( tmp );
  return tmp;
}

inline PrVector6 PrTransform::Xform( const PrVector6& rhs ) const
{
  PrVector6 tmp;
  Xform( rhs, tmp );
  return tmp;
}

inline PrVector6 PrTransform::Xform( const PrVector& rhs ) const
{
  PrVector6 tmp;
  Xform( rhs, tmp );
  return tmp;
}

inline PrVector6 PrTransform::XformT( const PrVector6& rhs ) const
{
  PrVector6 tmp;
  XformT( rhs, tmp );
  return tmp;
}

inline PrVector6 PrTransform::XformT( const PrVector& rhs ) const
{
  PrVector6 tmp;
  XformT( rhs, tmp );
  return tmp;
}

#endif // _PrTransform_h

