#ifndef UNITMSG_H
#define UNITMSG_H

#include <string.h>
#include <assert.h>
#ifdef WIN32
#include <WinSock.h>
#else
#include <netinet/in.h>
#endif
#include "PrGlobalDefn.h"
#include "PrNetworkDefn.h"
#include <stdio.h>

#define MAX_MSG_SIZE 1024


//-----------------------------------------------------------------------------
class AMsg
  //-----------------------------------------------------------------------------
  // Abstract message class
{
protected:
  int m_bufferSize;
  int m_byteCounter;
  int m_fEndian;
  short m_mesgType;
  char* m_szBuf;
  bool m_fBuffer;

  int InitByteCounter();

  int  Pack( const char *data, int num, int size );
  void Pack1B( char *text, char data );
  void Pack2B( char *text, short data );
  void Pack4B( char *text, long data );
  void Pack8B( char *text, double data );

  int    Unpack( char *data, int num, int size ) const;
  char   Unpack1B( const char *text ) const;
  short  Unpack2B( const char *text ) const;
  long   Unpack4B( const char *text ) const;
  double Unpack8B( const char *text ) const;

public:
  AMsg();

  void Init();

  int GetRawMsg( char*& szMsg )
  { 
    Pack2B(m_szBuf,(short)m_byteCounter); // fill in the size of the message
    szMsg = m_szBuf; 
    return m_byteCounter; 
  }

  int GetContent( char*& szMsg ) const
  {
    if( m_byteCounter >= 4 )
      szMsg = m_szBuf+4;
    else
      szMsg = NULL;
    return m_bufferSize-4;
  }

  int GetBufSize() const { return m_bufferSize; }

  virtual void SetMsg( const char* szMsg, int size ) = 0;

  // Write routines:
  int WriteChar( const char *data, int num )   { return Pack(data,num,1); }
  int WriteShort( const short *data, int num ) { return Pack((const char *)data,num,2); }
  int WriteInt( const int *data, int num )     { return Pack((const char *)data,num,4); }
  int WriteLong( const long *data, int num )   { return Pack((const char *)data,num,4); }
  int WriteFloat( const Float *data, int num ) { return Pack((const char *)data,num,sizeof(Float)); }

  int  WriteBool( bool value )    { char val = (value ? '\01' : '\0'); return Pack((const char *)(&val),1,1); }
  int  WriteInt( int value )     { return Pack((const char *)(&value),1,4); }
  int  WriteFloat( Float value ) { return Pack((const char *)(&value),1,sizeof(Float)); }
  int  WriteString( char * str ) { return Pack( str, (int)strlen(str)+1, 1 ); }
  void WriteMessageType( int mesgType ) 
  { 
    //printf("mt=%d\n", mesgType);
    assert( m_fBuffer );
    m_mesgType = (short) mesgType;
    Pack2B(m_szBuf+2,(short)m_mesgType);
  }


  int ReadChar( char *data, int num ) const     { return Unpack(data,num,1); }
  int ReadShort( short *data, int num ) const   { return Unpack((char *)data,num,2); }
  int ReadInt( int *data, int num ) const       { return Unpack((char *)data,num,4); }
  int ReadLong( long *data, int num ) const     { return Unpack((char *)data,num,4); }
  int ReadFloat( Float *data, int num ) const   { return Unpack((char *)data,num,sizeof(Float)); }


  // Read routines:
  bool  ReadBool()  const { char val = '\0'; Unpack(&val,1,1); return val != '\0'; }
  int   ReadInt()   const { int val=0;  Unpack((char *)(&val),1,4); return val; }
  Float ReadFloat() const { Float val=0; Unpack((char *)(&val),1,sizeof(Float)); return val; }
  void  ReadString( char * str, int bufLen ) const { Unpack(str,bufLen,1); }
  int   ReadMessageType() const 
  {
    return m_mesgType;
  }
};





//-----------------------------------------------------------------------------
class CMsg : public AMsg
  //-----------------------------------------------------------------------------
  // message with a local buffer.  message is to be used for unit of information
  // trasfered between server and ui and vice versa.
{
public:
  CMsg() { m_szBuf = &m_locBuf[0]; }

  void SetMsg( const char* szMsg, int size )
  {
    strncpy( m_szBuf, szMsg, size );
    m_mesgType = Unpack2B(m_szBuf+2);
    m_bufferSize = size;
    //fprintf( stderr, "mm of type %d, s=%d; ", m_mesgType, m_bufferSize );
    InitByteCounter();
  }

private:
  char m_locBuf[MAX_MSG_SIZE];
};


//-----------------------------------------------------------------------------
class CMappedMsg : public AMsg
  //-----------------------------------------------------------------------------
  // message whose buffer is mapped to some other memory
{
public:
  CMappedMsg() { m_fBuffer = false; }

  void SetMsg( const char* szMsg, int size )
  {
    m_szBuf = const_cast<char*> (szMsg);
    m_mesgType = Unpack2B(m_szBuf+2);
    m_bufferSize = size;
    //fprintf( stderr, "mm of s=%d; ", m_bufferSize );
    InitByteCounter();
  }

private:
};



//-----------------------------------------------------------------------------
// Inline funcitons
//-----------------------------------------------------------------------------

// Note: since we are commucating between x86 machines, we do not need 
// to conver to network format.  If you are computing between machines of
// different architectures, comment out the conversion functions.
inline void AMsg::Pack1B( char *text, char data )
{
  text[0] = data;
}

inline void AMsg::Pack2B( char *text, short data )
{
  *(short *)text = htons(data);
  //*(short *)text = data;
}

inline void AMsg::Pack4B( char *text, long data )
{
  *(long *)text = htonl(data);
  //*(long *)text = data;
}


typedef union
{
  double d;
  struct 
  {
    long l,h;
  } s;
} RemapDouble;


inline void AMsg::Pack8B( char *text, double data )
{
  RemapDouble x;
  x.d = data;
  if (m_fEndian == PR_LITTLE_ENDIAN)
  {
    Pack4B(text,x.s.l);
    Pack4B(text+4,x.s.h);
  }
  else
  {
    Pack4B(text+4,x.s.l);
    Pack4B(text,x.s.h);
  }
  //*(double *)text = data;
}



inline char AMsg::Unpack1B( const char *text ) const
{
  return text[0];
}

inline short AMsg::Unpack2B( const char *text ) const
{
  return ntohs(*(const short *)text);
  //return *(const short *)text;
}

inline long AMsg::Unpack4B( const char *text ) const
{
  return ntohl(*(const long *)text);
  //return *(const long *)text;
}

inline double AMsg::Unpack8B( const char *text ) const
{
  //return *(const double *)text;
  RemapDouble x;
  if (m_fEndian == PR_LITTLE_ENDIAN)
  {
    x.s.l = Unpack4B(text);
    x.s.h = Unpack4B(text+4);
  }
  else
  {
    x.s.l = Unpack4B(text+4);
    x.s.h = Unpack4B(text);
  }
  return (x.d);
}

#endif
