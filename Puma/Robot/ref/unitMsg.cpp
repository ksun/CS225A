#include "unitMsg.h"


AMsg::AMsg()
: m_bufferSize( MAX_MSG_SIZE )
, m_mesgType( -1 )
, m_szBuf( NULL )
, m_fBuffer( true )
{
  InitByteCounter();

  // little endian
  //   [0] [1] [2] [3]
  //    1   0   0   0
  // big endian
  //   [0] [1] [2] [3]
  //    0   0   0   1
  int aint=1;
  char *cptr = (char *)&aint;
  if (*cptr == '\0')
    m_fEndian = PR_BIG_ENDIAN; // 1 => big endian
  else
    m_fEndian = PR_LITTLE_ENDIAN;

}

void AMsg::Init()
{
  //m_bufferSize = MAX_MSG_SIZE; // is fixed
  memset(m_szBuf,'\0',m_bufferSize);
  InitByteCounter();
}

int AMsg::InitByteCounter()
{
  m_byteCounter = 4; // 2*sizeof(short);
  return m_byteCounter;
}


int AMsg::Pack( const char *data, int num, int size )
{
  assert( m_fBuffer );
  char *text = m_szBuf + m_byteCounter;
  int i;
  switch(size)
  {
  case 4:
    for(i=0;i<num;i++)
      Pack4B(text+i*4,*(const long *)(data+i*4));
    break;
  case 2:
    for(i=0;i<num;i++)
      Pack2B(text+i*2,*(const short *)(data+i*2));
    break;
  case 1:
    for(i=0;i<num;i++)
      Pack1B(text+i,data[i]);
    break;
  case 8:
    for(i=0;i<num;i++)
      Pack8B(text+i*8,*(double *)(data+i*8));
    break;
  }
  m_byteCounter += size*num;
  assert( m_byteCounter <= MAX_MSG_SIZE );
  return (size*num);
}


int AMsg::Unpack( char *data, int num, int size ) const
{
  const char *text = m_szBuf + m_byteCounter;
  int i;
  switch(size)
  {
  case 4:
    for(i=0;i<num;i++)
      *(long *)(data+i*4) = Unpack4B(text+i*4);
    break;
  case 2:
    for(i=0;i<num;i++)
      *(short *)(data+i*2) = Unpack2B(text+i*2);
    break;
  case 1:
    for(i=0;i<num;i++)
      data[i] = Unpack1B(text+i);
    break;
  case 8:
    for(i=0;i<num;i++)
      *(double *)(data+i*8) = Unpack8B(text+i*8);
    break;
  }

  (const_cast<AMsg*>(this))->m_byteCounter += size*num;
  assert( m_byteCounter <= MAX_MSG_SIZE );
  return (size*num);
}
