#include <iostream>
#include <string>
#include "RobotCom.h"
using namespace std;

//static const string DEFAULT_SERVER = "192.168.2.4";//"garuda.stanford.edu"; //"alpes.stanford.edu";
//static const string DEFAULT_SERVER = "192.168.0.101";
static const string DEFAULT_SERVER = "172.24.68.149";

/*********************************************************************
 * RobotCom constructor:  Create a socket connection to the system that
 * communicates with the robot.
 */
RobotCom::RobotCom()
{
	InitByteCounter();
	bufferSize_ = BUFFER_SIZE; //nbytes + InitByteCounter();
	buffer_ = new char[bufferSize_];

	memset(buffer_,'\0',bufferSize_);
	readSize_ = 1;


#ifdef WIN32
	// Initialize the WINSOCK module.  Since this is the only object
    // in the program that uses WINSOCK, we hide this initialization
    // here.
    //
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2,2), &wsaData);
#endif

	// Create the socket
    //
	robotSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	
	// Ask the user for the name of the robot server,
    // and connect the socket.
    //
	bool socketIsOpen = false;
	while (!socketIsOpen) {
		int rc;
		
		string robotServer;
		cout << "Please enter the name of the robot server ["
			<< DEFAULT_SERVER << "]: " << flush;
		
		getline(cin, robotServer);

		if (robotServer.empty()) {
			robotServer = DEFAULT_SERVER;
		}

		cout << "You entered "<<robotServer<<endl;

		// connect()
#ifdef WIN32
		SOCKADDR_IN serveraddr;
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(PR_NETWORK_PORT_CONTROL);
        serveraddr.sin_addr.s_addr = inet_addr( robotServer.c_str() );
#else
        struct sockaddr_in serveraddr;

        bzero((char *) &serveraddr, sizeof(serveraddr));

        serveraddr.sin_family = AF_INET;

        struct hostent *server;
        server = gethostbyname(robotServer.c_str());
        if (server == NULL) {fprintf(stderr,"ERROR, no such host\n"); exit(0);}
        bcopy((char *)server->h_addr,
              (char *)&serveraddr.sin_addr.s_addr,
              server->h_length);


        serveraddr.sin_port = htons(PR_NETWORK_PORT_CONTROL);
#endif

  
#ifdef WIN32
		rc = connect( robotSocket, (SOCKADDR *)&serveraddr, sizeof(serveraddr) );
#else
        rc = connect( robotSocket, (struct sockaddr *) &serveraddr, sizeof(serveraddr) );
#endif
		if (rc != 0) {
			cerr << "Error: cannot open socket to '" << robotServer
				<< "' port " << PR_NETWORK_PORT_CONTROL << ". Try again."
				<< endl;
			continue;
		}else
		{
			cout << "Connected!" << endl;
		}


		socketIsOpen = true;
	}
	
	// Make the socket non-blocking
	setSocketBlock( false );
}

void RobotCom::setSocketBlock( bool fBlock )
{
	// Make the socket non-blocking
	//
	unsigned long nonBlocking;
	if( !fBlock )
		nonBlocking = 1;
	else
		nonBlocking = 0;

#ifdef WIN32
	ioctlsocket(robotSocket, FIONBIO, &nonBlocking);
#else
   int flags = fcntl(robotSocket, F_GETFL, 0);
   if (flags < 0) {
       fprintf(stderr,"setBlocking. something wrong with socket");
       exit(0);
   }
   if (fBlock){
       flags = flags & ~O_NONBLOCK;
   } else {
       flags = flags | O_NONBLOCK;
   }
   flags = fcntl(robotSocket, F_SETFL, flags);
   if (flags < 0) {
       fprintf(stderr,"setBlocking. error setting.");
       exit(0);
   }
#endif
	
	//bufferPtr  = &buffers[0][0];    
	//bufferSize = 0;
}

/*********************************************************************
 * RobotCom destructor
 */
RobotCom::~RobotCom()
{
	delete[] buffer_;

	_break();
#ifdef WIN32
    closesocket(robotSocket);
    WSACleanup();
#else
    close(robotSocket);
#endif
}



/*********************************************************************
 * Assorted outgoing messages
 */
/*
void RobotCom::jtime()
{
    CMsg mOut;
    mOut.WriteMessageType(JTIME);
    mOut.WriteInt(VISION);
    sendMessage(mOut);
}

void RobotCom::getCalData()
{
    CMsg mOut;
    mOut.WriteMessageType(GET_CAL_DATA);
    sendMessage(mOut);
}
*/

void  RobotCom::_float()
{
    CMsg mOut;
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(FLOATMODE);
    sendMessage(mOut);
}

void RobotCom::control( ControlMode mode, float *arg, int numArgs )
{
    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(mode);
	if( numArgs > 0 )
		mOut.WriteFloat( arg, numArgs );
    sendMessage(mOut);
}

void RobotCom::jointControl( ControlMode mode, float q0, float q1, float q2, float q3, float q4, float q5)
{
	if( mode != NJMOVE && mode != JMOVE
	  && mode != NJGOTO && mode != JGOTO
	  && mode != NJTRACK && mode != JTRACK )
	  return;

    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(CONTROL_MODE);
    mOut.WriteInt(mode);
    mOut.WriteFloat(q0);
    mOut.WriteFloat(q1);
    mOut.WriteFloat(q2);
    mOut.WriteFloat(q3);
    mOut.WriteFloat(q4);
    mOut.WriteFloat(q5);
    sendMessage(mOut);
}

void RobotCom::controlGripper( float voltage )
{
    if( voltage < -10.0 || 10.0 < voltage ) return;

    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType(GRIPPER);
	mOut.WriteInt( (int)(voltage * 4096.0 / 10.0) );
	sendMessage(mOut);
}


void RobotCom::_break() {
    CMsg mOut;
    mOut.WriteMessageType(BREAK);
    sendMessage(mOut);
}
//void Robot::_reset() {
//    CMsg mOut;
//    mOut.WriteMessageType(RESET);
//    sendMessage(mOut);
//}

// get_type = GET_CURTIME (gv.curTime),
//            GET_JPOS (gv.q), GET_JVEL (gv.dq), GET_TORQ (gv.tau), 
//            GET_IPOS (gv.x)
void RobotCom::getStatus( UiToServoMessageType get_type, float *arg )
{
	if( get_type != GET_CURTIME && get_type != GET_JPOS
		&& get_type != GET_JVEL && get_type != GET_TORQ
		&& get_type != GET_IPOS )
		return;

    CMsg mOut;
	mOut.Init();
    mOut.WriteMessageType( get_type );
	mOut.WriteInt( GUI );
	sendMessage(mOut);

	short expectedMesgType;
	int numOfData;

	switch( get_type )
	{
	case GET_CURTIME:
		expectedMesgType = CURTIME_DATA;
		numOfData = 1;
		break;
	case GET_JPOS:
		expectedMesgType = JPOS_DATA;
		numOfData = 6;
		break;
	case GET_JVEL:
		expectedMesgType = JVEL_DATA;
		numOfData = 6;
		break;
	case GET_TORQ:
		expectedMesgType = TORQ_DATA;
		numOfData = 6;
		break;
	case GET_IPOS:
		expectedMesgType = IPOS_DATA;
		numOfData = 7;
		break;
	}

	for(;;)
	{
		if( IsDataAvailable() )
		{
			CMappedMsg mIn = GetMsg();
			short mesgType = mIn.ReadMessageType();
			//printf("Type = %d\n", mesgType );

			if( mesgType == expectedMesgType )
			{
				mIn.ReadFloat( arg, numOfData );
				return;
			}
		}
	}
}



/*********************************************************************
 * sendMessage(): Send a message to the robot.
 */
void RobotCom::sendMessage(AMsg &mOut)
{
    char *ptr;
    int bytesLeft = mOut.GetRawMsg(ptr);
    while (bytesLeft > 0) {
        int sendSize = send(robotSocket, ptr, bytesLeft, 0);
        if (sendSize <= 0) {
            processBrokenSocket();
            return;
        }

        ptr += sendSize;
        bytesLeft -= sendSize;
    }
}

/*********************************************************************
 * processIncomingMessages(): Process any messages received from the
 * robot.  If the robot hasn't sent any, then return immediately.
 */

CMappedMsg RobotCom::GetMsg()
{
  CMappedMsg msg;
  msg.SetMsg( buffer_, size_ );
  return msg;
}

int RobotCom::ready()
{
  size_ = 0;
  if (readSize_ == 1)
  {
    char s[2];
    if (Peek(s,2) < 2)
      return -1;
    size_ = Unpack2B(s);
    //fprintf(stderr, "got message of size %d\n", size_);
    memset(buffer_,'\0',bufferSize_);
    InitByteCounter();
    readSize_ = 0;
  }

  if (Peek() < size_ )
    return -1;

  if (Receive() != size_)
  {
    perror("PrDataInput: ready() error");
    return -2;
  }

  //type_ = Unpack2B(buffer_+2);

  readSize_ = 1;
  return size_;// - InitByteCounter());
}

// p279 in UNIX Network Programming by Stevens
int RobotCom::Receive()
{
  char *ptr = buffer_;

  int nleft,nread;
  nleft = size_;
  while(nleft>0)
  {
    nread = recv(robotSocket,ptr,nleft,0);
    if(nread<0) return (nread); /* error */
    else if(nread==0) break; /* EOF */
    nleft -= nread;
    ptr += nread;
  }
  return(size_ - nleft);  // return >= 0
}

int RobotCom::Peek()
{

  int arg=0;
#ifdef WIN32
  if ( ioctlsocket( robotSocket, FIONREAD, (unsigned long FAR*) &arg ) < 0 )
#else
  if (ioctl(robotSocket,FIONREAD,&arg)<0)
#endif
  {
    perror("RobotCom::Peek():ioctl() error");
    arg = -1;
  }

  //        fprintf( stderr, "peek %d\n", arg );
  return arg;
}

int RobotCom::Peek( char *buff, int nbytes )
{
  int arg = Peek();
  if (arg >= nbytes)
  {
    if (recv(robotSocket,buff,nbytes,MSG_PEEK)!=nbytes)
    {
      perror("RobotCom::Peek():recv() error");
      arg = -1;
    }
  }
  return arg;
}

short RobotCom::Unpack2B( char *text )
{
  return ntohs(*(short *)text);
  //return *(short *)text;
}

/*********************************************************************
 * processBrokenSocket(): Called when the socket connection to the
 * robot is broken.
 */
void RobotCom::processBrokenSocket()
{
	printf("error: processBrokenSocket()\n");
    return;
}


int XPrintf( const char* fmt, ... )
{
  /*
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
  */
    return 0;
}
