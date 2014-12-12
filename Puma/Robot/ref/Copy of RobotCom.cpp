#include <iostream>
#include <string>
#include "RobotCom.h"
using namespace std;

/* CHANGE TO ROBOT */
//static const string DEFAULT_SERVER = "perse.stanford.edu"; //"alpes.stanford.edu";
static const string DEFAULT_SERVER = "192.168.2.4";

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



	// Initialize the WINSOCK module.  Since this is the only object
    // in the program that uses WINSOCK, we hide this initialization
    // here.
    //
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2,2), &wsaData);

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

		/*
		sockaddr_in robotAddr;
		hostent *robotHost;
		if (isalpha(robotServer[0])) {
			robotHost = gethostbyname(robotServer.c_str());
		} else {
			robotHost = gethostbyaddr(robotServer.c_str(), 4, AF_INET);			
		}
		
		if (robotHost == NULL) {
			cerr << "Error: cannot find host '" << robotServer
				<< "'.  Try again." << endl;
			continue;
		}
		memset(&robotAddr, 0, sizeof(robotAddr));
		robotAddr.sin_family = AF_INET;
		robotAddr.sin_port   = htons(PR_NETWORK_PORT_CONTROL);
		robotAddr.sin_addr   = *(in_addr*)robotHost->h_addr_list[0];
		*/

		sockaddr_in robotAddr;
		hostent *robotHost;

		if (isalpha(robotServer[0])) {
			robotHost = gethostbyname(robotServer.c_str());
			if (robotHost == NULL) {
				cerr << "Error: cannot find host '" << robotServer
					<< "'.  Try again." << endl;
				continue;
			}

			memset(&robotAddr, 0, sizeof(robotAddr));
			robotAddr.sin_family = AF_INET;
			robotAddr.sin_port   = htons(PR_NETWORK_PORT_CONTROL);
			robotAddr.sin_addr   = *(in_addr*)robotHost->h_addr_list[0];
		} else { //! ip address by numbers
			//!! this gethostbyaddr() does not work well
			//robotHost = gethostbyaddr(robotServer.c_str(), 4, AF_INET);			

			memset(&robotAddr, 0, sizeof(robotAddr));
			robotAddr.sin_family = AF_INET;
			robotAddr.sin_port   = htons(PR_NETWORK_PORT_CONTROL);
			//robotAddr.sin_addr   = *(in_addr*)robotHost->h_addr_list[0];
			robotAddr.sin_addr.s_addr = inet_addr( robotServer.c_str() );
		}
	
		rc = connect(robotSocket, (sockaddr*)(&robotAddr), sizeof(robotAddr));
		if (rc != 0) {
			cerr << "Error: cannot open socket to '" << robotServer
				<< "' port " << PR_NETWORK_PORT_CONTROL << ". Try again."
				<< endl;
			continue;
		}

		socketIsOpen = true;
	}
	
	// Make the socket non-blocking
	//
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

	ioctlsocket(robotSocket, FIONBIO, &nonBlocking);
	
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
    closesocket(robotSocket);
    WSACleanup();
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
#ifndef WIN32
#ifdef PR_QNX
  if (ioctl((short)socketFD_,FIONREAD,&arg)<0)
#else // PR_QNX
  if (ioctl(socketFD_,FIONREAD,&arg)<0)
#endif // PR_QNX
#else //#ifndef WIN32
  if ( ioctlsocket( robotSocket, FIONREAD, (u_long FAR*) &arg ) < 0 )
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

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32