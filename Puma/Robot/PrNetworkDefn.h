/***************************************************************

* PrNetworkDefn.h

*

* This files contains global network constants.

*

****************************************************************/



/*

* modification history

*----------------------

*

* 02/17/98: K.C. Chang: created.

*/



#ifndef _PrNetworkDefn_h

#define _PrNetworkDefn_h



#define PR_BIG_ENDIAN    0

#define PR_LITTLE_ENDIAN 1



#define PR_NETWORK_BUFFER_SIZE     1020  // = 1024 - 2 - 2

#define PR_NETWORK_PORT_CONTROL    8189

#define PR_NETWORK_PORT_MOVEPROXY  8289




// *CHANGE */
//#define SERV_HOST_ADDR "171.64.68.143" // mann.stanford.edu

//#define SERV_HOST_ADDR "171.64.68.17" // mann.stanford.edu

//#define SERV_HOST_ADDR "172.24.68.108" // perse.stanford.edu

#define SERV_HOST_ADDR "192.168.2.4" // garuda.stanford.edu

#define SERV_TCP_PORT 8189 





#endif // PrNetworkDefn.h

