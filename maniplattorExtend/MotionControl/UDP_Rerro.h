#include <WinSock2.h> 
#define SDV_RESULT_OK (0)
#define SDV_RESULT_SOCK_ERR (1)
#define SDV_RESULT_BIND_ERR (2)
#define SDV_RESULT_HOOK_ERR (3)
#define SDV_RESULT_THREAD_ERR (4)
#define SDV_RESULT_CMD_ERR (5)

typedef long (*StreamProc)(LPVOID buffer,void*pUserData);