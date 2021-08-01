#ifndef _PINGCHECKER_H_
#define _PINGCHECKER_H_ 

#include <string>
int SetupPingChecker(std::string strAddr);
int GetPing(long double &ldJitterTime_ms);

#endif /* ifndef _PINGCHECKER_H_ */
