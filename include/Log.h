
#ifndef LOG_H
#define LOG_H

#include <stdio.h>
namespace ORB_SLAM3
{
//#define FN_ENTRY_LOG if(NULL!=log_file){fprintf(log_file,"FILE:%s,FUNC:%s,LINE:%d",__FILE__,__func__,__LINE__);}
//#define FN_ENTRY_LOG printf("FILE:%s,FUNC:%s,LINE:%d",__FILE__,__func__,__LINE__);
#define FN_ENTRY_LOG fprintf(log_file,"FILE:%s, FUNC:%s, LINE:%d\n",__FILE__,__func__,__LINE__);
#define DBG_LOG(str) fprintf(log_file,"FILE:%s, FUNC:%s, LINE:%d MSG: %s\n",__FILE__,__func__,__LINE__,str);
}
#endif
