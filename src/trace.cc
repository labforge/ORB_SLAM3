
extern "C"{
#define _GNU_SOURCE
#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <dlfcn.h>
#include <link.h>
#include <getopt.h>
#include <stdio.h>

void __cyg_profile_func_enter(void *this_fn, void *call_site)
                              __attribute__((no_instrument_function));
void __cyg_profile_func_exit(void *this_fn, void *call_site)
                             __attribute__((no_instrument_function)); 
static FILE *fp_trace;
 
void
__attribute__ ((constructor))
trace_begin (void)
{
 fp_trace = fopen("trace.out", "w");
}
 
void
__attribute__ ((destructor))
trace_end (void)
{
 if(fp_trace != NULL) {
 fclose(fp_trace);
 }
}
/* 
void
__cyg_profile_func_enter (void *func,  void *caller)
{
 if(fp_trace != NULL) {
 fprintf(fp_trace, "e %p %p %lu\n", func, caller, time(NULL) );
 }
}
*/
/*
void
__cyg_profile_func_exit (void *func, void *caller)
{
 if(fp_trace != NULL) {
 fprintf(fp_trace, "x %p %p %lu\n", func, caller, time(NULL));
 }
}
*/
void __cyg_profile_func_enter (void *func,  void *caller)
{
    if(fp_trace != NULL) {
        Dl_info a, b;
        struct link_map* link_mapa;
        struct link_map* link_mapb;
        dladdr1((void*)func,&a,(void**)&link_mapa,RTLD_DL_LINKMAP);
        dladdr1((void*)caller,&b,(void**)&link_mapb,RTLD_DL_LINKMAP);
        fprintf(fp_trace, "e %p %p %lu\n", func-link_mapa->l_addr, caller-link_mapb->l_addr, time(NULL) );
    }
}

void __cyg_profile_func_exit (void *func, void *caller)
{
    if(fp_trace != NULL) {
        Dl_info a, b;
        struct link_map* link_mapa;
        struct link_map* link_mapb;
        dladdr1((void*)func,&a,(void**)&link_mapa,RTLD_DL_LINKMAP);
        dladdr1((void*)caller,&b,(void**)&link_mapb,RTLD_DL_LINKMAP);
        fprintf(fp_trace, "x %p %p %lu\n", func-link_mapa->l_addr, caller-link_mapb->l_addr, time(NULL) );
    }
}
}
