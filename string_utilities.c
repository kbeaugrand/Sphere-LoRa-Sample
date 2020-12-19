#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "string_utilities.h"

char *substring(const char *string, int position, size_t length)
{
   char *p;
   int c;
 
   p = (char *)malloc(length + 1);
   
   if (p == NULL)
   {
      exit(1);
   }
 
   for (c = 0; c < length; c++)
   {
      *(p+c) = *(string+position-1);      
      string++;  
   }
 
   *(p+c) = '\0';
 
   return p;
}

char *ltrim(char *s) 
{     
    while(isspace(*s)) s++;     
    return s; 
}  

char *rtrim(char *s) 
{     
    char* back;
    size_t len = strlen(s);

    if(len == 0)
        return(s); 

    back = s + len;     
    while(isspace(*--back));     
    *(back+1) = '\0';     
    return s; 
}  

char *trim(char *s) 
{     
    return rtrim(ltrim(s));  
} 