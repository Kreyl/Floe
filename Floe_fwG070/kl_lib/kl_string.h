/*
 * kl_string.h
 *
 *  Created on: 5 дек. 2020 г.
 *      Author: layst
 */

#ifndef KL_STRING_H__
#define KL_STRING_H__

int kl_strcasecmp(const char *s1, const char *s2);
char* kl_strtok(char* s, const char* delim, char**PLast);
int kl_sscanf(const char* s, const char* format, ...);


#endif //KL_STRING_H__