/*
 * string_user.c
 *
 *  Created on: Feb 12, 2022
 *      Author: HAOHV6
 */
#include "../Mylib/string_user.h"

void String_process_backup_message(char* input, long t_on)
{
	char time_string_wait_fix[15] = {0};
	char time_string_fixed[15] = {0};
	char mess_fix_time[500] = {0};
	for(int i = 0; i < strlen(input); i++)
	{
		if(input[i] == 'T' && input[i-1] == '"' && input[i+1] == '"' && input[i+2] == ':')
		{
			int index = 0;
			for(int j =i+3; j < i+3+15; j++)
			{
				if(input[j] == ',')
				{
					break;
				}
				time_string_wait_fix[index++] = input[j];
			}
			break;
		}
	}
	long time_number = atol(time_string_wait_fix) + t_on;
	sprintf(time_string_fixed, "%ld", time_number);
	replace_sub_string(input, time_string_wait_fix, time_string_fixed,  mess_fix_time);
	memset(input, 0, strlen(input));
	strcpy(input, mess_fix_time);
}
void replace_sub_string(char* str, char* substr, char* replace, char* output)
{
     int i = 0, j = 0, flag = 0, start = 0;
     for(int i = 0; i < strlen(str); i ++)
     {
         if(str[i] == 'T' && str[i-1] == '"' && str[i+1] == '"' && str[i+2] == ':')
         {
             start = i+3;
             for(int j = i+3; j < strlen(str); j++)
             {
                if(strcmp(str + start, substr))
                {
                    flag = true;
                }
             }
             break;
         }
     }


    if (flag)
    {
            for (i = 0; i < start; i++)
                    output[i] = str[i];
            // replace substring with another string
            for (j = 0; j < strlen(replace); j++)
            {
                    output[i] = replace[j];
                    i++;
            }

            // copy remaining portion of the input string "str"
            for (j = start + strlen(substr); j < strlen(str); j++)
            {
                    output[i] = str[j];
                    i++;
            }

            // print the final string
            output[i] = '\0';
            //printf("Output: %s\n", output);
    } else {
            //printf("%s is not a substring of %s\n", substr, str);
    }
        return;
}
int filter_comma_sms(char *respond_data, char *word1, char *output)
{
    memset(output, 0, strlen(output));
    char *str;
    str = strstr(respond_data, word1);
//    printf("%s", str);
    for(int i = 0; i <strlen(str); i++)
    {
        output[i] = str[i];
        if(str[i] == '\n' || i == strlen(str) - 1)
        {
            output[i+1] = '\0';
            break;
        }
    }

    return 0;
}
int filter_comma(char *respond_data, int begin, int end, char *output, char exChar)
{
	memset(output, 0, strlen(output));
	int count_filter = 0, lim = 0, start = 0, finish = 0,i;
	for (i = 0; i < strlen(respond_data); i++)
	{
		if ( respond_data[i] == exChar)
		{
			count_filter ++;
			if (count_filter == begin)			start = i+1;
			if (count_filter == end)			finish = i;
		}

	}
	if(count_filter < end)
	{
	    finish = strlen(respond_data);
	}
	lim = finish - start;
	for (i = 0; i < lim; i++){
		output[i] = respond_data[start];
		start ++;
	}
	output[i] = 0;
	return 0;
}
int decodeMessage(char *str_in, char *str_out)
{
    int index = 0;
    int num_x =0;
    int n = 0;
    char str[500];
    for(int i = 0; i < strlen(str_in); i++)
    {
        if(i %4 == 2 || i %4 == 3)
        {
            str[index] = str_in[i];
            index++;
        }
    }
    char hex[3];
    int num = 0;
    for(int i = 0; i < index; i++)
    {
        if(i%2 == 0)
        {
            num = 0;
            hex[num] = str[i];
            num++;
        }
        else
        {
            hex[num] = str[i];
            num_x = (int)strtol(hex, NULL, 16);
            str_out[n] = num_x;
            n++;
        }
    }
    str_out[n]='\0';
    return 0;
}
