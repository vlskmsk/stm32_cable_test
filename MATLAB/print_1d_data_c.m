function print_1d_data_c(var, varName)
fid = fopen('matlab_data.c','wt');

len = length(var);
fprintf(fid,'#include "matlab_data.h" \n\n');
fprintf( fid,'float %s[%s_SIZE] = {\n',varName,upper(varName));
for i = 1:len
    fprintf(fid,'%f, \n', var(i));
end
fprintf(fid,'};\n');

fclose(fid);

fid = fopen('matlab_data.h','wt');

fprintf(fid,'#ifndef %s_H\n',upper(varName));
fprintf(fid,'#define %s_H\n\n\n',upper(varName));
fprintf(fid,'#define %s_SIZE %d\n\n\n',upper(varName),len);
fprintf( fid,'extern float %s;\n\n',varName);

fprintf(fid,'#endif\n');
fclose(fid);
end