function print_iir_filter_c(SOS,G,varName)
fid = fopen('filter.c','wt');

b = SOS(:,1:3);
a = SOS(:,4:6);
len = size(b,1);

fprintf(fid, '\n#define NUM_SECTIONS_%s %d\n',upper(varName),len);
fprintf('\n#define NUM_SECTIONS_%s %d\n',upper(varName),len);

fprintf('iirSOS %s[NUM_SECTIONS_%s] = {\n',varName,upper(varName));
fprintf(fid, 'iirSOS %s[NUM_SECTIONS_%s] = {\n',varName,upper(varName));

for i=1:len
    
    fprintf('    {\n        .a1 = %f,\n        .a2 = %f,\n', a(i,2),a(i,3));
    fprintf(fid, '    {\n        .a1 = %f,\n        .a2 = %f,\n', a(i,2),a(i,3));

    
    fprintf('        .b0 = %f,\n        .b1 = %f,\n        .b2 = %f,\n', b(i,1),b(i,2),b(i,3));
    fprintf(fid, '        .b0 = %f,\n        .b1 = %f,\n        .b2 = %f,\n', b(i,1),b(i,2),b(i,3));
    
    fprintf('        .gain = %f,\n        .w = {0, 0, 0}\n    }',G(i));    
    fprintf(fid, '        .gain = %f,\n        .w = {0, 0, 0}\n    }',G(i));    
    
    if(i<len)
        fprintf(',\n');
        fprintf(fid, ',\n');
    else
        fprintf('\n');
        fprintf(fid, '\n');
    end
end

fprintf('};\n\n');
fprintf(fid, '};\n\n');

fclose(fid);


end