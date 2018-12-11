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
    
    fprintf('{ %f, %f, ', a(i,2),a(i,3));
    fprintf(fid, '{ %f, %f, ', a(i,2),a(i,3));   
    
    fprintf('%f, %f, %f, ', b(i,1),b(i,2),b(i,3));
    fprintf(fid, '%f, %f, %f, ', b(i,1),b(i,2),b(i,3));
    
    fprintf('%f, {0, 0, 0} }',G(i));    
    fprintf(fid, '%f, {0, 0, 0} }',G(i));    
    
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